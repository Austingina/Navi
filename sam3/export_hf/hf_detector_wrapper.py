"""
ONNX-exportable wrapper for the HuggingFace SAM3 DetectorBody.

Combines text encoding, DETR encoder, DETR decoder, dot-product scoring,
and mask decoder into a single module with flat tensor I/O. Replicates the
non-vision portion of Sam3Model.forward (text-only path, no geometry).

Inputs (all with dynamic batch dimension N = number of text prompts):
    fpn_feat_0      [N, 256, 288, 288]
    fpn_feat_1      [N, 256, 144, 144]
    fpn_feat_2      [N, 256,  72,  72]
    fpn_pos_2       [N, 256,  72,  72]
    input_ids       [N, L]
    attention_mask  [N, L]

Outputs:
    pred_masks      [N, num_queries, Hm, Wm]
    pred_boxes      [N, num_queries, 4]          (xyxy normalised)
    pred_logits     [N, num_queries]
    presence_logits [N, 1]
"""

import types

import torch
import torch.nn as nn


def _inverse_sigmoid(x: torch.Tensor, eps: float = 1e-3) -> torch.Tensor:
    x = x.clamp(min=0, max=1)
    x1 = x.clamp(min=eps)
    x2 = (1 - x).clamp(min=eps)
    return torch.log(x1 / x2)


def _box_cxcywh_to_xyxy(x: torch.Tensor) -> torch.Tensor:
    cx, cy, w, h = x.unbind(-1)
    return torch.stack([cx - 0.5 * w, cy - 0.5 * h, cx + 0.5 * w, cy + 0.5 * h], dim=-1)


def _patch_clip_text_model_for_onnx(text_encoder):
    """Replace the CLIPTextTransformer.forward with an ONNX-friendly version.

    The stock ``create_causal_mask`` dispatcher from HF masking_utils uses
    runtime tensor-shapes in a way that JIT tracing cannot follow.  We
    replace it with a simple lower-triangular boolean mask built from the
    known (traced) sequence length.
    """
    text_model = text_encoder.text_model  # CLIPTextTransformer

    _orig_forward = text_model.forward.__wrapped__ if hasattr(text_model.forward, "__wrapped__") else None

    def _onnx_forward(self, input_ids=None, attention_mask=None, position_ids=None, **kwargs):
        if input_ids is None:
            raise ValueError("input_ids required")
        input_shape = input_ids.size()
        input_ids = input_ids.view(-1, input_shape[-1])
        hidden_states = self.embeddings(input_ids=input_ids, position_ids=position_ids)

        # Build a simple causal mask: True where attention is allowed
        bsz, seq_len, _ = hidden_states.shape
        causal = torch.tril(torch.ones(seq_len, seq_len, device=hidden_states.device, dtype=torch.bool))
        # Expand to [bsz, 1, seq_len, seq_len] and convert to float additive mask
        causal_4d = causal.unsqueeze(0).unsqueeze(0).expand(bsz, 1, seq_len, seq_len)
        attn_mask = torch.where(causal_4d, 0.0, torch.finfo(hidden_states.dtype).min)

        kwargs.pop("is_causal", None)
        encoder_outputs = self.encoder(
            inputs_embeds=hidden_states,
            attention_mask=attn_mask,
            is_causal=True,
            **kwargs,
        )
        last_hidden_state = encoder_outputs.last_hidden_state
        last_hidden_state = self.final_layer_norm(last_hidden_state)

        pooled_output = last_hidden_state[
            torch.arange(last_hidden_state.shape[0], device=last_hidden_state.device),
            input_ids.to(dtype=torch.int, device=last_hidden_state.device).argmax(dim=-1),
        ]
        from transformers.modeling_outputs import BaseModelOutputWithPooling
        return BaseModelOutputWithPooling(
            last_hidden_state=last_hidden_state,
            pooler_output=pooled_output,
        )

    text_model.forward = types.MethodType(_onnx_forward, text_model)


class HFDetectorBodyONNX(nn.Module):
    """Wraps everything after the vision encoder for ONNX export."""

    def __init__(self, sam3_model: nn.Module):
        super().__init__()
        self.text_encoder = sam3_model.text_encoder
        self.text_projection = sam3_model.text_projection
        self.detr_encoder = sam3_model.detr_encoder
        self.detr_decoder = sam3_model.detr_decoder
        self.dot_product_scoring = sam3_model.dot_product_scoring
        self.mask_decoder = sam3_model.mask_decoder

        _patch_clip_text_model_for_onnx(self.text_encoder)

    def forward(
        self,
        fpn_feat_0: torch.Tensor,
        fpn_feat_1: torch.Tensor,
        fpn_feat_2: torch.Tensor,
        fpn_pos_2: torch.Tensor,
        input_ids: torch.Tensor,
        attention_mask: torch.Tensor,
    ):
        # ── 1. Text encoding ────────────────────────────────────────
        text_outputs = self.text_encoder(
            input_ids=input_ids,
            attention_mask=attention_mask,
            return_dict=True,
        )
        text_features = self.text_projection(text_outputs.last_hidden_state)
        text_mask = attention_mask.bool()

        # ── 2. DETR encoder (single level: fpn level 2) ────────────
        encoder_outputs = self.detr_encoder(
            vision_features=[fpn_feat_2],
            text_features=text_features,
            vision_pos_embeds=[fpn_pos_2],
            text_mask=text_mask,
            return_dict=True,
        )

        # ── 3. DETR decoder ─────────────────────────────────────────
        decoder_outputs = self.detr_decoder(
            vision_features=encoder_outputs.last_hidden_state,
            text_features=encoder_outputs.text_features,
            vision_pos_encoding=encoder_outputs.pos_embeds_flattened,
            text_mask=text_mask,
            spatial_shapes=encoder_outputs.spatial_shapes,
            return_dict=True,
        )

        # ── 4. Box refinement + scoring (last decoder layer) ───────
        all_box_offsets = self.detr_decoder.box_head(
            decoder_outputs.intermediate_hidden_states
        )
        ref_inv = _inverse_sigmoid(decoder_outputs.reference_boxes)
        all_pred_boxes = _box_cxcywh_to_xyxy((ref_inv + all_box_offsets).sigmoid())

        all_pred_logits = self.dot_product_scoring(
            decoder_hidden_states=decoder_outputs.intermediate_hidden_states,
            text_features=encoder_outputs.text_features,
            text_mask=text_mask,
        ).squeeze(-1)

        pred_logits = all_pred_logits[-1]
        pred_boxes = all_pred_boxes[-1]
        presence_logits = decoder_outputs.presence_logits[-1]
        decoder_hidden_states = decoder_outputs.intermediate_hidden_states[-1]

        # ── 5. Mask decoder ─────────────────────────────────────────
        mask_outputs = self.mask_decoder(
            decoder_queries=decoder_hidden_states,
            backbone_features=[fpn_feat_0, fpn_feat_1, fpn_feat_2],
            encoder_hidden_states=encoder_outputs.last_hidden_state,
            prompt_features=text_features,
            prompt_mask=text_mask,
            return_dict=True,
        )

        return mask_outputs.pred_masks, pred_boxes, pred_logits, presence_logits
