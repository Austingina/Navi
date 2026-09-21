"""
ONNX-exportable wrapper for the HuggingFace SAM3 VisionEncoder.

Wraps Sam3VisionModel (ViT backbone + FPN neck) and returns flat tensors
suitable for ONNX. The last FPN level is dropped to match Sam3Model.forward
semantics.

Input:  pixel_values  [1, 3, 1008, 1008]
Output: fpn_feat_0    [1, 256, 288, 288]   highest-res FPN (for mask decoder)
        fpn_feat_1    [1, 256, 144, 144]   mid-res FPN     (for mask decoder)
        fpn_feat_2    [1, 256,  72,  72]   lowest-res FPN  (DETR encoder + mask decoder)
        fpn_pos_2     [1, 256,  72,  72]   position enc    (DETR encoder)
"""

import torch
import torch.nn as nn


class HFVisionEncoderONNX(nn.Module):
    """Wraps HuggingFace Sam3VisionModel for ONNX export with flat tensor I/O."""

    def __init__(self, vision_encoder: nn.Module):
        super().__init__()
        self.vision_encoder = vision_encoder

    def forward(self, pixel_values: torch.Tensor):
        vision_outputs = self.vision_encoder(pixel_values, return_dict=True)

        fpn = vision_outputs.fpn_hidden_states    # 4 levels
        pos = vision_outputs.fpn_position_encoding  # 4 levels

        # Sam3Model.forward drops the last FPN level:
        #   fpn_hidden_states    = fpn[:-1]  -> levels 0, 1, 2
        #   fpn_position_encoding = pos[:-1]  -> levels 0, 1, 2
        # DETR encoder uses only level 2 features + pos.
        # Mask decoder uses all three levels (0, 1, 2).
        return fpn[0], fpn[1], fpn[2], pos[2]
