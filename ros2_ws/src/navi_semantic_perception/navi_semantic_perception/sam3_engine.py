"""Local SAM3 image inference and conversion to Nav2 semantic class IDs."""

from pathlib import Path

import numpy as np


def merge_instances(shape, predictions):
    """Merge (class_id, binary masks, scores); the highest score wins overlaps."""
    labels = np.zeros(shape, dtype=np.uint8)
    confidence = np.zeros(shape, dtype=np.float32)
    for class_id, masks, scores in predictions:
        masks = np.asarray(masks, dtype=bool)
        scores = np.asarray(scores, dtype=np.float32)
        if not 1 <= class_id <= 254:
            raise ValueError('SAM3 foreground class IDs must be in 1..254')
        if masks.shape != (len(scores), *shape) or scores.ndim != 1:
            raise ValueError('SAM3 masks must match the original color image grid')
        for mask, score in zip(masks, scores):
            if not np.isfinite(score) or not 0 <= score <= 1:
                raise ValueError('SAM3 instance scores must be finite probabilities')
            update = mask & (score > confidence)
            labels[update] = class_id
            confidence[update] = score
    return labels, np.rint(confidence * 255).astype(np.uint8)


class Sam3Segmenter:
    """Use the locally validated Hugging Face/PyTorch SAM3 path, offline."""

    def __init__(self, model_path, processor_path, prompts, config_path, device='cuda',
                 precision='bfloat16', score_threshold=0.5, mask_threshold=0.5):
        """Load local weights and cache prompt tokens on the requested device."""
        # Lazy imports keep ROS/configuration tests independent of the GPU environment.
        import torch
        from transformers import (
            AutoImageProcessor, AutoTokenizer, Sam3Config, Sam3Model, Sam3Processor,
        )

        model_path = Path(model_path).expanduser()
        processor_path = Path(processor_path).expanduser()
        config_path = Path(config_path).expanduser()
        for path in (config_path / 'config.json', model_path / 'model.safetensors',
                     processor_path / 'preprocessor_config.json',
                     processor_path / 'tokenizer.json'):
            if not path.is_file():
                raise FileNotFoundError(f'SAM3 local asset missing: {path}')
        if not prompts or any(not text.strip() for _, text in prompts):
            raise ValueError('At least one nonempty SAM3 prompt is required')
        if any(not 1 <= class_id <= 254 for class_id, _ in prompts):
            raise ValueError('SAM3 foreground class IDs must be in 1..254')
        if not 0 < score_threshold < 1 or not 0 < mask_threshold < 1:
            raise ValueError('SAM3 score/mask thresholds must be between 0 and 1')
        if precision not in ('float32', 'float16', 'bfloat16'):
            raise ValueError('precision must be float32, float16 or bfloat16')
        self.device = torch.device(device)
        if self.device.type not in ('cpu', 'cuda'):
            raise ValueError('SAM3 device must be cpu or cuda[:index]')
        if self.device.type == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('SAM3 requested CUDA but this Python environment has no CUDA')
        self.dtype = getattr(torch, precision) if self.device.type == 'cuda' else torch.float32
        self.torch = torch
        self.prompts = prompts
        self.score_threshold = score_threshold
        self.mask_threshold = mask_threshold
        # Configuration and tokenizer ship with the ROS package; only weights are external.
        self.processor = Sam3Processor(
            image_processor=AutoImageProcessor.from_pretrained(
                str(processor_path), local_files_only=True),
            tokenizer=AutoTokenizer.from_pretrained(str(processor_path), local_files_only=True),
        )
        self.model = Sam3Model.from_pretrained(
            str(model_path), dtype=self.dtype, local_files_only=True,
            config=Sam3Config.from_pretrained(str(config_path), local_files_only=True),
        ).to(self.device).eval()
        self.text_inputs = [
            self.processor.tokenizer(
                [text], return_tensors='pt', padding='max_length',
                max_length=32, truncation=True,
            ).to(self.device)
            for _, text in prompts
        ]

    def infer(self, rgb):
        """Return uint8 class/confidence maps on the original H×W RGB grid."""
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('SAM3 input must be HxWx3 uint8 RGB')
        torch = self.torch
        predictions = []
        with torch.inference_mode():
            pixels = self.processor.image_processor(
                images=rgb, return_tensors='pt',
            )['pixel_values'].to(device=self.device, dtype=self.dtype)
            # Reuse the expensive vision encoder when more prompts are configured.
            vision = self.model.get_vision_features(pixel_values=pixels)
            for (class_id, _), tokens in zip(self.prompts, self.text_inputs):
                output = self.model(vision_embeds=vision, **tokens)
                result = self.processor.post_process_instance_segmentation(
                    output, threshold=self.score_threshold,
                    mask_threshold=self.mask_threshold, target_sizes=[rgb.shape[:2]],
                )[0]
                # Transformers leaves empty masks at decoder resolution (288x288).
                masks = (result['masks'].cpu().numpy() if len(result['scores'])
                         else np.empty((0, *rgb.shape[:2]), dtype=bool))
                predictions.append((class_id, masks, result['scores'].float().cpu().numpy()))
        return merge_instances(rgb.shape[:2], predictions)
