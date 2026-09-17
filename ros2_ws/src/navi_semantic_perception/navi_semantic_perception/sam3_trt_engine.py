"""Offline SAM3 TensorRT inference using the same processor as the torch backend."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np

from .sam3_engine import merge_instances


class TensorRTEngine:
    """Own one execution context; callers must serialize inference."""

    _logger = None

    def __init__(self, path, device):
        """Load a local engine compatible with the selected CUDA device."""
        import tensorrt as trt
        import torch

        self.trt, self.torch, self.device = trt, torch, device
        if TensorRTEngine._logger is None:
            TensorRTEngine._logger = trt.Logger(trt.Logger.WARNING)
        self.logger = TensorRTEngine._logger
        self.runtime = trt.Runtime(self.logger)
        path = Path(path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'SAM3 engine missing: {path}; run envs/sam3/build_trt.sh')
        with torch.cuda.device(device):
            self.engine = self.runtime.deserialize_cuda_engine(path.read_bytes())
            if self.engine is None:
                raise RuntimeError(f'Cannot load {path}; rebuild for this GPU/TensorRT version')
            self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError(f'Cannot create TensorRT context for {path}')
        self.stream = torch.cuda.Stream(device=device)
        self.names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]

    def infer(self, inputs):
        """Bind validated tensors and finish execution before returning their outputs."""
        torch, trt = self.torch, self.trt
        buffers, outputs = {}, {}
        with torch.cuda.device(self.device):
            self.stream.wait_stream(torch.cuda.current_stream(self.device))
        with torch.cuda.device(self.device), torch.cuda.stream(self.stream):
            for name in self.names:
                if self.engine.get_tensor_mode(name) != trt.TensorIOMode.INPUT:
                    continue
                dtype = torch.from_numpy(np.empty((), dtype=trt.nptype(
                    self.engine.get_tensor_dtype(name)))).dtype
                buffers[name] = inputs[name].to(device=self.device, dtype=dtype).contiguous()
                if not self.context.set_input_shape(name, tuple(buffers[name].shape)):
                    raise ValueError(f'SAM3 engine rejects {name} shape {buffers[name].shape}')
                self.context.set_tensor_address(name, buffers[name].data_ptr())
            for name in self.names:
                if self.engine.get_tensor_mode(name) != trt.TensorIOMode.OUTPUT:
                    continue
                shape = tuple(self.context.get_tensor_shape(name))
                if any(dim < 0 for dim in shape):
                    raise ValueError(f'Unresolved SAM3 output shape: {name}: {shape}')
                dtype = torch.from_numpy(np.empty((), dtype=trt.nptype(
                    self.engine.get_tensor_dtype(name)))).dtype
                outputs[name] = torch.empty(shape, dtype=dtype, device=self.device)
                self.context.set_tensor_address(name, outputs[name].data_ptr())
            stream = torch.cuda.current_stream(self.device)
            if not self.context.execute_async_v3(stream.cuda_stream):
                raise RuntimeError('SAM3 TensorRT execution failed')
            stream.synchronize()
        return outputs


class Sam3TRTSegmenter:
    """Run SAM3 vision and detector engines with cached text and original-grid masks."""

    def __init__(self, model_path, processor_path, prompts, config_path, device='cuda',
                 precision='bfloat16', score_threshold=0.5, mask_threshold=0.5):
        """model_path denotes the engine directory; precision is baked into engines."""
        import torch
        from transformers import AutoImageProcessor, AutoTokenizer, Sam3Processor

        if not prompts or any(not text.strip() or not 1 <= cid <= 254 for cid, text in prompts):
            raise ValueError('SAM3 requires nonempty prompts with class IDs in 1..254')
        if not 0 < score_threshold < 1 or not 0 < mask_threshold < 1:
            raise ValueError('SAM3 score/mask thresholds must be between 0 and 1')
        self.device = torch.device(device)
        if self.device.type != 'cuda' or not torch.cuda.is_available():
            raise ValueError('SAM3 TensorRT requires a CUDA device')
        self.torch, self.prompts = torch, prompts
        self.score_threshold, self.mask_threshold = score_threshold, mask_threshold
        self.processor = Sam3Processor(
            image_processor=AutoImageProcessor.from_pretrained(
                str(processor_path), local_files_only=True),
            tokenizer=AutoTokenizer.from_pretrained(str(processor_path), local_files_only=True),
        )
        self.vision = TensorRTEngine(Path(model_path) / 'vision_encoder.plan', self.device)
        self.detector = TensorRTEngine(Path(model_path) / 'detector_body.plan', self.device)
        # One prompt per detector call also works with engines built for just grass.
        self.tokens = [self.processor.tokenizer(
            [text], return_tensors='pt', padding='max_length', max_length=32, truncation=True,
        ).to(self.device) for _, text in prompts]

    def infer(self, rgb):
        """Return the same uint8 semantic IDs and confidence contract as PyTorch."""
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('SAM3 input must be HxWx3 uint8 RGB')
        predictions = []
        with self.torch.inference_mode(), self.torch.cuda.device(self.device):
            pixels = self.processor.image_processor(
                images=rgb, return_tensors='pt')['pixel_values'].to(self.device)
            features = self.vision.infer({'pixel_values': pixels})
            for (class_id, _), tokens in zip(self.prompts, self.tokens):
                output = SimpleNamespace(**self.detector.infer({**features, **tokens}))
                result = self.processor.post_process_instance_segmentation(
                    output, threshold=self.score_threshold, mask_threshold=self.mask_threshold,
                    target_sizes=[rgb.shape[:2]],
                )[0]
                masks = (result['masks'].cpu().numpy() if len(result['scores'])
                         else np.empty((0, *rgb.shape[:2]), dtype=bool))
                predictions.append((class_id, masks, result['scores'].float().cpu().numpy()))
        return merge_instances(rgb.shape[:2], predictions)
