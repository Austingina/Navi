#!/usr/bin/env python3
"""TensorRT 引擎构建与推理 —— 全 GPU 流水线。

为什么是 TensorRT 而不是 onnxruntime-gpu：
  本机是 NVIDIA Thor = **sm_110**。PyPI 的 onnxruntime-gpu 1.29.0 aarch64 wheel
  只烧了 sm_89/sm_90a/sm_120a/sm_121a 且**不含 PTX**（`cuobjdump --list-ptx` 为空），
  所以既跑不了也 JIT 不了，实测报 `cudaErrorNoKernelImageForDevice`。
  jetson-ai-lab 的 jp7/cu130 索引里没有 onnxruntime-gpu。
  TensorRT 在运行时针对实机架构编译，架构问题不存在；且 tensorrt/cupy 都在系统
  dist-packages 里，与 rclpy 同一解释器 —— 这个节点因此**不需要 venv**。

引擎缓存是设备 + TRT 版本 + 形状特定的产物，文件名里带全部三者。
**不要提交到 git**（models/.gitignore 已忽略 *.plan）。
"""
import os
import numpy as np

try:
    import tensorrt as trt
    import cupy as cp
    HAVE_TRT = True
except ImportError:  # pragma: no cover - 让 import 本模块永不致命
    trt = None
    cp = None
    HAVE_TRT = False

# ImageNet 归一化，与上游 nav2_semantic_segmentation_demo 一致
MEAN = (0.485, 0.456, 0.406)
STD = (0.229, 0.224, 0.225)


def cache_dir():
    """引擎缓存目录。**刻意放在仓库和 colcon build/ 之外**：

    引擎是本机产物（设备+TRT 版本绑定），不能进 git；而 --symlink-install 会把
    share/models/model.onnx 指到 build/ 里，`colcon build` 一清就没了 —— 若把
    引擎放在模型旁边，每次重编工作区都要白等 9 分钟重编引擎。
    """
    d = os.environ.get('NAVI_TRT_CACHE') or os.path.join(
        os.path.expanduser('~'), '.cache', 'navi_semantic_perception')
    os.makedirs(d, exist_ok=True)
    return d


def engine_path_for(onnx_path, width, height):
    """引擎缓存路径：模型内容哈希 + 设备 + 算力 + TRT 版本 + 形状。

    用 sha256 前 12 位而不是文件名：换了模型权重会自动换引擎，
    不会静默地拿旧引擎跑新模型。62 MB 的哈希只在启动时算一次。
    """
    import hashlib
    h = hashlib.sha256()
    with open(onnx_path, 'rb') as fp:
        for chunk in iter(lambda: fp.read(1 << 20), b''):
            h.update(chunk)
    dev = cp.cuda.runtime.getDeviceProperties(0)['name'].decode().replace(' ', '')
    cc = cp.cuda.Device(0).compute_capability
    name = (f"{os.path.basename(onnx_path).rsplit('.', 1)[0]}."
            f"{h.hexdigest()[:12]}.{dev}.sm{cc}.trt{trt.__version__}."
            f"{width}x{height}.plan")
    return os.path.join(cache_dir(), name)


def build_engine(onnx_path, width, height, out_path=None, workspace_mib=2048,
                 log=print):
    """从 ONNX 编译固定形状 FP16 引擎。几分钟量级，只需一次。"""
    out_path = out_path or engine_path_for(onnx_path, width, height)
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network()
    parser = trt.OnnxParser(network, logger)
    with open(onnx_path, 'rb') as f:
        if not parser.parse(f.read()):
            errs = [str(parser.get_error(i)) for i in range(parser.num_errors)]
            raise RuntimeError('ONNX 解析失败:\n' + '\n'.join(errs))

    cfg = builder.create_builder_config()
    cfg.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_mib << 20)
    cfg.set_flag(trt.BuilderFlag.FP16)  # 权重本身就是 float16

    inp = network.get_input(0)
    # 固定形状（min=opt=max）——比动态形状快，且我们只用一个分辨率
    shape = (1, 3, height, width)
    prof = builder.create_optimization_profile()
    prof.set_shape(inp.name, shape, shape, shape)
    cfg.add_optimization_profile(prof)

    log(f"编译 TRT 引擎 {inp.name}{shape} FP16 -> {out_path}（需几分钟）")
    plan = builder.build_serialized_network(network, cfg)
    if plan is None:
        raise RuntimeError('TensorRT 引擎编译失败（见上面的 TRT 日志）')
    tmp = out_path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(plan)
    os.replace(tmp, out_path)  # 原子替换，避免半截引擎被后续进程读到
    log(f"引擎已写出：{out_path}（{os.path.getsize(out_path) / 1e6:.1f} MB）")
    return out_path


class TrtSegmenter:
    """固定形状 TRT 分割器。预处理/后处理都在 GPU 上，主机侧只传 uint8。

    传输账：上行 W*H*3 字节（uint8 RGB），下行 2*W*H 字节（mask + confidence）。
    3*H*W 的 float16 logits 从不离开显存。
    """

    def __init__(self, onnx_path, width, height, engine_path=None,
                 allow_build=True, log=print):
        if not HAVE_TRT:
            raise RuntimeError('tensorrt / cupy 不可用；请用系统 python3 运行本节点')
        self.width, self.height = int(width), int(height)
        path = engine_path or engine_path_for(onnx_path, self.width, self.height)
        if not os.path.exists(path):
            if not allow_build:
                raise FileNotFoundError(
                    f'缺少引擎 {path}；先跑 `ros2 run navi_semantic_perception '
                    f'build_trt_engine` 或把 allow_build 设为 true')
            build_engine(onnx_path, self.width, self.height, path, log=log)

        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        with open(path, 'rb') as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(
                f'引擎 {path} 反序列化失败 —— 通常是 GPU 或 TRT 版本变了；删掉它重编')
        self.ctx = self.engine.create_execution_context()
        self.engine_path = path

        names = [self.engine.get_tensor_name(i)
                 for i in range(self.engine.num_io_tensors)]
        self.in_name = next(n for n in names
                            if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT)
        self.out_name = next(n for n in names
                             if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT)
        self.ctx.set_input_shape(self.in_name, (1, 3, self.height, self.width))
        out_shape = tuple(self.ctx.get_tensor_shape(self.out_name))
        self.num_classes = int(out_shape[1])

        self.stream = cp.cuda.Stream(non_blocking=True)
        # 常驻显存缓冲，逐帧复用，稳态零分配
        self.d_in = cp.empty((1, 3, self.height, self.width), dtype=cp.float16)
        self.d_out = cp.empty(out_shape, dtype=cp.float16)
        self.ctx.set_tensor_address(self.in_name, int(self.d_in.data.ptr))
        self.ctx.set_tensor_address(self.out_name, int(self.d_out.data.ptr))
        self._mean = cp.asarray(MEAN, dtype=cp.float32).reshape(3, 1, 1)
        self._std = cp.asarray(STD, dtype=cp.float32).reshape(3, 1, 1)
        log(f"TRT 就绪：{path.rsplit('/', 1)[-1]}  "
            f"{self.in_name}{(1, 3, self.height, self.width)} -> "
            f"{self.out_name}{out_shape}  classes={self.num_classes}")

    def infer(self, rgb_hw3_u8):
        """RGB uint8 (H,W,3) -> (mask uint8 (H,W), confidence uint8 (H,W))。

        输入必须已经是引擎的 W×H；缩放由调用方负责（见节点的 input_scale）。
        """
        h, w = rgb_hw3_u8.shape[:2]
        if (w, h) != (self.width, self.height):
            raise ValueError(f'期望 {self.width}x{self.height}，收到 {w}x{h}')
        with self.stream:
            d_u8 = cp.asarray(rgb_hw3_u8)                       # 上行唯一一次拷贝
            x = d_u8.transpose(2, 0, 1).astype(cp.float32) / 255.0
            self.d_in[0] = ((x - self._mean) / self._std).astype(cp.float16)
            self.ctx.execute_async_v3(self.stream.ptr)
            o = self.d_out[0].astype(cp.float32)                # (C,H,W)
            mask = cp.argmax(o, axis=0).astype(cp.uint8)
            e = cp.exp(o - o.max(axis=0, keepdims=True))
            conf = (e.max(axis=0) / e.sum(axis=0) * 255.0).astype(cp.uint8)
            mask_h = cp.asnumpy(mask)                           # 下行 2*W*H 字节
            conf_h = cp.asnumpy(conf)
        self.stream.synchronize()
        return mask_h, conf_h


def main(argv=None):
    """控制台入口：预编译引擎，省掉节点首启的几分钟。"""
    import argparse
    from ament_index_python.packages import get_package_share_directory
    ap = argparse.ArgumentParser(description='为本机预编译 TensorRT 引擎')
    ap.add_argument('--onnx', default=None)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    a = ap.parse_args(argv)
    onnx = a.onnx or os.path.join(
        get_package_share_directory('navi_semantic_perception'), 'models', 'model.onnx')
    print(build_engine(onnx, a.width, a.height))


if __name__ == '__main__':
    main()
