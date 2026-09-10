# 模型来源与许可

`model.onnx`（64.8 MB）取自
<https://github.com/ros-navigation/navigation2_tutorials/tree/rolling/nav2_semantic_segmentation_demo/semantic_segmentation_node/models>

上游包 `semantic_segmentation_node` 的 `package.xml` 声明 **BSD-3-Clause**，
维护者 Pedro Gonzalez <pedro.gonzalez@eia.edu.co>。`config/ontology.yaml` 同样取自该处。

## 模型接口（本机实测）

| | |
|---|---|
| 输入 | `input`，`[batch, 3, height, width]`，**float16**，H/W **动态** |
| 输出 | `output`，`[batch, 3, height, width]`，float16 |
| 类别 | 0=background、1=sidewalk、2=grass（ontology 里 0 恒为 background） |
| 预处理 | RGB、/255、ImageNet 归一化 (mean .485/.456/.406, std .229/.224/.225)、NCHW |
| 后处理 | argmax 出类别 id；softmax 的逐像素最大值 ×255 出 confidence |

H/W 动态这一点很关键：640×480 可以直接喂，**掩码原生就在彩色网格上**，
不需要 resize，天然满足 semantic_segmentation_layer 的像素对应契约。

## ⚠ 这个 65 MB 二进制要不要进 git

现在它在 git 里是**未提交**状态。65 MB 会永久留在 Navi 的仓库历史里。
三个选项，建议第 2 个：

1. 直接提交 —— 最省事，代价是仓库永久变大 65 MB。
2. **不提交，改为按需下载**：在 `models/.gitignore` 里忽略 `*.onnx`，
   提供一个 `scripts/fetch_model.sh` 从上游拉取并校验 sha256。部署时多一步。
3. git-lfs —— 这个仓库目前没在用 lfs，为一个文件引入它不值。

sha256（拉取后自校验用）：见本目录 `model.onnx.sha256`。
