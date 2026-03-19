# MapEditor — 多子地图编辑器

基于 Qt5 + PCL + OpenGL 的可视化地图编辑工具，用于加载、显示、编辑和保存 `multifloor.json` 格式的多楼层/多子地图数据。

## 功能概览

### 加载
- 基于 `multifloor.json` 格式加载完整工程
- 自动加载各 submap 的 PCD 点云文件
- 解析语义点位 (semantic_points)、传送门 (portals)、路线 (routes)

### 显示
- 以 `default_submap` 坐标系为全局坐标系，其他 submap 按 transform 对齐显示
- 每个 submap 可独立控制**可见性**和**点云颜色**
- VoxelGrid 降采样显示 (提升渲染性能)，保存时仍为原始精度
- 语义点位和传送门的可视化标记
- 坐标轴和网格地面参考

### 编辑
- **新增 Submap**: 导入 PCD 或 PCD+JSON，默认恒等变换
- **对齐 Transform**: 3D-BBS 粗搜索 + GICP 精配准自动计算变换
  - 在 default_submap 上选择初始位置
  - 从新 submap 原点附近裁剪 20×20×20 匹配点云
  - BBS 在初始位置附近搜索，GICP 验证 (fitness ≤ 0.2)
- **编辑语义点位**: 在点云上点击放置
- **编辑传送门**: 在 submap 上点击放置，指定目标 submap
- **编辑路线**: 通过树形面板管理

### 保存
- 保存为 `multifloor.json` + 各 PCD 文件
- 完整保留语义点位、传送门、路线等元数据

## 系统要求

- Ubuntu 20.04+ / 22.04
- Qt 5.12+
- PCL 1.10+
- Eigen 3.3+
- OpenGL 3.3+
- CMake 3.16+

## 编译

```bash
cd /home/nvidia/Navi/MapEditor
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## 运行

```bash
./build/MapEditor
```

启动后通过 **文件 → 打开** 加载 `multifloor.json`。

## 项目结构

```
MapEditor/
├── CMakeLists.txt
├── README.md
└── src/
    ├── main.cpp                    # 程序入口
    ├── core/
    │   ├── Types.h                 # 公共类型 (Pose, Transform, SemanticPoint...)
    │   ├── SubMap.h / .cpp         # 子地图数据模型
    │   └── MapProject.h / .cpp     # 工程管理 (JSON I/O)
    ├── ui/
    │   ├── MainWindow.h / .cpp     # 主窗口 (菜单、工具栏、Dock)
    │   ├── PointCloudViewer.h/.cpp # 3D OpenGL 点云渲染器
    │   ├── SubmapPanel.h / .cpp    # 左侧树形面板
    │   ├── PropertiesPanel.h/.cpp  # 右侧属性编辑面板
    │   ├── AddSubmapDialog.h/.cpp  # 新增子地图对话框
    │   └── AlignSubmapDialog.h/.cpp# 子地图对齐对话框
    └── algorithms/
        ├── VoxelGridFilter.h/.cpp  # 体素降采样
        └── CloudMatcher.h/.cpp     # 3D-BBS + GICP 匹配
```

## 界面布局

```
┌──────────────────────────────────────────────────────────────┐
│  文件  视图  编辑                          [ 工具栏 ]          │
├──────────┬────────────────────────────┬──────────────────────┤
│          │                            │                      │
│  子地图   │      3D 点云查看器           │     属性编辑          │
│  管理树   │                            │                      │
│          │  · 轨道球相机               │  · Submap 属性        │
│  📦 Submaps│  · 左键旋转                │  · Transform         │
│   ├ floor│  · 中键平移                 │  · 语义点属性          │
│   ├ B1   │  · 滚轮缩放                 │  · 传送门属性          │
│   └ 5F   │  · 点击拾取                 │  · 显示选项           │
│          │                            │                      │
│  🛤️ Routes│                            │                      │
│   └ 线路A│                            │                      │
│          │                            │                      │
├──────────┴────────────────────────────┴──────────────────────┤
│  状态栏                                                       │
└──────────────────────────────────────────────────────────────┘
```

## 操作指南

### 对齐新 Submap (3D-BBS + GICP)
1. **文件 → 打开** 已有的 `multifloor.json`
2. **+ Submap** 导入新的 PCD 文件
3. 右键新 submap → **对齐 Transform (3D-BBS)**
4. 在对话框中点击 **在地图上选择点** → 在 default_submap 点云上点击
5. 调整搜索参数 → 点击 **开始对齐**
6. 成功后点击 **应用结果**

### 添加语义点位
1. 右键目标 submap → **添加语义点位**
2. 在 3D 视图中点击目标位置
3. 输入点位名称

### 添加传送门
1. 右键目标 submap → **添加传送门**
2. 在 3D 视图中点击位置
3. 选择目标 submap 并输入传送门名称
