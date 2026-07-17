#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  SceneAlignDialog — 场景对齐窗口（分层地图模型：OSM 底图 + 多张 SLAM 地图）
//
//  功能（对应设计文档 §4）：
//    · 导入 OSM 底图 (.osm)                 —— 作为超大型场景的经纬度主干
//    · 导入 SLAM 地图 (工程目录 / .pcd)      —— 生成俯视足迹叠加到底图上
//    · 以 拖拽 / 缩放 / 旋转 把 SLAM 地图对齐到底图目标 → geo_anchor
//    · 保存 / 打开 场景 JSON (scene + slam_maps)
// ═══════════════════════════════════════════════════════════════════════

#include "core/SceneModel.h"
#include "core/OsmData.h"
#include "core/Types.h"

#include <QDialog>
#include <QVector>
#include <QString>
#include <memory>

class QListWidget;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;

namespace me {

class SceneAlignView;

class SceneAlignDialog : public QDialog {
    Q_OBJECT
public:
    explicit SceneAlignDialog(QWidget* parent = nullptr);
    ~SceneAlignDialog() override = default;

private slots:
    void onImportOsm();
    void onImportSlam();
    void onRemoveSlam();
    void onSaveScene();
    void onOpenScene();
    void onListRowChanged(int row);
    void onViewSelectionChanged(int index);
    void onViewAnchorEdited(int index, const GeoAnchor& anchor);
    void onAnchorSpinChanged();
    void onFitView();

private:
    void setupUi();
    void rebuildList();
    void syncAnchorEditors(int index);
    void setEditorsEnabled(bool on);
    QColor colorForIndex(int i) const;

    /// 从工程目录 / .pcd / .json 载入并合并为 root 系点云；outName 给出建议名。
    static CloudPtr loadCloudFromSource(const QString& path, QString& outName);

    // 数据
    SceneModel               scene_;
    std::shared_ptr<OsmData> osm_;
    QString                  sceneJsonPath_;
    QString                  absOsmSource_;         ///< OSM 绝对路径
    QVector<QString>         absSlamSources_;        ///< 与 scene_.slamMaps() 平行

    // UI
    SceneAlignView*  view_        = nullptr;
    QLineEdit*       sceneNameEdit_ = nullptr;
    QLabel*          originLabel_ = nullptr;
    QLabel*          osmLabel_    = nullptr;
    QListWidget*     list_        = nullptr;
    QDoubleSpinBox*  lonSpin_     = nullptr;
    QDoubleSpinBox*  latSpin_     = nullptr;
    QDoubleSpinBox*  altSpin_     = nullptr;
    QDoubleSpinBox*  headingSpin_ = nullptr;
    QDoubleSpinBox*  scaleSpin_   = nullptr;
    QPushButton*     removeBtn_   = nullptr;

    bool syncingEditors_ = false;   ///< 防止编辑器↔视图信号回环
};

} // namespace me
