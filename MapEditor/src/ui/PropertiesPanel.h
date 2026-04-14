#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  PropertiesPanel — 右侧属性编辑面板
// ═══════════════════════════════════════════════════════════════════════

#include "core/Types.h"
#include "core/SubMap.h"
#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QFormLayout>
#include <QGroupBox>
#include <QColorDialog>
#include <QStackedWidget>

namespace me {

class PropertiesPanel : public QWidget {
    Q_OBJECT
public:
    explicit PropertiesPanel(QWidget* parent = nullptr);

    /// 显示 submap 属性
    void showSubmapProperties(SubMap* sm);
    /// 显示语义点属性
    void showSemanticPointProperties(SubMap* sm, const QString& pointName);
    /// 新建语义点：在右侧面板编辑，「应用修改」写入工程；可用「从点云选点」填 position
    void beginNewSemanticPoint(SubMap* sm);
    /// 设置语义点 position 输入框（子地图局部坐标）
    void setSemanticPositionLocal(double x, double y, double z);
    /// 显示 portal 属性
    void showPortalProperties(SubMap* sm, int portalIdx, const QStringList& allSubmapIds);
    /// 清空面板
    void clear();

signals:
    void submapPropertiesChanged(const QString& submapId);
    void semanticPointChanged(const QString& submapId, const QString& pointName);
    void portalChanged(const QString& submapId, int portalIdx);
    void colorChanged(const QString& submapId, const QColor& color);
    void voxelSizeChanged(const QString& submapId, float size);
    /// 语义点页：请求在点云上拾取以填写 position
    void pickSemanticPositionRequested(const QString& submapId);

private:
    void setupUI();
    void setupSubmapPage();
    void setupSemanticPointPage();
    void setupPortalPage();

    QStackedWidget* stack_ = nullptr;

    // ── Submap 页面 ──────────────────────────────────────────────────
    QWidget*        smPage_ = nullptr;
    QLineEdit*      smId_ = nullptr;
    QLineEdit*      smName_ = nullptr;
    QLineEdit*      smDesc_ = nullptr;
    QLineEdit*      smMapFile_ = nullptr;
    // Transform
    QGroupBox*      tfGroup_ = nullptr;
    QDoubleSpinBox* smTx_ = nullptr;
    QDoubleSpinBox* smTy_ = nullptr;
    QDoubleSpinBox* smTz_ = nullptr;
    QDoubleSpinBox* smRx_ = nullptr;
    QDoubleSpinBox* smRy_ = nullptr;
    QDoubleSpinBox* smRz_ = nullptr;
    QDoubleSpinBox* smRw_ = nullptr;
    // Display
    QPushButton*    smColorBtn_ = nullptr;
    QDoubleSpinBox* smVoxelSize_ = nullptr;
    QDoubleSpinBox* smPointSize_ = nullptr;
    QLabel*         smCloudInfo_ = nullptr;

    SubMap*         currentSm_ = nullptr;

    // ── SemanticPoint 页面 ───────────────────────────────────────────
    QWidget*        spPage_ = nullptr;
    QLineEdit*      spName_ = nullptr;
    QComboBox*      spType_ = nullptr;
    QDoubleSpinBox* spPx_ = nullptr;
    QDoubleSpinBox* spPy_ = nullptr;
    QDoubleSpinBox* spPz_ = nullptr;
    QDoubleSpinBox* spYaw_ = nullptr; ///< 绕 Z 轴 yaw（度），与四元数双向同步
    QDoubleSpinBox* spOx_ = nullptr;
    QDoubleSpinBox* spOy_ = nullptr;
    QDoubleSpinBox* spOz_ = nullptr;
    QDoubleSpinBox* spOw_ = nullptr;
    QPushButton*    spPickPosBtn_ = nullptr;

    SubMap*  spSm_ = nullptr;
    QString  spCurrentName_;
    bool     spOrientationSyncGuard_ = false;

    void syncSemanticYawFromQuaternionSpins();
    void syncSemanticQuaternionSpinsFromYaw();

    // ── Portal 页面 ──────────────────────────────────────────────────
    QWidget*        ptPage_ = nullptr;
    QComboBox*      ptName_ = nullptr;   // 当前 submap 的语义点名称
    QComboBox*      ptTarget_ = nullptr;
    SubMap*         ptSm_ = nullptr;
    int             ptIdx_ = -1;  ///< >=0 编辑已有传送门；-1 表示未打开传送门页

    // helpers
    QDoubleSpinBox* makeSpinBox(double min = -9999, double max = 9999, int decimals = 4);
};

} // namespace me
