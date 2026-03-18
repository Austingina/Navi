#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  PointCloudViewer — 基于 QOpenGLWidget 的 3D 点云查看器
// ═══════════════════════════════════════════════════════════════════════

#include "core/Types.h"
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QVector3D>
#include <QMap>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimer>

namespace me {

/// 单个 submap 的 GPU 渲染数据
struct SubmapGLData {
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer            vbo{QOpenGLBuffer::VertexBuffer};
    int                      vertexCount = 0;
    QMatrix4x4               modelMatrix;
    bool                     visible     = true;
    QColor                   color;
    float                    pointSize   = 2.0f;
};

/// 标记点 (语义点/传送门) 的渲染数据
struct MarkerVertex {
    float x, y, z;
    float r, g, b;
};

/// 标记点元数据 (用于悬停提示)
struct MarkerInfo {
    QString name;         // 语义点名称
    QString submapId;     // 所属 submap
    QString type;         // "semantic" 或 "portal"
    QVector3D worldPos;   // 世界坐标系下位置
};

class PointCloudViewer : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core {
    Q_OBJECT
public:
    explicit PointCloudViewer(QWidget* parent = nullptr);
    ~PointCloudViewer() override;

    // ── 点云管理 ─────────────────────────────────────────────────────
    /// 更新某个 submap 的显示数据
    void updateSubmap(const QString& id, CloudPtr displayCloud,
                      const Eigen::Matrix4d& transform,
                      const QColor& color, float pointSize);
    /// 移除某个 submap 的显示
    void removeSubmap(const QString& id);
    /// 设置 submap 可见性
    void setSubmapVisible(const QString& id, bool visible);

    // ── 标记点管理 ───────────────────────────────────────────────────
    /// 更新语义点位和传送门标记
    void updateMarkers(const QVector<MarkerVertex>& markers,
                       const QVector<MarkerInfo>& markerInfos = {});
    void setMarkersVisible(bool v) { showMarkers_ = v; update(); }

    // ── 拾取模式 ─────────────────────────────────────────────────────
    enum PickMode { NoPickMode, PickPointOnCloud, PickGuessPosition };
    void setPickMode(PickMode mode) { pickMode_ = mode; }

    // ── 视角控制 ─────────────────────────────────────────────────────
    void resetCamera();
    void lookAt(const QVector3D& center);

signals:
    /// 用户在点云上拾取了一个 3D 点
    void pointPicked(const Eigen::Vector3d& position);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    // ── 着色器 ───────────────────────────────────────────────────────
    void initShaders();
    QOpenGLShaderProgram* cloudShader_  = nullptr;
    QOpenGLShaderProgram* markerShader_ = nullptr;

    // ── 相机 (轨道球) ────────────────────────────────────────────────
    QVector3D cameraTarget_{0, 0, 0};
    float     cameraDistance_ = 50.0f;
    float     cameraAzimuth_  = 45.0f;   // 水平角 (度)
    float     cameraElevation_= 30.0f;   // 仰角 (度)
    float     nearPlane_      = 0.1f;
    float     farPlane_       = 1000.0f;

    QMatrix4x4 viewMatrix() const;
    QMatrix4x4 projMatrix() const;

    // ── 鼠标交互 ─────────────────────────────────────────────────────
    QPoint     lastMousePos_;
    bool       rotating_ = false;
    bool       panning_  = false;

    // ── Submap 数据 ──────────────────────────────────────────────────
    QMap<QString, SubmapGLData*> submapGL_;

    // ── 标记点 ───────────────────────────────────────────────────────
    QOpenGLVertexArrayObject markerVAO_;
    QOpenGLBuffer            markerVBO_{QOpenGLBuffer::VertexBuffer};
    int                      markerCount_ = 0;
    bool                     showMarkers_ = true;
    QVector<MarkerInfo>      markerInfos_;   // 标记点元数据
    QVector<MarkerVertex>    markerVertices_; // 当前上传到 GPU 的标记顶点数据
    // 拖拽状态
    bool                     leftPressed_ = false;
    bool                     dragging_ = false;
    int                      potentialDragIndex_ = -1;
    int                      dragIndex_ = -1;
    QPoint                   pressPos_;
    int                      longPressMs_ = 250;

signals:
    /// 用户通过长按并拖拽标记点后释放，要求更新语义点位置
    void markerDragFinished(const QString& submapId, const QString& name, const Eigen::Vector3d& newPos);
    
private:
    QVector<int> hitTestMarkers(const QPoint& screenPos, float threshold = 15.0f);

    // ── 网格地面 ─────────────────────────────────────────────────────
    QOpenGLVertexArrayObject gridVAO_;
    QOpenGLBuffer            gridVBO_{QOpenGLBuffer::VertexBuffer};
    int                      gridVertexCount_ = 0;
    void buildGrid();

    // ── 拾取 ─────────────────────────────────────────────────────────
    PickMode pickMode_ = NoPickMode;
    Eigen::Vector3d unproject(const QPoint& screenPos, float depth = 0.0f);
    void performPick(const QPoint& screenPos);

    // ── 坐标轴 ───────────────────────────────────────────────────────
    QOpenGLVertexArrayObject axisVAO_;
    QOpenGLBuffer            axisVBO_{QOpenGLBuffer::VertexBuffer};
    void buildAxis();
};

} // namespace me
