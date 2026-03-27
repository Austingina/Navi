#include "ui/PointCloudViewer.h"
#include <QPainter>
#include <QDebug>
#include <QtMath>
#include <QToolTip>
#include <cmath>

namespace me {

// ─── 内联着色器代码 ──────────────────────────────────────────────────
static const char* kCloudVert = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uMVP;
out vec3 vColor;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vColor = aColor;
}
)";

static const char* kCloudFrag = R"(
#version 330 core
in vec3 vColor;
out vec4 FragColor;
uniform int uDrawMode; // 0 = lines, 1 = points
void main() {
    if (uDrawMode == 1) {
        vec2 coord = gl_PointCoord - vec2(0.5);
        if (dot(coord, coord) > 0.25) discard;
    }
    FragColor = vec4(vColor, 1.0);
}
)";

// ═══════════════════════════════════════════════════════════════════════
//  构造 / 析构
// ═══════════════════════════════════════════════════════════════════════

PointCloudViewer::PointCloudViewer(QWidget* parent) : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(400, 300);
    setMouseTracking(true);  // 接收无按键的鼠标移动事件
}

PointCloudViewer::~PointCloudViewer() {
    makeCurrent();
    for (auto* d : submapGL_) {
        d->vao.destroy();
        d->vbo.destroy();
        delete d;
    }
    for (auto* rd : routeGL_) {
        rd->vao.destroy();
        rd->vbo.destroy();
        delete rd;
    }
    for (auto* cb : cropBoxGL_) {
        cb->vao.destroy();
        cb->vbo.destroy();
        delete cb;
    }
    markerVAO_.destroy();
    markerVBO_.destroy();
    markerEmphasisVAO_.destroy();
    markerEmphasisVBO_.destroy();
    semanticArrowVAO_.destroy();
    semanticArrowVBO_.destroy();
    gridVAO_.destroy();
    gridVBO_.destroy();
    axisVAO_.destroy();
    axisVBO_.destroy();
    delete cloudShader_;
    doneCurrent();
}

// ═══════════════════════════════════════════════════════════════════════
//  初始化 OpenGL
// ═══════════════════════════════════════════════════════════════════════

void PointCloudViewer::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_POINT_SPRITE);

    initShaders();
    buildGrid();
    buildAxis();
}

void PointCloudViewer::initShaders() {
    cloudShader_ = new QOpenGLShaderProgram(this);
    cloudShader_->addShaderFromSourceCode(QOpenGLShader::Vertex, kCloudVert);
    cloudShader_->addShaderFromSourceCode(QOpenGLShader::Fragment, kCloudFrag);
    cloudShader_->link();

    // marker 使用同一组着色器
    markerShader_ = cloudShader_;
}

void PointCloudViewer::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

// ═══════════════════════════════════════════════════════════════════════
//  相机矩阵
// ═══════════════════════════════════════════════════════════════════════

QMatrix4x4 PointCloudViewer::viewMatrix() const {
    float azRad  = qDegreesToRadians(cameraAzimuth_);
    float elRad  = qDegreesToRadians(cameraElevation_);
    float cosEl  = std::cos(elRad);

    QVector3D eye(
        cameraTarget_.x() + cameraDistance_ * cosEl * std::cos(azRad),
        cameraTarget_.y() + cameraDistance_ * cosEl * std::sin(azRad),
        cameraTarget_.z() + cameraDistance_ * std::sin(elRad)
    );

    QMatrix4x4 view;
    view.lookAt(eye, cameraTarget_, QVector3D(0, 0, 1));
    return view;
}

QMatrix4x4 PointCloudViewer::projMatrix() const {
    QMatrix4x4 proj;
    float aspect = static_cast<float>(width()) / std::max(1, height());
    proj.perspective(45.0f, aspect, nearPlane_, farPlane_);
    return proj;
}

// ═══════════════════════════════════════════════════════════════════════
//  绘制
// ═══════════════════════════════════════════════════════════════════════

void PointCloudViewer::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 view = viewMatrix();
    QMatrix4x4 proj = projMatrix();

    cloudShader_->bind();

    // 绘制网格 (GL_LINES → uDrawMode=0)
    {
        QMatrix4x4 mvp = proj * view;
        cloudShader_->setUniformValue("uMVP", mvp);
        cloudShader_->setUniformValue("uDrawMode", 0);
        gridVAO_.bind();
        glDrawArrays(GL_LINES, 0, gridVertexCount_);
        gridVAO_.release();
    }

    // 绘制全局坐标轴 (GL_LINES → uDrawMode=0)
    {
        QMatrix4x4 mvp = proj * view;
        cloudShader_->setUniformValue("uMVP", mvp);
        cloudShader_->setUniformValue("uDrawMode", 0);
        axisVAO_.bind();
        glLineWidth(3.0f);
        glDrawArrays(GL_LINES, 0, 6);
        glLineWidth(1.0f);
        axisVAO_.release();
    }

    // 绘制各 submap 点云
    for (auto it = submapGL_.begin(); it != submapGL_.end(); ++it) {
        auto* d = it.value();
        if (!d->visible || d->vertexCount == 0) continue;

        // 点云 (GL_POINTS → uDrawMode=1)
        QMatrix4x4 mvp = proj * view * d->modelMatrix;
        cloudShader_->setUniformValue("uMVP", mvp);
        cloudShader_->setUniformValue("uDrawMode", 1);
        glPointSize(d->pointSize);
        d->vao.bind();
        glDrawArrays(GL_POINTS, 0, d->vertexCount);
        d->vao.release();

        // submap 原点坐标轴 (GL_LINES → uDrawMode=0)
        QMatrix4x4 axisMvp = proj * view * d->modelMatrix;
        cloudShader_->setUniformValue("uMVP", axisMvp);
        cloudShader_->setUniformValue("uDrawMode", 0);
        axisVAO_.bind();
        glLineWidth(2.0f);
        glDrawArrays(GL_LINES, 0, 6);
        glLineWidth(1.0f);
        axisVAO_.release();
    }

    // 绘制标记点 (GL_POINTS → uDrawMode=1)：普通点半径与选中语义点 2× 半径分两批
    if (showMarkers_ && markerCount_ > 0) {
        QMatrix4x4 mvp = proj * view;
        cloudShader_->setUniformValue("uMVP", mvp);
        cloudShader_->setUniformValue("uDrawMode", 1);
        if (markerNormalCount_ > 0) {
            glPointSize(kMarkerPointSizeBase_);
            markerVAO_.bind();
            glDrawArrays(GL_POINTS, 0, markerNormalCount_);
            markerVAO_.release();
        }
        if (markerEmphasisCount_ > 0) {
            glPointSize(kMarkerPointSizeEmphasis_);
            markerEmphasisVAO_.bind();
            glDrawArrays(GL_POINTS, 0, markerEmphasisCount_);
            markerEmphasisVAO_.release();
        }
    }

    // 语义点朝向箭头 (GL_LINES)：局部 +X 为箭头方向，与机器人/地图常用前进方向一致
    if (showMarkers_ && semanticArrowVertexCount_ >= 2) {
        QMatrix4x4 mvp = proj * view;
        cloudShader_->setUniformValue("uMVP", mvp);
        cloudShader_->setUniformValue("uDrawMode", 0);
        glLineWidth(2.5f);
        semanticArrowVAO_.bind();
        glDrawArrays(GL_LINES, 0, semanticArrowVertexCount_);
        semanticArrowVAO_.release();
        glLineWidth(1.0f);
    }

    // 绘制路线折线 (GL_LINE_STRIP → uDrawMode=0)
    for (auto it = routeGL_.begin(); it != routeGL_.end(); ++it) {
        auto* rd = it.value();
        if (!rd->visible || rd->vertexCount < 2) continue;
        QMatrix4x4 mvp = proj * view;
        cloudShader_->setUniformValue("uMVP", mvp);
        cloudShader_->setUniformValue("uDrawMode", 0);
        glLineWidth(3.0f);
        rd->vao.bind();
        glDrawArrays(GL_LINE_STRIP, 0, rd->vertexCount);
        rd->vao.release();
        glLineWidth(1.0f);
    }

    // 绘制裁剪框预览 (GL_LINES → uDrawMode=0)
    for (auto it = cropBoxGL_.begin(); it != cropBoxGL_.end(); ++it) {
        auto* cb = it.value();
        if (!cb->visible || cb->vertexCount < 2) continue;
        QMatrix4x4 mvp = proj * view;
        cloudShader_->setUniformValue("uMVP", mvp);
        cloudShader_->setUniformValue("uDrawMode", 0);
        glLineWidth(2.0f);
        cb->vao.bind();
        glDrawArrays(GL_LINES, 0, cb->vertexCount);
        cb->vao.release();
        glLineWidth(1.0f);
    }

    cloudShader_->release();

    // Overlay: draw submap IDs using QPainter (2D overlay)
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    QFont font = painter.font();
    font.setPointSize(12);
    painter.setFont(font);
    QMatrix4x4 view2 = view;
    QMatrix4x4 proj2 = proj;
    for (auto it = submapGL_.begin(); it != submapGL_.end(); ++it) {
        auto* d = it.value();
        if (!d->visible) continue;
        QVector4D worldPos = d->modelMatrix * QVector4D(0,0,0,1);
        QVector4D clip = proj2 * view2 * worldPos;
        if (clip.w() == 0.0f) continue;
        QVector3D ndc = clip.toVector3DAffine();
        int x = int((ndc.x() * 0.5f + 0.5f) * width());
        int y = int((-ndc.y() * 0.5f + 0.5f) * height());
        painter.setPen(Qt::white);
        painter.drawText(x+6, y-6, it.key());
    }
    painter.end();
}

// ═══════════════════════════════════════════════════════════════════════
//  Submap 点云上传
// ═══════════════════════════════════════════════════════════════════════

void PointCloudViewer::updateSubmap(const QString& id, CloudPtr displayCloud,
                                     const Eigen::Matrix4d& transform,
                                     const QColor& color, float pointSize) {
    makeCurrent();

    SubmapGLData* d = nullptr;
    if (submapGL_.contains(id)) {
        d = submapGL_[id];
    } else {
        d = new SubmapGLData();
        d->vao.create();
        d->vbo.create();
        submapGL_[id] = d;
    }

    d->color     = color;
    d->pointSize = pointSize;

    // 设置 model matrix
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            d->modelMatrix(r, c) = static_cast<float>(transform(r, c));

    if (!displayCloud || displayCloud->empty()) {
        d->vertexCount = 0;
        doneCurrent();
        update();
        return;
    }

    // 构建 VBO 数据: [x,y,z, r,g,b] per vertex
    int n = static_cast<int>(displayCloud->size());
    QVector<float> data(n * 6);
    float cr = color.redF(), cg = color.greenF(), cb = color.blueF();

    for (int i = 0; i < n; ++i) {
        const auto& pt = displayCloud->points[i];
        data[i * 6 + 0] = pt.x;
        data[i * 6 + 1] = pt.y;
        data[i * 6 + 2] = pt.z;
        data[i * 6 + 3] = cr;
        data[i * 6 + 4] = cg;
        data[i * 6 + 5] = cb;
    }

    d->vao.bind();
    d->vbo.bind();
    d->vbo.allocate(data.constData(), data.size() * sizeof(float));
    // position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    // color
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    d->vbo.release();
    d->vao.release();

    d->vertexCount = n;
    doneCurrent();
    update();
}

void PointCloudViewer::removeSubmap(const QString& id) {
    makeCurrent();
    if (submapGL_.contains(id)) {
        auto* d = submapGL_.take(id);
        d->vao.destroy();
        d->vbo.destroy();
        delete d;
    }
    doneCurrent();
    update();
}

void PointCloudViewer::setSubmapVisible(const QString& id, bool visible) {
    if (submapGL_.contains(id)) {
        submapGL_[id]->visible = visible;
        update();
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  标记点上传
// ═══════════════════════════════════════════════════════════════════════

void PointCloudViewer::uploadMarkerGlBuffers() {
    QVector<MarkerVertex> normal;
    QVector<MarkerVertex> emph;
    normal.reserve(markerVertices_.size());
    emph.reserve(markerVertices_.size());
    for (int i = 0; i < markerVertices_.size(); ++i) {
        const bool isEmphasis =
            (i < markerEmphasis_.size() && markerEmphasis_[i] != 0);
        if (isEmphasis) emph.append(markerVertices_[i]);
        else normal.append(markerVertices_[i]);
    }
    markerNormalCount_   = normal.size();
    markerEmphasisCount_ = emph.size();

    markerVAO_.bind();
    markerVBO_.bind();
    if (markerNormalCount_ > 0)
        markerVBO_.allocate(normal.constData(), markerNormalCount_ * sizeof(MarkerVertex));
    else
        markerVBO_.allocate(nullptr, 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    markerVBO_.release();
    markerVAO_.release();

    markerEmphasisVAO_.bind();
    markerEmphasisVBO_.bind();
    if (markerEmphasisCount_ > 0)
        markerEmphasisVBO_.allocate(emph.constData(), markerEmphasisCount_ * sizeof(MarkerVertex));
    else
        markerEmphasisVBO_.allocate(nullptr, 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    markerEmphasisVBO_.release();
    markerEmphasisVAO_.release();
}

void PointCloudViewer::updateMarkers(const QVector<MarkerVertex>& markers,
                                      const QVector<MarkerInfo>& markerInfos,
                                      const QVector<quint8>& markerEmphasis) {
    makeCurrent();
    markerInfos_ = markerInfos;
    // 保存顶点数据以便拖拽时可以局部更新并重新上传
    markerVertices_.clear();
    markerVertices_ = markers;
    markerCount_    = markerVertices_.size();

    markerEmphasis_.clear();
    if (static_cast<int>(markerEmphasis.size()) == markerCount_ && markerCount_ > 0)
        markerEmphasis_ = markerEmphasis;
    else if (markerCount_ > 0)
        markerEmphasis_.fill(0, markerCount_);

    if (!markerVAO_.isCreated()) markerVAO_.create();
    if (!markerVBO_.isCreated()) markerVBO_.create();
    if (!markerEmphasisVAO_.isCreated()) markerEmphasisVAO_.create();
    if (!markerEmphasisVBO_.isCreated()) markerEmphasisVBO_.create();

    if (markerCount_ == 0) {
        markerNormalCount_   = 0;
        markerEmphasisCount_ = 0;
        doneCurrent();
        update();
        return;
    }

    uploadMarkerGlBuffers();

    doneCurrent();
    update();
}

void PointCloudViewer::updateSemanticArrows(const QVector<MarkerVertex>& lineVertices) {
    makeCurrent();

    if (!semanticArrowVAO_.isCreated()) semanticArrowVAO_.create();
    if (!semanticArrowVBO_.isCreated()) semanticArrowVBO_.create();

    semanticArrowVertexCount_ = lineVertices.size();
    if (semanticArrowVertexCount_ < 2) {
        semanticArrowVertexCount_ = 0;
        doneCurrent();
        update();
        return;
    }

    semanticArrowVAO_.bind();
    semanticArrowVBO_.bind();
    semanticArrowVBO_.allocate(lineVertices.constData(),
                               semanticArrowVertexCount_ * sizeof(MarkerVertex));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    semanticArrowVBO_.release();
    semanticArrowVAO_.release();

    doneCurrent();
    update();
}

// ═══════════════════════════════════════════════════════════════════════
//  路线折线上传
// ═══════════════════════════════════════════════════════════════════════

void PointCloudViewer::updateRouteLines(const QString& routeName,
                                         const QVector<MarkerVertex>& vertices) {
    makeCurrent();

    RouteGLData* rd = nullptr;
    if (routeGL_.contains(routeName)) {
        rd = routeGL_[routeName];
    } else {
        rd = new RouteGLData();
        rd->vao.create();
        rd->vbo.create();
        routeGL_[routeName] = rd;
    }

    rd->vertexCount = vertices.size();
    if (rd->vertexCount == 0) {
        doneCurrent();
        update();
        return;
    }

    rd->vao.bind();
    rd->vbo.bind();
    rd->vbo.allocate(vertices.constData(), vertices.size() * sizeof(MarkerVertex));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    rd->vbo.release();
    rd->vao.release();

    doneCurrent();
    update();
}

void PointCloudViewer::setRouteVisible(const QString& routeName, bool visible) {
    if (routeGL_.contains(routeName)) {
        routeGL_[routeName]->visible = visible;
        update();
    }
}

void PointCloudViewer::removeRouteLines(const QString& routeName) {
    makeCurrent();
    if (routeGL_.contains(routeName)) {
        auto* rd = routeGL_.take(routeName);
        rd->vao.destroy();
        rd->vbo.destroy();
        delete rd;
    }
    doneCurrent();
    update();
}

void PointCloudViewer::clearAllRouteLines() {
    makeCurrent();
    for (auto* rd : routeGL_) {
        rd->vao.destroy();
        rd->vbo.destroy();
        delete rd;
    }
    routeGL_.clear();
    doneCurrent();
    update();
}

// ═══════════════════════════════════════════════════════════════════════
//  裁剪框预览
// ═══════════════════════════════════════════════════════════════════════

void PointCloudViewer::updateCropBox(const QString& name,
                                      const QVector<MarkerVertex>& lineVertices) {
    makeCurrent();

    RouteGLData* cb = nullptr;
    if (cropBoxGL_.contains(name)) {
        cb = cropBoxGL_[name];
    } else {
        cb = new RouteGLData();
        cb->vao.create();
        cb->vbo.create();
        cropBoxGL_[name] = cb;
    }

    cb->vertexCount = lineVertices.size();
    if (cb->vertexCount == 0) {
        doneCurrent();
        update();
        return;
    }

    cb->vao.bind();
    cb->vbo.bind();
    cb->vbo.allocate(lineVertices.constData(), lineVertices.size() * sizeof(MarkerVertex));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    cb->vbo.release();
    cb->vao.release();

    doneCurrent();
    update();
}

void PointCloudViewer::removeCropBox(const QString& name) {
    makeCurrent();
    if (cropBoxGL_.contains(name)) {
        auto* cb = cropBoxGL_.take(name);
        cb->vao.destroy();
        cb->vbo.destroy();
        delete cb;
    }
    doneCurrent();
    update();
}

void PointCloudViewer::clearCropBoxes() {
    makeCurrent();
    for (auto* cb : cropBoxGL_) {
        cb->vao.destroy();
        cb->vbo.destroy();
        delete cb;
    }
    cropBoxGL_.clear();
    doneCurrent();
    update();
}

// ═══════════════════════════════════════════════════════════════════════
//  网格地面
// ═══════════════════════════════════════════════════════════════════════

void PointCloudViewer::buildGrid() {
    QVector<float> data;
    float extent = 100.0f;
    float step   = 5.0f;
    float cr = 0.25f, cg = 0.25f, cb = 0.28f;

    for (float i = -extent; i <= extent; i += step) {
        // X 方向线
        data << i << -extent << 0 << cr << cg << cb;
        data << i <<  extent << 0 << cr << cg << cb;
        // Y 方向线
        data << -extent << i << 0 << cr << cg << cb;
        data <<  extent << i << 0 << cr << cg << cb;
    }

    gridVertexCount_ = data.size() / 6;
    gridVAO_.create();
    gridVBO_.create();
    gridVAO_.bind();
    gridVBO_.bind();
    gridVBO_.allocate(data.constData(), data.size() * sizeof(float));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    gridVBO_.release();
    gridVAO_.release();
}

// ═══════════════════════════════════════════════════════════════════════
//  坐标轴
// ═══════════════════════════════════════════════════════════════════════

void PointCloudViewer::buildAxis() {
    float len = 3.0f;
    float data[] = {
        0,0,0, 1,0,0,  len,0,0, 1,0,0,  // X 红
        0,0,0, 0,1,0,  0,len,0, 0,1,0,  // Y 绿
        0,0,0, 0,0,1,  0,0,len, 0,0,1,  // Z 蓝
    };

    axisVAO_.create();
    axisVBO_.create();
    axisVAO_.bind();
    axisVBO_.bind();
    axisVBO_.allocate(data, sizeof(data));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    axisVBO_.release();
    axisVAO_.release();
}

// ═══════════════════════════════════════════════════════════════════════
//  鼠标交互 — 轨道球相机
// ═══════════════════════════════════════════════════════════════════════

void PointCloudViewer::mousePressEvent(QMouseEvent* e) {
    lastMousePos_ = e->pos();
    if (e->button() == Qt::LeftButton) {
        leftPressed_ = true;
        // 检测是否点击到标记点，若点击则准备长按拖拽
        auto hits = hitTestMarkers(e->pos());
        if (!hits.isEmpty()) {
            potentialDragIndex_ = hits.first();
            pressPos_ = e->pos();
            // 启动长按定时器（单次）
            QTimer::singleShot(longPressMs_, this, [this]() {
                if (leftPressed_ && potentialDragIndex_ >= 0) {
                    dragging_ = true;
                    dragIndex_ = potentialDragIndex_;
                    setCursor(Qt::ClosedHandCursor);
                }
            });
            return;
        }
        // 若未点击到标记点，则进入旋转模式
        rotating_ = true;
    }

    if (e->button() == Qt::MiddleButton) panning_ = true;

    if (pickMode_ != NoPickMode && e->button() == Qt::LeftButton) {
        performPick(e->pos());
        return;
    }
}

void PointCloudViewer::mouseMoveEvent(QMouseEvent* e) {
    QPoint delta = e->pos() - lastMousePos_;
    lastMousePos_ = e->pos();

    if (rotating_) {
        cameraAzimuth_   -= delta.x() * 0.3f;
        cameraElevation_ += delta.y() * 0.3f;
        cameraElevation_  = qBound(-89.0f, cameraElevation_, 89.0f);
        update();
    }
    if (panning_) {
        float scale = cameraDistance_ * 0.002f;
        float azRad = qDegreesToRadians(cameraAzimuth_);
        // 计算相机右方向和上方向
        QVector3D right(-std::sin(azRad), std::cos(azRad), 0);
        QVector3D up(0, 0, 1);
        cameraTarget_ -= right * delta.x() * scale;
        cameraTarget_ += up * delta.y() * scale;
        update();
    }

    // 悬停检测: 非拖拽时检测标记点
    if (!rotating_ && !panning_ && showMarkers_) {
        auto hits = hitTestMarkers(e->pos());
        if (!hits.isEmpty()) {
            QStringList lines;
            for (int idx : hits) {
                const auto& mi = markerInfos_[idx];
                QString typeStr = (mi.type == "portal") ? "🚪传送门" : "📍语义点";
                lines << QString("%1  [%2] %3").arg(typeStr, mi.submapId, mi.name);
            }
            QToolTip::showText(e->globalPos(), lines.join("\n"), this);
        } else {
            QToolTip::hideText();
        }
    }

    // 拖拽处理: 若处于拖拽状态，则根据当前屏幕位置计算世界坐标并更新该标记
    if (dragging_ && dragIndex_ >= 0 && dragIndex_ < markerCount_) {
        // 将屏幕坐标转换为世界坐标（读取深度或投影到 z=0 平面）
        makeCurrent();
        float depth = 1.0f;
        glReadPixels(e->pos().x(), height() - e->pos().y(), 1, 1,
                     GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

        Eigen::Vector3d worldPos;
        if (depth >= 1.0f) {
            // 与 z=0 平面相交
            QMatrix4x4 invVP = (projMatrix() * viewMatrix()).inverted();
            float nx = (2.0f * e->pos().x()) / width() - 1.0f;
            float ny = 1.0f - (2.0f * e->pos().y()) / height();
            QVector4D nearPt = invVP * QVector4D(nx, ny, -1.0f, 1.0f);
            QVector4D farPt  = invVP * QVector4D(nx, ny, 1.0f, 1.0f);
            nearPt /= nearPt.w();
            farPt  /= farPt.w();
            QVector3D rayOrig(nearPt);
            QVector3D rayDir = QVector3D(farPt) - rayOrig;
            if (std::abs(rayDir.z()) > 1e-6) {
                float t = -rayOrig.z() / rayDir.z();
                if (t > 0) {
                    QVector3D hit = rayOrig + t * rayDir;
                    worldPos = Eigen::Vector3d(hit.x(), hit.y(), hit.z());
                }
            }
        } else {
            worldPos = unproject(e->pos(), depth);
        }

        // 更新本地顶点缓存与元数据，并重新上传 GPU
        markerVertices_[dragIndex_].x = static_cast<float>(worldPos.x());
        markerVertices_[dragIndex_].y = static_cast<float>(worldPos.y());
        markerVertices_[dragIndex_].z = static_cast<float>(worldPos.z());
        markerInfos_[dragIndex_].worldPos = QVector3D((float)worldPos.x(), (float)worldPos.y(), (float)worldPos.z());

        uploadMarkerGlBuffers();
        doneCurrent();
        update();
    }
}

void PointCloudViewer::mouseReleaseEvent(QMouseEvent* e) {
    if (e->button() == Qt::LeftButton) {
        leftPressed_ = false;
        rotating_ = false;
        // 如果处在拖拽结束，发出信号并清理状态
        if (dragging_ && dragIndex_ >= 0 && dragIndex_ < markerCount_) {
            auto& mi = markerInfos_[dragIndex_];
            Eigen::Vector3d p(mi.worldPos.x(), mi.worldPos.y(), mi.worldPos.z());
            emit markerDragFinished(mi.submapId, mi.name, p);
        }
        dragging_ = false;
        potentialDragIndex_ = -1;
        dragIndex_ = -1;
        setCursor(Qt::ArrowCursor);
    }
    if (e->button() == Qt::MiddleButton) panning_  = false;
}

void PointCloudViewer::wheelEvent(QWheelEvent* e) {
    float factor = (e->angleDelta().y() > 0) ? 0.9f : 1.1f;
    cameraDistance_ *= factor;
    cameraDistance_ = qBound(0.5f, cameraDistance_, 2000.0f);
    update();
}

// ═══════════════════════════════════════════════════════════════════════
//  标记点悬停命中检测
// ═══════════════════════════════════════════════════════════════════════

QVector<int> PointCloudViewer::hitTestMarkers(const QPoint& screenPos, float threshold) {
    QVector<int> result;
    if (markerInfos_.isEmpty()) return result;

    QMatrix4x4 vp = projMatrix() * viewMatrix();

    for (int i = 0; i < markerInfos_.size(); ++i) {
        const auto& mi = markerInfos_[i];
        // 世界坐标 → 屏幕坐标
        QVector4D clip = vp * QVector4D(mi.worldPos, 1.0f);
        if (clip.w() <= 0.0f) continue;  // 在相机后方
        float ndcX = clip.x() / clip.w();
        float ndcY = clip.y() / clip.w();
        float sx = (ndcX * 0.5f + 0.5f) * width();
        float sy = (1.0f - (ndcY * 0.5f + 0.5f)) * height();

        float dx = sx - screenPos.x();
        float dy = sy - screenPos.y();
        float th = threshold;
        if (i < markerEmphasis_.size() && markerEmphasis_[i] != 0)
            th = threshold * 2.0f;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < th) {
            result.append(i);
        }
    }
    return result;
}

// ═══════════════════════════════════════════════════════════════════════
//  拾取
// ═══════════════════════════════════════════════════════════════════════

void PointCloudViewer::performPick(const QPoint& screenPos) {
    makeCurrent();

    // 读取深度缓冲
    float depth = 1.0f;
    glReadPixels(screenPos.x(), height() - screenPos.y(), 1, 1,
                 GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

    if (depth >= 1.0f) {
        // 没有点击到任何物体, 使用地面平面 (z=0)
        // 通过射线与 z=0 平面求交
        QMatrix4x4 invVP = (projMatrix() * viewMatrix()).inverted();

        // NDC 坐标
        float nx = (2.0f * screenPos.x()) / width() - 1.0f;
        float ny = 1.0f - (2.0f * screenPos.y()) / height();

        QVector4D nearPt = invVP * QVector4D(nx, ny, -1.0f, 1.0f);
        QVector4D farPt  = invVP * QVector4D(nx, ny, 1.0f, 1.0f);
        nearPt /= nearPt.w();
        farPt  /= farPt.w();

        QVector3D rayOrig(nearPt);
        QVector3D rayDir = QVector3D(farPt) - rayOrig;

        // z = 0 平面
        if (std::abs(rayDir.z()) > 1e-6) {
            float t = -rayOrig.z() / rayDir.z();
            if (t > 0) {
                QVector3D hit = rayOrig + t * rayDir;
                Eigen::Vector3d pos(hit.x(), hit.y(), hit.z());
                emit pointPicked(pos);
                pickMode_ = NoPickMode;
                setCursor(Qt::ArrowCursor);
            }
        }
    } else {
        // 反投影深度值到世界坐标
        auto pos = unproject(screenPos, depth);
        emit pointPicked(pos);
        pickMode_ = NoPickMode;
        setCursor(Qt::ArrowCursor);
    }

    doneCurrent();
}

Eigen::Vector3d PointCloudViewer::unproject(const QPoint& screenPos, float depth) {
    QMatrix4x4 invVP = (projMatrix() * viewMatrix()).inverted();
    float nx = (2.0f * screenPos.x()) / width() - 1.0f;
    float ny = 1.0f - (2.0f * screenPos.y()) / height();
    float nz = 2.0f * depth - 1.0f;

    QVector4D clip(nx, ny, nz, 1.0f);
    QVector4D world = invVP * clip;
    world /= world.w();

    return Eigen::Vector3d(world.x(), world.y(), world.z());
}

void PointCloudViewer::resetCamera() {
    cameraTarget_    = QVector3D(0, 0, 0);
    cameraDistance_   = 50.0f;
    cameraAzimuth_   = 45.0f;
    cameraElevation_ = 30.0f;
    update();
}

void PointCloudViewer::lookAt(const QVector3D& center) {
    cameraTarget_ = center;
    update();
}

} // namespace me
