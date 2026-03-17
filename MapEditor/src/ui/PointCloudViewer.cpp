#include "ui/PointCloudViewer.h"
#include <QDebug>
#include <QtMath>
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
void main() {
    // 圆形点
    vec2 coord = gl_PointCoord - vec2(0.5);
    if (dot(coord, coord) > 0.25) discard;
    FragColor = vec4(vColor, 1.0);
}
)";

// ═══════════════════════════════════════════════════════════════════════
//  构造 / 析构
// ═══════════════════════════════════════════════════════════════════════

PointCloudViewer::PointCloudViewer(QWidget* parent) : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(400, 300);
}

PointCloudViewer::~PointCloudViewer() {
    makeCurrent();
    for (auto* d : submapGL_) {
        d->vao.destroy();
        d->vbo.destroy();
        delete d;
    }
    markerVAO_.destroy();
    markerVBO_.destroy();
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

    // 绘制网格
    {
        QMatrix4x4 mvp = proj * view;
        cloudShader_->setUniformValue("uMVP", mvp);
        glPointSize(1.0f);
        gridVAO_.bind();
        glDrawArrays(GL_LINES, 0, gridVertexCount_);
        gridVAO_.release();
    }

    // 绘制坐标轴
    {
        QMatrix4x4 mvp = proj * view;
        cloudShader_->setUniformValue("uMVP", mvp);
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

        QMatrix4x4 mvp = proj * view * d->modelMatrix;
        cloudShader_->setUniformValue("uMVP", mvp);
        glPointSize(d->pointSize);
        d->vao.bind();
        glDrawArrays(GL_POINTS, 0, d->vertexCount);
        d->vao.release();
    }

    // 绘制标记点 (语义点/传送门)
    if (showMarkers_ && markerCount_ > 0) {
        QMatrix4x4 mvp = proj * view;
        cloudShader_->setUniformValue("uMVP", mvp);
        glPointSize(10.0f);
        markerVAO_.bind();
        glDrawArrays(GL_POINTS, 0, markerCount_);
        markerVAO_.release();
    }

    cloudShader_->release();
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

void PointCloudViewer::updateMarkers(const QVector<MarkerVertex>& markers) {
    makeCurrent();

    if (!markerVAO_.isCreated()) markerVAO_.create();
    if (!markerVBO_.isCreated()) markerVBO_.create();

    markerCount_ = markers.size();
    if (markerCount_ == 0) {
        doneCurrent();
        update();
        return;
    }

    markerVAO_.bind();
    markerVBO_.bind();
    markerVBO_.allocate(markers.constData(), markerCount_ * sizeof(MarkerVertex));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MarkerVertex),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    markerVBO_.release();
    markerVAO_.release();

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

    if (pickMode_ != NoPickMode && e->button() == Qt::LeftButton) {
        performPick(e->pos());
        return;
    }

    if (e->button() == Qt::LeftButton)  rotating_ = true;
    if (e->button() == Qt::MiddleButton) panning_ = true;
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
}

void PointCloudViewer::mouseReleaseEvent(QMouseEvent* e) {
    if (e->button() == Qt::LeftButton)   rotating_ = false;
    if (e->button() == Qt::MiddleButton) panning_  = false;
}

void PointCloudViewer::wheelEvent(QWheelEvent* e) {
    float factor = (e->angleDelta().y() > 0) ? 0.9f : 1.1f;
    cameraDistance_ *= factor;
    cameraDistance_ = qBound(0.5f, cameraDistance_, 2000.0f);
    update();
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
