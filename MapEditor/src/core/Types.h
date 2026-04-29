#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  MapEditor 公共类型定义
// ═══════════════════════════════════════════════════════════════════════

#include <QString>
#include <QVector>
#include <QColor>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace me {  // MapEditor namespace

// ── 基本几何类型 ─────────────────────────────────────────────────────
using PointT   = pcl::PointXYZ;
using CloudT   = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

/// 6-DOF 位姿: position + orientation(quaternion)
struct Pose {
    Eigen::Vector3d position    = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    /// 转为 4×4 齐次矩阵
    Eigen::Matrix4d toMatrix() const {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        m.block<3,3>(0,0) = orientation.toRotationMatrix();
        m.block<3,1>(0,3) = position;
        return m;
    }

    static Pose fromMatrix(const Eigen::Matrix4d& m) {
        Pose p;
        p.position    = m.block<3,1>(0,3);
        p.orientation = Eigen::Quaterniond(m.block<3,3>(0,0)).normalized();
        return p;
    }
};

/// 刚体变换 (translation + rotation)
struct RigidTransform {
    Eigen::Vector3d    translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation    = Eigen::Quaterniond::Identity();

    bool isIdentity() const {
        return translation.isZero(1e-9) &&
               rotation.isApprox(Eigen::Quaterniond::Identity(), 1e-9);
    }

    Eigen::Matrix4d toMatrix() const {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        m.block<3,3>(0,0) = rotation.toRotationMatrix();
        m.block<3,1>(0,3) = translation;
        return m;
    }

    static RigidTransform fromMatrix(const Eigen::Matrix4d& m) {
        RigidTransform t;
        t.translation = m.block<3,1>(0,3);
        t.rotation    = Eigen::Quaterniond(m.block<3,3>(0,0)).normalized();
        return t;
    }

    static RigidTransform identity() { return {}; }
};

// ── 语义点位 ─────────────────────────────────────────────────────────
struct SemanticPoint {
    QString name;
    QString type;            // single_pose | multi_pose | elevator_call | elevator_wait | elevator_in | elevator_out
    QVector<Pose> poses;     // single_pose 时仅一个元素
    bool    needInspect = false; ///< 是否需要巡检；single_pose 默认 true，其他类型默认 false

    /// 根据 type 返回 need_inspect 的默认值
    static bool defaultNeedInspect(const QString& type) {
        return type == QStringLiteral("single_pose");
    }
};

// ── 传送门 ───────────────────────────────────────────────────────────
struct Portal {
    QString portalName;      // 传送门名称 (对应语义点位名)
    QString targetSubmap;    // 目标 submap id
};

// ── 路线 ─────────────────────────────────────────────────────────────
struct RouteWaypoint {
    QString submapId;
    QString semanticPoint;
    QString routeRef;          // 非空时表示嵌套引用另一条路线（此时 submapId/semanticPoint 无意义）

    bool isRouteRef() const { return !routeRef.isEmpty(); }
};

struct Route {
    QString name;
    QString description;
    QVector<RouteWaypoint> waypoints;
};

// ── 渲染选项 ─────────────────────────────────────────────────────────
struct SubmapDisplayOptions {
    bool    visible          = true;
    bool    showSemanticPts  = true;
    bool    showPortals      = true;
    QColor  color            = QColor(200, 200, 200);
    float   voxelSize        = 1.0f;   // 显示用降采样体素大小(米), 默认1m
    float   pointSize        = 2.0f;
};

} // namespace me
