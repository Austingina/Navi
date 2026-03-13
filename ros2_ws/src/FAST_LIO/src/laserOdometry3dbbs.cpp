// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <gpu_bbs3d/bbs3d.cuh>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;
std::string global_frame("odom"), robot_frame("imu_link");
double ndt_thresh = 1.0;
double bbs3d_score_threshold = 0.8;
double bbs3d_voxel_size = 0.15;
double bbs3d_min_level_res = 0.6;
int bbs3d_max_level = 6;
double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

// Relocalize search range parameters
bool use_relocalize_hint = false;  // Whether to use hint position for search range
Eigen::Vector3f relocalize_hint_pos(0.0f, 0.0f, 0.0f);  // Hint position for relocalize
float relocalize_xy_range = 5.0f;  // XY search range (±5m)
float relocalize_z_range = 1.0f;   // Z search range (±1m)
Eigen::Vector3f map_min_z(0.0f, 0.0f, 0.0f);  // Map min z from point cloud
Eigen::Vector3f map_max_z(0.0f, 0.0f, 0.0f);  // Map max z from point cloud

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
        time_buffer.clear();
        lidar_pushed = false;
        flg_first_scan = true;
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(get_time_sec(msg->header.stamp));
    last_timestamp_lidar = get_time_sec(msg->header.stamp);
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
        time_buffer.clear();
        lidar_pushed = false;
        flg_first_scan = true;
    }
    last_timestamp_lidar = get_time_sec(msg->header.stamp);
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    //cout << "LIDAR time: " << std::fixed << std::setprecision(10) << last_timestamp_lidar <<" lidar_buffer.size() = " << lidar_buffer.size() << endl;
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in) 
{
    publish_count ++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        get_ros_time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "imu loop back, clear buffer" << std::endl;
        imu_buffer.clear();
        flg_first_scan = true;
    }

    last_timestamp_imu = timestamp;
    
    imu_buffer.push_back(msg);
    //std::cout << "IMU time: " << std::fixed << std::setprecision(10) << last_timestamp_imu << " imu_buffer.size() =" << imu_buffer.size() << std::endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        //std::cout << "lidar_buffer.empty()=" <<lidar_buffer.empty() <<   "  imu_buffer.empty() =" << imu_buffer.empty()<< std::endl;
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n" << std::endl;
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        //std::cout << "IMU not enough, skip this scan last_timestamp_imu:" << last_timestamp_imu << " lidar_end_time: " << lidar_end_time << std::endl;
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = global_frame;
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = robot_frame;
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    // std::cout << "effct_feat_num: " << effct_feat_num << std::endl;
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = global_frame;
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    laserCloudMap.header.frame_id = global_frame;
    pubLaserCloudMap->publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = global_frame;
    odomAftMapped.child_frame_id = robot_frame;
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    //rclcpp::Time now = rclcpp::Clock().now();
    //double header_stamp_sec = odomAftMapped.header.stamp.sec + odomAftMapped.header.stamp.nanosec * 1e-9;
    //std::cout << std::fixed << std::setprecision(9)
    //        << "[publish_odometry] now: " << now.seconds()
    //        << "  header.stamp: " << header_stamp_sec << std::endl;

    // odomAftMapped.pose.pose.position.z = 0;
    // pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }
    Eigen::Matrix3d m(state_point.rot);
    auto vel_body = m.inverse() * state_point.vel;
    odomAftMapped.twist.twist.linear.x = vel_body(0);
    odomAftMapped.twist.twist.linear.y = vel_body(1);
    odomAftMapped.twist.twist.linear.z = vel_body(2);
    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = global_frame;
    trans.child_frame_id = robot_frame;
    trans.header.stamp = get_ros_time(lidar_end_time);
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = global_frame;

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points! \n" << std::endl;
        flg_first_scan = true;
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

// teaser::PointCloud tgt_cloud;
// teaser::FPFHCloudPtr scene_descriptors;
// ====3D-BBS====
std::unique_ptr<gpu::BBS3D> bbs3d_ptr(new gpu::BBS3D);
// pcl::NormalDistributionsTransform<PointType, PointType> ndt;
pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
bool load_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap, const std::string &pcd_file)
{
    PointCloudXYZI::Ptr global_map(new PointCloudXYZI);
    PointCloudXYZI::Ptr ds_map_cloud(new PointCloudXYZI);
    if ((access(pcd_file.c_str(), F_OK) != -1) && (pcl::io::loadPCDFile<PointType>(pcd_file, *global_map) != -1)) // load existed pcd file
    {
        printf("\033[1mLoaded %ld data points from %s\033[0m\n", global_map->points.size(), pcd_file.c_str());
        sensor_msgs::msg::PointCloud2 laserCloudMap;
        pcl::toROSMsg(*global_map, laserCloudMap);
        laserCloudMap.header.stamp = rclcpp::Clock().now();
        laserCloudMap.header.frame_id = global_frame;
        pubLaserCloudMap->publish(laserCloudMap);
        printf("\033[1mpublished global map cloud to /Laser_map\033[0m\n");
    }
    else
    {
        std::cerr << "Cannot load map from " << pcd_file << std::endl;
        return false;
    }

    // downsample map points
    downSizeFilterMap.setInputCloud(global_map);
    downSizeFilterMap.filter(*ds_map_cloud);
    // build ikdtree and publish map data
    ikdtree.Build(ds_map_cloud->points);
    std::cout << "Map Kdtree Initial Done, kdtree size: " << ikdtree.size() << std::endl;;

    // initializing for global registration
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(bbs3d_voxel_size, bbs3d_voxel_size, bbs3d_voxel_size);
    voxel_grid_filter.setInputCloud(global_map);
    voxel_grid_filter.filter(*ds_map_cloud);
    // Convert to teaser point cloud
    std::vector<Eigen::Vector3f> tar_points;
    for (auto p : ds_map_cloud->points)
        tar_points.push_back({p.x, p.y, p.z});
    // Compute FPFH
    // teaser::FPFHEstimation fpfh;
    // scene_descriptors = fpfh.computeFPFHFeatures(tgt_cloud, voxel_size * 2, voxel_size * 5);

    bbs3d_ptr->set_tar_points(tar_points, bbs3d_min_level_res, bbs3d_max_level);
    bbs3d_ptr->set_trans_search_range(tar_points);
    bbs3d_ptr->set_angular_search_range({-0.02,-0.02,0.0}, {0.02,0.02,6.28});
    bbs3d_ptr->set_score_threshold_percentage(bbs3d_score_threshold);
    
    // Store map z range for relocalize with hint
    map_min_z = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    map_max_z = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
    for (const auto& p : tar_points) {
        map_min_z = map_min_z.cwiseMin(p);
        map_max_z = map_max_z.cwiseMax(p);
    }
    std::cout << "Map z range: [" << map_min_z.z() << ", " << map_max_z.z() << "]" << std::endl;
    
    std::cout << "3D BBS initialized! voxel_size: " << bbs3d_voxel_size << ", min_level_res: " << bbs3d_min_level_res << ", score_threshold: " << bbs3d_score_threshold << std::endl;
    // num_threads
    // int num_threads = 4;
    // bbs3d_ptr->set_num_threads(num_threads);
    // ndt.setInputTarget(ds_map_cloud);
    gicp.setInputTarget(ds_map_cloud);

    return true;
}

M3D imu_init_rot = Eye3d;
bool global_localize(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull, const double thresh)
{
    const PointCloudXYZI::Ptr lidar_frame = Measures.lidar;
    if (lidar_frame->points.size() < 10) return false;

    PointCloudXYZI::Ptr ds_lidar_frame(new PointCloudXYZI);
    double voxel_size = 0.5;
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid_filter.setInputCloud(lidar_frame);
    voxel_grid_filter.filter(*ds_lidar_frame);

    // teaser::PointCloud src_cloud;
    // Convert to teaser point cloud
    std::vector<Eigen::Vector3f> src_points;
    for (auto p : ds_lidar_frame->points)
        if (std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) > p_pre->blind)
            src_points.push_back({p.x, p.y, p.z});
    if (src_points.size() < 10) return false;

    double t0 = rclcpp::Clock().now().seconds();

    // Set search range based on hint position if available
    if (use_relocalize_hint) {
        // Calculate z search range, clamp to map bounds
        float z_min = std::max(relocalize_hint_pos.z() - relocalize_z_range, map_min_z.z());
        float z_max = std::min(relocalize_hint_pos.z() + relocalize_z_range, map_max_z.z());
        
        Eigen::Vector3f min_xyz(
            relocalize_hint_pos.x() - relocalize_xy_range,
            relocalize_hint_pos.y() - relocalize_xy_range,
            z_min  // Use hint z with range
        );
        Eigen::Vector3f max_xyz(
            relocalize_hint_pos.x() + relocalize_xy_range,
            relocalize_hint_pos.y() + relocalize_xy_range,
            z_max  // Use hint z with range
        );
        bbs3d_ptr->set_trans_search_range(min_xyz, max_xyz);
        std::cout << "[Relocalize] Using hint position (" << relocalize_hint_pos.x() << ", " 
                  << relocalize_hint_pos.y() << ", " << relocalize_hint_pos.z() << "), search range: xy=[" 
                  << min_xyz.x() << "~" << max_xyz.x() << ", " 
                  << min_xyz.y() << "~" << max_xyz.y() << "], z=[" 
                  << min_xyz.z() << "~" << max_xyz.z() << "]" << std::endl;
    }

    bbs3d_ptr->set_src_points(src_points);
    bbs3d_ptr->localize();

    std::cout << "[Localize] Execution time: " << bbs3d_ptr->get_elapsed_time() << "[msec] " << std::endl;
    std::cout << "[Localize] Score: " << bbs3d_ptr->get_best_score() << std::endl;

    if (!bbs3d_ptr->has_localized()) {
      if (bbs3d_ptr->has_timed_out())
        std::cout << "[Failed] Localization timed out." << std::endl;
      else
        std::cout << "[Failed] Score is below the threshold." << std::endl;
      return false;
    }

    // Compute FPFH
    // teaser::FPFHEstimation fpfh;
    // teaser::FPFHCloudPtr obj_descriptors = fpfh.computeFPFHFeatures(src_cloud, voxel_size * 2, voxel_size * 5);
    // double t1 = rclcpp::Clock().now().seconds();

    // teaser::Matcher matcher;
    // auto correspondences = matcher.calculateCorrespondences(
    //     src_cloud, tgt_cloud, *obj_descriptors, *scene_descriptors, false, true, false, 0.95);
    // std::cout << "Matcher correspondences: " << correspondences.size() << std::endl;
    // if (correspondences.size() < 10 || correspondences.size() > 1e4)
    // {
    //     std::cerr << "inappropriate correspondence quantities, abandon this frame data!" << std::endl;
    //     return false;
    // }
    double t2 = rclcpp::Clock().now().seconds();
    std::cout << "bbs3d solved pose:" << std::endl;
    std::cout << bbs3d_ptr->get_global_pose() << std::endl;

    // Run TEASER++ registration
    // Prepare solver parameters
    // teaser::RobustRegistrationSolver::Params params;
    // params.noise_bound = voxel_size;
    // params.cbar2 = 1.0;
    // params.estimate_scaling = false;
    // params.rotation_max_iterations = 100;
    // params.rotation_gnc_factor = 1.4;
    // params.rotation_estimation_algorithm =
    //     teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    // params.rotation_cost_threshold = 0.001;

    // // Solve with TEASER++
    // teaser::RobustRegistrationSolver solver(params);
    // Eigen::Matrix4d teaser_pose = Eigen::Matrix4d::Identity();
    // auto solution = solver.solve(src_cloud, tgt_cloud, correspondences);
    // std::cout << "TEASER SOLVED" << std::endl;
    // if (solution.valid)
    // {
    //     teaser_pose.block<3, 3>(0, 0) = solution.rotation;
    //     teaser_pose.block<3, 1>(0, 3) = solution.translation;
    //     std::cout << teaser_pose << std::endl;
    //     double t3 = rclcpp::Clock().now().seconds();
    //     // Cost times
    //     std::cout << "Global registration time taken (s): " << t3 - t0 << " (" << t1 - t0 << " + " << t2 - t1 << " + "
    //               << t3 - t2 << ")" << std::endl;
    // }
    // else
    // {
    //     std::cerr << "Teaser global registration is not valid!" << std::endl;
    //     return false;
    // }

    // ndt.setInputSource(ds_lidar_frame);
    gicp.setInputSource(ds_lidar_frame);
    PointCloudXYZI::Ptr ndt_aligned(new PointCloudXYZI);
    // ndt.align(*ndt_aligned, bbs3d_ptr->get_global_pose().cast<float>());
    gicp.align(*ndt_aligned, bbs3d_ptr->get_global_pose().cast<float>());
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*ndt_aligned, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(Measures.lidar_beg_time);
    laserCloudmsg.header.frame_id = global_frame;
    pubLaserCloudFull->publish(laserCloudmsg);

    // double ndt_score = ndt.getFitnessScore();
    // double probability = ndt.getTransformationProbability();
    // Eigen::Matrix4d ndt_pose = ndt.getFinalTransformation().cast<double>(); // lidar pose in global frame
    Eigen::Matrix4d ndt_pose = gicp.final_transformation_.cast<double>();
    std::cout << "gicp fitness score: " << gicp.getFitnessScore() << std::endl;
    if (gicp.hasConverged() == false || gicp.getFitnessScore() > 0.2)
        return false;
    std::cout << "lidar pose in global frame\n" << ndt_pose << std::endl;
    // printf("NDT registration score = %f, alignment probability = %f\n", ndt_score, probability);
    // if (ndt_score > thresh)
    // {
    //     std::cout << "Initial pose registration failed, it's not accurate enough!" << std::endl;
    //     return false;
    // }

    Eigen::Matrix4d lidar_in_imu_pose_matrix = Eigen::Matrix4d::Identity();
    lidar_in_imu_pose_matrix.block<3, 1>(0, 3) = Lidar_T_wrt_IMU;
    lidar_in_imu_pose_matrix.block<3, 3>(0, 0) = Lidar_R_wrt_IMU;
    // lidar_pose = lidar_in_imu_pose_matrix * imu_pose;
    Eigen::Matrix4d imu_pose = ndt_pose * lidar_in_imu_pose_matrix.inverse(); // imu pose in global frame
    imu_init_rot = imu_pose.block<3, 3>(0, 0).cast<double>();

    // set init pose to kf
    Eigen::Vector3d init_pos = imu_pose.block<3, 1>(0, 3).cast<double>();
    Eigen::Quaterniond init_rot(imu_pose.block<3, 3>(0, 0).cast<double>());

    state_ikfom init_state; // = kf.get_x();
    init_state.pos = init_pos;
    init_state.rot = init_rot;
    init_state.vel = Eigen::Vector3d(0.0, 0.0, 0.0);
    kf.change_x(init_state);

    return true;
}

class LaserOdometryNode : public rclcpp::Node
{
public:
    LaserOdometryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
        this->declare_parameter<double>("ndt_thresh", 1.0);
        this->declare_parameter<double>("bbs3d.score_threshold", 0.8);
        this->declare_parameter<double>("bbs3d.voxel_size", 0.15);
        this->declare_parameter<double>("bbs3d.min_level_res", 0.6);
        this->declare_parameter<int>("bbs3d.max_level", 6);
        this->declare_parameter<float>("bbs3d.xy_range", 5.0);
        this->declare_parameter<float>("bbs3d.z_range", 1.0);

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());
        this->get_parameter_or<double>("ndt_thresh", ndt_thresh, 1.0);
        this->get_parameter_or<double>("bbs3d.score_threshold", bbs3d_score_threshold, 0.8);
        this->get_parameter_or<double>("bbs3d.voxel_size", bbs3d_voxel_size, 0.15);
        this->get_parameter_or<double>("bbs3d.min_level_res", bbs3d_min_level_res, 0.6);
        this->get_parameter_or<int>("bbs3d.max_level", bbs3d_max_level, 6);
        this->get_parameter_or<float>("bbs3d.xy_range", relocalize_xy_range, 5.0f);
        this->get_parameter_or<float>("bbs3d.z_range", relocalize_z_range, 1.0f);

	pcd_save_en = false;

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = global_frame;

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 20, standard_pcl_cbk);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);
        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", custom_qos);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        if (!load_map(pubLaserCloudMap_, map_file_path)) {rclcpp::shutdown(); return;}
        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0)); // 10ms, 100hz
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserOdometryNode::timer_callback, this));
	    relocalize_srv_ = this->create_service<std_srvs::srv::Trigger>("global_relocalize", std::bind(&LaserOdometryNode::relocalize_callback, this, std::placeholders::_1, std::placeholders::_2));
        relocalize_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "relocalize_pose", 10, 
            std::bind(&LaserOdometryNode::relocalize_pose_callback, this, std::placeholders::_1));

        change_map_sub_ = this->create_subscription<std_msgs::msg::String>(
            "change_map", 10,
            std::bind(&LaserOdometryNode::change_map_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserOdometryNode()
    {
        fout_out.close();
        fout_pre.close();
        fclose(fp);
    }

private:
    void timer_callback()
    {
        // // 打印IMU和激光buffer队列长度
        // std::cout << "[fastlio_odometry-3] imu_buffer size: " << imu_buffer.size()
        //           << ", lidar_buffer size: " << lidar_buffer.size() << std::endl;
                  
        // rclcpp::Time now = rclcpp::Clock().now();
        // std::cout << std::fixed << std::setprecision(9)
        //      << "timer_callback now: " << now.seconds() << std::endl;
        if(sync_packages(Measures))
        {
            // rclcpp::Time now = rclcpp::Clock().now();
            // std::cout << std::fixed << std::setprecision(9)
            //     << "sync_packages now: " << now.seconds() << std::endl;
            if (flg_first_scan)
            {
                std::cout<<"flg_first_scan"<<std::endl;
                if(global_localize(pubLaserCloudFull_, ndt_thresh))
                {first_lidar_time = Measures.lidar_beg_time;
                feats_undistort.reset(new PointCloudXYZI());
                p_imu->Reset();
                p_imu->set_mapping_mode(false);
                p_imu->set_imu_init_rot(imu_init_rot);
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                path.poses.clear();}
                return;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            // std::cout << "point num before process" << Measures.lidar->points.size() <<"point num after process: " << feats_undistort->points.size() << std::endl;
            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
        
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
        
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
        
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            // map_incremental();
            t5 = omp_get_wtime();
        
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
            // publish_effect_world(pubLaserCloudEffect_);
            // publish_map(pubLaserCloudMap);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }
    }

    void relocalize_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "trigger global relocalization (full map search)...");
        use_relocalize_hint = false;  // Full map search
        flg_first_scan = true;
        res->success = true;
        res->message = "relocalize mode trigged (full map search)";
    }

    void relocalize_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "trigger global relocalization with hint position (%.2f, %.2f, %.2f)...", 
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        use_relocalize_hint = true;
        relocalize_hint_pos = Eigen::Vector3f(
            static_cast<float>(msg->pose.position.x),
            static_cast<float>(msg->pose.position.y),
            static_cast<float>(msg->pose.position.z)  // Use z value for search range
        );
        flg_first_scan = true;
    }

    void change_map_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string new_map_path = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received change_map request: %s", new_map_path.c_str());

        // Clear existing ikdtree by rebuilding with empty point cloud
        PointVector empty_cloud;
        ikdtree.Build(empty_cloud);

        // Load new map
        if (!load_map(pubLaserCloudMap_, new_map_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load new map: %s", new_map_path.c_str());
            return;
        }

        // Update map file path
        map_file_path = new_map_path;
        // 同步更新 ROS2 参数服务器
        this->set_parameter(rclcpp::Parameter("map_file_path", new_map_path));

        // Trigger relocalization
        flg_first_scan = true;
        RCLCPP_INFO(this->get_logger(), "Map switched to: %s, relocalization triggered.", new_map_path.c_str());
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr relocalize_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr change_map_sub_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);
    rclcpp::spin(std::make_shared<LaserOdometryNode>());
    // auto node = std::make_shared<LaserOdometryNode>();

    // rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
    // exec.add_node(node);
    // exec.spin();

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
