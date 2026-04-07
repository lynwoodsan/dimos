// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// FAR Planner native module for dimos NativeModule framework.
//
// Replaces the ROS2 FARMaster node with CLI arg parsing + LCM pub/sub.
// The algorithm modules (ContourDetector, DynamicGraph, GraphPlanner,
// ContourGraph, MapHandler, ScanHandler) are called directly — FARMaster
// is bypassed because it is too tightly coupled to ROS.
//
// Usage:
//   ./far_planner_native \
//       --terrain_map_ext '/terrain_map_ext#sensor_msgs.PointCloud2' \
//       --terrain_map '/terrain_map#sensor_msgs.PointCloud2' \
//       --registered_scan '/registered_scan#sensor_msgs.PointCloud2' \
//       --odometry '/odometry#nav_msgs.Odometry' \
//       --goal '/goal#geometry_msgs.PointStamped' \
//       --way_point '/way_point#geometry_msgs.PointStamped' \
//       --goal_path '/goal_path#nav_msgs.Path' \
//       --update_rate 5.0 --sensor_range 15.0

#include <lcm/lcm-cpp.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// dimos-lcm message types
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "nav_msgs/Odometry.hpp"
#include "geometry_msgs/PointStamped.hpp"
#include "nav_msgs/Path.hpp"
#include "std_msgs/Header.hpp"

// SmartNav common helpers
#include "dimos_native_module.hpp"
#include "point_cloud_utils.hpp"

// FAR Planner core (via compat layer — no ROS deps)
#include "far_planner/compat.h"
#include "far_planner/utility.h"
#include "far_planner/contour_detector.h"
#include "far_planner/dynamic_graph.h"
#include "far_planner/graph_planner.h"
#include "far_planner/contour_graph.h"
#include "far_planner/map_handler.h"
#include "far_planner/scan_handler.h"

// ---------------------------------------------------------------------------
// FARUtil static definitions (normally in far_planner.cpp — we bypass that file)
// ---------------------------------------------------------------------------

PointCloudPtr  FARUtil::surround_obs_cloud_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::surround_free_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_new_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_new_cloud_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_dyobs_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_dyobs_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_scan_cloud_      = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::local_terrain_obs_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::local_terrain_free_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_new_cloud_    = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_filter_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());

const float FARUtil::kEpsilon = 1e-7f;
const float FARUtil::kINF     = std::numeric_limits<float>::max();
std::string FARUtil::worldFrameId;
float   FARUtil::kAngleNoise;
Point3D FARUtil::robot_pos;
Point3D FARUtil::odom_pos;
Point3D FARUtil::map_origin;
Point3D FARUtil::free_odom_p;
float   FARUtil::robot_dim;
float   FARUtil::vehicle_height;
float   FARUtil::kLeafSize;
float   FARUtil::kHeightVoxel;
float   FARUtil::kNavClearDist;
float   FARUtil::kCellLength;
float   FARUtil::kCellHeight;
float   FARUtil::kNewPIThred;
float   FARUtil::kSensorRange;
float   FARUtil::kMarginDist;
float   FARUtil::kMarginHeight;
float   FARUtil::kTerrainRange;
float   FARUtil::kLocalPlanRange;
float   FARUtil::kFreeZ;
float   FARUtil::kVizRatio;
double  FARUtil::systemStartTime;
float   FARUtil::kObsDecayTime;
float   FARUtil::kNewDecayTime;
float   FARUtil::kNearDist;
float   FARUtil::kMatchDist;
float   FARUtil::kProjectDist;
int     FARUtil::kDyObsThred;
int     FARUtil::KNewPointC;
int     FARUtil::kObsInflate;
float   FARUtil::kTolerZ;
float   FARUtil::kAcceptAlign;
bool    FARUtil::IsStaticEnv;
bool    FARUtil::IsDebug;
bool    FARUtil::IsMultiLayer;
TimeMeasure FARUtil::Timer;

// DynamicGraph statics
DynamicGraphParams DynamicGraph::dg_params_;
NodePtrStack DynamicGraph::globalGraphNodes_;
std::size_t  DynamicGraph::id_tracker_;
std::unordered_map<std::size_t, NavNodePtr> DynamicGraph::idx_node_map_;
std::unordered_map<NavNodePtr, std::pair<int, std::unordered_set<NavNodePtr>>> DynamicGraph::out_contour_nodes_map_;

// ContourGraph statics
CTNodeStack ContourGraph::polys_ctnodes_;
CTNodeStack ContourGraph::contour_graph_;
PolygonStack ContourGraph::contour_polygons_;
std::vector<PointPair> ContourGraph::global_contour_;
std::vector<PointPair> ContourGraph::unmatched_contour_;
std::vector<PointPair> ContourGraph::inactive_contour_;
std::vector<PointPair> ContourGraph::boundary_contour_;
std::vector<PointPair> ContourGraph::local_boundary_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::global_contour_set_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::boundary_contour_set_;

// MapHandler statics
PointKdTreePtr MapHandler::kdtree_terrain_clould_;
std::vector<int> MapHandler::terrain_grid_occupy_list_;
std::vector<int> MapHandler::terrain_grid_traverse_list_;
std::unordered_set<int> MapHandler::neighbor_obs_indices_;
std::unordered_set<int> MapHandler::extend_obs_indices_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_free_cloud_grid_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_obs_cloud_grid_;
std::unique_ptr<grid_ns::Grid<std::vector<float>>> MapHandler::terrain_height_grid_;

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

static std::atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;

// Topic strings (set from CLI args)
static std::string g_terrain_map_ext_topic;
static std::string g_terrain_map_topic;
static std::string g_registered_scan_topic;
static std::string g_odometry_topic;
static std::string g_goal_topic;
static std::string g_way_point_topic;
static std::string g_goal_path_topic;

// World frame for published messages
static std::string g_world_frame = "map";

// Thread-safe input buffers: odometry
static std::mutex g_odom_mutex;
static nav_msgs::Odometry g_latest_odom;
static bool g_has_odom = false;

// Thread-safe input buffers: terrain_map_ext
static std::mutex g_terrain_ext_mutex;
static sensor_msgs::PointCloud2 g_latest_terrain_ext;
static bool g_has_terrain_ext = false;

// Thread-safe input buffers: terrain_map
static std::mutex g_terrain_mutex;
static sensor_msgs::PointCloud2 g_latest_terrain;
static bool g_has_terrain = false;

// Thread-safe input buffers: registered_scan
static std::mutex g_scan_mutex;
static sensor_msgs::PointCloud2 g_latest_scan;
static bool g_has_scan = false;

// Thread-safe input buffers: goal
static std::mutex g_goal_mutex;
static geometry_msgs::PointStamped g_latest_goal;
static bool g_has_goal = false;
static bool g_goal_consumed = true;  // tracks whether current goal was processed

// ---------------------------------------------------------------------------
// LCM callback handlers
// ---------------------------------------------------------------------------

class Handlers {
public:
    void on_odometry(const lcm::ReceiveBuffer* /*rbuf*/,
                     const std::string& /*channel*/,
                     const nav_msgs::Odometry* msg) {
        std::lock_guard<std::mutex> lock(g_odom_mutex);
        g_latest_odom = *msg;
        g_has_odom = true;
    }

    void on_terrain_ext(const lcm::ReceiveBuffer* /*rbuf*/,
                        const std::string& /*channel*/,
                        const sensor_msgs::PointCloud2* msg) {
        std::lock_guard<std::mutex> lock(g_terrain_ext_mutex);
        g_latest_terrain_ext = *msg;
        g_has_terrain_ext = true;
    }

    void on_terrain(const lcm::ReceiveBuffer* /*rbuf*/,
                    const std::string& /*channel*/,
                    const sensor_msgs::PointCloud2* msg) {
        std::lock_guard<std::mutex> lock(g_terrain_mutex);
        g_latest_terrain = *msg;
        g_has_terrain = true;
    }

    void on_registered_scan(const lcm::ReceiveBuffer* /*rbuf*/,
                            const std::string& /*channel*/,
                            const sensor_msgs::PointCloud2* msg) {
        std::lock_guard<std::mutex> lock(g_scan_mutex);
        g_latest_scan = *msg;
        g_has_scan = true;
    }

    void on_goal(const lcm::ReceiveBuffer* /*rbuf*/,
                 const std::string& /*channel*/,
                 const geometry_msgs::PointStamped* msg) {
        std::lock_guard<std::mutex> lock(g_goal_mutex);
        g_latest_goal = *msg;
        g_has_goal = true;
        g_goal_consumed = false;
    }
};

// ---------------------------------------------------------------------------
// Conversion helpers
// ---------------------------------------------------------------------------

/// Convert dimos-lcm PointCloud2 to pcl::PointCloud<PCLPoint> via point_cloud_utils
static PointCloudPtr lcm_pc2_to_pcl(const sensor_msgs::PointCloud2& lcm_msg) {
    PointCloudPtr cloud(new pcl::PointCloud<PCLPoint>());
    auto points = smart_nav::parse_pointcloud2(lcm_msg);
    cloud->reserve(points.size());
    for (const auto& p : points) {
        PCLPoint pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = p.intensity;
        cloud->push_back(pt);
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

/// Convert Point3D to dimos-lcm geometry_msgs::PointStamped
static geometry_msgs::PointStamped point3d_to_lcm_point_stamped(
    const Point3D& pt, double timestamp) {
    geometry_msgs::PointStamped msg;
    msg.header = dimos::make_header(g_world_frame, timestamp);
    msg.point.x = pt.x;
    msg.point.y = pt.y;
    msg.point.z = pt.z;
    return msg;
}

/// Convert a path (vector of NavNodePtr) to dimos-lcm nav_msgs::Path
static nav_msgs::Path path_to_lcm_path(
    const NodePtrStack& path_nodes, double timestamp) {
    nav_msgs::Path msg;
    msg.header = dimos::make_header(g_world_frame, timestamp);
    msg.poses_length = static_cast<int32_t>(path_nodes.size());
    msg.poses.resize(path_nodes.size());
    for (size_t i = 0; i < path_nodes.size(); ++i) {
        auto& pose = msg.poses[i];
        pose.header = msg.header;
        pose.pose.position.x = path_nodes[i]->position.x;
        pose.pose.position.y = path_nodes[i]->position.y;
        pose.pose.position.z = path_nodes[i]->position.z;
        pose.pose.orientation.w = 1.0;
    }
    return msg;
}

/// Extract robot position from odometry message
static Point3D odom_to_point3d(const nav_msgs::Odometry& odom) {
    return Point3D(
        static_cast<float>(odom.pose.pose.position.x),
        static_cast<float>(odom.pose.pose.position.y),
        static_cast<float>(odom.pose.pose.position.z));
}

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------

static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    dimos::NativeModule mod(argc, argv);

    // ── Get topic strings ──────────────────────────────────────────────────
    g_terrain_map_ext_topic = mod.has("terrain_map_ext") ? mod.topic("terrain_map_ext") : "";
    g_terrain_map_topic     = mod.has("terrain_map") ? mod.topic("terrain_map") : "";
    g_registered_scan_topic = mod.has("registered_scan") ? mod.topic("registered_scan") : "";
    g_odometry_topic        = mod.has("odometry") ? mod.topic("odometry") : "";
    g_goal_topic            = mod.has("goal") ? mod.topic("goal") : "";
    g_way_point_topic       = mod.has("way_point") ? mod.topic("way_point") : "";
    g_goal_path_topic       = mod.has("goal_path") ? mod.topic("goal_path") : "";

    // ── Get config params (mirrors LoadROSParams in original FARMaster) ────
    float update_rate         = mod.arg_float("update_rate", 5.0f);
    float robot_dim           = mod.arg_float("robot_dim", 0.5f);
    float voxel_dim           = mod.arg_float("voxel_dim", 0.1f);
    float sensor_range        = mod.arg_float("sensor_range", 15.0f);
    float terrain_range       = mod.arg_float("terrain_range", 7.5f);
    float local_planner_range = mod.arg_float("local_planner_range", 2.5f);
    float vehicle_height      = mod.arg_float("vehicle_height", 0.75f);
    float visibility_range    = mod.arg_float("visibility_range", 15.0f);
    bool  is_static_env       = mod.arg_bool("is_static_env", false);
    bool  is_viewpoint_extend = mod.arg_bool("is_viewpoint_extend", true);
    float converge_dist       = mod.arg_float("converge_dist", 0.8f);
    g_world_frame             = mod.arg("world_frame", "map");

    // Remaining params are read inline via mod.arg_*() where they're used.

    printf("[far_planner] Starting FAR Planner native module\n");
    printf("[far_planner] terrain_map_ext: %s\n",
           g_terrain_map_ext_topic.empty() ? "(disabled)" : g_terrain_map_ext_topic.c_str());
    printf("[far_planner] terrain_map: %s\n",
           g_terrain_map_topic.empty() ? "(disabled)" : g_terrain_map_topic.c_str());
    printf("[far_planner] registered_scan: %s\n",
           g_registered_scan_topic.empty() ? "(disabled)" : g_registered_scan_topic.c_str());
    printf("[far_planner] odometry: %s\n",
           g_odometry_topic.empty() ? "(disabled)" : g_odometry_topic.c_str());
    printf("[far_planner] goal: %s\n",
           g_goal_topic.empty() ? "(disabled)" : g_goal_topic.c_str());
    printf("[far_planner] way_point: %s\n",
           g_way_point_topic.empty() ? "(disabled)" : g_way_point_topic.c_str());
    printf("[far_planner] goal_path: %s\n",
           g_goal_path_topic.empty() ? "(disabled)" : g_goal_path_topic.c_str());
    printf("[far_planner] update_rate=%.1f Hz  sensor_range=%.1f  vehicle_height=%.2f\n",
           update_rate, sensor_range, vehicle_height);

    // ── Init LCM ───────────────────────────────────────────────────────────
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[far_planner] Error: LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    // ── Subscribe to input topics ──────────────────────────────────────────
    Handlers handlers;
    if (!g_odometry_topic.empty())
        lcm.subscribe(g_odometry_topic, &Handlers::on_odometry, &handlers);
    if (!g_terrain_map_ext_topic.empty())
        lcm.subscribe(g_terrain_map_ext_topic, &Handlers::on_terrain_ext, &handlers);
    if (!g_terrain_map_topic.empty())
        lcm.subscribe(g_terrain_map_topic, &Handlers::on_terrain, &handlers);
    if (!g_registered_scan_topic.empty())
        lcm.subscribe(g_registered_scan_topic, &Handlers::on_registered_scan, &handlers);
    if (!g_goal_topic.empty())
        lcm.subscribe(g_goal_topic, &Handlers::on_goal, &handlers);

    // ── Set FARUtil static parameters ──────────────────────────────────────
    // These correspond to LoadROSParams() in the original FARMaster.
    // Mirror LoadROSParams() from original FARMaster — set FARUtil statics.
    // Note: kEpsilon and kINF are const statics, already initialized.
    FARUtil::vehicle_height  = vehicle_height;
    FARUtil::kSensorRange    = sensor_range;
    FARUtil::kTerrainRange   = std::min(terrain_range, sensor_range);
    FARUtil::kLocalPlanRange = local_planner_range;
    FARUtil::kLeafSize       = voxel_dim;
    FARUtil::kNearDist       = robot_dim;
    FARUtil::kHeightVoxel    = voxel_dim * 2.0f;
    FARUtil::kMatchDist      = robot_dim * 2.0f + voxel_dim;
    FARUtil::kNavClearDist   = robot_dim / 2.0f + voxel_dim;
    FARUtil::kProjectDist    = voxel_dim;
    FARUtil::kMarginDist     = sensor_range - FARUtil::kMatchDist;
    FARUtil::kFreeZ          = 0.1f;
    float floor_height_val   = mod.arg_float("floor_height", 2.0f);
    FARUtil::kTolerZ         = floor_height_val - FARUtil::kHeightVoxel;
    FARUtil::kCellLength     = mod.arg_float("cell_length", 5.0f);
    FARUtil::kCellHeight     = floor_height_val / 2.5f;
    FARUtil::kMarginHeight   = FARUtil::kTolerZ - FARUtil::kCellHeight / 2.0f;
    FARUtil::kVizRatio       = mod.arg_float("visualize_ratio", 0.4f);
    FARUtil::worldFrameId    = g_world_frame;
    FARUtil::kObsInflate     = mod.arg_int("obs_inflate_size", 2);
    FARUtil::kAngleNoise     = mod.arg_float("angle_noise", 15.0f) / 180.0f * M_PI;
    FARUtil::kAcceptAlign    = mod.arg_float("accept_max_align_angle", 15.0f) / 180.0f * M_PI;
    FARUtil::kNewPIThred     = mod.arg_float("new_intensity_thred", 2.0f);
    FARUtil::kObsDecayTime   = mod.arg_float("dynamic_obs_decay_time", 10.0f);
    FARUtil::kNewDecayTime   = mod.arg_float("new_points_decay_time", 2.0f);
    FARUtil::kDyObsThred     = mod.arg_int("dyobs_update_thred", 4);
    FARUtil::KNewPointC      = mod.arg_int("new_point_counter", 10);
    FARUtil::IsStaticEnv     = is_static_env;
    FARUtil::IsDebug         = mod.arg_bool("is_debug_output", false);
    FARUtil::IsMultiLayer    = mod.arg_bool("is_multi_layer", false);
    FARUtil::robot_dim       = robot_dim;
    FARUtil::systemStartTime = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // Init FARUtil kdtrees
    FARUtil::kdtree_new_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
    PointCloudPtr init_cloud(new pcl::PointCloud<PCLPoint>());
    PCLPoint zero_p;
    zero_p.x = zero_p.y = zero_p.z = 0.0f;
    zero_p.intensity = 0.0f;
    init_cloud->push_back(zero_p);
    FARUtil::kdtree_new_cloud_->setInputCloud(init_cloud);
    FARUtil::robot_pos = Point3D(0, 0, 0);
    FARUtil::free_odom_p = Point3D(0, 0, 0);
    FARUtil::odom_pos = Point3D(0, 0, 0);

    // ── Create stub CompatNode for algorithm modules that need rclcpp::Node ─
    auto nh = std::make_shared<CompatNode>("far_planner");

    // ── Initialize algorithm modules ───────────────────────────────────────

    // ContourDetector
    ContourDetector contour_detector;
    {
        ContourDetectParams cd_params;
        cd_params.sensor_range = sensor_range;
        cd_params.voxel_dim    = voxel_dim;
        cd_params.kRatio       = mod.arg_float("resize_ratio", 5.0f);
        cd_params.kThredValue  = mod.arg_int("filter_count_value", 5);
        cd_params.kBlurSize    = static_cast<int>(std::round(FARUtil::kNavClearDist / voxel_dim));
        cd_params.is_save_img  = false;
        cd_params.img_path     = "";
        contour_detector.Init(cd_params);
    }

    // ContourGraph
    ContourGraph contour_graph;
    {
        ContourGraphParams cg_params;
        cg_params.kPillarPerimeter = robot_dim * 4.0f;
        contour_graph.Init(nh, cg_params);
    }

    // DynamicGraph
    DynamicGraph dynamic_graph;
    {
        DynamicGraphParams dg_params;
        dg_params.dumper_thred             = mod.arg_int("clear_dumper_thred", 3);
        dg_params.finalize_thred           = mod.arg_int("node_finalize_thred", 3);
        dg_params.pool_size                = mod.arg_int("filter_pool_size", 12);
        dg_params.votes_size               = mod.arg_int("connect_votes_size", 10);
        dg_params.kConnectAngleThred       = FARUtil::kAcceptAlign;
        dg_params.filter_pos_margin        = FARUtil::kNavClearDist;
        dg_params.filter_dirs_margin       = FARUtil::kAngleNoise;
        dg_params.frontier_perimeter_thred = FARUtil::kMatchDist * 4.0f;
        dynamic_graph.Init(nh, dg_params);
    }

    // GraphPlanner
    GraphPlanner graph_planner;
    {
        GraphPlannerParams gp_params;
        gp_params.converge_dist  = converge_dist;
        gp_params.adjust_radius  = mod.arg_float("goal_adjust_radius", 10.0f);
        gp_params.momentum_dist  = robot_dim / 2.0f;
        gp_params.is_autoswitch  = mod.arg_bool("is_attempt_autoswitch", true);
        gp_params.free_thred     = mod.arg_int("free_counter_thred", 5);
        gp_params.votes_size     = mod.arg_int("reach_goal_vote_size", 5);
        gp_params.momentum_thred = mod.arg_int("path_momentum_thred", 5);
        graph_planner.Init(nh, gp_params);
    }

    // MapHandler
    MapHandler map_handler;
    {
        MapHandlerParams mh_params;
        mh_params.sensor_range     = sensor_range;
        mh_params.floor_height     = floor_height_val;
        mh_params.cell_length      = FARUtil::kCellLength;
        mh_params.cell_height      = FARUtil::kCellHeight;
        mh_params.grid_max_length  = mod.arg_float("map_grid_max_length", 1000.0f);
        mh_params.grid_max_height  = mod.arg_float("map_grad_max_height", 100.0f);
        mh_params.height_voxel_dim = FARUtil::kHeightVoxel;
        map_handler.Init(mh_params);
    }

    // ScanHandler
    ScanHandler scan_handler;
    {
        ScanHandlerParams sh_params;
        sh_params.terrain_range = terrain_range;
        sh_params.voxel_size    = voxel_dim;
        sh_params.ceil_height   = floor_height_val;
        scan_handler.Init(sh_params);
    }

    // ── Tracking state for main loop ───────────────────────────────────────
    bool is_map_init = false;
    bool has_received_odom = false;
    Point3D robot_pos(0, 0, 0);
    Point3D last_robot_pos(0, 0, 0);
    int frame_count = 0;

    // ── Main loop ──────────────────────────────────────────────────────────
    float loop_period_ms = 1000.0f / update_rate;
    printf("[far_planner] Starting main loop at %.1f Hz\n", update_rate);

    while (g_running.load()) {
        auto loop_start = std::chrono::high_resolution_clock::now();

        // Handle all pending LCM messages (non-blocking)
        while (lcm.handleTimeout(0) > 0) {}

        double now_sec = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // ── Snapshot input buffers ─────────────────────────────────────────
        nav_msgs::Odometry cur_odom;
        bool have_odom = false;
        {
            std::lock_guard<std::mutex> lock(g_odom_mutex);
            if (g_has_odom) {
                cur_odom = g_latest_odom;
                have_odom = true;
            }
        }

        sensor_msgs::PointCloud2 cur_terrain_ext;
        bool have_terrain_ext = false;
        {
            std::lock_guard<std::mutex> lock(g_terrain_ext_mutex);
            if (g_has_terrain_ext) {
                cur_terrain_ext = g_latest_terrain_ext;
                have_terrain_ext = true;
                g_has_terrain_ext = false;  // consume
            }
        }

        sensor_msgs::PointCloud2 cur_terrain;
        bool have_terrain = false;
        {
            std::lock_guard<std::mutex> lock(g_terrain_mutex);
            if (g_has_terrain) {
                cur_terrain = g_latest_terrain;
                have_terrain = true;
                g_has_terrain = false;
            }
        }

        sensor_msgs::PointCloud2 cur_scan;
        bool have_scan = false;
        {
            std::lock_guard<std::mutex> lock(g_scan_mutex);
            if (g_has_scan) {
                cur_scan = g_latest_scan;
                have_scan = true;
                g_has_scan = false;
            }
        }

        geometry_msgs::PointStamped cur_goal_msg;
        bool have_new_goal = false;
        {
            std::lock_guard<std::mutex> lock(g_goal_mutex);
            if (g_has_goal && !g_goal_consumed) {
                cur_goal_msg = g_latest_goal;
                have_new_goal = true;
                g_goal_consumed = true;
            }
        }

        // Need odometry to do anything
        if (!have_odom) {
            auto elapsed = std::chrono::high_resolution_clock::now() - loop_start;
            auto elapsed_ms = std::chrono::duration<float, std::milli>(elapsed).count();
            if (elapsed_ms < loop_period_ms) {
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    static_cast<int>(loop_period_ms - elapsed_ms)));
            }
            continue;
        }

        // ── Update robot position ──────────────────────────────────────────
        robot_pos = odom_to_point3d(cur_odom);
        FARUtil::robot_pos = robot_pos;
        FARUtil::odom_pos = robot_pos;

        if (!has_received_odom) {
            has_received_odom = true;
            last_robot_pos = robot_pos;
            printf("[far_planner] First odometry received: (%.2f, %.2f, %.2f)\n",
                   robot_pos.x, robot_pos.y, robot_pos.z);
        }

        // Init map origin on first odometry
        if (!is_map_init) {
            map_handler.SetMapOrigin(robot_pos);
            scan_handler.ReInitGrids();
            is_map_init = true;
            printf("[far_planner] Map origin set\n");
        }

        // Update positions in algorithm modules
        map_handler.UpdateRobotPosition(robot_pos);
        scan_handler.UpdateRobotPosition(robot_pos);
        dynamic_graph.UpdateRobotPosition(robot_pos);

        // ── Process terrain_map_ext (obstacle + free separation) ───────────
        PointCloudPtr obs_cloud(new pcl::PointCloud<PCLPoint>());
        PointCloudPtr free_cloud(new pcl::PointCloud<PCLPoint>());
        PointCloudPtr terrain_height_cloud(new pcl::PointCloud<PCLPoint>());
        bool have_new_clouds = false;

        if (have_terrain_ext) {
            PointCloudPtr terrain_ext_pcl = lcm_pc2_to_pcl(cur_terrain_ext);
            if (!terrain_ext_pcl->empty()) {
                // Crop to sensor range around robot
                FARUtil::CropPCLCloud(terrain_ext_pcl, robot_pos, sensor_range);
                FARUtil::RemoveNanInfPoints(terrain_ext_pcl);

                // Separate free and obstacle points (based on intensity/z threshold)
                FARUtil::ExtractFreeAndObsCloud(terrain_ext_pcl, free_cloud, obs_cloud);
                have_new_clouds = true;

                // Update cloud grids
                map_handler.UpdateObsCloudGrid(obs_cloud);
                map_handler.UpdateFreeCloudGrid(free_cloud);
                map_handler.UpdateTerrainHeightGrid(free_cloud, terrain_height_cloud);
            }
        }

        // ── Process terrain_map (alternative terrain input) ────────────────
        if (have_terrain && !have_terrain_ext) {
            PointCloudPtr terrain_pcl = lcm_pc2_to_pcl(cur_terrain);
            if (!terrain_pcl->empty()) {
                FARUtil::CropPCLCloud(terrain_pcl, robot_pos, sensor_range);
                FARUtil::RemoveNanInfPoints(terrain_pcl);
                FARUtil::ExtractFreeAndObsCloud(terrain_pcl, free_cloud, obs_cloud);
                have_new_clouds = true;

                map_handler.UpdateObsCloudGrid(obs_cloud);
                map_handler.UpdateFreeCloudGrid(free_cloud);
                map_handler.UpdateTerrainHeightGrid(free_cloud, terrain_height_cloud);
            }
        }

        // ── Process registered_scan (for dynamic obstacle detection) ───────
        if (have_scan) {
            PointCloudPtr scan_pcl = lcm_pc2_to_pcl(cur_scan);
            if (!scan_pcl->empty()) {
                FARUtil::CropPCLCloud(scan_pcl, robot_pos, sensor_range);
                FARUtil::RemoveNanInfPoints(scan_pcl);

                PointCloudPtr scan_free_cloud(new pcl::PointCloud<PCLPoint>());
                scan_handler.SetCurrentScanCloud(scan_pcl, scan_free_cloud);
            }
        }

        // ── Handle new goal ────────────────────────────────────────────────
        if (have_new_goal) {
            Point3D goal_pos(
                static_cast<float>(cur_goal_msg.point.x),
                static_cast<float>(cur_goal_msg.point.y),
                static_cast<float>(cur_goal_msg.point.z));
            graph_planner.UpdateGoal(goal_pos);
            printf("[far_planner] New goal received: (%.2f, %.2f, %.2f)\n",
                   goal_pos.x, goal_pos.y, goal_pos.z);
        }

        // ── Main graph update loop (mirrors FARMaster::MainLoopCallBack) ───
        if (have_new_clouds && is_map_init) {
            // Get surrounding obstacle and free clouds from grid map
            PointCloudPtr surround_obs(new pcl::PointCloud<PCLPoint>());
            PointCloudPtr surround_free(new pcl::PointCloud<PCLPoint>());
            map_handler.GetSurroundObsCloud(surround_obs);
            map_handler.GetSurroundFreeCloud(surround_free);

            // Set surround obstacle cloud for scan handler
            scan_handler.SetSurroundObsCloud(surround_obs, !FARUtil::IsStaticEnv);

            // Extract new observation points for graph update
            PointCloudPtr new_obs_cloud(new pcl::PointCloud<PCLPoint>());
            if (!surround_obs->empty()) {
                FARUtil::ExtractNewObsPointCloud(surround_obs, obs_cloud, new_obs_cloud);
                FARUtil::UpdateKdTrees(new_obs_cloud);
            }

            // Update node heights based on terrain
            const NodePtrStack& nav_graph = dynamic_graph.GetNavGraph();
            map_handler.AdjustNodesHeight(nav_graph);

            // Build contour image and extract contours
            const NavNodePtr odom_node = dynamic_graph.GetOdomNode();
            if (odom_node != nullptr) {
                // Contour detection from obstacle cloud
                std::vector<PointStack> realworld_contour;
                contour_detector.BuildTerrainImgAndExtractContour(
                    odom_node, surround_obs, realworld_contour);

                // Update contour graph with detected contours
                contour_graph.UpdateContourGraph(odom_node, realworld_contour);

                // Match contour vertices with navigation graph nodes
                CTNodeStack new_convex_vertices;
                const NodePtrStack& near_nodes = dynamic_graph.GetExtendLocalNode();
                contour_graph.MatchContourWithNavGraph(nav_graph, near_nodes, new_convex_vertices);

                // Adjust heights of contour nodes
                map_handler.AdjustCTNodeHeight(ContourGraph::contour_graph_);

                // Extract navigation graph nodes from contour vertices
                dynamic_graph.ExtractGraphNodes(new_convex_vertices);

                // Update near nodes (local range classification)
                dynamic_graph.UpdateGlobalNearNodes();

                // Update navigation graph (edges, visibility, etc.)
                const bool is_freeze_vgraph = false;
                NodePtrStack clear_nodes;
                dynamic_graph.UpdateNavGraph(
                    dynamic_graph.GetNewNodes(), is_freeze_vgraph, clear_nodes);

                // Remove cleared nodes from map
                if (!clear_nodes.empty()) {
                    PointCloudPtr clear_cloud(new pcl::PointCloud<PCLPoint>());
                    for (const auto& node : clear_nodes) {
                        clear_cloud->push_back(FARUtil::Point3DToPCLPoint(node->position));
                    }
                    map_handler.RemoveObsCloudFromGrid(clear_cloud);
                }

                // Extract global contour lines for polygon checks
                contour_graph.ExtractGlobalContours();
            }
        }

        // ── Planning loop (mirrors FARMaster::PlanningCallBack) ────────────
        const NavNodePtr goal_ptr = graph_planner.GetGoalNodePtr();
        if (goal_ptr != nullptr && is_map_init) {
            const NavNodePtr odom_node = dynamic_graph.GetOdomNode();
            if (odom_node != nullptr) {
                // Update graph in planner
                graph_planner.UpdaetVGraph(dynamic_graph.GetNavGraph());

                // Update goal node connectivity
                graph_planner.UpdateGoalNavNodeConnects(goal_ptr);

                // Re-evaluate goal position with terrain
                graph_planner.ReEvaluateGoalPosition(goal_ptr, true);

                // Update free terrain grid for goal adjustment
                PointCloudPtr goal_obs(new pcl::PointCloud<PCLPoint>());
                PointCloudPtr goal_free(new pcl::PointCloud<PCLPoint>());
                map_handler.GetCloudOfPoint(goal_ptr->position, goal_obs,
                                            CloudType::OBS_CLOUD, true);
                map_handler.GetCloudOfPoint(goal_ptr->position, goal_free,
                                            CloudType::FREE_CLOUD, true);
                graph_planner.UpdateFreeTerrainGrid(goal_ptr->position, goal_obs, goal_free);

                // Compute graph traversability
                graph_planner.UpdateGraphTraverability(odom_node, goal_ptr);

                // Plan path to goal
                NodePtrStack global_path;
                NavNodePtr nav_waypoint = nullptr;
                Point3D goal_p;
                bool is_fails = false;
                bool is_succeed = false;
                bool is_free_nav = false;

                bool plan_success = graph_planner.PathToGoal(
                    goal_ptr, global_path, nav_waypoint, goal_p,
                    is_fails, is_succeed, is_free_nav);

                if (plan_success && nav_waypoint != nullptr) {
                    // Publish waypoint
                    if (!g_way_point_topic.empty()) {
                        auto wp_msg = point3d_to_lcm_point_stamped(
                            nav_waypoint->position, now_sec);
                        g_lcm->publish(g_way_point_topic, &wp_msg);
                    }

                    // Publish path
                    if (!g_goal_path_topic.empty() && !global_path.empty()) {
                        auto path_msg = path_to_lcm_path(global_path, now_sec);
                        g_lcm->publish(g_goal_path_topic, &path_msg);
                    }
                }

                if (is_succeed) {
                    printf("[far_planner] Goal reached!\n");
                } else if (is_fails) {
                    printf("[far_planner] Planning failed — no path to goal\n");
                }
            }
        }

        frame_count++;
        last_robot_pos = robot_pos;

        // ── Rate control ───────────────────────────────────────────────────
        auto elapsed = std::chrono::high_resolution_clock::now() - loop_start;
        auto elapsed_ms = std::chrono::duration<float, std::milli>(elapsed).count();
        if (elapsed_ms < loop_period_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(loop_period_ms - elapsed_ms)));
        }
    }

    printf("[far_planner] Shutting down\n");
    g_lcm = nullptr;
    return 0;
}
