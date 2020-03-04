#include <voxblox_ros/conversions.h>

#include "voxblox_ros/esdf_server.h"
#include "voxblox_ros/ros_params.h"

#include <tf/LinearMath/Transform.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace voxblox {

EsdfServer::EsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : EsdfServer(nh, nh_private, getEsdfMapConfigFromRosParam(nh_private),
                 getEsdfIntegratorConfigFromRosParam(nh_private),
                 getTsdfMapConfigFromRosParam(nh_private),
                 getTsdfIntegratorConfigFromRosParam(nh_private),
                 getMeshIntegratorConfigFromRosParam(nh_private)) {}

EsdfServer::EsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private,
                       const EsdfMap::Config& esdf_config,
                       const EsdfIntegrator::Config& esdf_integrator_config,
                       const TsdfMap::Config& tsdf_config,
                       const TsdfIntegratorBase::Config& tsdf_integrator_config,
                       const MeshIntegratorConfig& mesh_config)
    : TsdfServer(nh, nh_private, tsdf_config, tsdf_integrator_config,
                 mesh_config),
      clear_sphere_for_planning_(false),
      publish_esdf_map_(false),
      publish_traversable_(false),
      traversability_radius_(1.0),
      incremental_update_(true),
      num_subscribers_esdf_map_(0),
      publish_global_info_(false) {
  // Set up map and integrator.
  esdf_map_.reset(new EsdfMap(esdf_config));
  esdf_global_map_.reset(new EsdfMap(esdf_config));
  esdf_integrator_.reset(new EsdfIntegrator(
      esdf_integrator_config, tsdf_map_->getTsdfLayerPtr(),
      esdf_map_->getEsdfLayerPtr(), esdf_global_map_->getEsdfGlobalLayerPtr()));

  setupRos();
}

void EsdfServer::setupRos() {
  // Set up publisher.
  esdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_pointcloud",
                                                              1, true);
  esdf_pointcloud_local_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "esdf_pointcloud_local", 1, true);
  esdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "esdf_slice", 1, true);
  traversable_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "traversable", 1, true);

  esdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("esdf_map_out", 1, false);

  // Set up subscriber.
  esdf_map_sub_ = nh_private_.subscribe("esdf_map_in", 1,
                                        &EsdfServer::esdfMapCallback, this);

  // Whether to clear each new pose as it comes in, and then set a sphere
  // around it to occupied.
  nh_private_.param("clear_sphere_for_planning", clear_sphere_for_planning_,
                    clear_sphere_for_planning_);
  nh_private_.param("publish_esdf_map", publish_esdf_map_, publish_esdf_map_);

  // Special output for traversable voxels. Publishes all voxels with distance
  // at least traversibility radius.
  nh_private_.param("publish_traversable", publish_traversable_,
                    publish_traversable_);
  nh_private_.param("traversability_radius", traversability_radius_,
                    traversability_radius_);

  double update_esdf_every_n_sec = 1.0;
  nh_private_.param("update_esdf_every_n_sec", update_esdf_every_n_sec,
                    update_esdf_every_n_sec);

  if (update_esdf_every_n_sec > 0.0) {
    update_esdf_timer_ =
        nh_private_.createTimer(ros::Duration(update_esdf_every_n_sec),
                                &EsdfServer::updateEsdfEvent, this);
  }

  nh_private_.param("publish_global_info", publish_global_info_,
                    publish_global_info_);

  timing::Timer load_saved_map("load_saved_map");
  tsdf_map_ = TsdfServer::getTsdfMapPtr();
  CHECK(tsdf_map_);

  nh_private_.param("voxblox_path", input_filepath, input_filepath);
  nh_private_.param("load_saved_map", load_saved_map_, load_saved_map_);
  if (load_saved_map_) {
    if (!input_filepath.empty()) {
      // Verify that the map has an ESDF layer, otherwise generate it.
      if (!EsdfServer::loadMap(input_filepath)) {
        ROS_ERROR("Couldn't load ESDF map!");
        std::cout << "Input filepath> " << input_filepath;
        // Check if the TSDF layer is non-empty...
        if (tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) {
          ROS_INFO("Generating ESDF layer from TSDF.");
          // If so, generate the ESDF layer!
          const bool full_euclidean_distance = true;
          updateEsdfBatch(full_euclidean_distance);
        } else {
          ROS_ERROR("TSDF map also empty! Check voxel size!");
        }
      }
    }
  }
  load_saved_map.Stop();
}

void EsdfServer::publishAllUpdatedEsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  pcl::PointCloud<pcl::PointXYZI> pointcloud_local;
  if (publish_global_info_) {
    createDistancePointcloudFromEsdfLayer(
        esdf_global_map_->getEsdfGlobalLayer(), &pointcloud);
  } else {
    createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(),
                                          &pointcloud);
  }
  createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(),
                                        &pointcloud_local);
  pointcloud.header.frame_id = world_frame_;
  pointcloud_local.header.frame_id = world_frame_;
  esdf_pointcloud_pub_.publish(pointcloud);
  esdf_pointcloud_local_pub_.publish(pointcloud_local);
}

void EsdfServer::publishSlices() {
  TsdfServer::publishSlices();

  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  constexpr int kZAxisIndex = 2;
  if (publish_global_info_) {
    createDistancePointcloudFromEsdfLayerSlice(
        esdf_global_map_->getEsdfGlobalLayer(), kZAxisIndex, slice_level_,
        &pointcloud);
  } else {
    createDistancePointcloudFromEsdfLayerSlice(
        esdf_map_->getEsdfLayer(), kZAxisIndex, slice_level_, &pointcloud);
  }

  pointcloud.header.frame_id = world_frame_;
  esdf_slice_pub_.publish(pointcloud);
}

bool EsdfServer::generateEsdfCallback(
    std_srvs::Empty::Request& /*request*/,      // NOLINT
    std_srvs::Empty::Response& /*response*/) {  // NOLINT
  const bool clear_esdf = true;
  if (clear_esdf) {
    esdf_integrator_->updateFromTsdfLayerBatch();
  } else {
    const bool clear_updated_flag = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
  }
  publishAllUpdatedEsdfVoxels();
  publishSlices();
  return true;
}

void EsdfServer::updateEsdfEvent(const ros::TimerEvent& /*event*/) {
  updateEsdf();
  updateGlobalEsdf();
  esdf_integrator_->clearGlobalEsdfMap();
  esdf_integrator_->clearEsdfMap();
}

void EsdfServer::publishPointclouds() {
  timing::Timer publish_pointclouds_timer("pointclouds/publish");
  publishAllUpdatedEsdfVoxels();
  if (publish_slices_) {
    publishSlices();
  }
  if (publish_traversable_) {
    publishTraversable();
  }

  TsdfServer::publishPointclouds();
  publish_pointclouds_timer.Stop();
}

void EsdfServer::publishTraversable() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  createFreePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(),
                                    traversability_radius_, &pointcloud);
  pointcloud.header.frame_id = world_frame_;
  traversable_pub_.publish(pointcloud);
}

void EsdfServer::publishMap(bool reset_remote_map) {
  if (!publish_esdf_map_) {
    return;
  }

  int subscribers = this->esdf_map_pub_.getNumSubscribers();
  if (subscribers > 0) {
    if (num_subscribers_esdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_esdf");
    voxblox_msgs::Layer layer_msg;
    if (publish_global_info_) {
      serializeLayerAsMsg<EsdfVoxel>(
          this->esdf_global_map_->getEsdfGlobalLayer(), only_updated,
          &layer_msg);
    } else {
      // serializeLayerAsMsg<EsdfVoxel>(this->esdf_map_->getEsdfLayer(),
      //                                only_updated, &layer_msg);
      serializeLayerAsMsg<EsdfVoxel>(
          this->esdf_global_map_->getEsdfGlobalLayer(), only_updated,
          &layer_msg);
    }

    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    this->esdf_map_pub_.publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_esdf_map_ = subscribers;
  TsdfServer::publishMap();
}

bool EsdfServer::saveMap(const std::string& file_path) {
  // Output TSDF map first, then ESDF.
  const bool success = TsdfServer::saveMap(file_path);

  constexpr bool kClearFile = false;
  return success &&
         io::SaveLayer(esdf_global_map_->getEsdfLayer(), file_path, kClearFile);
}

bool EsdfServer::loadMap(const std::string& file_path) {
  // Load in the same order: TSDF first, then ESDF.
  bool success_tsdf = TsdfServer::loadMap(file_path);

  constexpr bool kMultipleLayerSupport = true;
  bool success_esdf = io::LoadBlocksFromFile(
      file_path, Layer<EsdfVoxel>::BlockMergingStrategy::kReplace,
      kMultipleLayerSupport, esdf_global_map_->getEsdfGlobalLayerPtr());
  esdf_integrator_->setGlobalLayer();

  return success_tsdf && success_esdf;
}

void EsdfServer::updateEsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
  }
}

void EsdfServer::updateGlobalEsdf() { esdf_integrator_->updateFromEsdfLayer(); }

void EsdfServer::updateEsdfBatch(bool full_euclidean) {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    esdf_integrator_->setFullEuclidean(full_euclidean);
    esdf_integrator_->updateFromTsdfLayerBatch();
  }
}

float EsdfServer::getEsdfMaxDistance() const {
  return esdf_integrator_->getEsdfMaxDistance();
}

void EsdfServer::setEsdfMaxDistance(float max_distance) {
  esdf_integrator_->setEsdfMaxDistance(max_distance);
}

float EsdfServer::getTraversabilityRadius() const {
  return traversability_radius_;
}

void EsdfServer::setTraversabilityRadius(float traversability_radius) {
  traversability_radius_ = traversability_radius;
}

void EsdfServer::newPoseCallback(const Transformation& T_G_C) {
  if (clear_sphere_for_planning_) {
    Eigen::Quaterniond quat{T_G_C.getRotation().w(), T_G_C.getRotation().x(),
                            T_G_C.getRotation().y(), T_G_C.getRotation().z()};
    // Transform it to camera_link ref. frame. Now, it is in
    // /camera_depth_optical_frame
    Eigen::Quaterniond quat_rot1{0.707, 0.0, 0.0, 0.707};
    Eigen::Quaterniond quat_mult = quat_rot1 * quat;
    esdf_integrator_->addNewRobotPosition(T_G_C.getPosition(), quat_mult);
    esdf_integrator_->setLimitArea(T_G_C.getPosition());
  }

  timing::Timer block_remove_timer("remove_distant_blocks");
  esdf_map_->getEsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  block_remove_timer.Stop();
}

void EsdfServer::esdfMapCallback(const voxblox_msgs::Layer& layer_msg) {
  timing::Timer receive_map_timer("map/receive_esdf");
  bool success;
  if (publish_global_info_) {
    success = deserializeMsgToLayer<EsdfVoxel>(layer_msg,
                                               esdf_map_->getEsdfLayerPtr());
  } else {
    success = deserializeMsgToLayer<EsdfVoxel>(layer_msg,
                                               esdf_map_->getEsdfLayerPtr());
  }

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid ESDF map message!");
  } else {
    ROS_INFO_ONCE("Got an ESDF map from ROS topic!");
    // publishPointclouds();
  }
}

void EsdfServer::clear() {
  if (publish_global_info_) {
    esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
  } else {
    esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
  }

  esdf_integrator_->clear();
  CHECK_EQ(
      esdf_global_map_->getEsdfGlobalLayerPtr()->getNumberOfAllocatedBlocks(),
      0u);

  TsdfServer::clear();

  // Publish a message to reset the map to all subscribers.
  constexpr bool kResetRemoteMap = true;
  publishMap(kResetRemoteMap);
}

}  // namespace voxblox
