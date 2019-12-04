#ifndef VOXBLOX_UTILS_PLANNING_UTILS_INL_H_
#define VOXBLOX_UTILS_PLANNING_UTILS_INL_H_

#include <algorithm>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/utils/camera_model.h"
#include <voxblox/integrator/integrator_utils.h>

#include <mav_msgs/eigen_mav_msgs.h>
#include <iostream>
#include "voxblox/core/common.h"
#include <kindr/minimal/quat-transformation.h>
#include <cmath>
//#include <voxblox_planning_common/gain_evaluator.h>

namespace voxblox {
namespace utils {

template <typename VoxelType>
void getSphereAroundPoint(const Layer<VoxelType>& layer, const Point& center,
                          FloatingPoint radius,
                          HierarchicalIndexMap* block_voxel_list) {
  CHECK_NOTNULL(block_voxel_list);
  float voxel_size = layer.voxel_size();
  float voxel_size_inv = 1.0 / layer.voxel_size();
  int voxels_per_side = layer.voxels_per_side();

  const GlobalIndex center_index =
      getGridIndexFromPoint<GlobalIndex>(center, voxel_size_inv);
  const FloatingPoint radius_in_voxels = radius / voxel_size;

  for (FloatingPoint x = -radius_in_voxels; x <= radius_in_voxels; x++) {
    for (FloatingPoint y = -radius_in_voxels; y <= radius_in_voxels; y++) {
      for (FloatingPoint z = -radius_in_voxels; z <= radius_in_voxels; z++) {
        Point point_voxel_space(x, y, z);

        // check if point is inside the spheres radius
        if (point_voxel_space.norm() <= radius_in_voxels) {
          GlobalIndex voxel_offset_index(std::floor(point_voxel_space.x()),
                                         std::floor(point_voxel_space.y()),
                                         std::floor(point_voxel_space.z()));
          // Get the block and voxel indices from this.
          BlockIndex block_index;
          VoxelIndex voxel_index;

          getBlockAndVoxelIndexFromGlobalVoxelIndex(
              voxel_offset_index + center_index, voxels_per_side, &block_index,
              &voxel_index);
          (*block_voxel_list)[block_index].push_back(voxel_index);
        }
      }
    }
  }
}

template <typename VoxelType>
void getAndAllocateSphereAroundPoint(const Point& center, FloatingPoint radius,
                                     Layer<VoxelType>* layer,
                                     HierarchicalIndexMap* block_voxel_list) {
  CHECK_NOTNULL(layer);
  CHECK_NOTNULL(block_voxel_list);
  getSphereAroundPoint(*layer, center, radius, block_voxel_list);
  for (auto it = block_voxel_list->begin(); it != block_voxel_list->end();
       ++it) {
    layer->allocateBlockPtrByIndex(it->first);
  }
}

template <typename VoxelType>
void getFOVAroundPoint(const Layer<VoxelType>& layer, const Point& center, mav_msgs::EigenTrajectoryPoint pose,
                          FloatingPoint radius,
                          HierarchicalIndexMap* block_voxel_list) {
  CHECK_NOTNULL(block_voxel_list);
  float voxel_size = layer.voxel_size();
  float voxel_size_inv = 1.0 / layer.voxel_size();
  int voxels_per_side = layer.voxels_per_side();

  const GlobalIndex center_index =
      getGridIndexFromPoint<GlobalIndex>(center, voxel_size_inv);
  const FloatingPoint radius_in_voxels = radius / voxel_size;

  //Get FOV of camera 
  // Initialize camera 
  const double pi = std::acos(-1);
  double horizontal_fov = (60 * pi)/180;
  double vertical_fov = (49.5 * pi)/180;
  double min_distance = 0.6;
  double max_distance = 8;
  const FloatingPoint max_distance_in_voxels = max_distance / voxel_size;
  voxblox::CameraModel cam_model_;

  Point origin(0.128,  0.000,  0.090);
  Transformation T_C_B_(origin,
                         Eigen::Quaternion<FloatingPoint>(0.0, 0.0, 0.0, 1.0));
  cam_model_.setExtrinsics(T_C_B_);

  
  cam_model_.setIntrinsicsFromFoV( horizontal_fov, vertical_fov, min_distance,
                                   max_distance);

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));


  // Get the boundaries of the current view.
  Eigen::Vector3f aabb_min, aabb_max;
  cam_model_.getAabb(&aabb_min, &aabb_max);
  std::cout << "[VOXLOX] Exploration gain, cam model boundaries \n"<< aabb_min.x() << " " << aabb_min.y() << " "  <<aabb_min.z() ;
  std::cout << "[VOXLOX] Exploration gain, cam model boundaries \n"<< aabb_max.x() << " " << aabb_max.y() << " " <<aabb_max.z() ;
  std::cout << "[VOXLOX] Pose \n"<< pose.position_W.x() << " " << pose.position_W.y() << " " << pose.position_W.z() ;
  std::cout << "[VOXLOX] Orientation \n"<< pose.orientation_W_B.w() << " " <<  pose.orientation_W_B.x() << " " << pose.orientation_W_B.y() << " " << pose.orientation_W_B.z() ;
  Eigen::Vector3f pos = aabb_min/voxel_size;
  aabb_min = aabb_min/voxel_size;
  aabb_max = aabb_max/voxel_size;
  
  double tan_half_horizontal_fov = tanf(horizontal_fov / 2.0);
  double tan_half_vertical_fov = tanf(vertical_fov / 2.0);
 
  double hoz_fov_voxels = tan_half_horizontal_fov / voxel_size;
  double vert_fov_voxels = tan_half_vertical_fov / voxel_size;

   std::cout << "[VOXLOX] horz, vert, vox h, vox v, distance \n"<< tan_half_horizontal_fov << " " << tan_half_vertical_fov << " "  << hoz_fov_voxels << " "  << vert_fov_voxels << " " << max_distance_in_voxels;

  // for (FloatingPoint x = -10; x < 10; x ++) {
  //   for ( FloatingPoint y = -30; y < 30; y ++) {
  //     for (FloatingPoint z = -70; z < 70; z ++) {
 for (FloatingPoint x = 0; x <= max_distance_in_voxels; x++) {
      float ybound = (x*voxel_size * tan_half_horizontal_fov)/voxel_size;
      float zbound = (x*voxel_size * tan_half_vertical_fov)/voxel_size;
    for (FloatingPoint y = -ybound ; y <= ybound; y++) {
      for (FloatingPoint z = -zbound; z <= zbound; z++) {

          Point point_voxel_space(x, y, z);
          Eigen::Vector3d position{x,y,z};
          Eigen::Vector3d position_rot = pose.orientation_W_B.toRotationMatrix() * position;
          Point point_voxel_space_rot(position_rot[0], position_rot[1],position_rot[2]);
                  // check if point is inside the spheres radius
        if (point_voxel_space_rot.norm() <= max_distance_in_voxels) {
          GlobalIndex voxel_offset_index(std::floor(point_voxel_space_rot.x()),
                                         std::floor(point_voxel_space_rot.y()),
                                         std::floor(point_voxel_space_rot.z()));
          // Get the block and voxel indices from this.
          BlockIndex block_index;
          VoxelIndex voxel_index;

          getBlockAndVoxelIndexFromGlobalVoxelIndex(
              voxel_offset_index + center_index, voxels_per_side, &block_index,
              &voxel_index);
          (*block_voxel_list)[block_index].push_back(voxel_index);
        }

        }
      }
  }
}

template <typename VoxelType>
void getAndAllocateFOVAroundPoint(const Point& center, mav_msgs::EigenTrajectoryPoint pose, FloatingPoint radius,
                                     Layer<VoxelType>* layer,
                                     HierarchicalIndexMap* block_voxel_list) {
  CHECK_NOTNULL(layer);
  CHECK_NOTNULL(block_voxel_list);
  getFOVAroundPoint(*layer, center, pose, radius, block_voxel_list);
  for (auto it = block_voxel_list->begin(); it != block_voxel_list->end();
       ++it) {
    layer->allocateBlockPtrByIndex(it->first);
  }
}





// This function sets all voxels within a Euclidean distance of the center
// to a value equal to the distance of the point from the center, essentially
// making it a filled sphere with that center and radius.
// Marks the new points as halllucinated and fixed.
template <typename VoxelType>
void fillSphereAroundPoint(const Point& center, const FloatingPoint radius,
                           const FloatingPoint max_distance_m,
                           Layer<VoxelType>* layer) {
  CHECK_NOTNULL(layer);
  HierarchicalIndexMap block_voxel_list;
  getAndAllocateSphereAroundPoint(center, radius, layer, &block_voxel_list);

  for (auto it = block_voxel_list.begin(); it != block_voxel_list.end(); ++it) {
    typename Block<VoxelType>::Ptr block_ptr =
        layer->getBlockPtrByIndex(it->first);
    for (const VoxelIndex& voxel_index : it->second) {
      Point point = block_ptr->computeCoordinatesFromVoxelIndex(voxel_index);
      Point voxel_center_vec = point - center;

      VoxelType& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
      // The distance of the voxel should map to actual meaningful
      // Euclidean distance; in this case, the sphere center has the max
      // distance, and the edges should have the lowest distance.
      // Also compress this to the max distance given.
      const FloatingPoint new_distance =
          std::max(voxel_center_vec.norm() - radius, -max_distance_m);

      if (!voxel.observed || new_distance < voxel.distance) {
        voxel.distance = new_distance;
        voxel.observed = true;
        voxel.hallucinated = true;
        voxel.fixed = true;
        block_ptr->updated().set();
        block_ptr->has_data() = true;
      }
    }
  }
}

// Similar to above, clears the area around the specified point, marking it as
// hallucinated and fixed.
template <typename VoxelType>
void clearSphereAroundPoint(const Point& center, const FloatingPoint radius,
                            const FloatingPoint max_distance_m,
                            Layer<VoxelType>* layer) {
  CHECK_NOTNULL(layer);
  HierarchicalIndexMap block_voxel_list;
  getAndAllocateSphereAroundPoint(center, radius, layer, &block_voxel_list);

  for (auto it = block_voxel_list.begin(); it != block_voxel_list.end(); ++it) {
    typename Block<VoxelType>::Ptr block_ptr =
        layer->getBlockPtrByIndex(it->first);
    for (const VoxelIndex& voxel_index : it->second) {
      Point point = block_ptr->computeCoordinatesFromVoxelIndex(voxel_index);
      Point voxel_center_vec = point - center;

      VoxelType& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
      // How far is voxel from edge of free sphere. The values should be
      // biggest in the center, smallest outside.
      const FloatingPoint new_distance =
          std::min(radius - voxel_center_vec.norm(), max_distance_m);

      if (!voxel.observed || new_distance > voxel.distance) {
        voxel.distance = new_distance;
        voxel.observed = true;
        voxel.hallucinated = true;
        voxel.fixed = true;
        block_ptr->updated().set();
        block_ptr->has_data() = true;
      }
    }
  }
}

// Utility function to get map bounds from an arbitrary layer.
template <typename VoxelType>
void computeMapBoundsFromLayer(const voxblox::Layer<VoxelType>& layer,
                               Eigen::Vector3d* lower_bound,
                               Eigen::Vector3d* upper_bound) {
  FloatingPoint block_size = layer.block_size();

  BlockIndexList all_blocks;
  layer.getAllAllocatedBlocks(&all_blocks);

  BlockIndex lower_bound_index;
  BlockIndex upper_bound_index;

  bool first_block = true;

  for (const voxblox::BlockIndex& block_index : all_blocks) {
    if (first_block) {
      lower_bound_index = block_index;
      upper_bound_index = block_index;
      first_block = false;
      continue;
    }

    lower_bound_index = lower_bound_index.array().min(block_index.array());
    upper_bound_index = upper_bound_index.array().max(block_index.array());
  }

  *lower_bound =
      getOriginPointFromGridIndex(lower_bound_index, block_size).cast<double>();
  *upper_bound =
      (getOriginPointFromGridIndex(upper_bound_index, block_size).array() +
       block_size)
          .cast<double>();
}

}  // namespace utils
}  // namespace voxblox

#endif  // VOXBLOX_UTILS_PLANNING_UTILS_INL_H_
