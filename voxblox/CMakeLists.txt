cmake_minimum_required(VERSION 2.8.12)
project(voxblox)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11 -Wall -Wextra)

############
# PROTOBUF #
############
# General idea: first check if we have protobuf catkin, then use that.
# Otherwise use system protobuf.
set(PROTO_DEFNS proto/voxblox/Block.proto
                proto/voxblox/Layer.proto)
set(PROTO_LIBRARIES "")
find_package(protobuf_catkin QUIET)
if (protobuf_catkin_FOUND)
  message(STATUS "Using protobuf_catkin")
  PROTOBUF_CATKIN_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
  set(PROTO_LIBRARIES ${protobuf_catkin_LIBRARIES})
else()
  message(STATUS "Using system protobuf")
  find_package(Protobuf REQUIRED)
  PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
  set(PROTO_LIBRARIES ${PROTOBUF_LIBRARIES})
endif()

####################
# SET SOURCE FILES #
####################

set("${PROJECT_NAME}_SRCS"
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
)

#############
# LIBRARIES #
#############
# NOTE(mereweth@jpl.nasa.gov) - Be careful when compiling Proto cc files. It is
# best to do so only once (make a shared library for each set of Proto files).
# Otherwise, at some point, you will get errors from double-adding protobuf
# formats, of the following form:
# [libprotobuf ERROR google/protobuf/descriptor_database.cc:57]
# File already exists in database: Block.proto

# Avoid having multiple compiled copies of the same .pb.cc
# file sharing a single copy of libprotobuf.so
cs_add_library(${PROJECT_NAME}_proto
  ${PROTO_SRCS}
)
# Specify proto headers as public headers so we can install them later.
set_target_properties(${PROJECT_NAME}_proto PROPERTIES PUBLIC_HEADER "${PROTO_HDRS}")
# We need the binary folder in the include directories so we can find the proto
# headers within this library and all dependent packages.
target_include_directories(${PROJECT_NAME}_proto
                           PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(${PROJECT_NAME}_proto ${PROTO_LIBRARIES})

cs_add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto)

############
# BINARIES #
############



#########
# TESTS #
#########

##########
# EXPORT #
##########

# The proto headers are not installed automatically with cs_install,
# we need to specify this here.
install(TARGETS ${PROJECT_NAME}_proto
        LIBRARY DESTINATION "lib"
        PUBLIC_HEADER DESTINATION "include"
)
cs_install()
cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
          CFG_EXTRAS voxblox-extras.cmake)
