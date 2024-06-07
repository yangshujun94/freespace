message(STATUS "================  -] building x86-64 [-  ================")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Modules")

find_package(PCL QUIET)
include_directories(${PCL_INCLUDE_DIRS})
#list(APPEND ALL_LIBRARIES ${GLOG_LIBRARIES})
list(APPEND ALL_LIBRARIES 
        pcl_kdtree	
        pcl_octree
        pcl_features
	pcl_ml
	pcl_tracking
	pcl_2d
	pcl_common
	pcl_io
	pcl_stereo
	pcl_in_hand_scanner
	pcl_keypoints
	pcl_people
	pcl_apps
	pcl_outofcore
	pcl_filters
	pcl_sample_consensus
	pcl_geometry
	pcl_registration
	pcl_point_cloud_editor
	pcl_segmentation
	pcl_visualization
	pcl_search
	pcl_surface
	pcl_recognition
	)

