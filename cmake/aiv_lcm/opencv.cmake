if (USE_CC_IECU2.0)
    message(STATUS "================  -] opencv building IECU2.0 [-  ================")
    find_package(OpenCV COMPONENTS
            opencv_ccalib
            opencv_plot
            opencv_rgbd
            opencv_stereo
            opencv_ximgproc
            opencv_calib3d
            opencv_core
            opencv_features2d
            opencv_flann
            opencv_highgui
            opencv_imgcodecs
            opencv_imgproc
            opencv_shape
            opencv_stitching
            opencv_superres
            opencv_aruco)
    include_directories(${OpenCV_INCLUDE_DIRS})
#     message(opencv: ${OpenCV_INCLUDE_DIRS})
    #list(APPEND ALL_LIBRARIES ${OpenCV_LIBS})
    list(APPEND ALL_LIBRARIES
            opencv_ccalib
            opencv_plot
            opencv_rgbd
            opencv_stereo
            opencv_ximgproc
            opencv_calib3d
            opencv_core
            opencv_features2d
            opencv_flann
            opencv_highgui
            opencv_imgcodecs
            opencv_imgproc
            opencv_shape
            opencv_stitching
            opencv_superres
            opencv_aruco)
    message("opencv: ${OpenCV_LIBS}")
elseif (USE_CC_IECU3.1)
    message(STATUS "================  -] opencv building IECU3.1 [-  ================")
    find_package(OpenCV COMPONENTS
            opencv_ccalib
            opencv_plot
            opencv_rgbd
            opencv_stereo
            opencv_ximgproc
            opencv_calib3d
            opencv_core
            opencv_features2d
            opencv_flann
            opencv_highgui
            opencv_imgcodecs
            opencv_imgproc
            opencv_shape
            opencv_stitching
            opencv_superres
            opencv_aruco)
    include_directories(${OpenCV_INCLUDE_DIRS})
#     message(opencv: ${OpenCV_INCLUDE_DIRS})
    #list(APPEND ALL_LIBRARIES ${OpenCV_LIBS})
    list(APPEND ALL_LIBRARIES
            opencv_ccalib
            opencv_plot
            opencv_rgbd
            opencv_stereo
            opencv_ximgproc
            opencv_calib3d
            opencv_core
            opencv_features2d
            opencv_flann
            opencv_highgui
            opencv_imgcodecs
            opencv_imgproc
            opencv_shape
            opencv_stitching
            opencv_superres
            opencv_aruco)
    message("opencv: ${OpenCV_LIBS}")
else ()
    message(STATUS "================  -] opencv building x86-64 [-  ================")
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${OpenCV_LIBS})
endif ()
