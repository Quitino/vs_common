# vs_common
The vs_common package is a versatile C++ library for vision/robotics system, which provides a lot of useful functions including data structure, multithread processing, image processing, random generation, numeric utils, vector utils, string utils, system logger, opencv tools and so on.

We try to build this library as a basic dependency of other vision/robotics packages. So it only contain algorithm independent tools and basic data structures, so as to put more focus on algorithm rather than miscellaneous functions

## Prerequisites
- C++ 11
- Eigen3(http://eigen.tuxfamily.org): used for data convertion between Eigen and opencv.
- OpenCV(https://opencv.org): there are some image tools based on opencv.

## What can we do?
The charactor after header indicate the dependency of E:Eigen, C:OpenCV B:Boost.
- vs_cam_capture.h[C]: camera capture using opencv, real-time capture image with rolling buffer, auto save video.
- vs_cam_odom_calib.h[E]: hand-eye calibration with odometer poses and camera poses.
- vs_color_filter.h[C]: color mask for image. support: yellow, red, white.
- vs_cv_convert.h[E,C]: data convertion between Eigen and opencv.
- vs_data_buffer.h[B]: data buffer with mutual exclusion, read/write lock in multi threads.
- vs_data_struct.h: 2D array, fixed length queue, median filter.
- vs_debug.h: macros for debugging.
- vs_debug_draw.h[C]: feature tracking drawing based on opencv.
- vs_exposure.h[C]: exposure control for camera.
- vs_geometry2d.h: geometry utils of points and lines in 2D plane.
- vs_gridmap2d.h: 2D gridmap used for robot navigation and image drawing.
- vs_improc.h[C]: bgr2hue, grab red region, sobel filter, region filter.
- vs_kdtree.h[B]: implementation of KD tree for 2D/3D nearest search.
- vs_lane_detect[C]: detect lane with special color.
- vs_line_match[C]: match line pairs, iteratively closest line(ICL).
- vs_mapping.h: 2D/3D grid mapping with raycasting.
- vs_numeric.h: numeric funtions such as min, max, clip, deg2rad, linear interpolation.
- vs_os.h: cpu handle, file utils, path utils which is similar to os.path.
- vs_pca.h[E]: principal component analysis(PCA) using SVD, line fitting.
- vs_perf.h: performance evaluation for algorithms time cost.
- vs_pf.h: general particle filter with importance resampling.
- vs_pnp.h[C]: closed form for translation-only pnp, pnl, as well as 4-Dof pnp, pnl.
- vs_random.h: random generator and sampling. Similar to numpy.random.
- vs_rater.h: control the frequency of loop. Similar to ros::Rater.
- vs_raycast.h: ray casting for 2D/3D line voxel tracersal, using Bresenham's algorithm
- vs_rot.h: convertion between quaternion, eularangle and rotation matrix
- vs_shape_detect.h[C]: detection for special shape such as: circle, triangle, arrow, con-centric shape.
- vs_singleton.h: singleton.
- vs_stdout.h: cout with color.
- vs_stereo_calib[C]: rectify stereo image and calc recity camera intrinsics.
- vs_strutils.h: string functions such as find, match, cut, str2vec, str2num, num2str...
- vs_syslog.h: system logger that create log dir in rules and log system info.
- vs_tictoc.h: timer, tictoc to evaluate the time cost.
- vs_time_buffer.h: buffer to store timestamped data, find the nereast data for specific timestamp.
- vs_vecutils.h: vector utils, such as find the min, max, sum, mean, median, Kth item, subset of a vector. 
- vs_video_saver[C].h: save video in an asynchronous manner.
- vs_viz3d.h[C]: 3D visulization based on cv::Viz in an asynchronous manner.
- vs_yaml_parser[C]: param reader and writer of file in Yaml format, based on opencv.


## How to use?
```
mkdir build
cd build
cmake ..
make -j
```

## License
The source code is released under GPLv3 license.

Note: This library is not full tested. Please send any feedback and bugreports to maoshuyuan123@gmail.com
