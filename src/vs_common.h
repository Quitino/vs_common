#ifndef __VS_COMMON_H__
#define __VS_COMMON_H__

#define VS_VERSION_MAJOR    0
#define VS_VERSION_MINOR    0
#define VS_VERSION_REVISION 1

#include "vs_cam_capture.h"
#include "vs_cam_odom_calib.h"
#include "vs_color_filter.h"
#include "vs_cv_convert.h"
#include "vs_data_buffer.h"
#include "vs_data_saver.h"
#include "vs_data_struct.h"
#include "vs_debug.h"
#include "vs_debug_draw.h"
#include "vs_exposure.h"
#include "vs_gridmap2d.h"
#include "vs_improc.h"
#include "vs_kdtree.h"
#include "vs_lane_detect.h"
#include "vs_line_match.h"
#include "vs_mapping.h"
#include "vs_numeric.h"
#include "vs_os.h"
#include "vs_pca.h"
#include "vs_perf.h"
#include "vs_pf.h"
#include "vs_pnp.h"
#include "vs_random.h"
#include "vs_rater.h"
#include "vs_raycast.h"
#include "vs_rot.h"
#include "vs_shape_detect.h"
#include "vs_singleton.h"
#include "vs_stdout.h"
#include "vs_stereo_calib.h"
#include "vs_strutils.h"
#include "vs_syslog.h"
#include "vs_tictoc.h"
#include "vs_time_buffer.h"
#include "vs_vecutils.h"
#include "vs_video_saver.h"
#include "vs_viz3d.h"
#include "vs_yaml_parser.h"

#endif//__VS_COMMON_H__