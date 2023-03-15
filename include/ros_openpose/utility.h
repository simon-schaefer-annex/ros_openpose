#ifndef ROS_OPENPOSE_UTILITY_H
#define ROS_OPENPOSE_UTILITY_H

#include <ros/ros.h>
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>


#define OPENPOSE1POINT6_OR_HIGHER OpenPose_VERSION_MAJOR >= 1 && OpenPose_VERSION_MINOR >= 6
#define OPENPOSE1POINT7POINT1_OR_HIGHER OpenPose_VERSION_MAJOR >= 1 && OpenPose_VERSION_MINOR >= 7 && OpenPose_VERSION_PATCH >=1


void configureOpenPose(op::Wrapper& opWrapper) {
  try
  {
#if OPENPOSE1POINT6_OR_HIGHER
        op::checkBool(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
#else
        op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
#endif
                  "Wrong logging_level value.",
                  __LINE__,
                  __FUNCTION__,
                  __FILE__);

        op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
        op::Profiler::setDefaultX(FLAGS_profile_speed);

// Applying user defined configuration - GFlags to program variables
// outputSize
#if OPENPOSE1POINT6_OR_HIGHER
        const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
#else
        const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
#endif

// netInputSize
#if OPENPOSE1POINT6_OR_HIGHER
        const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
#else
        const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
#endif

// faceNetInputSize
#if OPENPOSE1POINT6_OR_HIGHER
        const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
#else
        const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
#endif

// handNetInputSize
#if OPENPOSE1POINT6_OR_HIGHER
        const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
#else
        const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
#endif

        // poseMode
        const auto poseMode = op::flagsToPoseMode(FLAGS_body);

// poseModel
#if OPENPOSE1POINT6_OR_HIGHER
        const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
#else
        const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
#endif

        // JSON saving
        if (!FLAGS_write_keypoint.empty())
            ROS_INFO("Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json` instead.");

        // keypointScaleMode
        const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);

        // heatmaps to add
        const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts,
                                                      FLAGS_heatmaps_add_bkg,
                                                      FLAGS_heatmaps_add_PAFs);

        const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);

        // >1 camera view?
        // const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
        const auto multipleView = false;

        // Face and hand detectors
        const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
        const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);

        // Enabling Google Logging
        const bool enableGoogleLogging = true;

        // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
        const op::WrapperStructPose wrapperStructPose{poseMode,
                                                      netInputSize,
#if OPENPOSE1POINT7POINT1_OR_HIGHER
                FLAGS_net_resolution_dynamic,
                                                  outputSize,
#else
                                                      outputSize,
#endif
                                                      keypointScaleMode,
                                                      FLAGS_num_gpu,
                                                      FLAGS_num_gpu_start,
                                                      FLAGS_scale_number,
                                                      (float)FLAGS_scale_gap,
                                                      op::flagsToRenderMode(FLAGS_render_pose,
                                                                            multipleView),
                                                      poseModel,
                                                      !FLAGS_disable_blending,
                                                      (float)FLAGS_alpha_pose,
                                                      (float)FLAGS_alpha_heatmap,
                                                      FLAGS_part_to_show,
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_model_folder),
#else
                                                      FLAGS_model_folder,
#endif
                                                      heatMapTypes,
                                                      heatMapScaleMode,
                                                      FLAGS_part_candidates,
                                                      (float)FLAGS_render_threshold,
                                                      FLAGS_number_people_max,
                                                      FLAGS_maximize_positives,
                                                      FLAGS_fps_max,
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_prototxt_path),
                                                  op::String(FLAGS_caffemodel_path),
#else
                                                      FLAGS_prototxt_path,
                                                      FLAGS_caffemodel_path,
#endif
                                                      (float)FLAGS_upsampling_ratio,
                                                      enableGoogleLogging};
        opWrapper.configure(wrapperStructPose);

        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{FLAGS_face,
                                                      faceDetector,
                                                      faceNetInputSize,
                                                      op::flagsToRenderMode(FLAGS_face_render,
                                                                            multipleView,
                                                                            FLAGS_render_pose),
                                                      (float)FLAGS_face_alpha_pose,
                                                      (float)FLAGS_face_alpha_heatmap,
                                                      (float)FLAGS_face_render_threshold};
        opWrapper.configure(wrapperStructFace);

        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{FLAGS_hand,
                                                      handDetector,
                                                      handNetInputSize,
                                                      FLAGS_hand_scale_number,
                                                      (float)FLAGS_hand_scale_range,
                                                      op::flagsToRenderMode(FLAGS_hand_render,
                                                                            multipleView,
                                                                            FLAGS_render_pose),
                                                      (float)FLAGS_hand_alpha_pose,
                                                      (float)FLAGS_hand_alpha_heatmap,
                                                      (float)FLAGS_hand_render_threshold};
        opWrapper.configure(wrapperStructHand);

        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
        const op::WrapperStructExtra wrapperStructExtra{FLAGS_3d,
                                                        FLAGS_3d_min_views,
                                                        FLAGS_identification,
                                                        FLAGS_tracking,
                                                        FLAGS_ik_threads};
        opWrapper.configure(wrapperStructExtra);

        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{FLAGS_cli_verbose,
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_write_keypoint),
#else
                                                          FLAGS_write_keypoint,
#endif
                                                          op::stringToDataFormat(FLAGS_write_keypoint_format),
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_write_json),
                                                      op::String(FLAGS_write_coco_json),
#else
                                                          FLAGS_write_json,
                                                          FLAGS_write_coco_json,
#endif
                                                          FLAGS_write_coco_json_variants,
                                                          FLAGS_write_coco_json_variant,
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_write_images),
                                                      op::String(FLAGS_write_images_format),
                                                      op::String(FLAGS_write_video),
#else
                                                          FLAGS_write_images,
                                                          FLAGS_write_images_format,
                                                          FLAGS_write_video,
#endif
                                                          FLAGS_write_video_fps,
                                                          FLAGS_write_video_with_audio,
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_write_heatmaps),
                                                      op::String(FLAGS_write_heatmaps_format),
                                                      op::String(FLAGS_write_video_3d),
                                                      op::String(FLAGS_write_video_adam),
                                                      op::String(FLAGS_write_bvh),
                                                      op::String(FLAGS_udp_host),
                                                      op::String(FLAGS_udp_port)};
#else
                                                          FLAGS_write_heatmaps,
                                                          FLAGS_write_heatmaps_format,
                                                          FLAGS_write_video_3d,
                                                          FLAGS_write_video_adam,
                                                          FLAGS_write_bvh,
                                                          FLAGS_udp_host,
                                                          FLAGS_udp_port};
#endif
        opWrapper.configure(wrapperStructOutput);

        // GUI (comment or use default argument to disable any visual output)
        const op::WrapperStructGui wrapperStructGui{op::flagsToDisplayMode(FLAGS_display,
                                                                           FLAGS_3d),
                                                    !FLAGS_no_gui_verbose,
                                                    FLAGS_fullscreen};
        opWrapper.configure(wrapperStructGui);
        // clang-format on

        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapper.disableMultiThreading();
    }
  catch (const std::exception& e)
  {
      ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
  }
}

#endif  // ROS_OPENPOSE_UTILITY_H
