#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>
#include <ros_openpose/Frame.h>
#include <ros_openpose/BodyPart.h>

#include<ros_openpose/utility.h>

using namespace sensor_msgs;
class rosOpenPose
{
private:
    op::Wrapper* _op_wrapper;
    ros::NodeHandle* _nh;
    ros::Publisher _pub;
    message_filters::Subscriber<Image> _color_sub;
    message_filters::Subscriber<Image> _depth_sub;
    typedef message_filters::sync_policies::ApproximateTime<Image, Image> ColorDepthSyncPolicy;
    typedef message_filters::Synchronizer<ColorDepthSyncPolicy> ColorDepthSync;
    std::shared_ptr<ColorDepthSync> _sync;

    bool _no_depth;
    float _fx, _fy, _cx, _cy;
    float _mm_to_m;

    cv::Mat _color_img, _depth_img;
    ros_openpose::Frame _frame_msg;
public:
    rosOpenPose(ros::NodeHandle* nh, op::Wrapper* op_wrapper, const std::string& color_topic, const std::string& depth_topic,
                const std::string& cam_info_topic, const std::string& pub_topic, const std::string& frame_id, const bool& no_depth):
                _nh(nh), _op_wrapper(op_wrapper), _no_depth(no_depth) {

        _frame_msg.header.frame_id = frame_id;

        // Populate camera intrinsic matrix values.
        auto cam_info = ros::topic::waitForMessage<CameraInfo>(cam_info_topic);
        _fx = cam_info->K.at(0);
        _fy = cam_info->K.at(4);
        _cx = cam_info->K.at(2);
        _cy = cam_info->K.at(5);

        // Obtain depth encoding.
        auto depth_encoding = ros::topic::waitForMessage<Image>(depth_topic)->encoding;
        _mm_to_m = (depth_encoding == image_encodings::TYPE_16UC1) ? 0.001 : 1.;

        // Initialize frame publisher
        _pub = _nh->advertise<ros_openpose::Frame>(pub_topic, 10);

        // Start color & depth subscribers.
        _color_sub.subscribe(*_nh, color_topic, 1);
        _depth_sub.subscribe(*_nh, depth_topic, 1);
        _sync.reset(new ColorDepthSync(ColorDepthSync(10), _color_sub, _depth_sub));
        _sync->registerCallback(boost::bind(&rosOpenPose::callback, this, _1, _2));
    }

    template <typename key_points>
    void assign_msg_vals(ros_openpose::BodyPart& part, const key_points& kp, const int& i) {
        // Assign pixel position and score from Openpose.
        float u = kp[i], v = kp[i+1], s = kp[i+2];
        part.pixel.x = u;
        part.pixel.y = v;
        part.score = s;

        // Compute 3D Pose if depth is provided.
        if (!_no_depth) {
            auto depth = _depth_img.at<float>(static_cast<int>(v), static_cast<int> (u)) * _mm_to_m;
            if (depth <= 0) return;
            part.point.x = (depth / _fx) * (u - _cx);
            part.point.y = (depth / _fy) * (v - _cy);
            part.point.z = depth;
        }
    }

    void callback(const ImageConstPtr& color_msg, const ImageConstPtr& depth_msg) {
        _frame_msg.header.stamp = color_msg->header.stamp;
        _frame_msg.persons.clear();

        _color_img = cv_bridge::toCvShare(color_msg, image_encodings::BGR8)->image;
        _depth_img = cv_bridge::toCvShare(depth_msg, image_encodings::TYPE_32FC1)->image;

        // Fill datum
#if OPENPOSE1POINT6_OR_HIGHER
        auto datum_ptr = _op_wrapper->emplaceAndPop(OP_CV2OPCONSTMAT(_color_img));
#else
        auto datum_ptr = _op_wrapper->emplaceAndPop(_color_img);
#endif

        const auto& pose_kp = datum_ptr->at(0)->poseKeypoints;
        const auto& hand_kp = datum_ptr->at(0)->handKeypoints;
        const auto& face_kp = datum_ptr->at(0)->faceKeypoints;

        // get the size
        const auto num_persons = pose_kp.getSize(0);
        const auto body_part_count = pose_kp.getSize(1);
        const auto hand_part_count = hand_kp[0].getSize(1);
        const auto face_part_count = face_kp.getSize(1);

        _frame_msg.persons.resize(num_persons);
        int i;
        for (auto p = 0; p < num_persons; p++) {
            auto& curr_person = _frame_msg.persons[p];

            curr_person.bodyParts.resize(body_part_count);
            curr_person.leftHandParts.resize(hand_part_count);
            curr_person.rightHandParts.resize(hand_part_count);
            curr_person.faceParts.resize(face_part_count);


            // Fill body parts
            for (auto bp = 0; bp < body_part_count; bp++) {
                auto& curr_body_part = curr_person.bodyParts[bp];
                i = pose_kp.getSize(2) * (p * body_part_count + bp);
                assign_msg_vals(curr_body_part, pose_kp, i);
            }

            // Fill left and right hands
            for (auto hp = 0; hp < hand_part_count; hp++) {
                i = hand_kp[0].getSize(2) * (p * hand_part_count + hp);

                // Left Hand
                auto& curr_left_hand = curr_person.leftHandParts[hp];
                assign_msg_vals(curr_left_hand, hand_kp[0], i);

                // Right Hand
                auto& curr_right_hand = curr_person.rightHandParts[hp];
                assign_msg_vals(curr_right_hand, hand_kp[1], i);
            }

            // Fill face parts
            for (auto fp = 0; fp < face_part_count; fp++) {
                auto& curr_face_part = curr_person.faceParts[fp];
                i = face_kp.getSize(2) * (p * face_part_count + fp);
                assign_msg_vals(curr_face_part, face_kp, i);
            }
        }
        _pub.publish(_frame_msg);
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_openpose_synchronous");
    ros::NodeHandle nh("~");

    // Get params
    bool no_depth;
    std::string color_topic, depth_topic, cam_info_topic, pub_topic, frame_id;
    nh.getParam("color_topic", color_topic);
    nh.getParam("depth_topic", depth_topic);
    nh.getParam("cam_info_topic", cam_info_topic);
    nh.getParam("pub_topic", pub_topic);
    nh.getParam("frame_id", frame_id);
    nh.param("no_depth", no_depth, false);  // default value is false

    // Parse Openpose Args
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    try {
        ROS_INFO("Starting ros_openpose...");

        // Initialize Openpose wrapper
        op::Wrapper op_wrapper{op::ThreadManagerMode::Asynchronous};
        configureOpenPose(op_wrapper);
        op_wrapper.start();

        // Start ROS wrapper
        rosOpenPose rop(&nh, &op_wrapper, color_topic, depth_topic, cam_info_topic, pub_topic, frame_id, no_depth);

        ros::spin();

        ROS_INFO("Exiting ros_openpose...");

        op_wrapper.stop();

        return 0;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
        return -1;
    }
}
