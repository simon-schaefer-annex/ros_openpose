#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>

#include <ros_openpose/Frame.h>
#include <ros_openpose/BodyPart.h>
#include <ros_openpose/utility.h>


using namespace sensor_msgs;
class RosOpenPoseRGB
{
private:
    op::Wrapper* _op_wrapper;
    ros::NodeHandle* _nh;
    ros::Publisher _pub;
    image_transport::ImageTransport* imgTransport_ = nullptr;
    image_transport::Subscriber _color_sub;

    cv::Mat _color_img;
    ros_openpose::Frame _frame_msg = ros_openpose::Frame();
public:
    RosOpenPoseRGB(ros::NodeHandle& nh, op::Wrapper* op_wrapper, const std::string& color_topic,
                   const std::string& pub_topic, const std::string& frame_id):
                   _nh(&nh), _op_wrapper(op_wrapper) {

        _frame_msg.header.frame_id = frame_id;

        // Initialize frame publisher
        _pub = _nh->advertise<ros_openpose::Frame>(pub_topic, 10);

        // Start color subscribers.
        imgTransport_ = new image_transport::ImageTransport(nh);
        _color_sub = imgTransport_->subscribe(color_topic, 1, &RosOpenPoseRGB::callback, this);
    }

    template <typename key_points>
    void assign_msg_vals(ros_openpose::BodyPart& part, const key_points& kp, const int& i) {
        // Assign pixel position and score from Openpose.
        float u = kp[i], v = kp[i+1], s = kp[i+2];
        part.pixel.x = u;
        part.pixel.y = v;
        part.score = s;
    }

    void callback(const sensor_msgs::ImageConstPtr& color_msg) {
        _frame_msg.header.stamp = color_msg->header.stamp;
        _frame_msg.persons.clear();

        _color_img = cv_bridge::toCvShare(color_msg, image_encodings::BGR8)->image;

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
    std::string color_topic, cam_info_topic, pub_topic, frame_id;
    nh.getParam("color_topic", color_topic);
    nh.getParam("pub_topic", pub_topic);
    nh.getParam("frame_id", frame_id);

    // Parse Openpose Args
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    try {
        ROS_INFO("Starting ros_openpose...");

        // Initialize Openpose wrapper
        op::Wrapper op_wrapper{op::ThreadManagerMode::Asynchronous};
        configureOpenPose(op_wrapper);
        op_wrapper.start();

        // Start ROS wrapper
        RosOpenPoseRGB rop(nh, &op_wrapper, color_topic, pub_topic, frame_id);

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
