/**
* rosOpenpose.cpp: the main file. it consists of two workers input and output worker.
*                  the job of the input worker is to provide color images to openpose wrapper.
*                  the job of the output worker is to receive the keypoints detected in 2D
*                  space. it then converts 2D pixels to 3D coordinates (wrt camera coordinate
*                  system)/
* Author: Ravi Joshi
* Date: 2019/09/27
* src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/tree/master/examples/tutorial_api_cpp
*/

// OpenPose headers
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>

// ros_openpose headers
#include <ros_openpose/Frame.h>
#include <ros_openpose/cameraReader.hpp>
#include <ros_openpose/utility.h>

// define sleep for input and output worker in milliseconds
const int SLEEP_MS = 1;

// define a few datatype
typedef std::shared_ptr<op::Datum> sPtrDatum;
typedef std::shared_ptr<std::vector<sPtrDatum>> sPtrVecSPtrDatum;

// the input worker. the job of this worker is to provide color imagees to
// openpose wrapper
class WUserInput : public op::WorkerProducer<sPtrVecSPtrDatum>
{
public:
  explicit WUserInput(const std::shared_ptr<ros_openpose::CameraReader>& sPtrCameraReader) : mSPtrCameraReader(sPtrCameraReader)
  {
  }

  void initializationOnThread() override
  {
  }

  sPtrVecSPtrDatum workProducer() override
  {
    try
    {
      auto frameNumber = mSPtrCameraReader->getFrameNumber();
      if (frameNumber == 0 || frameNumber == mFrameNumber)
      {
        // display the error at most once per 10 seconds
        ROS_WARN_THROTTLE(10, "Waiting for color image frame...");
        std::this_thread::sleep_for(std::chrono::milliseconds{SLEEP_MS});
        return nullptr;
      }
      else
      {
        // update frame number
        mFrameNumber = frameNumber;

        // get the latest color image from the camera
        auto& colorImage = mSPtrCameraReader->getColorFrame();

        if (!colorImage.empty())
        {
          // create new datum
          auto datumsPtr = std::make_shared<std::vector<sPtrDatum>>();
          datumsPtr->emplace_back();
          auto& datumPtr = datumsPtr->at(0);
          datumPtr = std::make_shared<op::Datum>();

          // fill the datum
          datumPtr->frameNumber = frameNumber;  // frame number = timestamp in ns
#if OPENPOSE1POINT6_OR_HIGHER
          datumPtr->cvInputData = OP_CV2OPCONSTMAT(colorImage);
#else
          datumPtr->cvInputData = colorImage;
#endif
          return datumsPtr;
        }
        else
        {
          // display the error at most once per 10 seconds
          ROS_WARN_THROTTLE(10, "Empty color image frame detected. Ignoring...");
          return nullptr;
        }
      }
    }
    catch (const std::exception& e)
    {
      this->stop();
      // display the error at most once per 10 seconds
      ROS_ERROR_THROTTLE(10, "Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__,
                         __FILE__);
      return nullptr;
    }
  }

private:
  ullong mFrameNumber = 0ULL;
  const std::shared_ptr<ros_openpose::CameraReader> mSPtrCameraReader;
};

// the outpout worker. the job of the output worker is to receive the keypoints
// detected in 2D space. it then converts 2D pixels to 3D coordinates (wrt
// camera coordinate system).
class WUserOutput : public op::WorkerConsumer<sPtrVecSPtrDatum>
{
public:
  // clang-format off
  WUserOutput(const ros::Publisher& framePublisher,
              const std::shared_ptr<ros_openpose::CameraReader>& sPtrCameraReader,
              const std::string& frameId, const bool noDepth)
    : mFramePublisher(framePublisher), mSPtrCameraReader(sPtrCameraReader), mNoDepth(noDepth)
  {
    mFrame.header.frame_id = frameId;
  }
  // clang-format on

  void initializationOnThread() override
  {
  }

  template <typename Array>
  void fillBodyROSMsg(Array& poseKeypoints, int person, int bodyPartCount)
  {
#pragma omp parallel for
    for (auto bodyPart = 0; bodyPart < bodyPartCount; bodyPart++)
    {
      // src:
      // https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md#keypoint-format-in-the-c-api
      const auto baseIndex = poseKeypoints.getSize(2) * (person * bodyPartCount + bodyPart);
      const auto x = poseKeypoints[baseIndex];
      const auto y = poseKeypoints[baseIndex + 1];
      const auto score = poseKeypoints[baseIndex + 2];

      float point3D[3];
      // compute 3D point only if depth flag is set
      if (!mNoDepth)
        mSPtrCameraReader->compute3DPoint(x, y, point3D);

      mFrame.persons[person].bodyParts[bodyPart].pixel.x = x;
      mFrame.persons[person].bodyParts[bodyPart].pixel.y = y;
      mFrame.persons[person].bodyParts[bodyPart].score = score;
      mFrame.persons[person].bodyParts[bodyPart].point.x = point3D[0];
      mFrame.persons[person].bodyParts[bodyPart].point.y = point3D[1];
      mFrame.persons[person].bodyParts[bodyPart].point.z = point3D[2];
    }
  }

  template <typename ArrayOfArray>
  void fillHandROSMsg(ArrayOfArray& handKeypoints, int person, int handPartCount)
  {
#pragma omp parallel for
    for (auto handPart = 0; handPart < handPartCount; handPart++)
    {
      const auto baseIndex = handKeypoints[0].getSize(2) * (person * handPartCount + handPart);

      // left hand
      const auto xLeft = handKeypoints[0][baseIndex];
      const auto yLeft = handKeypoints[0][baseIndex + 1];
      const auto scoreLeft = handKeypoints[0][baseIndex + 2];

      // right hand
      const auto xRight = handKeypoints[1][baseIndex];
      const auto yRight = handKeypoints[1][baseIndex + 1];
      const auto scoreRight = handKeypoints[1][baseIndex + 2];

      float point3DLeft[3];
      float point3DRight[3];

      // compute 3D point only if depth flag is set
      if (!mNoDepth)
      {
        mSPtrCameraReader->compute3DPoint(xLeft, yLeft, point3DLeft);
        mSPtrCameraReader->compute3DPoint(xRight, yRight, point3DRight);
      }

      mFrame.persons[person].leftHandParts[handPart].pixel.x = xLeft;
      mFrame.persons[person].leftHandParts[handPart].pixel.y = yLeft;
      mFrame.persons[person].leftHandParts[handPart].score = scoreLeft;
      mFrame.persons[person].leftHandParts[handPart].point.x = point3DLeft[0];
      mFrame.persons[person].leftHandParts[handPart].point.y = point3DLeft[1];
      mFrame.persons[person].leftHandParts[handPart].point.z = point3DLeft[2];

      mFrame.persons[person].rightHandParts[handPart].pixel.x = xRight;
      mFrame.persons[person].rightHandParts[handPart].pixel.y = yRight;
      mFrame.persons[person].rightHandParts[handPart].score = scoreRight;
      mFrame.persons[person].rightHandParts[handPart].point.x = point3DRight[0];
      mFrame.persons[person].rightHandParts[handPart].point.y = point3DRight[1];
      mFrame.persons[person].rightHandParts[handPart].point.z = point3DRight[2];
    }
  }

  template <typename ArrayOfArray>
  void fillFaceROSMsg(ArrayOfArray& faceKeypoints, int person, int facePartCount)
  {
#pragma omp parallel for
    for (auto facePart = 0; facePart < facePartCount; facePart++)
    {
      const auto baseIndex = faceKeypoints.getSize(2) * (person * facePartCount + facePart);

      // face
      const auto xFace = faceKeypoints[baseIndex];
      const auto yFace = faceKeypoints[baseIndex + 1];
      const auto scoreFace = faceKeypoints[baseIndex + 2];

      float point3DFace[3];

      // compute 3D point only if depth flag is set
      if (!mNoDepth)
      {
        mSPtrCameraReader->compute3DPoint(xFace, yFace, point3DFace);
      }

      mFrame.persons[person].faceParts[facePart].pixel.x = xFace;
      mFrame.persons[person].faceParts[facePart].pixel.y = yFace;
      mFrame.persons[person].faceParts[facePart].score = scoreFace;
      mFrame.persons[person].faceParts[facePart].point.x = point3DFace[0];
      mFrame.persons[person].faceParts[facePart].point.y = point3DFace[1];
      mFrame.persons[person].faceParts[facePart].point.z = point3DFace[2];
    }
  }

  void workConsumer(const sPtrVecSPtrDatum& datumsPtr)
  {
    try
    {
      if (datumsPtr != nullptr && !datumsPtr->empty())
      {
        // update timestamp
        mFrame.header.stamp = ros::Time().fromNSec((uint64_t)datumsPtr->at(0)->frameNumber);

        // make sure to clear previous data
        mFrame.persons.clear();

        // we use the latest depth image for computing point in 3D space
        mSPtrCameraReader->lockLatestDepthImage();

        // accesing each element of the keypoints
        const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
        const auto& handKeypoints = datumsPtr->at(0)->handKeypoints;
        const auto& faceKeypoints = datumsPtr->at(0)->faceKeypoints;

        // get the size
        const auto personCount = poseKeypoints.getSize(0);
        const auto bodyPartCount = poseKeypoints.getSize(1);
        const auto handPartCount = handKeypoints[0].getSize(1);
        const auto facePartCount = faceKeypoints.getSize(1);        

        mFrame.persons.resize(personCount);

        // update with the new data
        for (auto person = 0; person < personCount; person++)
        {
          mFrame.persons[person].bodyParts.resize(bodyPartCount);
          mFrame.persons[person].leftHandParts.resize(handPartCount);
          mFrame.persons[person].rightHandParts.resize(handPartCount);
          mFrame.persons[person].faceParts.resize(facePartCount);          

          fillBodyROSMsg(poseKeypoints, person, bodyPartCount);
          fillHandROSMsg(handKeypoints, person, handPartCount);
          fillFaceROSMsg(faceKeypoints, person, facePartCount);
        }

        mFramePublisher.publish(mFrame);
      }
      else
      {
        // display the error at most once per 10 seconds
        ROS_WARN_THROTTLE(10, "Waiting for datum...");
        std::this_thread::sleep_for(std::chrono::milliseconds{SLEEP_MS});
      }
    }
    catch (const std::exception& e)
    {
      this->stop();
      ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
  }

private:
  const bool mNoDepth;
  ros_openpose::Frame mFrame;
  const ros::Publisher mFramePublisher;
  const std::shared_ptr<ros_openpose::CameraReader> mSPtrCameraReader;
};


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_openpose_node");
  ros::NodeHandle nh("~");

  // define the parameters, we are going to read
  bool noDepth;
  std::string colorTopic, depthTopic, camInfoTopic, frameId, pubTopic;

  // read the parameters from relative nodel handle
  nh.getParam("frame_id", frameId);
  nh.getParam("pub_topic", pubTopic);
  nh.param("no_depth", noDepth, false);  // default value is false
  nh.getParam("color_topic", colorTopic);
  nh.getParam("depth_topic", depthTopic);
  nh.getParam("cam_info_topic", camInfoTopic);

  if (pubTopic.empty())
  {
    ROS_FATAL("Missing 'pub_topic' info in launch file. Please make sure that you have executed 'run.launch' file.");
    exit(-1);
  }

  // parsing command line flags
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const auto cameraReader = std::make_shared<ros_openpose::CameraReader>(nh, colorTopic, depthTopic, camInfoTopic);

  // the frame consists of the location of detected body parts of each person
  const ros::Publisher framePublisher = nh.advertise<ros_openpose::Frame>(pubTopic, 1);

  try
  {
    ROS_INFO("Starting ros_openpose...");
    op::Wrapper opWrapper;
    configureOpenPose(opWrapper);
    // Initializing the user custom classes
    auto wUserInput = std::make_shared<WUserInput>(cameraReader);
    auto wUserOutput = std::make_shared<WUserOutput>(framePublisher, cameraReader, frameId, noDepth);

    // Add custom processing
    const auto workerInputOnNewThread = true;
    opWrapper.setWorker(op::WorkerType::Input, wUserInput, workerInputOnNewThread);
    const auto workerOutputOnNewThread = true;
    opWrapper.setWorker(op::WorkerType::Output, wUserOutput, workerOutputOnNewThread);

    // start and run
    opWrapper.start();

    // exit when Ctrl-C is pressed, or the node is shutdown by the master
    ros::spin();

    // return successful message
    ROS_INFO("Exiting ros_openpose...");

    // stop processing
    opWrapper.stop();
    return 0;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
    return -1;
  }
}
