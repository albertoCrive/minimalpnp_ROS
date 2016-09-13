
#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<stdlib.h>
#include<signal.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "PoseEstimator.hpp"


ros::Publisher posePublisher;


// TODO: copied from Utilities
void ReadIntrinsicFromOpencvFile(const std::string & fileName, cv::Matx33f * internalCalibrationMatrix, cv::Mat * distortionParameters)
{
    cv::FileStorage fs(fileName, cv::FileStorage::READ);
    cv::Mat temp;
    fs["camera_matrix"] >> temp;
    if (temp.type() != CV_64F)
        throw std::runtime_error("could not read internal parameters matrix! aborting...");
    for (int i(0); i < 3; ++i) {
        for (int j(0); j < 3; ++j)
            (*internalCalibrationMatrix) (i, j) = temp.at < double >(i, j);
    }
    fs["distortion_coefficients"] >> (*distortionParameters);
    fs.release();
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {

        cv::Mat imageRGB = cv_bridge::toCvShare(msg, "bgr8")->image;
        // dirty workaround for receiving the frame pose in the header of
        // the frame !
        cv::Vec6f poseORBSLAM;
        std::stringstream ss(msg->header.frame_id);
        for(int i=0;i<6;i++)
            ss >> poseORBSLAM[i];

        std::cout << " img received the pose is  :"<<  poseORBSLAM<<std::endl<<std::endl;
        
        // TODO call minimalpnp to compute the absolute pose of the box
        // TODO compute transform pose ORBSLAM -> pose of the box
        geometry_msgs::Pose poseObjInORB;
        poseObjInORB.position.x = 0;
        poseObjInORB.position.y = 0;
        poseObjInORB.position.z = 0.1*double (rand()) / double(RAND_MAX);
        poseObjInORB.orientation.x = 0;
        poseObjInORB.orientation.y = 0;
        poseObjInORB.orientation.z = 0.01*double (rand()) / double(RAND_MAX);
        poseObjInORB.orientation.w = 1;
        posePublisher.publish(poseObjInORB);
        std::cout << " hope i published the pose!  :"<<  poseObjInORB.position.z<<std::endl<<std::endl;

        cv::imshow("minimal pnp->received image", imageRGB);
        cv::waitKey(5);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    std::cout<<"Signal caught. Exiting ... " <<std::endl;
    ros::shutdown();
}

// void PoseReceivedCallback(const geometry_msgs::PoseStamped & poseStamped)
// {
//     ROS_INFO_STREAM("fico received a pose ! "<< poseStamped.pose.position.x<< poseStamped.pose.position.y << poseStamped.pose.position.z);
// }



int main(int argc, char ** argv)
{

    // POSEESTIMATOR

    std::cout << "trying to initialize pose estimator .."<<std::endl;
    PoseEstimator poseEstimator;
    poseEstimator.Initialize();
    std::cout << " ORCO CAZZO CHE FIGATA !@@ "<<std::endl;



    // I n i t i a l i z e the ROS system .
    ros::init(argc, argv, "minimalpnp");

    // E s t a b l i s h t h i s program as a ROS node .
    ros::NodeHandle nodeHandler;
    cv::namedWindow("minimal pnp->received image");
    cv::startWindowThread();

    posePublisher = nodeHandler.advertise<geometry_msgs::Pose>("minimalpnp/relativePose", 1000); //1000 is the max lenght of the message queue

    image_transport::ImageTransport it(nodeHandler);
    image_transport::Subscriber trackedImageSubscriber = it.subscribe("/ORB_SLAM2/trackedImage", 1, imageCallback);

    // not necessary anymore !
    // ros::Subscriber trackedPoseSubscriber = nodeHandler.subscribe("/ORB_SLAM2/SLAMPose", 1000, &PoseReceivedCallback); //1000 is the max lenght of the message queue

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);


    ros::spin();
    cv::destroyWindow("minimal pnp->received image");

    ros::Rate rate(2);

    // while(ros::ok)
    //   {
    //     geometry_msgs::Pose pose;
    //     pose.position.x = 1;
    //     pose.position.y = 2;
    //     pose.position.z = double (rand()) / double(RAND_MAX);
    //     //      Point position
    //     // Quaternion orientation

    //     posePublisher.publish(pose);
    //     // Send some output as a l o g message .
    //     ROS_INFO_STREAM( "HelloOrcoCazzoROS! Sending pose message : "<< " pose z " << pose.position.z) ;
    //     // Wait u n t i l i t ' s time f o r another i t e r a t i o n .
    //     rate.sleep () ;
    //   }
}
