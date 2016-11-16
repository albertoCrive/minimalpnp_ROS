
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
PoseEstimator poseEstimator;
cv::Mat undistortMapx, undistortMapy;

#define MAX_NPOSES 13
std::list<cv::Vec6f> posesSLAM2Cam;
std::list<cv::Vec6f> posesObj2Cam;
// temp
std::vector<cv::Mat> imagesSLAM;
const float distForManualCalibrationMeters=0.297;

bool bScaleManuallySet = false;
double absoluteScale = -1;
cv::Mat cameraSLAMCenter1;
cv::Mat cameraSLAMCenter2;
int nMinimumPartForAcceptingPose = 4;
int nMinimumPartComputingScale = 4;


cv::Vec6f poseSLAM2Cam;


void RTFromPose(const cv::Vec6f &pose, cv::Mat * R, cv::Mat *t)
{
    (*R) = cv::Mat(3,1, CV_64FC1);
    (*t) = cv::Mat(3,1, CV_64FC1);
    R->at<double>(0,0)=  pose[0];     R->at<double>(1,0)=  pose[1];     R->at<double>(2,0)=  pose[2];
    t->at<double>(0,0)=  pose[3];     t->at<double>(1,0)=  pose[4];     t->at<double>(2,0)=  pose[5];
}

cv::Vec6f InvertPose(const cv::Vec6f &pose)
{
    cv::Mat expMap,t, R;
    RTFromPose(pose, &expMap, &t);
    cv::Rodrigues(expMap, R);
    cv::Mat invR = R.inv();
    cv::Mat invExpMap, invt;
    cv::Rodrigues(invR, invExpMap);
    invt = - invR * t;
    return cv::Vec6f(invExpMap.at<double>(0,0), invExpMap.at<double>(1,0), invExpMap.at<double>(2,0),
                     invt.at<double>(0,0), invt.at<double>(1,0), invt.at<double>(2,0));
}

cv::Vec6f ComposePoses(const cv::Vec6f &poseIn, const cv::Vec6f & poseOut)
{
    cv::Mat rvec1(3,1,CV_64F); rvec1.at<double>(0,0)=poseIn(0); rvec1.at<double>(1,0)=poseIn(1); rvec1.at<double>(2,0)=poseIn(2);
    cv::Mat rvec2(3,1,CV_64F); rvec2.at<double>(0,0)=poseOut(0); rvec2.at<double>(1,0)=poseOut(1); rvec2.at<double>(2,0)=poseOut(2);
    cv::Mat tvec1(3,1,CV_64F); tvec1.at<double>(0,0)=poseIn(3); tvec1.at<double>(1,0)=poseIn(4); tvec1.at<double>(2,0)=poseIn(5);
    cv::Mat tvec2(3,1,CV_64F); tvec2.at<double>(0,0)=poseOut(3); tvec2.at<double>(1,0)=poseOut(4); tvec2.at<double>(2,0)=poseOut(5);

    cv::Mat rvec3, tvec3;
    cv::composeRT(rvec1, tvec1, rvec2, tvec2, rvec3, tvec3);
    cv::Vec6f newPose(rvec3.at<double>(0,0), rvec3.at<double>(1,0), rvec3.at<double>(2,0), tvec3.at<double>(0,0), tvec3.at<double>(1,0), tvec3.at<double>(2,0));
    return newPose;
}



cv::Mat ComputeCameraCenter(const cv::Vec6f &poseWorldToCam)
{
    cv::Mat rot(cv::Size(1,3),CV_32F);
    rot.at<float>(0,0) = poseWorldToCam[0]; rot.at<float>(1,0) = poseWorldToCam[1]; rot.at<float>(2,0) = poseWorldToCam[2];
    cv::Mat rotMat;
    cv::Rodrigues(rot, rotMat);

    cv::Mat t(cv::Size(1,3),CV_32F);
    t.at<float>(0,0) = poseWorldToCam[3]; t.at<float>(1,0) = poseWorldToCam[4]; t.at<float>(2,0) = poseWorldToCam[5];

    cv::Mat cameraCenter =-rotMat.inv() * t;
    return cameraCenter;
}

void PublishPose(const cv::Mat& translation, const cv::Mat& rotationExp, const float &scale)
{
  // ATTENTION : THE first 3 components of the orientation are the exp map, the fourth is the estimated scale !!!
  geometry_msgs::Pose poseObjToSLAMmsg;
  if(!translation.data || ! rotationExp.data)
    {
      poseObjToSLAMmsg.position.x = NAN;
      poseObjToSLAMmsg.position.y = NAN;
      poseObjToSLAMmsg.position.z = NAN;
      poseObjToSLAMmsg.orientation.x = NAN;
      poseObjToSLAMmsg.orientation.y = NAN;
      poseObjToSLAMmsg.orientation.z = NAN;
      poseObjToSLAMmsg.orientation.w = NAN;
    }
  else
    {
      poseObjToSLAMmsg.position.x = translation.at<double>(0,0);
      poseObjToSLAMmsg.position.y = translation.at<double>(1,0);
      poseObjToSLAMmsg.position.z = translation.at<double>(2,0);
      poseObjToSLAMmsg.orientation.x = rotationExp.at<double>(0,0);
      poseObjToSLAMmsg.orientation.y = rotationExp.at<double>(1,0);
      poseObjToSLAMmsg.orientation.z = rotationExp.at<double>(2,0);
      poseObjToSLAMmsg.orientation.w = scale;
    }
	    
  posePublisher.publish(poseObjToSLAMmsg);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat frameRGB = cv_bridge::toCvShare(msg, "bgr8")->image;
        // dirty workaround for receiving the frame pose in the header of
        // the frame !
        std::stringstream ss(msg->header.frame_id);
        for(int i=0;i<6;i++)
            ss >> poseSLAM2Cam[i];
        //        std::cout << " I should have read :"<<  ss.str()<<std::endl<<std::endl;
        //        std::cout << " img received the pose is  :"<<  poseSLAM2Cam<<std::endl<<std::endl;
        ////////////////////////////////////////////////////////////////////////
        // call minimalpnp to compute the absolute pose of the box
        // TODO compute transform pose ORBSLAM -> pose of the box
        if (!frameRGB.data || frameRGB.cols != 640 || frameRGB.rows != 480)
            throw std::runtime_error("frame are not VGA ! Aborting.");

        cv::Mat frameRGBUndistorted;
        cv::remap(frameRGB, frameRGBUndistorted, undistortMapx, undistortMapy, cv::INTER_LINEAR);
        cv::Matx66f outPoseCovariance;
        cv::Vec6f poseObj2Cam;

        int nEmployedParts =  poseEstimator.EstimatePoseOnFrame(frameRGBUndistorted, &outPoseCovariance, & poseObj2Cam);
        std::cout << " ...another frame tracked ! number of detected parts : " <<nEmployedParts <<std::endl;
	std::cout << "SCALE : " << absoluteScale << " "  << bScaleManuallySet << " "  << nMinimumPartForAcceptingPose <<std::endl;

	if(nEmployedParts >= nMinimumPartComputingScale)
	      {
			    std::cout << "I need to automatically estimate the scale  !" <<std::endl;

		//////////////////BEGIN SCALE ESTIMATION //////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////
	  
		//////////////// update lists of poses////////////////////////////////
		if(posesSLAM2Cam.size() >= MAX_NPOSES)
		  posesSLAM2Cam.pop_front();
		if(posesObj2Cam.size() >= MAX_NPOSES)
		  posesObj2Cam.pop_front();

		posesSLAM2Cam.push_back(poseSLAM2Cam);
		posesObj2Cam.push_back(poseObj2Cam);

		const int nPoses = posesObj2Cam.size();
		// if I can not estimate the scale then return
		if(nPoses < 3)
		  {
		    PublishPose( cv::Mat(), cv::Mat(), NAN);
		    return;
		  }
	    
		// if too big displacement between 2 last poses then re-initialize poses list
		// TODO keep this code ???????????????
		float maxDisplacementMeters = 0.7;
		auto itObj = posesObj2Cam.end(); itObj--;
		auto itObjPrev = std::prev(itObj);
		cv::Mat centerObj = ComputeCameraCenter((*itObj));
		cv::Mat centerObjPrev = ComputeCameraCenter((*itObjPrev));
		if(cv::norm(centerObj - centerObjPrev) > maxDisplacementMeters)
		  {
		    posesSLAM2Cam.clear();
		    posesObj2Cam.clear();
		    posesSLAM2Cam.push_back(poseSLAM2Cam);
		    posesObj2Cam.push_back(poseObj2Cam);
		
		    return;
		  }

		//////////////// estimate scale transform ////////////////////////////////
		auto itSLAM2Cam = posesSLAM2Cam.begin();
		auto itObj2Cam = posesObj2Cam.begin();
		int iPose(0);
		if (!bScaleManuallySet)
		  {
		    float furthestScale = 0.0;
		    float maxDistance = 0.0;
		    while(iPose < nPoses-1)
		      {
			auto nextItSLAM = std::next(itSLAM2Cam);
			auto nextItObj = std::next(itObj2Cam);
			while(nextItObj != posesObj2Cam.end())
			  {

			    cv::Mat centerObj1 = ComputeCameraCenter((*itObj2Cam));
			    cv::Mat centerObj2 = ComputeCameraCenter((*nextItObj));
			    float deltatObj = cv::norm(centerObj1 - centerObj2);

			    cv::Mat centerSLAM1 = ComputeCameraCenter((*itSLAM2Cam));
			    cv::Mat centerSLAM2 = ComputeCameraCenter((*nextItSLAM));
			    float deltatSLAM = cv::norm(centerSLAM1 - centerSLAM2);

			    float ss = deltatSLAM > 1e-3 ?  deltatObj / deltatSLAM : 1.;
			    if(deltatSLAM > maxDistance && deltatSLAM > 1e-3)
			      {
				// std::cout << "setting furthest scale ! old : " << maxDistance << " ; new : " << deltatObj << "; scale : " << ss <<std::endl;
				maxDistance = deltatSLAM;
				furthestScale = ss;
			      }
			    nextItSLAM++;
			    nextItObj++;
			  }
			itObj2Cam++;
			itSLAM2Cam++;
			iPose++;
		      }
		    if(furthestScale != 0.)
		      absoluteScale = furthestScale;
		    else
		      {
			std::cout<< " Failed estimating the scale :(" <<std::endl;
			PublishPose( cv::Mat(), cv::Mat(), NAN);
			return;
		      }
		  }
	      }
	

        //////////////////END SCALE ESTIMATION //////////////////////////////////////
   	  ////////////////////////////////////////////////////////////////////////////

        if(absoluteScale > 1e-2 && absoluteScale < 10 && nEmployedParts >= nMinimumPartForAcceptingPose)
	  {
	    std::cout << "scale seems ok I compute abs pose !" <<std::endl;
	    
            const cv::Mat canonicalBasis = cv::Mat::eye(3,4,CV_64FC1);

            // auto itSLAM2Cam = posesSLAM2Cam.begin();
            // auto itObj2Cam = posesObj2Cam.begin();
            // int iPose = 0;
            // std::string method = bScaleManuallySet ? " manually set " : " automatically estimated";
            // std::cout << " retained scale  " << absoluteScale <<  ",   " << method <<std::endl;

            // while(iPose < nPoses-1)
            // {
            //     itObj2Cam++;
            //     itSLAM2Cam++;
            //     iPose++;
            // }
            // // now the pointers should point to the last computed pose
            
            // cv::Vec6f poseCam2Obj = InvertPose((*itObj2Cam));
            // // points SLAM
            // cv::Vec6f poseCam2SLAM = InvertPose((*itSLAM2Cam));

	    ///////////////////////
            cv::Vec6f poseCam2Obj = InvertPose(poseObj2Cam);
	    cv::Vec6f poseCam2SLAM = InvertPose(poseSLAM2Cam);
	    ////////////////////////////////////
            cv::Mat Rexp,R, t;
            RTFromPose(poseCam2Obj, &Rexp, &t);
            cv::Rodrigues(Rexp, R);
            cv::Mat ppOBJ = R * canonicalBasis * absoluteScale + cv::repeat(t, 1, 4);

		RTFromPose(poseCam2SLAM, &Rexp, &t);
            cv::Rodrigues(Rexp, R);
            cv::Mat ppSLAM = R * canonicalBasis + cv::repeat(t, 1, 4);

            cv::Mat rotation, translation;
            double scale;
            AlignTrajectories(ppOBJ, ppSLAM,  &rotation, &translation, &scale); // pointsSLAM = s*R*pointsOBJ + translation


            // CHEKC compute residual
            cv::Mat supposedPoints = scale * rotation * ppOBJ + cv::repeat(translation, 1, ppOBJ.cols);
            cv::Mat delta;
            absdiff(supposedPoints, ppSLAM, delta);
            //            double mmin(0), mmax(0);
            //            minMaxLoc(delta, &mmin, &mmax);
            //std::cout<< " residual norm " << cv::norm(delta)/ppOBJ.cols <<std::endl;
            ///////////////////////////////////////////////////////////////////////////////////
            //            cv::Vec6f poseObj2SLAM = ComposePoses(poseObj2Cam, InvertPose(poseSLAM2Cam));
            cv::Mat rotationExp;
            cv::Rodrigues(rotation, rotationExp);
	    PublishPose( translation, rotationExp, scale);
	    

            // ATTENTION : THE first 3 components of the orientation are the exp map, the fourth is the estimated scale !!!
            // geometry_msgs::Pose poseObjToSLAMmsg;
            // poseObjToSLAMmsg.position.x = translation.at<double>(0,0);
            // poseObjToSLAMmsg.position.y = translation.at<double>(1,0);
            // poseObjToSLAMmsg.position.z = translation.at<double>(2,0);
            // poseObjToSLAMmsg.orientation.x = rotationExp.at<double>(0,0);
            // poseObjToSLAMmsg.orientation.y = rotationExp.at<double>(1,0);
            // poseObjToSLAMmsg.orientation.z = rotationExp.at<double>(2,0);
            // poseObjToSLAMmsg.orientation.w = scale;
            // posePublisher.publish(poseObjToSLAMmsg);
        }// end of if (tracked)

	// cv::imshow("minimal pnp->undistorted image", frameRGB );
	
        char key = cv::waitKey(30);
        if (key ==' ')
            key = cv::waitKey(0);
        else if (key ==27)
            exit(0);
        else if (key == 's')
        {
            std::cout << " Please enter the scale value ! " <<std::endl;
            std:: cin >>absoluteScale;
            std::cout <<" absolute scale set to : " << absoluteScale << "; press a key to continue or c to abandon" << std::endl;
            char ccc;
            std:: cin >> ccc;
            if(ccc != 'c')
                bScaleManuallySet = true;
        }
        else if (key == 'p')
        {
            std::cout << " setting first camera center : " << std::endl;
            cameraSLAMCenter1 = ComputeCameraCenter(poseSLAM2Cam);
            std::cout << " camera center set at : " <<cameraSLAMCenter1.t() << std::endl;
            std::cout << " Now move the camera of " << distForManualCalibrationMeters << " m and press q" <<std::endl;
        }
        else if (key == 'q')
        {
            if(!cameraSLAMCenter1.data)
                std::cout << "you first have to set the first camera center. Press p"<<std::endl;
            else
            {
                std::cout << " setting second camera center and scale : " << std::endl;
                cameraSLAMCenter2 = ComputeCameraCenter(poseSLAM2Cam);
                absoluteScale = distForManualCalibrationMeters/cv::norm(cameraSLAMCenter1 - cameraSLAMCenter2);
                bScaleManuallySet = true;
                std::cout << " second camera center set at : " <<cameraSLAMCenter2.t() << std::endl;
                std::cout << " scale manually set at : " << absoluteScale << std::endl;
            }
        }
        else if (key == 'r')
        {
            std::cout << "automatically estimating the scale "<<std::endl;
            bScaleManuallySet = false;
            absoluteScale = -1;
            PublishPose( cv::Mat(), cv::Mat(), NAN);
        }
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
    cv::destroyWindow("minimal pnp->undistorted image");

}

// void PoseReceivedCallback(const geometry_msgs::PoseStamped & poseStamped)
// {
//     ROS_INFO_STREAM("fico received a pose ! "<< poseStamped.pose.position.x<< poseStamped.pose.position.y << poseStamped.pose.position.z);
// }



void ButtonSetP(int state, void* userdata)
{
  std::cout << " setting first camera center : " << std::endl;
  cameraSLAMCenter1 = ComputeCameraCenter(poseSLAM2Cam);
  std::cout << " camera center set at : " <<cameraSLAMCenter1.t() << std::endl;
  return;
}

void ButtonSetQ(int state, void* userdata)
{
  if(!cameraSLAMCenter1.data)
    std::cout << "you first have to set the first camera center. Press p"<<std::endl;
  else
    {
      std::cout << " setting second camera center and scale : " << std::endl;
      cameraSLAMCenter2 = ComputeCameraCenter(poseSLAM2Cam);
      absoluteScale = distForManualCalibrationMeters/cv::norm(cameraSLAMCenter1 - cameraSLAMCenter2);
      bScaleManuallySet = true;
      std::cout << " second camera center set at : " <<cameraSLAMCenter2.t() << std::endl;
      std::cout << " scale manually set at : " << absoluteScale << std::endl;
    }
  return;
}

void ButtonReset(int state, void* userdata)
{
  std::cout << "automatically estimating the scale "<<std::endl;
  bScaleManuallySet = false;
  absoluteScale = -1;
  PublishPose( cv::Mat(), cv::Mat(), NAN);
  return;
}



int main(int argc, char ** argv)
{

    // POSEESTIMATOR

    std::cout << "trying to initialize pose estimator .."<<std::endl;
    poseEstimator.Initialize();

    cv::Matx33f internalCalibrationMatrix;
    cv::Mat distortionParam;
    std::string intrinsicsXmlFile = "./Data/calib/LogitechPro9000.xml";
    ReadIntrinsicFromOpencvFile(intrinsicsXmlFile, &internalCalibrationMatrix, &distortionParam);
    cv::initUndistortRectifyMap(internalCalibrationMatrix, distortionParam, cv::Mat(), internalCalibrationMatrix, cv::Size(640, 480), CV_32FC1, undistortMapx, undistortMapy);
    if(internalCalibrationMatrix(0,2) > 500)
        throw std::runtime_error("The intrinsic matrix is likely to not be VGA!! ! Aborting.");

    std::cout << " ... pose estimator initialized !"<<std::endl;

    // I n i t i a l i z e the ROS system .
    ros::init(argc, argv, "minimalpnp");

    // E s t a b l i s h t h i s program as a ROS node .
    ros::NodeHandle nodeHandler;
       // cv::namedWindow("minimal pnp->undistorted image");

       cv::createButton("set P1", ButtonSetP, NULL, CV_PUSH_BUTTON, 0);       
       cv::createButton("Set P2=P1+dist", ButtonSetQ, NULL, CV_PUSH_BUTTON, 0);       
       cv::createButton("reset scale", ButtonReset, NULL, CV_PUSH_BUTTON, 0);       
       // cv::startWindowThread();


       
    posePublisher = nodeHandler.advertise<geometry_msgs::Pose>("minimalpnp/relativePose", 1000); //1000 is the max lenght of the message queue
    image_transport::ImageTransport it(nodeHandler);
    image_transport::Subscriber trackedImageSubscriber = it.subscribe("/SMART_ORB_SLAM2/trackedImage", 1, imageCallback);

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    ros::spin();

    // TODO : add cleanup !
    //minimalpnp::Cleanup();

    // ros::Rate rate(2);
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
