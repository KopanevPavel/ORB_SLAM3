/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include <tuple>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb):mpSLAM(pSLAM), mpImuGb(pImuGb){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    // pair<cv::Mat,cv::Mat> GetImage(const sensor_msgs::ImageConstPtr &img_msg);

    ORB_SLAM3::System* mpSLAM;

    // queue<sensor_msgs::ImageConstPtr> imgRGB, imgD;
    queue<tuple<cv::Mat, cv::Mat, double>> imgRGBDBuf;
    std::mutex mBufMutex;
    void SyncWithImu();

    ImuGrabber *mpImuGb;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Inertial");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD_IMU,true);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb);
   
    ros::NodeHandle nh;

    ros::Subscriber sub_imu = nh.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);  
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/stereo/left/image_rect", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/stereo/left/depth", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(20), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mBufMutex.lock();
    std::cout << "NEW RGBD msg!" << std::endl;
    if (!imgRGBDBuf.empty())
        imgRGBDBuf.pop();
    if ((cv_ptrRGB->image.type()==0) && (cv_ptrD->image.type()==0))
    {
        imgRGBDBuf.push(make_tuple(cv_ptrRGB->image.clone(), cv_ptrD->image.clone(), cv_ptrRGB->header.stamp.toSec()));
    }
    else
    {
        // std::cout << "Image types! RGB: " << cv_ptrRGB->image.type() << "; Depth: " << cv_ptrD->image.type() << std::endl;
        imgRGBDBuf.push(make_tuple(cv_ptrRGB->image.clone(), cv_ptrD->image.clone(), cv_ptrRGB->header.stamp.toSec()));
    } 
    mBufMutex.unlock();

    //mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  //cout<<"here";
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}

void ImageGrabber::SyncWithImu()
{
  const double maxTimeDiff = 0.01;
  while(1)
  {
    pair<cv::Mat, cv::Mat> imRGBD;
    double tImRGBD = 0;
    if (!imgRGBDBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tImRGBD = get<2>(imgRGBDBuf.front());
      // std::cout << std::fixed << "T RGBD: " << tImRGBD << std::endl;

      if(tImRGBD>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;

      this->mBufMutex.lock();
      imRGBD = make_pair(get<0>(imgRGBDBuf.front()), get<1>(imgRGBDBuf.front()));
      imgRGBDBuf.pop();
      this->mBufMutex.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        std::cout << std::fixed << "T RGBD: " << tImRGBD << "; IMU: " << mpImuGb->imuBuf.front()->header.stamp.toSec() << std::endl;
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImRGBD)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          // std::cout << std::fixed << "T IMU current: " << t << std::endl;
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }

      std::cout << std::fixed << "IMU vector size: " << vImuMeas.size() << std::endl;
      mpImuGb->mBufMutex.unlock();

      mpSLAM->TrackRGBD(imRGBD.first,imRGBD.second,tImRGBD,vImuMeas);

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

