/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Helio Perroni Filho.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef CV_VIDEO_CAMERA_H
#define CV_VIDEO_CAMERA_H

#include <cv_video/cv_video.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

namespace cv_video
{

class Camera
{
  /** \brief Reference to the current ROS node. */
  ros::NodeHandle node_;

  /** \brief Object used to communicate with the service. */
  ros::ServiceClient client_;

  /** \brief Data exchanged with the service. */
  cv_video::Record exchange_;
  
  /** \brief Connection to video feed. */
  CvVideo::Ptr dispatcher_;
  
  /** \brief Last frame recovered from the video feed, if any. */
  cv::Mat frame_;
  
  /**
   * \brief Callback function for retrieving images from a video topic.
   */
  void callback(CvVideo& dispatcher, const cv::Mat& image);

public:    
  /** \brief Default constructor. */
  Camera();

  /** \brief Creates a new camera and start recording immediately. */
  Camera(const std::string& topic,
         const std::string& path,
         const std::string& format,
         double fps,
         int width,
         int height);

  /** \brief Object destructor. */
  virtual ~Camera();

  /**
   * \brief Returns a single frame from the video feed.
   */
  cv::Mat operator () ();
  
  /**
   * \brief Returns a single frame from the video feed.
   */
  cv::Mat grab();
  
  /**
   * \brief Connect this camera object to the default video feed.
   */
  void open();

  /**
   * \brief Connect this camera object to the given video feed.
   */
  void open(const std::string& topic);

  /**
   * \brief Disconnect this camera object from the current video feed.
   */
  void close();

  /**
   * \brief Starts a new recording.
   * 
   * Recording parameters are taken from environment parameters, or if these are unavailable,
   * set to reasonable defaults.
   */
  void record();

  /** \brief Starts a new recording. */
  void record(const std::string& path,
              const std::string& format,
              double fps,
              int width,
              int height);

  /** \brief Sets the video feed topic and starts a new recording. */
  void record(const std::string& topic,
              const std::string& path,
              const std::string& format,
              double fps,
              int width,
              int height);

  /** \brief Stops an ongoing recording. */
  void stop();
};

} // namespace cv_video

#endif
