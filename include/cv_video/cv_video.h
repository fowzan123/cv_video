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

#ifndef CV_VIDEO_CV_VIDEO_H
#define CV_VIDEO_CV_VIDEO_H

#include <cv_video/environment.h>

#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>

#include <string>

namespace cv_video
{

/**
 * \brief Select the appropriate cv_bridge encoding for the given image.
 */
std::string encoding(const cv::Mat& image);

class CvVideo
{
  struct Publisher
  {
    /** \brief Image publisher. */
    image_transport::Publisher publisher_;

    /** \brief Topic under which to publish images. */
    std::string topic_;

    /** \brief Incrementing message ID. */
    uint32_t index_;

    /** \brief Publish the given image. */
    void publish(const cv::Mat& image);

    /** \brief Publish the given image. */
    void publish(const cv_bridge::CvImagePtr& image);

    /** \brief Publish the given image. */
    void publish(const cv_bridge::CvImageConstPtr& image);
    
    /** \brief Setup the publisher. */
    void reset(const std::string& topic, image_transport::ImageTransport& transport);
  };

  /** \brief Reference to the current ROS node. */
  ros::NodeHandle node_;

  /** \brief Image topic to listen to. */
  std::string topic_;

  /** \brief Access to the image channel. */
  image_transport::ImageTransport transport_;

  /** \brief Image topic subscribers. */
  std::map<std::string, image_transport::Subscriber> subscribers_;
  
  /** \brief Image topic publishers. */
  std::map<std::string, Publisher> publishers_;

public:
  /** \brief Smart pointer type alias. */
  typedef boost::shared_ptr<CvVideo> Ptr;
  
  /** \brief Callback function type. */
  typedef boost::function<void(CvVideo&, const sensor_msgs::ImageConstPtr& message)> Callback;

  typedef boost::function<void(CvVideo&, const cv_bridge::CvImagePtr&)> CallbackCvImagePtr;

  typedef boost::function<void(CvVideo&, const cv_bridge::CvImageConstPtr&)> CallbackCvImageConstPtr;

  typedef boost::function<void(CvVideo&, cv::Mat&)> CallbackMat;

  typedef boost::function<void(CvVideo&, const cv::Mat&)> CallbackMatConst;

  /** \brief Create a new object bound to the default \c image topic. */
  CvVideo();
  
  /** \brief Create a new object bound to the given topic. */
  CvVideo(const std::string& topic);
  
  /**
   * \brief Publish the given image to the given image topic.
   */
  template<class T>
  void publish(const std::string& topic, const T& image);
  
  /**
   * \brief Removes the publisher for the given topic.
   */
  void unpublish(const std::string& topic);

  /**
   * \brief Subscribe the given default callback.
   *
   * The default callback is stored under the empty name \c "".
   */
  void subscribe(Callback callback);

  /**
   * \brief Subscribe the given named callback.
   *
   * A named callback can later be unsubscribed by its name.
   */
  void subscribe(const std::string& name, Callback callback);

  /**
   * \brief Subscribe the given default callback.
   *
   * The default callback is stored under the empty name \c "".
   */
  void subscribe(CallbackCvImagePtr callback);

  /**
   * \brief Subscribe the given named callback.
   *
   * A named callback can later be unsubscribed by its name.
   */
  void subscribe(const std::string& name, CallbackCvImagePtr callback);

  /**
   * \brief Subscribe the given default callback.
   *
   * The default callback is stored under the empty name \c "".
   */
  void subscribe(CallbackCvImageConstPtr callback);

  /**
   * \brief Subscribe the given named callback.
   *
   * A named callback can later be unsubscribed by its name.
   */
  void subscribe(const std::string& name, CallbackCvImageConstPtr callback);

  /**
   * \brief Subscribe the given default callback.
   *
   * The default callback is stored under the empty name \c "".
   */
  void subscribe(CallbackMat callback);

  /**
   * \brief Subscribe the given named callback.
   *
   * A named callback can later be unsubscribed by its name.
   */
  void subscribe(const std::string& name, CallbackMat callback);
  
  /**
   * \brief Subscribe the given default callback.
   *
   * The default callback is stored under the empty name \c "".
   */
  void subscribe(CallbackMatConst callback);

  /**
   * \brief Subscribe the given named callback.
   *
   * A named callback can later be unsubscribed by its name.
   */
  void subscribe(const std::string& name, CallbackMatConst callback);
  
  /**
   * \brief Subscribe the given object callback as default.
   */
  template<class T>
  void subscribe(void(T::*callback)(CvVideo& dispatcher, const cv::Mat&), T* object);
  
  /**
   * \brief Subscribe the given object callback under the given name.
   */
  template<class T>
  void subscribe(const std::string& name,
                 void(T::*callback)(CvVideo& dispatcher, const cv::Mat&),
                 T* object);
  
  /**
   * \brief Unsubscribe the given named callback.
   *
   * If the \c name parameter is omitted, the default callback is unsubscribed.
   */
  void unsubscribe(const std::string& name = "");
};

template<class T>
void CvVideo::publish(const std::string& topic, const T& image)
{
  if (publishers_.count(topic) == 0)
    publishers_[topic].reset(topic, transport_);

  publishers_[topic].publish(image);
}

template<class T>
void CvVideo::subscribe(void(T::*callback)(CvVideo& dispatcher, const cv::Mat&), T* object)
{
  boost::function<void(T*, CvVideo&, const cv::Mat&)> member_function = callback;
  subscribe(boost::bind(member_function, object, _1, _2));
}

template<class T>
void CvVideo::subscribe(const std::string& name,
                        void(T::*callback)(CvVideo& dispatcher, const cv::Mat&),
                        T* object)
{
  boost::function<void(T*, CvVideo&, const cv::Mat&)> member_function = callback;
  CallbackMatConst enclosed = boost::bind(member_function, object, _1, _2);
  subscribe(name, enclosed);
}

} //namespace cv_video

#endif
