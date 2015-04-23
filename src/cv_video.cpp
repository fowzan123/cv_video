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

#include <cv_video/cv_video.h>

namespace enc = sensor_msgs::image_encodings;

namespace cv_video
{

std::string encoding(const cv::Mat& image)
{
  switch (image.type())
  {
    case CV_8UC1:
    {
      return enc::MONO8;
    }
    case CV_8UC3:
    {
      return enc::BGR8;
    }
    default:
    {
      throw cv_bridge::Exception("Incompatible encoding (not CV_8UC1 nor CV_8UC3)");
    }
  }
}

CvVideo::CvVideo():
    transport_(node_),
    topic_(node_.resolveName("image"))
{
  // Nothing to do.
}

CvVideo::CvVideo(const std::string& topic):
    transport_(node_),
    topic_(topic)
{
  // Nothing to do.
}

void CvVideo::Publisher::reset(const std::string& topic, image_transport::ImageTransport& transport)
{
  publisher_ = transport.advertise(topic, 100);
  topic_ = topic;
  index_ = 0;
}

void CvVideo::Publisher::publish(const cv::Mat& image)
{
  cv_bridge::CvImage bridged;
  bridged.header.seq = index_++;
  bridged.header.stamp = ros::Time::now();
  bridged.header.frame_id = topic_;
  bridged.image = image;
  bridged.encoding = encoding(image);

  publisher_.publish(bridged.toImageMsg());
}

void CvVideo::Publisher::publish(const cv_bridge::CvImagePtr& image)
{
  publisher_.publish(image->toImageMsg());
}

void CvVideo::Publisher::publish(const cv_bridge::CvImageConstPtr& image)
{
  publisher_.publish(image->toImageMsg());
}

void CvVideo::unpublish(const std::string& topic)
{
  publishers_.erase(topic);
}

static void subscriber(CvVideo::Callback callback, CvVideo* dispatcher, const sensor_msgs::ImageConstPtr& message)
{
  try
  {
    callback(*dispatcher, message);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR("OpenCV exception: %s", e.what());
  }
  catch (...)
  {
    ROS_ERROR("Unknown error");
  }
}

static void subscriberCvImagePtr(CvVideo::CallbackCvImagePtr callback,
                                 CvVideo& dispatcher,
                                 const sensor_msgs::ImageConstPtr& message)
{
  cv_bridge::CvImagePtr pointer = cv_bridge::toCvCopy(message, param<std::string>("~encoding", ""));
  callback(dispatcher, pointer);
}

static void subscriberCvImageConstPtr(CvVideo::CallbackCvImageConstPtr callback,
                                      CvVideo& dispatcher,
                                      const sensor_msgs::ImageConstPtr& message)
{
  cv_bridge::CvImageConstPtr pointer = cv_bridge::toCvShare(message, param<std::string>("~encoding", ""));
  callback(dispatcher, pointer);
}

static void subscriberMat(CvVideo::CallbackMat callback,
                          CvVideo& dispatcher,
                          const sensor_msgs::ImageConstPtr& message)
{
  cv_bridge::CvImagePtr pointer = cv_bridge::toCvCopy(message, param<std::string>("~encoding", ""));
  callback(dispatcher, pointer->image);
}

static void subscriberMatConst(CvVideo::CallbackMatConst callback,
                               CvVideo& dispatcher,
                               const sensor_msgs::ImageConstPtr& message)
{
  cv_bridge::CvImageConstPtr pointer = cv_bridge::toCvShare(message, param<std::string>("~encoding", ""));
  callback(dispatcher, pointer->image);
}

void CvVideo::subscribe(Callback callback)
{
  subscribe("", callback);
}

void CvVideo::subscribe(const std::string& name, Callback callback)
{
  subscribers_[name] = transport_.subscribe(topic_, 1, boost::bind(subscriber, callback, this, _1));
}

void CvVideo::subscribe(CallbackMat callback)
{
  subscribe("", callback);
}

void CvVideo::subscribe(const std::string& name, CallbackMat callback)
{
  Callback enclosed = boost::bind(subscriberMat, callback, _1, _2);
  subscribe(name, enclosed);
}

void CvVideo::subscribe(CallbackMatConst callback)
{
  subscribe("", callback);
}

void CvVideo::subscribe(const std::string& name, CallbackMatConst callback)
{
  Callback enclosed = boost::bind(subscriberMatConst, callback, _1, _2);
  subscribe(name, enclosed);
}

void CvVideo::subscribe(CallbackCvImagePtr callback)
{
  subscribe("", callback);
}

void CvVideo::subscribe(const std::string& name, CallbackCvImagePtr callback)
{
  Callback enclosed = boost::bind(subscriberCvImagePtr, callback, _1, _2);
  subscribe(name, enclosed);
}

void CvVideo::subscribe(CallbackCvImageConstPtr callback)
{
  subscribe("", callback);
}

void CvVideo::subscribe(const std::string& name, CallbackCvImageConstPtr callback)
{
  Callback enclosed = boost::bind(subscriberCvImageConstPtr, callback, _1, _2);
  subscribe(name, enclosed);
}

void CvVideo::unsubscribe(const std::string& name)
{
  subscribers_.erase(name);
}

} //namespace cv_video
