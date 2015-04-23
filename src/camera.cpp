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

#include <cv_video/camera.h>

#include <stdexcept>

namespace cv_video
{

Camera::Camera()
{
  exchange_.request.topic = "";
  exchange_.request.path = "";
}

Camera::Camera(const std::string& topic,
               const std::string& path,
               const std::string& format,
               double fps,
               int width,
               int height)
{
  record(topic, path, format, fps, width, height);
}

Camera::~Camera()
{
  stop();
}

cv::Mat Camera::operator () ()
{
  return grab();
}

void Camera::callback(CvVideo& dispatcher, const cv::Mat& image)
{
  frame_ = image;
}

cv::Mat Camera::grab()
{
  frame_ = cv::Mat();
  if (exchange_.request.topic == "")
    return frame_;

  if (dispatcher_.get() == NULL)
  {
    dispatcher_.reset(new CvVideo());
    dispatcher_->subscribe(exchange_.request.topic, &Camera::callback, this);
  }
  
  for (ros::Rate rate(params().fps); ros::ok();)
  {
    ros::spinOnce();
    if (!frame_.empty())
      return frame_;
    
    rate.sleep();
  }
}

void Camera::open()
{
  open(node_.resolveName("image"));
}

void Camera::open(const std::string& topic)
{
  dispatcher_.reset();
  exchange_.request.topic = topic;
}

void Camera::close()
{
  stop();
  dispatcher_.reset();
  exchange_.request.topic = "";
}

void Camera::record()
{
  Record::Request request = params();

  if (exchange_.request.topic == "")
    open(request.topic);

  record(request.path,
         request.format,
         request.fps,
         request.width,
         request.height);
}

void Camera::record(const std::string& path,
                    const std::string& format,
                    double fps,
                    int width,
                    int height)
{
    exchange_.request.path = path;
    exchange_.request.format = format;
    exchange_.request.fps = fps;
    exchange_.request.width = width;
    exchange_.request.height = height;

    std::string name = node_.resolveName("camcorder");
    client_ = node_.serviceClient<cv_video::Record>(name);
    if (!client_.call(exchange_))
      throw std::runtime_error(exchange_.response.status);
}

void Camera::record(const std::string& topic,
                    const std::string& path,
                    const std::string& format,
                    double fps,
                    int width,
                    int height)
{
  open(topic);
  record(path, format, fps, width, height);
}

void Camera::stop()
{
  if (exchange_.request.path == "")
    return;

  exchange_.request.path = "";

  if (!client_.call(exchange_))
    ROS_ERROR("Error stopping video record: %s", exchange_.response.status.c_str());
}

} // namespace cv_video
