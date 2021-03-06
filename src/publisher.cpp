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

#include <cv_video/publisher.h>

#include <cv_video/settings.h>

namespace cv_video
{

Publisher::Publisher():
  index_(0)
{
  // Nothing to do.
}

Publisher::Publisher(image_transport::ImageTransport& transport, std::string topic, bool latch):
  publisher_(transport.advertise(topic, param::framerate(), latch)),
  index_(0)
{
  // Nothing to do.
}

Publisher::~Publisher()
{
  // Nothing to do.
}

void Publisher::publish(const cv::Mat& image)
{
  cv_bridge::CvImage bridged;
  bridged.header.seq = index_++;
  bridged.header.stamp = ros::Time::now();
  bridged.header.frame_id = "";
  bridged.image = image;
  bridged.encoding = encoding(image);

  publisher_.publish(bridged.toImageMsg());
}

void Publisher::publish(const cv_bridge::CvImagePtr& image)
{
  publisher_.publish(image->toImageMsg());
}

void Publisher::publish(const cv_bridge::CvImageConstPtr& image)
{
  publisher_.publish(image->toImageMsg());
}

void Publisher::publish(const Frame& frame)
{
  publisher_.publish(frame.buffer());
}

} //namespace cv_video
