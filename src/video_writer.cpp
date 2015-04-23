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

#include <cv_video/video_writer.h>

#include <cv_bridge/cv_bridge.h>

namespace enc = sensor_msgs::image_encodings;

namespace cv_video
{
  
VideoWriter::VideoWriter()
{
  Record::Request request = params();
  start(
    request.topic,
    request.path,
    request.format,
    request.fps,
    request.width,
    request.height
  );
}

VideoWriter::VideoWriter(const std::string& topic,
                         const std::string& path,
                         const std::string& format,
                         double fps,
                         int width,
                         int height)
{
  start(topic, path, format, fps, width, height);
}

VideoWriter::VideoWriter(Record::Request& request)
{
  start(request.topic,
        request.path,
        request.format,
        request.fps,
        request.width,
        request.height);
}

void VideoWriter::record(CvVideo& dispatcher, const cv::Mat& image)
{
  recorder_.write(image);
}

void VideoWriter::start(const std::string& topic,
                        const std::string& path,
                        const std::string& format,
                        double fps,
                        int width,
                        int height)
{
  int fourcc = CV_FOURCC(format[0], format[1], format[2], format[3]);
  recorder_.open(path, fourcc, fps, cv::Size(width, height));
  dispatcher_.subscribe(topic, &VideoWriter::record, this);
}
  
} // namespace cv_video
