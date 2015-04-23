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

#ifndef CV_VIDEO_VIDEO_WRITER_H
#define CV_VIDEO_VIDEO_WRITER_H

#include <cv_video/cv_video.h>

namespace cv_video
{

class VideoWriter
{
  /** \brief Access to the image channel. */
  CvVideo dispatcher_;
  
  /** \brief Video recorder object. */
  cv::VideoWriter recorder_;

  /** \brief Callback for the image channel. */
  void record(CvVideo& dispatcher, const cv::Mat& image);
  
public:
  /** \brief Smart pointer type alias. */
  typedef boost::shared_ptr<VideoWriter> Ptr;
  
  /** \brief Creates a new video writer from environment parameters. */
  VideoWriter();
  
  /** \brief Creates a new video writer from given arguments. */
  VideoWriter(const std::string& topic,
              const std::string& path,
              const std::string& format,
              double fps,
              int width,
              int height);
  
  /** \brief Creates a new video writer from a service request. */
  VideoWriter(Record::Request& request);
  
  /** \brief Start a recording session. */
  void start(const std::string& topic,
             const std::string& path,
             const std::string& format,
             double fps,
             int width,
             int height);
};

} // namespace cv_video

#endif
