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
using cv_video::VideoWriter;

#include <cv_video/Record.h>
using cv_video::Record;

#include <cv_bridge/cv_bridge.h>

#include <map>

struct Recorder
{
  /** \brief ROS node handler. */
  ros::NodeHandle node_;

  /** \brief Service advertiser object. */
  ros::ServiceServer service_;

  /** \brief Video feeds currently being recorded. */
  std::map<std::string, VideoWriter::Ptr> feeds_;

  Recorder()
  {
    std::string topic = node_.resolveName("camcorder");
    service_ = node_.advertiseService(topic, &Recorder::record, this);
  }

  bool record(Record::Request& request, Record::Response& response)
  {
    ROS_INFO_STREAM("Request: record " << request.topic << " on \"" << request.path << '"');

    try
    {
      feeds_[request.topic].reset(request.path != "" ? new VideoWriter(request) : NULL);
      response.status = "";
      return true;
    }
    catch (cv_bridge::Exception& e)
    {
      response.status = std::string("cv_bridge exception: ") + e.what();
    }
    catch (cv::Exception& e)
    {
      response.status = std::string("OpenCV exception: ") + e.what();
    }
    catch (ros::Exception& e)
    {
      response.status = std::string("ROS exception: ") + e.what();
    }
    catch (std::exception& e)
    {
      response.status = std::string("Base exception: ") + e.what();
    }
    catch (...)
    {
      response.status = "unknown error";
    }

    return false;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recorder");
  Recorder recorder;
  ros::spin();
  return 0;
}
