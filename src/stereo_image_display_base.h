/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef STEREO_IMAGE_DISPLAY_BASE_H
#define STEREO_IMAGE_DISPLAY_BASE_H

#include <QObject>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <message_filters/subscriber.h>
# include <tf/message_filter.h>
# include <sensor_msgs/Image.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

# include <image_transport/image_transport.h>
# include <image_transport/subscriber_filter.h>

# include <rviz/display_context.h>
# include <rviz/frame_manager.h>
# include <rviz/properties/ros_topic_property.h>
# include <rviz/properties/enum_property.h>
# include <rviz/properties/int_property.h>

# include <rviz/display.h>

#endif

namespace rviz
{
/** @brief Display subclass for subscribing and displaying to image messages.
 *
 * This class brings together some common things used for subscribing and displaying image messages in Display
 * types.  It has a tf::MessageFilter and image_tranport::SubscriberFilter to filter incoming image messages, and
 * it handles subscribing and unsubscribing when the display is
 * enabled or disabled.  */

class StereoImageDisplayBase : public rviz::Display
{
Q_OBJECT
public:

  /** @brief Constructor. */
  StereoImageDisplayBase();
  virtual ~StereoImageDisplayBase();

  virtual void setLeftTopic( const QString &topic, const QString &datatype );
  virtual void setRightTopic( const QString &topic, const QString &datatype );

protected Q_SLOTS:
  /** @brief Update topic and resubscribe */
  virtual void updateTopics();

  /** @brief Update queue size of tf filter  */
  virtual void updateQueueSize();

  /** @brief Fill list of available and working transport options */
  void fillTransportOptionList(EnumProperty* property);

protected:
  virtual void onInitialize();

  /** @brief Reset display. */
  virtual void reset();

  /** @brief Enabling TF filtering by defining a target frame. */
  void enableTFFilter(std::string& targetFrame)
  {
    targetFrame_ = targetFrame;
    reset();
  }

  /** @brief ROS topic management. */
  virtual void subscribe();
  virtual void unsubscribe();

  virtual void fixedFrameChanged();

  /** @brief Incoming message callback.  Checks if the message pointer
   * is valid, increments messages_received_, then calls
   * processMessage(). */
  void incomingMessages(
      const sensor_msgs::Image::ConstPtr& left_msg,
      const sensor_msgs::Image::ConstPtr& right_msg);

  /** @brief Implement this to process the contents of a message.
   *
   * This is called by incomingMessage(). */
  virtual void processMessages(
      const sensor_msgs::Image::ConstPtr& left_msg,
      const sensor_msgs::Image::ConstPtr& right_msg) = 0;

  void scanForTransportSubscriberPlugins();

  boost::scoped_ptr<image_transport::ImageTransport> it_;
  boost::shared_ptr<image_transport::SubscriberFilter> 
    left_sub_,
    right_sub_;
  boost::shared_ptr<tf::MessageFilter<sensor_msgs::Image> > tf_filter_;

  std::string targetFrame_;

  uint32_t messages_received_;

  rviz::RosTopicProperty 
    *left_topic_property_,
    *right_topic_property_;
  rviz::EnumProperty 
    *transport_property_;
  rviz::IntProperty *queue_size_property_;

  std::string transport_;

  std::set<std::string> transport_plugin_types_;

  typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> StereoSyncPolicy;

  boost::shared_ptr<StereoSyncPolicy> sync_;
};

} // end namespace rviz

#endif // IMAGE_DISPLAY_BASE_H
