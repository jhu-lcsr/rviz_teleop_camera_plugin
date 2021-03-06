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

#include <boost/algorithm/string/erase.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/typeof/std/utility.hpp>

#include <pluginlib/class_loader.h>

#include <image_transport/subscriber_plugin.h>

#include "stereo_image_display_base.h"

namespace rviz
{

StereoImageDisplayBase::StereoImageDisplayBase() :
    Display()
    , left_sub_()
    , right_sub_()
    , left_tf_filter_()
    , right_tf_filter_()
    , messages_received_(0)
    , use_approx_sync_(false)
{
  left_topic_property_ = new RosTopicProperty("Left Image Topic", "",
                                              QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                              "sensor_msgs::Image topic to subscribe to.", this, SLOT( updateTopics() ));
  right_topic_property_ = new RosTopicProperty("Right Image Topic", "",
                                               QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                               "sensor_msgs::Image topic to subscribe to.", this, SLOT( updateTopics() ));

  transport_property_ = new EnumProperty(
      "Transport Hint", "raw", "Preferred method of sending images.", 
      this, SLOT( updateTopics() ));

  bool ok = connect(transport_property_,
          SIGNAL( requestOptions( EnumProperty* )), 
          this, SLOT( fillTransportOptionList( EnumProperty* )));

  Q_ASSERT(ok);

  queue_size_property_ = new IntProperty( "Queue Size", 2,
                                          "Advanced: set the size of the incoming message queue.  Increasing this "
                                          "is useful if your incoming TF data is delayed significantly from your"
                                          " image data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));
  queue_size_property_->setMin( 1 );

  approx_sync_property_ = new BoolProperty( "Approximate Sync", false,
                                          "Advanced: set this to true if your timestamps aren't synchronized.",
                                          this, SLOT( updateApproxSync() ));

  transport_property_->setStdString("raw");

}

StereoImageDisplayBase::~StereoImageDisplayBase()
{
  unsubscribe();
}

void StereoImageDisplayBase::onInitialize()
{
  it_.reset( new image_transport::ImageTransport( update_nh_ ));
  scanForTransportSubscriberPlugins();
}

void StereoImageDisplayBase::setLeftTopic( const QString &topic, const QString &datatype )
{
  if ( datatype == ros::message_traits::datatype<sensor_msgs::Image>() )
  {
    transport_property_->setStdString( "raw" );
    left_topic_property_->setString( topic );
  }
  else
  {
    int index = topic.lastIndexOf("/");
    if ( index == -1 )
    {
      ROS_WARN("StereoImageDisplayBase::setTopic() Invalid topic name: %s",
               topic.toStdString().c_str());
      return;
    }
    QString transport = topic.mid(index + 1);
    QString base_topic = topic.mid(0, index);

    transport_property_->setString( transport );
  }
}

void StereoImageDisplayBase::setRightTopic( const QString &topic, const QString &datatype )
{
  if ( datatype == ros::message_traits::datatype<sensor_msgs::Image>() )
  {
    transport_property_->setStdString( "raw" );
    right_topic_property_->setString( topic );
  }
  else
  {
    int index = topic.lastIndexOf("/");
    if ( index == -1 )
    {
      ROS_WARN("StereoImageDisplayBase::setTopic() Invalid topic name: %s",
               topic.toStdString().c_str());
      return;
    }
    QString transport = topic.mid(index + 1);
    QString base_topic = topic.mid(0, index);

    transport_property_->setString( transport );
    right_topic_property_->setString( base_topic );
  }
}

void StereoImageDisplayBase::incomingMessages(
    const sensor_msgs::Image::ConstPtr& left_msg,
    const sensor_msgs::Image::ConstPtr& right_msg)
{
  if (!left_msg || !right_msg || context_->getFrameManager()->getPause() )
  {
    return;
  }

  ++messages_received_;
  setStatus(StatusProperty::Ok, "Image", QString::number(messages_received_) + " images received");

  emitTimeSignal( left_msg->header.stamp );

  processMessages(left_msg, right_msg);
}


void StereoImageDisplayBase::reset()
{
  Display::reset();
  if (left_tf_filter_ && right_tf_filter_) {
    left_tf_filter_->clear();
    right_tf_filter_->clear();
  }
  messages_received_ = 0;
}

void StereoImageDisplayBase::updateQueueSize()
{
  uint32_t size = queue_size_property_->getInt();
  if (left_tf_filter_ && right_tf_filter_) {
    left_tf_filter_->setQueueSize(size);
    right_tf_filter_->setQueueSize(size);
  }
}

void StereoImageDisplayBase::updateApproxSync()
{
  use_approx_sync_ = approx_sync_property_->getBool();
  this->updateTopics();
}


void StereoImageDisplayBase::subscribe()
{
  if (!isEnabled())
  {
    ROS_ERROR_STREAM("Can't subscribe because this display element isn't enabled.");
    return;
  }

  try
  {
    tf_approx_sync_.reset();
    tf_sync_.reset();

    left_tf_filter_.reset();
    right_tf_filter_.reset();

    if(!left_sub_)
      left_sub_.reset(new image_transport::SubscriberFilter());
    if(!right_sub_)
      right_sub_.reset(new image_transport::SubscriberFilter());

    ROS_INFO_STREAM(
        "Subscribing to "
        <<left_topic_property_->getTopicStd()
        <<" transported as "<<transport_property_->getStdString());
    ROS_INFO_STREAM(
        "Subscribing to "
        <<right_topic_property_->getTopicStd()
        <<" transported as "<<transport_property_->getStdString());

    if (!left_topic_property_->getTopicStd().empty() &&
        !right_topic_property_->getTopicStd().empty() && 
        !transport_property_->getStdString().empty() )
    {
      left_sub_->subscribe(
          *it_, 
          left_topic_property_->getTopicStd(),
          (uint32_t)queue_size_property_->getInt(),
          image_transport::TransportHints(transport_property_->getStdString()));

      right_sub_->subscribe(
          *it_, 
          right_topic_property_->getTopicStd(),
          (uint32_t)queue_size_property_->getInt(),
          image_transport::TransportHints(transport_property_->getStdString()));

      if (targetFrame_.empty())
      {
        if(use_approx_sync_) {
          if(!approx_sync_) {
            approx_sync_.reset(new StereoApproxSync(StereoApproxSyncPolicy(10), *left_sub_, *right_sub_));
            BOOST_AUTO(f,boost::bind(&StereoImageDisplayBase::incomingMessages, this, _1, _2));
            approx_sync_->registerCallback(f);
          } 
        } else {
          if(!sync_) {
            sync_.reset(new StereoSync(*left_sub_, *right_sub_, 10));
            BOOST_AUTO(f,boost::bind(&StereoImageDisplayBase::incomingMessages, this, _1, _2));
            sync_->registerCallback(f);
          }
        }
      }
      else
      {
        // TODO: Pipe output of message synchronizer into tf messagefilter
        // or maybe pipe output of tf messagefilter into time synchronizer?
        // http://docs.ros.org/hydro/api/tf/html/c++/classtf_1_1MessageFilter.html#abcaaa2112aede9acd8cddbd2acda75e2

        left_tf_filter_.reset(
            new tf::MessageFilter<sensor_msgs::Image>(
                *left_sub_,
                (tf::Transformer&)*(context_->getTFClient()), 
                targetFrame_, 
                (uint32_t)queue_size_property_->getInt(), 
                update_nh_));
        right_tf_filter_.reset(
            new tf::MessageFilter<sensor_msgs::Image>(
                *right_sub_,
                (tf::Transformer&)*(context_->getTFClient()), 
                targetFrame_, 
                (uint32_t)queue_size_property_->getInt(), 
                update_nh_));


        //tf_filter_->registerCallback(boost::bind(&StereoImageDisplayBase::incomingMessage, this, _1));
        if(use_approx_sync_) {
          if(!tf_approx_sync_) {
            tf_approx_sync_.reset(new TFStereoApproxSync(TFStereoApproxSyncPolicy(10), *left_tf_filter_, *right_tf_filter_));
            tf_approx_sync_->registerCallback(boost::bind(&StereoImageDisplayBase::incomingMessages, this, _1, _2));
          } 
        } else {
          if(!tf_sync_) {
            tf_sync_.reset(new TFStereoSync(*left_tf_filter_, *right_tf_filter_, 10));
            tf_sync_->registerCallback(boost::bind(&StereoImageDisplayBase::incomingMessages, this, _1, _2));
          }
        }
      }
    }
    setStatus(StatusProperty::Ok, "Topic", "OK");
  }
  catch ( ros::Exception& e)
  {
    setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }
  catch (image_transport::Exception& e)
  {
    setStatus( StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }

  messages_received_ = 0;
  setStatus(StatusProperty::Warn, "Image", "No Image received through TF message filter");
}

void StereoImageDisplayBase::unsubscribe()
{
  if (left_tf_filter_ && right_tf_filter_ && left_sub_ && right_sub_) {
    tf_approx_sync_.reset();
    tf_sync_.reset();
    left_tf_filter_.reset();
    right_tf_filter_.reset();
    left_sub_->unsubscribe();
    right_sub_->unsubscribe();
    left_sub_.reset();
    right_sub_.reset();
  }
}

void StereoImageDisplayBase::fixedFrameChanged()
{
  if (left_tf_filter_ && right_tf_filter_) {
    left_tf_filter_->setTargetFrame(fixed_frame_.toStdString());
    right_tf_filter_->setTargetFrame(fixed_frame_.toStdString());
    reset();
  }
}

void StereoImageDisplayBase::scanForTransportSubscriberPlugins()
{
  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader("image_transport",
                                                                       "image_transport::SubscriberPlugin");

  BOOST_FOREACH( const std::string& lookup_name, sub_loader.getDeclaredClasses() )
  {
    // lookup_name is formatted as "pkg/transport_sub", for instance
    // "image_transport/compressed_sub" for the "compressed"
    // transport.  This code removes the "_sub" from the tail and
    // everything up to and including the "/" from the head, leaving
    // "compressed" (for example) in transport_name.
    std::string transport_name = boost::erase_last_copy(lookup_name, "_sub");
    transport_name = transport_name.substr(lookup_name.find('/') + 1);

    // If the plugin loads without throwing an exception, add its
    // transport name to the list of valid plugins, otherwise ignore
    // it.
    try
    {
      boost::shared_ptr<image_transport::SubscriberPlugin> sub = sub_loader.createInstance(lookup_name);
      transport_plugin_types_.insert(transport_name);
    }
    catch (const pluginlib::LibraryLoadException& e)
    {
    }
    catch (const pluginlib::CreateClassException& e)
    {
    }
  }
}

void StereoImageDisplayBase::updateTopics()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void StereoImageDisplayBase::fillTransportOptionList(EnumProperty* property)
{
  ROS_WARN_STREAM("Filling stereo transport option list...");
  property->clearOptions();

  std::vector<std::string> choices;

  choices.push_back("raw");

  // Loop over all current ROS topic names
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  ros::master::V_TopicInfo::iterator it = topics.begin();
  ros::master::V_TopicInfo::iterator end = topics.end();
  for (; it != end; ++it)
  {
    // If the beginning of this topic name is the same as topic_,
    // and the whole string is not the same,
    // and the next character is /
    // and there are no further slashes from there to the end,
    // then consider this a possible transport topic.
    const ros::master::TopicInfo& ti = *it;
    const std::string& topic_name = ti.name;
    const std::string& topic = left_topic_property_->getStdString();

    if (topic_name.find(topic) == 0 && topic_name != topic && topic_name[topic.size()] == '/'
        && topic_name.find('/', topic.size() + 1) == std::string::npos)
    {
      std::string transport_type = topic_name.substr(topic.size() + 1);

      // If the transport type string found above is in the set of
      // supported transport type plugins, add it to the list.
      if (transport_plugin_types_.find(transport_type) != transport_plugin_types_.end())
      {
        choices.push_back(transport_type);
      } else {
        ROS_WARN_STREAM("Not a transport type: " <<transport_type);
      }
    }
  }

  for (size_t i = 0; i < choices.size(); i++)
  {
    property->addOptionStd(choices[i]);
  }
}

} // end namespace rviz
