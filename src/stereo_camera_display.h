/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef RVIZ_STEREO_CAMERA_DISPLAY_H
#define RVIZ_STEREO_CAMERA_DISPLAY_H

#include <QObject>

#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>

#ifndef Q_MOC_RUN
# include <sensor_msgs/CameraInfo.h>

# include <message_filters/subscriber.h>
# include <tf/message_filter.h>

# include <rviz/image/image_display_base.h>
# include <rviz/image/ros_image_texture.h>
# include <rviz/render_panel.h>

# include "stereo_image_display_base.h"
#endif

namespace Ogre
{
class SceneNode;
class ManualObject;
class Rectangle2D;
class Camera;
}

namespace rviz
{

class EnumProperty;
class FloatProperty;
class IntProperty;
class RosTopicProperty;
class DisplayGroupVisibilityProperty;
class StereoImageRenderPanel;
}

namespace rviz
{

/**
 * \class StereoCameraDisplay
 *
 */
class StereoCameraDisplay: public StereoImageDisplayBase, public Ogre::RenderTargetListener
{
Q_OBJECT
public:
  /** \brief Create additional rviz panel options
   * - image position
   * - overlay alpha
   * - image zoom
   */
  StereoCameraDisplay();

  /** \brief deletes and re-sets orgre resources **/
  virtual ~StereoCameraDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();

  // Overrides from Ogre::RenderTargetListener
  virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
  virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );

  static const QString BACKGROUND;
  static const QString OVERLAY;
  static const QString BOTH;

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  ROSImageTexture left_texture_;
  ROSImageTexture right_texture_;
  StereoImageRenderPanel* render_panel_;

private Q_SLOTS:
  void forceRender();
  void updateAlpha();

  virtual void updateQueueSize();

private:
  void subscribe();
  void unsubscribe();

  virtual void processMessages(
      const sensor_msgs::Image::ConstPtr& left_msg,
      const sensor_msgs::Image::ConstPtr& right_msg);
  void leftCaminfoCallback( 
    const sensor_msgs::CameraInfo::ConstPtr& msg);
  void rightCaminfoCallback( 
    const sensor_msgs::CameraInfo::ConstPtr& msg);

  bool updateCamera(
      ROSImageTexture &texture, 
      sensor_msgs::CameraInfo::ConstPtr &current_caminfo,
      Ogre::Rectangle2D* &bg_screen_rect,
      Ogre::Rectangle2D* &fg_screen_rect,
      Ogre::Camera* camera);

  void clear();
  void updateStatus();

  // Scene node for 3d view
  Ogre::SceneNode* bg_scene_node_;
  Ogre::SceneNode* fg_scene_node_;

  Ogre::Vector2 right_offset_;

  // Image rect for camera image underlaying
  Ogre::Rectangle2D* left_bg_screen_rect_;
  Ogre::Rectangle2D* right_bg_screen_rect_;
  Ogre::MaterialPtr left_bg_material_;
  Ogre::MaterialPtr right_bg_material_;

  // Image rect for camera image overlaying
  Ogre::Rectangle2D* left_fg_screen_rect_;
  Ogre::Rectangle2D* right_fg_screen_rect_;
  Ogre::MaterialPtr left_fg_material_;
  Ogre::MaterialPtr right_fg_material_;

  message_filters::Subscriber<sensor_msgs::CameraInfo> left_caminfo_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> right_caminfo_sub_;
  tf::MessageFilter<sensor_msgs::CameraInfo>* left_caminfo_tf_filter_;
  tf::MessageFilter<sensor_msgs::CameraInfo>* right_caminfo_tf_filter_;

  FloatProperty* alpha_property_;
  EnumProperty* image_position_property_;
  FloatProperty* zoom_property_;
  DisplayGroupVisibilityProperty* visibility_property_;

  sensor_msgs::CameraInfo::ConstPtr left_current_caminfo_;
  sensor_msgs::CameraInfo::ConstPtr right_current_caminfo_;
  boost::mutex caminfo_mutex_;

  bool left_new_caminfo_;
  bool right_new_caminfo_;

  bool caminfo_ok_;

  bool force_render_;

  uint32_t vis_bit_;
};

} // namespace rviz

 #endif
