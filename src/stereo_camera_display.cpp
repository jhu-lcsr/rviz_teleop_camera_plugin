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

#include <boost/bind.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "rviz/bit_allocator.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/render_panel.h"
#include "rviz/uniform_string_stream.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"
#include "rviz/properties/display_group_visibility_property.h"
#include "rviz/load_resource.h"

#include <image_transport/camera_common.h>

#include "stereo_camera_display.h"
#include "stereo_image_render_panel.h"

namespace rviz
{

const QString StereoCameraDisplay::BACKGROUND( "background" );
const QString StereoCameraDisplay::OVERLAY( "overlay" );
const QString StereoCameraDisplay::BOTH( "background and overlay" );

bool validateFloats(const sensor_msgs::CameraInfo& msg)
{
  bool valid = true;
  valid = valid && validateFloats( msg.D );
  valid = valid && validateFloats( msg.K );
  valid = valid && validateFloats( msg.R );
  valid = valid && validateFloats( msg.P );
  return valid;
}

StereoCameraDisplay::StereoCameraDisplay()
  : StereoImageDisplayBase()
  , left_texture_()
  , right_texture_()
  , render_panel_( 0 )
  , left_caminfo_tf_filter_( 0 )
  , right_caminfo_tf_filter_( 0 )
  , left_new_caminfo_( false )
  , right_new_caminfo_( false )
  , force_render_( false )
  , caminfo_ok_(false)
{
  image_position_property_ = new EnumProperty( "Image Rendering", OVERLAY,
                                               "Render the image behind all other geometry or overlay it on top, or both.",
                                               this, SLOT( forceRender() ));
  image_position_property_->addOption( BACKGROUND );
  image_position_property_->addOption( OVERLAY );
  image_position_property_->addOption( BOTH );

  alpha_property_ = new FloatProperty( "Overlay Alpha", 0.5,
                                       "The amount of transparency to apply to the camera image when rendered as overlay.",
                                       this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0 );
  alpha_property_->setMax( 1 );

  zoom_property_ = new FloatProperty( "Zoom Factor", 1.0,
                                      "Set a zoom factor below 1 to see a larger part of the world, above 1 to magnify the image.",
                                      this, SLOT( forceRender() ));
  zoom_property_->setMin( 0.00001 );
  zoom_property_->setMax( 100000 );
}

StereoCameraDisplay::~StereoCameraDisplay()
{
  if ( initialized() )
  {
    render_panel_->getRenderWindow()->removeListener( this );

    unsubscribe();
    left_caminfo_tf_filter_->clear();
    right_caminfo_tf_filter_->clear();

    //workaround. delete results in a later crash
    //render_panel_->hide();
    delete render_panel_;

    // Remove Ogre rects and scene nodes
    delete left_bg_screen_rect_;
    delete right_bg_screen_rect_;
    delete left_fg_screen_rect_;
    delete right_fg_screen_rect_;

    bg_scene_node_->getParentSceneNode()->removeAndDestroyChild( bg_scene_node_->getName() );
    fg_scene_node_->getParentSceneNode()->removeAndDestroyChild( fg_scene_node_->getName() );

    // Clean up
    delete left_caminfo_tf_filter_;
    delete right_caminfo_tf_filter_;

    // Disable ogre rendering of this context
    context_->visibilityBits()->freeBits(vis_bit_);
  }
}

static void create_material(
    const std::string &material_name, 
    ROSImageTexture &texture, 
    Ogre::MaterialPtr &material) 
{
  material = Ogre::MaterialManager::getSingleton().create( 
      material_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );

  material->setDepthWriteEnabled(false);
  material->setReceiveShadows(false);
  material->setDepthCheckEnabled(false);
  material->getTechnique(0)->setLightingEnabled(false);

  Ogre::TextureUnitState* tu = material->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureName(texture.getTexture()->getName());
  tu->setTextureFiltering( Ogre::TFO_NONE );
  tu->setAlphaOperation( 
      Ogre::LBX_SOURCE1, 
      Ogre::LBS_MANUAL, 
      Ogre::LBS_CURRENT, 
      0.0 );

  material->setCullingMode(Ogre::CULL_NONE);
  material->setSceneBlending( Ogre::SBT_REPLACE );
}

static void create_rect(
    Ogre::Rectangle2D* &screen_rect,
    Ogre::SceneNode* &scene_node,
    Ogre::MaterialPtr &material)
{
  screen_rect = new Ogre::Rectangle2D(true);
  screen_rect->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

  screen_rect->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
  screen_rect->setMaterial(material->getName());

  scene_node->attachObject(screen_rect);
  scene_node->setVisible(false);
}

static void create_fg_rect(
    Ogre::Rectangle2D* &screen_rect,
    Ogre::SceneNode* &scene_node,
    Ogre::MaterialPtr &material)
{
  create_rect(screen_rect, scene_node, material);

  screen_rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
}

static void create_bg_rect(
    Ogre::Rectangle2D* &screen_rect,
    Ogre::SceneNode* &scene_node,
    Ogre::MaterialPtr &material)
{
  create_rect(screen_rect, scene_node, material);

  screen_rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
}


void StereoCameraDisplay::onInitialize()
{
  StereoImageDisplayBase::onInitialize();

  // TODO: add a second tf filter, note that this is only a filter for
  // CameraInfo and not the sensor_msgs::Image message
  // TODO: pipe them into a(n) (approximate)synchronizer
  left_caminfo_tf_filter_ = new tf::MessageFilter<sensor_msgs::CameraInfo>(
      *context_->getTFClient(), fixed_frame_.toStdString(),
      queue_size_property_->getInt(), update_nh_ );

  right_caminfo_tf_filter_ = new tf::MessageFilter<sensor_msgs::CameraInfo>(
      *context_->getTFClient(), fixed_frame_.toStdString(),
      queue_size_property_->getInt(), update_nh_ );

  // Create underlay and overlay scene nodes
  bg_scene_node_ = scene_node_->createChildSceneNode();
  fg_scene_node_ = scene_node_->createChildSceneNode();

  // Create underlay and overlay rects for drawing the image
  {
    // Create material name
    static int count = 0;
    UniformStringStream mat_name_ss;
    mat_name_ss << "StereoCameraDisplayObject" << count++;
    mat_name_ss << "Material";

    // create the image underlay material
    create_material(mat_name_ss.str()+"_bg_left", left_texture_, left_bg_material_);
    create_material(mat_name_ss.str()+"_bg_right", right_texture_, right_bg_material_);

    // create the image underlay rect
    create_bg_rect(left_bg_screen_rect_, bg_scene_node_, left_bg_material_);
    create_bg_rect(right_bg_screen_rect_, bg_scene_node_, right_bg_material_);
    
    // create the image overlay material
    create_material(mat_name_ss.str()+"_fg_left", left_texture_, left_fg_material_);
    left_fg_material_->setSceneBlending( Ogre::SBT_ADD );
    create_material(mat_name_ss.str()+"_fg_right", right_texture_, right_fg_material_);
    right_fg_material_->setSceneBlending( Ogre::SBT_ADD );

    // create the image overlay rect
    create_fg_rect(left_fg_screen_rect_, fg_scene_node_, left_fg_material_);
    create_fg_rect(right_fg_screen_rect_, fg_scene_node_, right_fg_material_);
  }

  updateAlpha();

  render_panel_ = new StereoImageRenderPanel();
  //render_panel_->addLeftRect(left_bg_screen_rect_);
  render_panel_->addLeftRect(left_fg_screen_rect_);
  //render_panel_->addRightRect(right_bg_screen_rect_);
  render_panel_->addRightRect(right_fg_screen_rect_);
  render_panel_->getRenderWindow()->addListener( this );
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive( false );
  render_panel_->resize( 640, 480 );
  render_panel_->initialize( context_->getSceneManager(), context_ );

  setAssociatedWidget( render_panel_ );

  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance( 0.01f );

  left_caminfo_tf_filter_->connectInput(left_caminfo_sub_);
  left_caminfo_tf_filter_->registerCallback(boost::bind(&StereoCameraDisplay::leftCaminfoCallback, this, _1));

  right_caminfo_tf_filter_->connectInput(right_caminfo_sub_);
  right_caminfo_tf_filter_->registerCallback(boost::bind(&StereoCameraDisplay::rightCaminfoCallback, this, _1));

  //context_->getFrameManager()->registerFilterForTransformStatusCheck(caminfo_tf_filter_, this);

  vis_bit_ = context_->visibilityBits()->allocBit();
  render_panel_->getViewport()->setVisibilityMask( vis_bit_ );
  render_panel_->getRightViewport()->setVisibilityMask( vis_bit_ );

  visibility_property_ = new DisplayGroupVisibilityProperty(
      vis_bit_, context_->getRootDisplayGroup(), this, "Visibility", true,
      "Changes the visibility of other Displays in the camera view.");

  visibility_property_->setIcon( loadPixmap("package://rviz/icons/visibility.svg",true) );

  this->addChild( visibility_property_, 0 );
}

void StereoCameraDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  QString image_position = image_position_property_->getString();
  bg_scene_node_->setVisible( caminfo_ok_ && (image_position == BACKGROUND || image_position == BOTH) );
  fg_scene_node_->setVisible( caminfo_ok_ && (image_position == OVERLAY || image_position == BOTH) );

  // set view flags on all displays
  visibility_property_->update();
}

void StereoCameraDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  bg_scene_node_->setVisible( false );
  fg_scene_node_->setVisible( false );
}

void StereoCameraDisplay::onEnable()
{
  subscribe();
  render_panel_->getRenderWindow()->setActive(true);
}

void StereoCameraDisplay::onDisable()
{
  render_panel_->getRenderWindow()->setActive(false);
  unsubscribe();
  clear();
}

void StereoCameraDisplay::subscribe()
{
  if ( (!isEnabled()) || (left_topic_property_->getTopicStd().empty()) || (right_topic_property_->getTopicStd().empty()) )
  {
    return;
  }

  std::string target_frame = fixed_frame_.toStdString();
  StereoImageDisplayBase::enableTFFilter(target_frame);

  StereoImageDisplayBase::subscribe();

  std::string left_topic = left_topic_property_->getTopicStd();
  std::string left_caminfo_topic = image_transport::getCameraInfoTopic(left_topic_property_->getTopicStd());

  std::string right_topic = right_topic_property_->getTopicStd();
  std::string right_caminfo_topic = image_transport::getCameraInfoTopic(right_topic_property_->getTopicStd());

  try
  {
    left_caminfo_sub_.subscribe( update_nh_, left_caminfo_topic, 1 );
    right_caminfo_sub_.subscribe( update_nh_, right_caminfo_topic, 1 );
    setStatus( StatusProperty::Ok, "Camera Info", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( StatusProperty::Error, "Camera Info", QString( "Error subscribing: ") + e.what() );
  }
}

void StereoCameraDisplay::unsubscribe()
{
  StereoImageDisplayBase::unsubscribe();
  left_caminfo_sub_.unsubscribe();
  right_caminfo_sub_.unsubscribe();
}

static void set_material_alpha(Ogre::MaterialPtr &material, double alpha)
{
  Ogre::Pass* pass = material->getTechnique( 0 )->getPass( 0 );
  if( pass->getNumTextureUnitStates() > 0 )
  {
    Ogre::TextureUnitState* tex_unit = pass->getTextureUnitState( 0 );
    tex_unit->setAlphaOperation( Ogre::LBX_ADD_SIGNED, Ogre::LBS_TEXTURE, Ogre::LBS_CURRENT);
  }
  else
  {
    material->setAmbient( Ogre::ColourValue( 0.0f, 1.0f, 1.0f, alpha ));
    material->setDiffuse( Ogre::ColourValue( 0.0f, 1.0f, 1.0f, alpha ));
  }
}

void StereoCameraDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();

  set_material_alpha(left_fg_material_, alpha);
  set_material_alpha(right_fg_material_, alpha);

  force_render_ = true;
  context_->queueRender();
}

void StereoCameraDisplay::forceRender()
{
  force_render_ = true;
  context_->queueRender();
}

void StereoCameraDisplay::updateQueueSize()
{
  left_caminfo_tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
  right_caminfo_tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
  StereoImageDisplayBase::updateQueueSize();
}

void StereoCameraDisplay::clear()
{
  left_texture_.clear();
  right_texture_.clear();
  force_render_ = true;
  context_->queueRender();

  left_new_caminfo_ = false;
  right_new_caminfo_ = false;
  left_current_caminfo_.reset();
  right_current_caminfo_.reset();

  setStatus( StatusProperty::Warn, "Camera Info",
             "No CameraInfo received on [" + QString::fromStdString( left_caminfo_sub_.getTopic() ) + "].  Topic may not exist.");
  setStatus( StatusProperty::Warn, "Image", "No Image received");

  render_panel_->getCamera()->setPosition( Ogre::Vector3( 999999, 999999, 999999 ));
}

void StereoCameraDisplay::update( float wall_dt, float ros_dt )
{
  try
  {
    if( left_texture_.update() || force_render_ )
    {
      caminfo_ok_ &= updateCamera(
          left_texture_,
          left_current_caminfo_,
          left_bg_screen_rect_,
          left_fg_screen_rect_);
    }
    if( right_texture_.update() || force_render_ )
    {
      caminfo_ok_ &= updateCamera(
          right_texture_,
          right_current_caminfo_,
          right_bg_screen_rect_,
          right_fg_screen_rect_);
    }

    force_render_ = false;
  }
  catch( UnsupportedImageEncoding& e )
  {
    setStatus( StatusProperty::Error, "Image", e.what() );
  }

  render_panel_->getRenderWindow()->update();
}

bool StereoCameraDisplay::updateCamera(
  ROSImageTexture &texture, 
  sensor_msgs::CameraInfo::ConstPtr &current_caminfo,
  Ogre::Rectangle2D* &bg_screen_rect,
  Ogre::Rectangle2D* &fg_screen_rect)
{
  sensor_msgs::CameraInfo::ConstPtr info;
  sensor_msgs::Image::ConstPtr image;
  {
    //boost::mutex::scoped_lock lock( caminfo_mutex_ );

    info = current_caminfo;
    image = texture.getImage();
  }

  if( !info || !image )
  {
    return false;
  }

  if( !validateFloats( *info ))
  {
    setStatus( StatusProperty::Error, "Camera Info", "Contains invalid floating point values (nans or infs)" );
    return false;
  }

  // if we're in 'exact' time mode, only show image if the time is exactly right
  ros::Time rviz_time = context_->getFrameManager()->getTime();
  if ( context_->getFrameManager()->getSyncMode() == FrameManager::SyncExact &&
      rviz_time != image->header.stamp )
  {
    std::ostringstream s;
    s << "Time-syncing active and no image at timestamp " << rviz_time.toSec() << ".";
    setStatus( StatusProperty::Warn, "Time", s.str().c_str() );
    return false;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  context_->getFrameManager()->getTransform( image->header.frame_id, image->header.stamp, position, orientation );

  //printf( "StereoCameraDisplay:updateCamera(): pos = %.2f, %.2f, %.2f.\n", position.x, position.y, position.z );

  // convert vision (Z-forward) frame to ogre frame (Z-out)
  orientation = orientation * Ogre::Quaternion( Ogre::Degree( 180 ), Ogre::Vector3::UNIT_X );

  float img_width = info->width;
  float img_height = info->height;

  // If the image width is 0 due to a malformed caminfo, try to grab the width from the image.
  if( img_width == 0 )
  {
    ROS_DEBUG( "Malformed CameraInfo on camera [%s], width = 0", qPrintable( getName() ));
    img_width = texture.getWidth();
  }

  if (img_height == 0)
  {
    ROS_DEBUG( "Malformed CameraInfo on camera [%s], height = 0", qPrintable( getName() ));
    img_height = texture.getHeight();
  }

  if( img_height == 0.0 || img_width == 0.0 )
  {
    setStatus( StatusProperty::Error, "Camera Info",
               "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)" );
    return false;
  }

  double fx = info->P[0];
  double fy = info->P[5];

  float win_width = render_panel_->width();
  float win_height = render_panel_->height();
  float zoom_x = zoom_property_->getFloat();
  float zoom_y = zoom_x;

  // Preserve aspect ratio
  if( win_width != 0 && win_height != 0 )
  {
    float img_aspect = (img_width/fx) / (img_height/fy);
    float win_aspect = win_width / win_height;

    if ( img_aspect > win_aspect )
    {
      zoom_y = zoom_y / img_aspect * win_aspect;
    }
    else
    {
      zoom_x = zoom_x / win_aspect * img_aspect;
    }
  }

  Ogre::Vector3 position_right(position);

  // Add the camera's translation relative to the left camera (from P[3]);
  Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  //position = position + (right * right_offset_.x)/2.0;
  position_right = position_right - (right * right_offset_.x);

  Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
  //position = position - (down * right_offset_.y);
  //position_right = position_right - (down * right_offset_.y);

  //printf( "StereoCameraDisplay:updateCamera(): pos = %.2f, %.2f, %.2f.\n", position.x, position.y, position.z );
  //printf( "StereoCameraDisplay:updateCamera(): frustum offset = %.2f.\n", tx );

  if( !validateFloats( position ))
  {
    setStatus( StatusProperty::Error, "Camera Info", "CameraInfo/P resulted in an invalid position calculation (nans or infs)" );
    return false;
  }

  render_panel_->getCamera()->setPosition( position );
  render_panel_->getCamera()->setOrientation( orientation );

  render_panel_->getRightCamera()->setPosition( position_right );
  render_panel_->getRightCamera()->setOrientation( orientation );

  // calculate the projection matrix
  double cx = info->P[2];
  double cy = info->P[6];

  double far_plane = 100;
  double near_plane = 0.01;

  Ogre::Matrix4 proj_matrix;
  proj_matrix = Ogre::Matrix4::ZERO;

 
  proj_matrix[0][0]= 2.0 * fx/img_width * zoom_x;
  proj_matrix[1][1]= 2.0 * fy/img_height * zoom_y;

  proj_matrix[0][2]= 2.0 * (0.5 - (cx)/img_width) * zoom_x;
  proj_matrix[1][2]= 2.0 * (cy/img_height - 0.5) * zoom_y;

  proj_matrix[2][2]= -(far_plane+near_plane) / (far_plane-near_plane);
  proj_matrix[2][3]= -2.0*far_plane*near_plane / (far_plane-near_plane);

  proj_matrix[3][2]= -1;

  render_panel_->getCamera()->setCustomProjectionMatrix( true, proj_matrix );

  proj_matrix[0][2]= 2.0 * (0.5 - (cx)/img_width) * zoom_x;

  render_panel_->getRightCamera()->setCustomProjectionMatrix( true, proj_matrix );

  //std::cerr<<proj_matrix<<std::endl<<std::endl;

  setStatus( StatusProperty::Ok, "Camera Info", "OK" );

#if 1
  static Axes* debug_axes = new Axes(scene_manager_, 0, 0.2, 0.01);
  debug_axes->setPosition(position);
  debug_axes->setOrientation(orientation);

  static Axes* debug_axes_right = new Axes(scene_manager_, 0, 0.2, 0.01);
  debug_axes_right->setPosition(position_right);
  debug_axes_right->setOrientation(orientation);
#endif

  //adjust the image rectangles to fit the zoom & aspect ratio
  bg_screen_rect->setCorners( -1.0f*zoom_x, 1.0f*zoom_y, 1.0f*zoom_x, -1.0f*zoom_y );
  fg_screen_rect->setCorners( -1.0f*zoom_x, 1.0f*zoom_y, 1.0f*zoom_x, -1.0f*zoom_y );

  bg_screen_rect->setBoundingBox( Ogre::AxisAlignedBox::BOX_INFINITE );
  fg_screen_rect->setBoundingBox( Ogre::AxisAlignedBox::BOX_INFINITE );

  setStatus( StatusProperty::Ok, "Time", "ok" );
  setStatus( StatusProperty::Ok, "Camera Info", "ok" );

  return true;
}

void StereoCameraDisplay::processMessages(
    const sensor_msgs::Image::ConstPtr& left_msg,
    const sensor_msgs::Image::ConstPtr& right_msg)
{
  if(left_msg->encoding != right_msg->encoding) {
    ROS_ERROR("Encodings must be the same for stereo images!");
    return;
  }

  left_texture_.addMessage(left_msg);
  right_texture_.addMessage(right_msg);
}

void StereoCameraDisplay::leftCaminfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  //boost::mutex::scoped_lock lock( caminfo_mutex_ );
  left_current_caminfo_ = msg;
  left_new_caminfo_ = true;
}

void StereoCameraDisplay::rightCaminfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  //boost::mutex::scoped_lock lock( caminfo_mutex_ );
  right_current_caminfo_ = msg;
  right_new_caminfo_ = true;
  double fx = msg->P[0];
  double fy = msg->P[5];
  double tx = msg->P[3] / fx;
  double ty = msg->P[7] / fy;
  right_offset_ = Ogre::Vector2(tx, ty);
}
   

void StereoCameraDisplay::fixedFrameChanged()
{
  std::string targetFrame = fixed_frame_.toStdString();
  left_caminfo_tf_filter_->setTargetFrame(targetFrame);
  right_caminfo_tf_filter_->setTargetFrame(targetFrame);
  StereoImageDisplayBase::fixedFrameChanged();
}

void StereoCameraDisplay::reset()
{
  StereoImageDisplayBase::reset();
  clear();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::StereoCameraDisplay, rviz::Display )
