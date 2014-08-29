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
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>

#include <tf/transform_listener.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/render_panel.h>
#include <rviz/validate_floats.h>

#include <sensor_msgs/image_encodings.h>

#include "stereo_image_display.h"

namespace rviz
{

class StereoImageRenderPanel : public rviz::RenderPanel
  {
  public:
    StereoImageRenderPanel(
        Ogre::Rectangle2D* left_rect,
        Ogre::Rectangle2D* right_rect) :
      rviz::RenderPanel(),
      left_rect_(left_rect),
      right_rect_(right_rect)
    { }

  protected:
    void preViewportUpdate(
        const Ogre::RenderTargetViewportEvent& evt)
    {
      rviz::RenderPanel::preViewportUpdate(evt);
      Ogre::Viewport* viewport = evt.source;

      if (viewport == this->getViewport()) {
        // Set texture to left image
        left_rect_->setVisible(true);
        right_rect_->setVisible(false);
      } else {
        // Set texture to right image
        left_rect_->setVisible(false);
        right_rect_->setVisible(true);
      }
    }

    Ogre::Rectangle2D *left_rect_, *right_rect_;
  };

StereoImageDisplay::StereoImageDisplay()
  : StereoImageDisplayBase()
  , left_texture_()
  , right_texture_()
{
  normalize_property_ = new BoolProperty( "Normalize Range", true,
                                          "If set to true, will try to estimate the range of possible values from the received images.",
                                          this, SLOT( updateNormalizeOptions() ));

  min_property_ = new FloatProperty( "Min Value", 0.0, "Value which will be displayed as black.", this, SLOT( updateNormalizeOptions() ));

  max_property_ = new FloatProperty( "Max Value", 1.0, "Value which will be displayed as white.", this, SLOT( updateNormalizeOptions() ));

  median_buffer_size_property_ = new IntProperty( "Median window", 5, "Window size for median filter used for computin min/max.",
                                                  this, SLOT( updateNormalizeOptions() ) );

  got_float_image_ = false;
}

void create_material(const std::string &material_name, ROSImageTexture &texture, Ogre::MaterialPtr &material) 
{
  material = Ogre::MaterialManager::getSingleton().create( material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  material->setSceneBlending( Ogre::SBT_REPLACE );
  material->setDepthWriteEnabled(false);
  material->setReceiveShadows(false);
  material->setDepthCheckEnabled(false);

  material->getTechnique(0)->setLightingEnabled(false);
  Ogre::TextureUnitState* tu = material->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureName(texture.getTexture()->getName());
  tu->setTextureFiltering( Ogre::TFO_NONE );

  material->setCullingMode(Ogre::CULL_NONE);
}

void StereoImageDisplay::onInitialize()
{
  StereoImageDisplayBase::onInitialize();
  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "StereoImageDisplay" << count++;
    img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
  }

  img_scene_node_ = img_scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "StereoImageDisplayObject" << count++;

    left_screen_rect_ = new Ogre::Rectangle2D(true);
    left_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    left_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    right_screen_rect_ = new Ogre::Rectangle2D(true);
    right_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    right_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    create_material(ss.str() + "Left", left_texture_, left_material_);
    create_material(ss.str() + "Right", right_texture_, right_material_);

    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();

    left_screen_rect_->setBoundingBox(aabInf);
    left_screen_rect_->setMaterial(left_material_->getName());
    img_scene_node_->attachObject(left_screen_rect_);

    right_screen_rect_->setBoundingBox(aabInf);
    right_screen_rect_->setMaterial(right_material_->getName());
    img_scene_node_->attachObject(right_screen_rect_);
  }

  render_panel_ = new StereoImageRenderPanel(
      left_screen_rect_,
      right_screen_rect_);

  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive( false );

  render_panel_->resize( 640, 480 );
  render_panel_->initialize(img_scene_manager_, context_);

  setAssociatedWidget( render_panel_ );

  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance( 0.01f );
  render_panel_->enableStereo(true);

  updateNormalizeOptions();
}

StereoImageDisplay::~StereoImageDisplay()
{
  if ( initialized() )
  {
    delete render_panel_;
    delete left_screen_rect_;
    delete right_screen_rect_;
    img_scene_node_->getParentSceneNode()->removeAndDestroyChild( img_scene_node_->getName() );
  }
}

void StereoImageDisplay::onEnable()
{
  StereoImageDisplayBase::subscribe();
  render_panel_->getRenderWindow()->setActive(true);
}

void StereoImageDisplay::onDisable()
{
  render_panel_->getRenderWindow()->setActive(false);
  StereoImageDisplayBase::unsubscribe();
  clear();
}

void StereoImageDisplay::updateNormalizeOptions()
{
  if (got_float_image_)
  {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    left_texture_.setNormalizeFloatImage( normalize, min_property_->getFloat(), max_property_->getFloat());
    left_texture_.setMedianFrames( median_buffer_size_property_->getInt() );

    right_texture_.setNormalizeFloatImage( normalize, min_property_->getFloat(), max_property_->getFloat());
    right_texture_.setMedianFrames( median_buffer_size_property_->getInt() );
  }
  else
  {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}

void StereoImageDisplay::clear()
{
  left_texture_.clear();
  right_texture_.clear();

  if( render_panel_->getCamera() )
  {
    render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
  }
}

void resize(StereoImageRenderPanel *render_panel, ROSImageTexture &texture, Ogre::Rectangle2D *screen_rect)
{

    //make sure the aspect ratio of the image is preserved
    float win_width = render_panel->width();
    float win_height = render_panel->height();

    float img_width = texture.getWidth();
    float img_height = texture.getHeight();

    if ( img_width != 0 && img_height != 0 && win_width !=0 && win_height != 0 )
    {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if ( img_aspect > win_aspect )
      {
        screen_rect->setCorners(-1.0f, 1.0f * win_aspect/img_aspect, 1.0f, -1.0f * win_aspect/img_aspect, false);
      }
      else
      {
        screen_rect->setCorners(-1.0f * img_aspect/win_aspect, 1.0f, 1.0f * img_aspect/win_aspect, -1.0f, false);
      }
    }
}

void StereoImageDisplay::update( float wall_dt, float ros_dt )
{
  try
  {
    left_texture_.update();
    right_texture_.update();

    resize(render_panel_, left_texture_, left_screen_rect_);
    resize(render_panel_, right_texture_, right_screen_rect_);

    render_panel_->getRenderWindow()->update();
  }
  catch( UnsupportedImageEncoding& e )
  {
    setStatus(StatusProperty::Error, "Image", e.what());
  }
}

void StereoImageDisplay::reset()
{
  StereoImageDisplayBase::reset();
  clear();
}

bool is_float_image(
    const sensor_msgs::Image::ConstPtr& msg)
{
  return msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
    msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
    msg->encoding == sensor_msgs::image_encodings::MONO16;
}

/* This is called by incomingMessage(). */
void StereoImageDisplay::processMessages(
    const sensor_msgs::Image::ConstPtr& left_msg,
    const sensor_msgs::Image::ConstPtr& right_msg)
{
  if(left_msg->encoding != right_msg->encoding) {
    ROS_ERROR("Encodings must be the same for stereo images!");
    return;
  }

  bool got_float_image = is_float_image(left_msg);
  if ( got_float_image != got_float_image_ )
  {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }

  left_texture_.addMessage(left_msg);
  right_texture_.addMessage(right_msg);
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::StereoImageDisplay, rviz::Display )
