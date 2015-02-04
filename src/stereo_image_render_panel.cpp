
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

#include "stereo_image_render_panel.h"

using namespace rviz;

StereoImageRenderPanel::StereoImageRenderPanel(
    Ogre::Rectangle2D* left_rect,
    Ogre::Rectangle2D* right_rect) :
  rviz::RenderPanel()
{
  left_rects_.push_back(left_rect);
  right_rects_.push_back(right_rect);
}

StereoImageRenderPanel::StereoImageRenderPanel() :
  rviz::RenderPanel()
{ }

void StereoImageRenderPanel::clearRects()
{
  left_rects_.clear();
  right_rects_.clear();
}

void StereoImageRenderPanel::addLeftRect(Ogre::Rectangle2D *rect)
{
  left_rects_.push_back(rect);
}

void StereoImageRenderPanel::addRightRect(Ogre::Rectangle2D *rect)
{
  right_rects_.push_back(rect);
}

void StereoImageRenderPanel::preViewportUpdate(
    const Ogre::RenderTargetViewportEvent& evt)
{
  rviz::RenderPanel::preViewportUpdate(evt);
  Ogre::Viewport* viewport = evt.source;

  const bool left = viewport == this->getViewport();
  
  // TODO: NOTE: FIXME: this ignored fg/bg separation

  // Show left rects
  for(std::vector<Ogre::Rectangle2D*>::iterator it = left_rects_.begin();
      it != left_rects_.end();
      ++it)
  {
    (*it)->setVisible(left);
  }

  // Show right rects
  for(std::vector<Ogre::Rectangle2D*>::iterator it = right_rects_.begin();
      it != right_rects_.end();
      ++it)
  {
    (*it)->setVisible(!left);
  }
}

Ogre::Camera* StereoImageRenderPanel::getRightCamera() {
  return right_camera_;
}

Ogre::Viewport* StereoImageRenderPanel::getRightViewport() {
  return right_viewport_;
}
