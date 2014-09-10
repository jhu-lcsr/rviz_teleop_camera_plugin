#ifndef __RVIZ_STEREO_IMAGE_RENDER_PANEL_H
#define __RVIZ_STEREO_IMAGE_RENDER_PANEL_H

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
#include <rviz/render_panel.h>

# include <rviz/display.h>

#endif


namespace rviz {

  class StereoImageRenderPanel : public rviz::RenderPanel
  {
  public:
    StereoImageRenderPanel();

    StereoImageRenderPanel(
        Ogre::Rectangle2D* left_rect,
        Ogre::Rectangle2D* right_rect);

    void clearRects();
    void addLeftRect(Ogre::Rectangle2D *rect);
    void addRightRect(Ogre::Rectangle2D *rect);

    Ogre::Camera* getRightCamera();
    Ogre::Viewport* getRightViewport();
  protected:
    void preViewportUpdate( const Ogre::RenderTargetViewportEvent& evt);

    std::vector<Ogre::Rectangle2D*> left_rects_;
    std::vector<Ogre::Rectangle2D*> right_rects_;
  };

}

#endif // ifndef __RVIZ_STEREO_IMAGE_RENDER_PANEL_H
