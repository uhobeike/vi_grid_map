#ifndef RVIZ_GRID_CELLS_DISPLAY_H
#define RVIZ_GRID_CELLS_DISPLAY_H

#include "rviz/display.h"

#include <nav_msgs/GridCells.h>
#include <nav_msgs/MapMetaData.h>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#endif

#include <boost/shared_ptr.hpp>

namespace Ogre
{
class ManualObject;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class PointCloud;
class RosTopicProperty;

/**
 * \class ViGridCellsDisplay
 * \brief Displays a nav_msgs::GridCells message
 */
class ViGridCellsDisplay : public Display
{
  Q_OBJECT
public:
  ViGridCellsDisplay();
  ~ViGridCellsDisplay() override;

  void onInitialize() override;

  // Overrides from Display
  void fixedFrameChanged() override;
  void reset() override;

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateAlpha();
  void updateTopic();

private:
  void subscribe();
  void unsubscribe();
  void clear();
  void incomingMessage(const nav_msgs::GridCells::ConstPtr& msg);

  PointCloud* cloud_;

  message_filters::Subscriber<nav_msgs::GridCells> sub_;
  tf::MessageFilter<nav_msgs::GridCells>* tf_filter_;

  ColorProperty* color_property_;
  RosTopicProperty* topic_property_;
  FloatProperty* alpha_property_;

  uint32_t messages_received_;
  uint64_t last_frame_count_;
};

} // namespace rviz

#endif /* RVIZ_GRID_CELLS_DISPLAY_H */
