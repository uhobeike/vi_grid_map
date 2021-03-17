#ifndef RVIZ_VIGRID_CELLS_DISPLAY_H
#define RVIZ_VIGRID_CELLS_DISPLAY_H

#include "rviz/display.h"

#include <vi_grid_map_msgs/ViGridCells.h>
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
 * \brief Displays a vi_grid_map_msgs::ViGridCells message
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
    void incomingMessage(const vi_grid_map_msgs::ViGridCells::ConstPtr& msg);

    PointCloud* cloud_;

    message_filters::Subscriber<vi_grid_map_msgs::ViGridCells> sub_;
    tf::MessageFilter<vi_grid_map_msgs::ViGridCells>* tf_filter_;

    ColorProperty* color_property_;
    RosTopicProperty* topic_property_;
    FloatProperty* alpha_property_;

    uint32_t messages_received_;
    uint64_t last_frame_count_;
};

} // namespace rviz

#endif /* RVIZ_VIGRID_CELLS_DISPLAY_H */
