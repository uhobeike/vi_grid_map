#include <boost/bind.hpp>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/parse_color.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "vigrid_cells_display.h"

namespace rviz
{
ViGridCellsDisplay::ViGridCellsDisplay() : Display(), messages_received_(0), last_frame_count_(uint64_t(-1))
{
  color_property_ = new ColorProperty("Color", QColor(25, 255, 0), "Color of the grid cells.", this);

  alpha_property_ = new FloatProperty("Alpha", 1.0, "Amount of transparency to apply to the cells.",
                                      this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  topic_property_ =
      new RosTopicProperty("Topic", "",
                           QString::fromStdString(ros::message_traits::datatype<nav_msgs::GridCells>()),
                           "nav_msgs::GridCells topic to subscribe to.", this, SLOT(updateTopic()));
}

void ViGridCellsDisplay::onInitialize()
{
// TODO(wjwwood): remove this and use tf2 interface instead
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

  auto tf_client = context_->getTFClient();

#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

  tf_filter_ =
      new tf::MessageFilter<nav_msgs::GridCells>(*tf_client, fixed_frame_.toStdString(), 10, update_nh_);
  static int count = 0;
  std::stringstream ss;
  ss << "PolyLine" << count++;

  cloud_ = new PointCloud();
  cloud_->setRenderMode(PointCloud::RM_TILES);
  cloud_->setCommonDirection(Ogre::Vector3::UNIT_Z);
  cloud_->setCommonUpVector(Ogre::Vector3::UNIT_Y);
  scene_node_->attachObject(cloud_);
  updateAlpha();

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&ViGridCellsDisplay::incomingMessage, this, _1));
// TODO(wjwwood): remove this and use tf2 interface instead
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

  context_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);

#ifndef _WIN32
#pragma GCC diagnostic pop
#endif
}

ViGridCellsDisplay::~ViGridCellsDisplay()
{
  if (initialized())
  {
    unsubscribe();
    clear();
    scene_node_->detachObject(cloud_);
    delete cloud_;
    delete tf_filter_;
  }
}

void ViGridCellsDisplay::clear()
{
  cloud_->clear();

  messages_received_ = 0;
  setStatus(StatusProperty::Warn, "Topic", "No messages received");
}

void ViGridCellsDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
  context_->queueRender();
}

void ViGridCellsDisplay::updateAlpha()
{
  cloud_->setAlpha(alpha_property_->getFloat());
  context_->queueRender();
}

void ViGridCellsDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  try
  {
    sub_.subscribe(update_nh_, topic_property_->getTopicStd(), 10);
    setStatus(StatusProperty::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }
}

void ViGridCellsDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void ViGridCellsDisplay::onEnable()
{
  subscribe();
}

void ViGridCellsDisplay::onDisable()
{
  unsubscribe();
  clear();
}

void ViGridCellsDisplay::fixedFrameChanged()
{
  clear();

  tf_filter_->setTargetFrame(fixed_frame_.toStdString());
}

bool validateFloats(const nav_msgs::GridCells& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.cell_width);
  valid = valid && validateFloats(msg.cell_height);
  valid = valid && validateFloats(msg.cells);
  return valid;
}

void ViGridCellsDisplay::incomingMessage(const nav_msgs::GridCells::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  ++messages_received_;

  if (context_->getFrameCount() == last_frame_count_)
  {
    return;
  }
  last_frame_count_ = context_->getFrameCount();

  cloud_->clear();

  if (!validateFloats(*msg))
  {
    setStatus(StatusProperty::Error, "Topic",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  setStatus(StatusProperty::Ok, "Topic", QString::number(messages_received_) + " messages received");

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  if (msg->cell_width == 0)
  {
    setStatus(StatusProperty::Error, "Topic", "Cell width is zero, cells will be invisible.");
  }
  else if (msg->cell_height == 0)
  {
    setStatus(StatusProperty::Error, "Topic", "Cell height is zero, cells will be invisible.");
  }

  cloud_->setDimensions(msg->cell_width, msg->cell_height, 0.0);

  Ogre::ColourValue color_int = qtToOgre(color_property_->getColor());
  uint32_t num_points = msg->cells.size();

  typedef std::vector<PointCloud::Point> V_Point;
  V_Point points;
  points.resize(num_points);
  for (uint32_t i = 0; i < num_points; i++)
  {
    PointCloud::Point& current_point = points[i];
    current_point.position.x = msg->cells[i].x;
    current_point.position.y = msg->cells[i].y;
    current_point.position.z = msg->cells[i].z;
    current_point.color = color_int;
  }

  cloud_->clear();

  if (!points.empty())
  {
    cloud_->addPoints(&points.front(), points.size());
  }
}

void ViGridCellsDisplay::reset()
{
  Display::reset();
  clear();
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::ViGridCellsDisplay, rviz::Display)
