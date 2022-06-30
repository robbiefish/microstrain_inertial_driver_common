#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_PUBLISHER_POOL_H_
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_PUBLISHER_POOL_H_

#include "microstrain_inertial_driver_common/microstrain_defs.h"
#include "microstrain_inertial_driver_common/mip_topic_mapping.h"

namespace microstrain
{

// Type alias for the publisher map
template<class PublisherType, class MessageType>
class MIPPublisherPool
{
 public:
  MIPPublisherPool() = default;

  void configurePublisher(RosNodeType* node, const MIPTopicMapping& topic_mapping, const std::string& topic, const uint32_t queue_size = 100)
  {
    if (topic_mapping.shouldPublish(topic))
    {
      MICROSTRAIN_INFO(node, "Publishing topic %s at %d hz", topic.c_str(), topic_mapping.getDataRate(topic));
      publisher_ = create_publisher<MessageType>(node, topic, queue_size);
    }
    else
    {
      if (topic_mapping.canPublish(topic))
        MICROSTRAIN_INFO(node, "Not publishing topic %s because the data rate was set to 0", topic.c_str());
      else
        MICROSTRAIN_INFO(node, "Not publishing topic %s because the device does not support it", topic.c_str());
    }
  }

  void configureEventPublisher(RosNodeType* node, const std::string& topic, const uint8_t event_id, const uint32_t queue_size = 100)
  {
    if (event_publisher_map_.find(event_id) != event_publisher_map_.end())
      MICROSTRAIN_WARN(node, "Overriding event %d publisher for topic %s", static_cast<int32_t>(event_id), topic.c_str());
    else
      MICROSTRAIN_INFO(node, "Publishing to topic %s on event %d", topic.c_str(), static_cast<int32_t>(event_id));
    event_publisher_map_[event_id] = create_publisher<MessageType>(node, topic, queue_size);
  }

  void activate()
  {
    if (publisher_ != nullptr)
      publisher_->on_activate();
    for (const auto& event_publisher_entry : event_publisher_map_)
      event_publisher_entry.second->on_activate();
  }

  void deactivate()
  {
    if (publisher_ != nullptr)
      publisher_->on_deactivate();
    for (const auto& event_publisher_entry : event_publisher_map_)
      event_publisher_entry.second->on_deactivate();
  }

  void reset()
  {
    if (publisher_ != nullptr)
      publisher_ = nullptr;
    event_publisher_map_.clear();
  }

  void publish(const MessageType& message, const uint8_t event_id = 0)
  {
    if (event_id == 0)
    {
      if (publisher_ != nullptr)
        publisher_->publish(message);
    }
    else if (event_publisher_map_.find(event_id) != event_publisher_map_.end())
    {
      event_publisher_map_[event_id]->publish(message);
    }
  }

 private:
  PublisherType publisher_ = nullptr;
  std::map<uint8_t, PublisherType> event_publisher_map_;
};
}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_PUBLISHER_POOL_H