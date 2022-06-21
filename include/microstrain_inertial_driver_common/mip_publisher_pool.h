#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_PUBLISHER_POOL_H_
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_PUBLISHER_POOL_H_

#include "microstrain_inertial_driver_common/microstrain_defs.h"

namespace microstrain
{

// Type alias for the publisher map
template<class PublisherType, class MessageType>
class MIPPublisherPool
{
 public:
  MIPPublisherPool() = default;
  MIPPublisherPool(RosNodeType* node) : node_(node) {}

  void configurePublisher(const std::string& topic, const uint32_t queue_size = 100)
  {
    publisher_ = create_publisher<MessageType>(node_, topic, queue_size);
  }

  void configureEventPublisher(const std::string& topic, const uint32_t queue_size, const uint8_t event_id)
  {
    if (event_publisher_map_.find(topic) != event_publisher_map_.end())
      MICROSTRAIN_WARN(node_, "Overriding event %u publisher for topic %s", event_id, topic.c_str());
    event_publisher_map_[event_id] = create_publisher<MessageType>(node_, topic, queue_size);
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
    else
    {
      MICROSTRAIN_ERROR(node_, "No publisher found for event ID %u", event_id);
    }
  }

 private:
  RosNodeType* node_;

  PublisherType publisher_ = nullptr;
  std::map<uint8_t, PublisherType> event_publisher_map_;
};
}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_PUBLISHER_POOL_H