#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_TOPIC_MAPPING_H_
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_TOPIC_MAPPING_H_

#include <string>
#include <memory>

#include "mscl/mscl.h"

#include "microstrain_inertial_driver_common/microstrain_defs.h"

namespace microstrain
{

// Aliases to some vectors of MSCL types
using MipDataClasses = std::vector<mscl::MipTypes::DataClass>;
using MipChannelFields = mscl::MipTypes::MipChannelFields;
using MipChannels = mscl::MipChannels;

// Topic names
static constexpr auto IMU_DATA_TOPIC = "imu/data";
static constexpr auto IMU_INTERNAL_TIME_REF_TOPIC = "imu/internal_time_ref";
static constexpr auto IMU_MAG_TOPIC = "mag";
static constexpr auto IMU_GPS_CORR_TOPIC = "gps_corr";

static constexpr auto GNSS1_NAVSATFIX_TOPIC = "gnss1/fix";
static constexpr auto GNSS1_ODOM_TOPIC = "gnss1/odom";
static constexpr auto GNSS1_TIME_REF_TOPIC = "gnss1/time_ref";
static constexpr auto GNSS1_FIX_INFO_TOPIC = "gnss1/fix_info";
static constexpr auto GNSS1_AIDING_STATUS_TOPIC = "gnss1/aiding_status";

static constexpr auto GNSS2_NAVSATFIX_TOPIC = "gnss2/fix";
static constexpr auto GNSS2_ODOM_TOPIC = "gnss2/odom";
static constexpr auto GNSS2_TIME_REF_TOPIC = "gnss2/time_ref";
static constexpr auto GNSS2_FIX_INFO_TOPIC = "gnss2/fix_info";
static constexpr auto GNSS2_AIDING_STATUS_TOPIC = "gnss2/aiding_status";

static constexpr auto RTK_STATUS_TOPIC = "rtk/status";

static constexpr auto FILTER_STATUS_TOPIC = "nav/status";
static constexpr auto FILTER_HEADING_TOPIC = "nav/heading";
static constexpr auto FILTER_HEADING_STATE_TOPIC = "nav/heading_state";
static constexpr auto FILTER_ODOM_TOPIC = "nav/odom";
static constexpr auto FILTER_IMU_DATA_TOPIC = "nav/filtered_imu/data";
static constexpr auto FILTER_RELATIVE_ODOM_TOPIC = "nav/relative_pos/odom";
static constexpr auto FILTER_DUAL_ANTENNA_STATUS_TOPIC = "nav/dual_antenna_status";
static constexpr auto FILTER_AIDING_SUMMARY_TOPIC = "nav/aiding_summary";

/**
 * Helper class used to lookup MIP or device information given a topic name
 */
class MIPTopicMapping
{
 public:
  /**
   * \brief Default constructor
   */
  MIPTopicMapping() = default;

  /**
   * \brief Constructs the mapping with a reference to the ROS node and the device. The reference to the ROS node will be saved as a member variable for later usage
   * \param node  The ROS node that is constructing this object
   * \param inertial_device  Pointer to the inertial device that we will use to read information from the device 
   */
  MIPTopicMapping(RosNodeType* node, const std::shared_ptr<mscl::InertialNode> inertial_device);

  /**
   * \brief Gets the data classes (descriptor sets) that are used by the topic
   * \param topic  Name of the topic to search for
   * \return List of data classes for the requested topic, or an empty vector if the topic cannot be found or is not supported by the device
   */
  MipDataClasses getDataClasses(const std::string& topic);

  /**
   * \brief Gets the channel fields (field descriptors) that are used by the topic
   * \param topic  Name of the topic to search for
   * \return List of channel fields for the requested topic, or an empty vector if the topic cannot be found or is not supported by the device
   */
  MipChannelFields getChannelFields(const std::string& topic);

  /**
   * \brief Streams the channels associated with the requested "topic" at "data_rate" hertz
   *        NOTE: This will only append the channel fields to a list. In order to start streaming, "startStreaming" must be called after this function
   * \param topic  Name of the topic to stream
   * \param data_rate  Hertz to stream the requested topic at
   */
  void streamTopic(const std::string& topic, const int data_rate);

  /**
   * \brief Sends the requested data rates for all channel fields to the device, and enables the appropriate data streams
   *        NOTE: This should be run after calling "streamTopic". If "streamTopic" is called after "startStreaming", you will need to call "startStreaming" again
   */
  void startStreaming();

 private:

  /**
   * \brief Helper function used when initializing this class to check if the channels are supported by the device before mapping the channels to the topic
   * \param topic  The topic to add a mapping for
   * \param channels  The channels associated with this topic
   */
  void setChannelFieldsMapping(const std::string& topic, const MipChannelFields& channels);

  RosNodeType* node_;
  std::shared_ptr<mscl::InertialNode> inertial_device_;
  std::map<std::string, MipDataClasses> data_class_mapping_;
  std::map<std::string, MipChannelFields> channel_field_mapping_;
  std::map<mscl::MipTypes::DataClass, MipChannels> streamed_channels_mapping_;
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MIP_TOPIC_MAPPING_H