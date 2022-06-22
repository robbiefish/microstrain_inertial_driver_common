#include <algorithm>

#include "microstrain_inertial_driver_common/microstrain_ros_funcs.h"
#include "microstrain_inertial_driver_common/mip_topic_mapping.h"

namespace microstrain
{

MIPTopicMapping::MIPTopicMapping(RosNodeType* node, const std::shared_ptr<mscl::InertialNode> inertial_device) : node_(node), inertial_device_(inertial_device)
{
  // Generate the channel fields mapping for this device
  for (const auto& mapping : getTopicToChannelFieldsMapping())
    setChannelFieldsMapping(mapping.first, mapping.second);

  // Generate the data class mappings for this device
  for (auto& mapping : topic_info_mapping_)
  {
    const auto& topic = mapping.first;
    auto& info = mapping.second;
    const auto& channels = info.channel_fields;
    auto& data_classes = info.data_classes;
    for (const auto& channel : channels)
    {
      const mscl::MipTypes::DataClass data_class = static_cast<mscl::MipTypes::DataClass>(mscl::Utils::msb(static_cast<uint16_t>(channel)));
      if (inertial_device_->features().supportsCategory(data_class))
        if (std::find(data_classes.begin(), data_classes.end(), data_class) == data_classes.end())
          data_classes.push_back(data_class);
    }
  }
}

bool MIPTopicMapping::configureDataRates(RosNodeType* config_node)
{
  // Add the data rates to the topic info map
  const auto& field_data_rate_mapping = getTopicToDataRateConfigKeyMapping();
  const auto& data_class_data_rate_mapping = getDataClassToDataRateConfigKeyMapping();
  for (auto& mapping : topic_info_mapping_)
  {
    const auto& topic = mapping.first;
    auto& topic_info = mapping.second;

    // Get the data rate for the topic, and if it is not the default, use it, otherwise use the data class data rate
    if (field_data_rate_mapping.find(topic) != field_data_rate_mapping.end())
    {
      get_param<int32_t>(config_node, field_data_rate_mapping.at(topic), topic_info.data_rate, FIELD_DATA_RATE_USE_DATA_CLASS);
      if (topic_info.data_rate != FIELD_DATA_RATE_USE_DATA_CLASS)
        continue;
    }
    else
    {
      MICROSTRAIN_ERROR(node_, "Topic %s does not have an associated data rate, this should be added to the map in getTopicToDataRateConfigKeyMapping", topic.c_str());
      return false;
    }

    // Get the maximum data rate for the data classes associated with the topic if no specific field data rate was configured
    std::vector<int32_t> data_class_data_rates;
    for (const auto& data_class : topic_info.data_classes)
    {
      if (data_class_data_rate_mapping.find(data_class) != data_class_data_rate_mapping.end())
      {
        int32_t data_class_rate;
        get_param<int32_t>(config_node, data_class_data_rate_mapping.at(data_class), data_class_rate, DATA_CLASS_DATA_RATE_DO_NOT_STREAM);
        data_class_data_rates.push_back(data_class_rate);
      }
      else
      {
        MICROSTRAIN_ERROR(node_, "Data class 0x%x used by topic %s does not have an associated data rate. This should be added to the map in getDataClassToDataRateConfigKeyMapping", data_class, topic.c_str());
        return false;
      }
    }
    if (!data_class_data_rates.empty())
      topic_info.data_rate = *std::max_element(data_class_data_rates.begin(), data_class_data_rates.end());
    else
      topic_info.data_rate = DATA_CLASS_DATA_RATE_DO_NOT_STREAM;
  }
  return true;
}

MipDataClasses MIPTopicMapping::getDataClasses(const std::string& topic) const
{
  if (topic_info_mapping_.find(topic) != topic_info_mapping_.end())
    return topic_info_mapping_.at(topic).data_classes;
  else
    return {};
}

MipChannelFields MIPTopicMapping::getChannelFields(const std::string& topic) const
{
  if (topic_info_mapping_.find(topic) != topic_info_mapping_.end())
    return topic_info_mapping_.at(topic).channel_fields;
  else
    return {};
}

int32_t MIPTopicMapping::getDataRate(const std::string& topic) const
{
  if (topic_info_mapping_.find(topic) != topic_info_mapping_.end())
    return topic_info_mapping_.at(topic).data_rate;
  else
    return DATA_CLASS_DATA_RATE_DO_NOT_STREAM;
}

int32_t MIPTopicMapping::getMaxDataRate() const
{
  std::vector<int32_t> data_rates;
  for (const auto& element : getTopicToChannelFieldsMapping())
    data_rates.push_back(getDataRate(element.first));
  return *std::max_element(data_rates.begin(), data_rates.end());
}

bool MIPTopicMapping::canPublish(const std::string& topic) const
{
  return !getChannelFields(topic).empty();
}

bool MIPTopicMapping::shouldPublish(const std::string& topic) const
{
  return canPublish(topic) && getDataRate(topic) != DATA_CLASS_DATA_RATE_DO_NOT_STREAM;
}

void MIPTopicMapping::streamTopic(const std::string& topic)
{
  const MipChannelFields& channels_to_stream = getChannelFields(topic);
  const uint32_t data_rate = getDataRate(topic);
  for (const auto& channel_to_stream : channels_to_stream)
  {
    // Append the channel to the proper entry for it's descriptor set
    const mscl::MipChannel channel_with_rate(channel_to_stream, mscl::SampleRate::Hertz(data_rate));
    const mscl::MipTypes::DataClass data_class = static_cast<mscl::MipTypes::DataClass>(channel_with_rate.descriptorSet());
    if (streamed_channels_mapping_.find(data_class) == streamed_channels_mapping_.end())
      streamed_channels_mapping_[data_class] = {};

    // If this channel was already being streamed, update it if it was being streamed at a slower rate, otherwise just add it to the list
    auto existing_channel = std::find_if(streamed_channels_mapping_[data_class].begin(), streamed_channels_mapping_[data_class].end(), [channel_to_stream](const mscl::MipChannel& m)
    {
      return m.channelField() == channel_to_stream;
    }
    );
    if (existing_channel != streamed_channels_mapping_[data_class].end())
      *existing_channel = channel_with_rate;
    else
      streamed_channels_mapping_[data_class].push_back(channel_with_rate);
  }
}

void MIPTopicMapping::startStreaming()
{
  // Enable each of the data classes and their associated fields
  for (const auto& mapping : streamed_channels_mapping_)
  {
    const mscl::MipTypes::DataClass data_class = mapping.first;
    const MipChannels channels = mapping.second;

    // Attempt to configure the device to stream
    try
    {
      MICROSTRAIN_DEBUG(node_, "Enabling data class 0x%x", data_class);
      inertial_device_->setActiveChannelFields(data_class, channels);
      inertial_device_->enableDataStream(data_class);
    }
    catch (mscl::Error& e)
    {
      MICROSTRAIN_ERROR(node_, "Unable to configure 0x%x data to stream", data_class);
      MICROSTRAIN_ERROR(node_, "  Error: %s", e.what());
      throw e;
    }
  }

  // Resume the device
  MICROSTRAIN_INFO(node_, "Resuming the device data streams");
  inertial_device_->resume();
}

std::map<std::string, MipChannelFields> MIPTopicMapping::getTopicToChannelFieldsMapping() const
{
  // Get some information from the device to determine what mappings we need to create
  const bool multi_gnss = inertial_device_->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS1);

  // Generate the channel field mappings
  return {
    // IMU topic mappings
    {IMU_DATA_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_ACCEL_VEC,
      mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_GYRO_VEC,
      mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_ORIENTATION_QUATERNION,
    }},
    {IMU_INTERNAL_TIME_REF_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SHARED_REFERENCE_TIMESTAMP,
    }},
    {IMU_MAG_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_MAG_VEC,
    }},
    {IMU_GPS_CORR_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_GPS_CORRELATION_TIMESTAMP,
    }},

    // GNSS/GNSS1 topic mappings
    {GNSS1_NAVSATFIX_TOPIC, {
      multi_gnss ? mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_LLH_POSITION : mscl::MipTypes::ChannelField::CH_FIELD_GNSS_LLH_POSITION,
    }},
    {GNSS1_ODOM_TOPIC, {
      multi_gnss ? mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_LLH_POSITION : mscl::MipTypes::ChannelField::CH_FIELD_GNSS_LLH_POSITION,
      multi_gnss ? mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_NED_VELOCITY : mscl::MipTypes::ChannelField::CH_FIELD_GNSS_NED_VELOCITY,
    }},
    {GNSS1_TIME_REF_TOPIC, {
      multi_gnss ? mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_GPS_TIME : mscl::MipTypes::ChannelField::CH_FIELD_GNSS_GPS_TIME,
    }},
    {GNSS1_FIX_INFO_TOPIC, {
      multi_gnss ? mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_FIX_INFO : mscl::MipTypes::ChannelField::CH_FIELD_GNSS_FIX_INFO,
    }},
    {GNSS1_AIDING_STATUS_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_POSITION_AIDING_STATUS,
    }},

    // GNSS2 topic mappings
    {GNSS2_NAVSATFIX_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_LLH_POSITION,
    }},
    {GNSS2_ODOM_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_LLH_POSITION,
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_NED_VELOCITY,
    }},
    {GNSS2_TIME_REF_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_GPS_TIME,
    }},
    {GNSS2_FIX_INFO_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_FIX_INFO,
    }},
    {GNSS2_AIDING_STATUS_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_POSITION_AIDING_STATUS,
    }},

    // RTK topic mappings
    {RTK_STATUS_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_3_RTK_CORRECTIONS_STATUS,
    }},
    {RTK_STATUS_V1_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_GNSS_3_RTK_CORRECTIONS_STATUS,
    }},

    // Filter topic mappings
    {FILTER_STATUS_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_FILTER_STATUS,
    }},
    {FILTER_HEADING_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER,
    }},
    {FILTER_HEADING_STATE_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_HEADING_UPDATE_SOURCE,
    }},
    {FILTER_ODOM_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_POS,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_VELOCITY,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_UNCERT,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
    }},
    {FILTER_IMU_DATA_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_COMPENSATED_ACCEL,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER,
    }},
    {FILTER_RELATIVE_ODOM_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_NED_RELATIVE_POS,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_VELOCITY,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_UNCERT,
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
    }},
    {FILTER_DUAL_ANTENNA_STATUS_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_GNSS_DUAL_ANTENNA_STATUS,
    }},
    {FILTER_AIDING_SUMMARY_TOPIC, {
      mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_AIDING_MEASURE_SUMMARY,
    }},
  };

}

std::map<std::string, std::string> MIPTopicMapping::getTopicToDataRateConfigKeyMapping() const
{
  return {
    // IMU data rates
    {IMU_DATA_TOPIC,              "imu_raw_data_rate"},
    {IMU_INTERNAL_TIME_REF_TOPIC, "imu_raw_data_rate"},  // Same rate for the raw IMU data as the time ref
    {IMU_MAG_TOPIC,               "imu_mag_data_rate"},
    {IMU_GPS_CORR_TOPIC,          "imu_gps_corr_data_rate"},

    // GNSS/GNSS1 data rates
    {GNSS1_NAVSATFIX_TOPIC,     "gnss1_nav_sat_fix_data_rate"},
    {GNSS1_ODOM_TOPIC,          "gnss1_odom_data_rate"},
    {GNSS1_TIME_REF_TOPIC,      "gnss1_time_reference_data_rate"},
    {GNSS1_FIX_INFO_TOPIC,      "gnss1_fix_info_data_rate"},
    {GNSS1_AIDING_STATUS_TOPIC, "filter_aiding_status_data_rate"},

    // GNSS2 data rates
    {GNSS2_NAVSATFIX_TOPIC,     "gnss2_nav_sat_fix_data_rate"},
    {GNSS2_ODOM_TOPIC,          "gnss2_odom_data_rate"},
    {GNSS2_TIME_REF_TOPIC,      "gnss2_time_reference_data_rate"},
    {GNSS2_FIX_INFO_TOPIC,      "gnss2_fix_info_data_rate"},
    {GNSS2_AIDING_STATUS_TOPIC, "filter_aiding_status_data_rate"},

    // RTK data rates
    {RTK_STATUS_TOPIC,    "rtk_status_data_rate"},
    {RTK_STATUS_V1_TOPIC, "rtk_status_data_rate"},

    // Filter data rates
    {FILTER_STATUS_TOPIC,              "filter_status_data_rate"},
    {FILTER_HEADING_TOPIC,             "filter_heading_data_rate"},
    {FILTER_HEADING_STATE_TOPIC,       "filter_heading_state_data_rate"},
    {FILTER_ODOM_TOPIC,                "filter_heading_state_data_rate"},
    {FILTER_IMU_DATA_TOPIC,            "filter_imu_data_rate"},
    {FILTER_RELATIVE_ODOM_TOPIC,       "filter_relative_odom_data_rate"},
    {FILTER_DUAL_ANTENNA_STATUS_TOPIC, "filter_gnss_dual_antenna_data_rate"},
    {FILTER_AIDING_SUMMARY_TOPIC,      "filter_aiding_measurement_summary_data_rate"},
  };
}

std::map<mscl::MipTypes::DataClass, std::string> MIPTopicMapping::getDataClassToDataRateConfigKeyMapping() const
{
  return {
    {mscl::MipTypes::DataClass::CLASS_AHRS_IMU,  "imu_data_rate"},
    {mscl::MipTypes::DataClass::CLASS_GNSS,      "gnss1_data_rate"},
    {mscl::MipTypes::DataClass::CLASS_GNSS1,     "gnss1_data_rate"},
    {mscl::MipTypes::DataClass::CLASS_GNSS2,     "gnss2_data_rate"},
    {mscl::MipTypes::DataClass::CLASS_GNSS3,     "rtk_status_data_rate"},  // Only one RTK field right now, so just use rtk_status_data_rate
    {mscl::MipTypes::DataClass::CLASS_ESTFILTER, "filter_data_rate"},
  };
}

void MIPTopicMapping::setChannelFieldsMapping(const std::string& topic, const MipChannelFields& channels)
{
  MipChannelFields channels_to_add;
  for (const auto& channel : channels)
  {
    // If the field is supported, add it to the list
    try
    {
      if (inertial_device_->features().supportsChannelField(channel))
        channels_to_add.push_back(channel);
      else
        MICROSTRAIN_DEBUG(node_, "Note: The device does not support field 0x%x associated with topic %s", channel, topic.c_str());
    }
    catch (mscl::Error_NotSupported& e)
    {
      MICROSTRAIN_DEBUG(node_, "Note: The device does not support the field descriptor of field 0x%x associated with topic %s", channel, topic.c_str());
    }
  }

  // Add the channels to the map, or log a warning
  if (!channels_to_add.empty())
  {
    if (topic_info_mapping_.find(topic) == topic_info_mapping_.end())
      topic_info_mapping_[topic] = MIPTopicMappingInfo();
    topic_info_mapping_[topic].channel_fields = channels_to_add;
    if (channels_to_add.size() < channels.size())
      MICROSTRAIN_DEBUG(node_, "Note: The device only supports some of the fields for topic %s, some of the fields in the message may be blank. Enable debug logging for more info", topic.c_str());
  }
  else
  {
    MICROSTRAIN_DEBUG(node_, "Note: The device does not support any of the fields required for topic %s, any settings to stream that topic will have no affect", topic.c_str());
  }
} 


}  // namespace microstrain
