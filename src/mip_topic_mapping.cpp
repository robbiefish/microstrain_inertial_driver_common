#include "microstrain_inertial_driver_common/mip_topic_mapping.h"

namespace microstrain
{

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
    channel_field_mapping_[topic] = channels_to_add;
    if (channels_to_add.size() < channels.size())
      MICROSTRAIN_WARN(node_, "Note: The device only supports some of the fields for topic %s, some of the fields in the message may be blank. Enable debug logging for more info", topic.c_str());
  }
  else
  {
    MICROSTRAIN_WARN(node_, "This device does not support any of the fields required for topic %s, any settings to stream that topic will have no affect", topic.c_str());
  }
} 

MIPTopicMapping::MIPTopicMapping(RosNodeType* node, const std::shared_ptr<mscl::InertialNode> inertial_device) : node_(node), inertial_device_(inertial_device)
{
  // Get some information from the device to determine what mappings we need to create
  const bool multi_gnss = inertial_device_->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS1);

  // Generate the channel field mappings
  // IMU topic to MIP field mappings
  setChannelFieldsMapping(IMU_DATA_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_ACCEL_VEC,
    mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_GYRO_VEC,
    mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_ORIENTATION_QUATERNION,
  });
  setChannelFieldsMapping(IMU_INTERNAL_TIME_REF_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SHARED_REFERENCE_TIMESTAMP,
  });
  setChannelFieldsMapping(IMU_MAG_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_MAG_VEC,
  });
  setChannelFieldsMapping(IMU_GPS_CORR_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_GPS_CORRELATION_TIMESTAMP,
  });

  // GNSS1 topic to MIP field mappings
  setChannelFieldsMapping(GNSS1_NAVSATFIX_TOPIC, {
    multi_gnss ? mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_LLH_POSITION : mscl::MipTypes::ChannelField::CH_FIELD_GNSS_LLH_POSITION,
  });
  setChannelFieldsMapping(GNSS1_ODOM_TOPIC, {
    multi_gnss ? mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_LLH_POSITION : mscl::MipTypes::ChannelField::CH_FIELD_GNSS_LLH_POSITION,
    multi_gnss ? mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_NED_VELOCITY : mscl::MipTypes::ChannelField::CH_FIELD_GNSS_NED_VELOCITY,
  });
  setChannelFieldsMapping(GNSS1_TIME_REF_TOPIC, {
    multi_gnss ? mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_GPS_TIME : mscl::MipTypes::ChannelField::CH_FIELD_GNSS_GPS_TIME,
  });
  setChannelFieldsMapping(GNSS1_FIX_INFO_TOPIC, {
    multi_gnss ? mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_FIX_INFO : mscl::MipTypes::ChannelField::CH_FIELD_GNSS_FIX_INFO,
  });
  setChannelFieldsMapping(GNSS1_AIDING_STATUS_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_POSITION_AIDING_STATUS,
  });

  // GNSS2 topic to MIP field mapping
  setChannelFieldsMapping(GNSS2_NAVSATFIX_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_LLH_POSITION,
  });
  setChannelFieldsMapping(GNSS2_ODOM_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_LLH_POSITION,
    mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_NED_VELOCITY,
  });
  setChannelFieldsMapping(GNSS2_TIME_REF_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_GPS_TIME,
  });
  setChannelFieldsMapping(GNSS2_FIX_INFO_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_FIX_INFO,
  });
  setChannelFieldsMapping(GNSS2_AIDING_STATUS_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_POSITION_AIDING_STATUS,
  });

  // RTK topic to MIP field mapping
  setChannelFieldsMapping(RTK_STATUS_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_GNSS_3_RTK_CORRECTIONS_STATUS,
  });

  // Filter topic to MIP field mappings
  setChannelFieldsMapping(FILTER_STATUS_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_FILTER_STATUS,
  });
  setChannelFieldsMapping(FILTER_HEADING_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER,
  });
  setChannelFieldsMapping(FILTER_HEADING_STATE_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_HEADING_UPDATE_SOURCE,
  });
  setChannelFieldsMapping(FILTER_ODOM_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_POS,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_VELOCITY,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_UNCERT,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
  });
  setChannelFieldsMapping(FILTER_IMU_DATA_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_COMPENSATED_ACCEL,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER,
  });
  setChannelFieldsMapping(FILTER_RELATIVE_ODOM_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_NED_RELATIVE_POS,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_VELOCITY,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_UNCERT,
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
  });
  setChannelFieldsMapping(FILTER_DUAL_ANTENNA_STATUS_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_GNSS_DUAL_ANTENNA_STATUS,
  });
  setChannelFieldsMapping(FILTER_AIDING_SUMMARY_TOPIC, {
    mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_AIDING_MEASURE_SUMMARY,
  });

  // Generate the data class mappings
  for (const auto& mapping : channel_field_mapping_)
  {
    const auto& topic = mapping.first;
    const auto& channels = mapping.second;
    for (const auto& channel : channels)
    {
      const mscl::MipTypes::DataClass data_class = static_cast<mscl::MipTypes::DataClass>(mscl::Utils::msb(static_cast<uint16_t>(channel)));
      if (inertial_device_->features().supportsCategory(data_class))
      {
        if (data_class_mapping_.find(topic) == data_class_mapping_.end())
          data_class_mapping_[topic] = {};
        if (std::find(data_class_mapping_[topic].begin(), data_class_mapping_[topic].end(), data_class) != data_class_mapping_[topic].end())
          data_class_mapping_[topic].push_back(data_class);
      }
    }
  }
}

MipDataClasses MIPTopicMapping::getDataClasses(const std::string& topic)
{
  if (data_class_mapping_.find(topic) != data_class_mapping_.end())
    return data_class_mapping_[topic];
  else
    return {};
}

MipChannelFields MIPTopicMapping::getChannelFields(const std::string& topic)
{
  if (channel_field_mapping_.find(topic) != channel_field_mapping_.end())
    return channel_field_mapping_[topic];
  else
    return {};
}

void MIPTopicMapping::streamTopic(const std::string& topic, const int data_rate)
{
  const MipChannelFields& channels_to_stream = getChannelFields(topic);
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
}

}  // namespace microstrain
