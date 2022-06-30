/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "microstrain_inertial_driver_common/microstrain_publishers.h"

namespace microstrain
{
bool MicrostrainPublishers::configure(RosNodeType* node, const MIPTopicMapping& topic_mapping)
{
  // TODO(robbiefish): Rename some of these variables to be consistent
  // IMU publishers
  imu_pub_map_.configurePublisher(node, topic_mapping, IMU_DATA_TOPIC);
  imu_time_pub_map_.configurePublisher(node, topic_mapping, IMU_INTERNAL_TIME_REF_TOPIC);
  mag_pub_map_.configurePublisher(node, topic_mapping, IMU_MAG_TOPIC);
  gps_corr_pub_map_.configurePublisher(node, topic_mapping, IMU_GPS_CORR_TOPIC);

  // GNSS/GNSS1 publishers
  gnss_pub_map_[GNSS1_ID].configurePublisher(node, topic_mapping, GNSS1_NAVSATFIX_TOPIC);
  gnss_odom_pub_map_[GNSS1_ID].configurePublisher(node, topic_mapping, GNSS1_ODOM_TOPIC);
  gnss_time_pub_map_[GNSS1_ID].configurePublisher(node, topic_mapping, GNSS1_TIME_REF_TOPIC);
  gnss_fix_info_pub_map_[GNSS1_ID].configurePublisher(node, topic_mapping, GNSS1_FIX_INFO_TOPIC);
  gnss_aiding_status_pub_map_[GNSS1_ID].configurePublisher(node, topic_mapping, GNSS1_AIDING_STATUS_TOPIC);

  // GNSS2 publishers
  gnss_pub_map_[GNSS2_ID].configurePublisher(node, topic_mapping, GNSS2_NAVSATFIX_TOPIC);
  gnss_odom_pub_map_[GNSS2_ID].configurePublisher(node, topic_mapping, GNSS2_ODOM_TOPIC);
  gnss_time_pub_map_[GNSS2_ID].configurePublisher(node, topic_mapping, GNSS2_TIME_REF_TOPIC);
  gnss_fix_info_pub_map_[GNSS2_ID].configurePublisher(node, topic_mapping, GNSS2_FIX_INFO_TOPIC);
  gnss_aiding_status_pub_map_[GNSS2_ID].configurePublisher(node, topic_mapping, GNSS2_AIDING_STATUS_TOPIC);

  // RTK publishers
  rtk_pub_map_.configurePublisher(node, topic_mapping, RTK_STATUS_TOPIC);

  // Filter publishers
  filter_status_pub_map_.configurePublisher(node, topic_mapping, FILTER_STATUS_TOPIC);
  filter_heading_pub_map_.configurePublisher(node, topic_mapping, FILTER_HEADING_TOPIC);
  filter_heading_state_pub_map_.configurePublisher(node, topic_mapping, FILTER_HEADING_STATE_TOPIC);
  filter_pub_map_.configurePublisher(node, topic_mapping, FILTER_ODOM_TOPIC);
  filtered_imu_pub_map_.configurePublisher(node, topic_mapping, FILTER_IMU_DATA_TOPIC);
  filter_relative_pos_pub_map_.configurePublisher(node, topic_mapping, FILTER_RELATIVE_ODOM_TOPIC);
  if (topic_mapping.shouldPublish(FILTER_RELATIVE_ODOM_TOPIC) && transform_broadcaster_ == nullptr)
    transform_broadcaster_ = create_transform_broadcaster(node);
  gnss_dual_antenna_status_pub_map_.configurePublisher(node, topic_mapping, FILTER_DUAL_ANTENNA_STATUS_TOPIC);
  filter_aiding_measurement_summary_pub_map_.configurePublisher(node, topic_mapping, FILTER_AIDING_SUMMARY_TOPIC);

  return true;
}

bool MicrostrainPublishers::configureEventPublisher(RosNodeType* node, const std::string& topic, const std::string& data_topic, const uint8_t event_id)
{
  /*
  // IMU events
  if (data_topic == IMU_DATA_TOPIC)
    imu_pub_map_.configureEventPublisher(node, topic, event_id);
  else if (data_topic == IMU_INTERNAL_TIME_REF_TOPIC)
    imu_time_pub_map_.configureEventPublisher(node, topic, event_id);
  else if (data_topic == IMU_MAG_TOPIC)
    mag_pub_map_.configureEventPublisher(node, topic, event_id);
  else if (data_topic == IMU_GPS_CORR_TOPIC)
    gps_corr_pub_map_.configureEventPublisher(node, topic, event_id);
  
  // GNSS/GNSS1 events
  else if (data_topic == GNSS1_NAVSATFIX_TOPIC)
    gnss_pub_map_[GNSS1_ID].configureEventPublisher(node, topic, event_id);
  else if (data_topic == GNSS1_ODOM_TOPIC)
    gnss_odom_pub_map_[GNSS1_ID].configureEventPublisher(node, topic, event_id);
  else if (data_topic == GNSS1_TIME_REF_TOPIC)
    gnss_time_pub_map_[GNSS1_ID].configureEventPublisher(node, topic, event_id);
  else if (data_topic == GNSS1_FIX_INFO_TOPIC)
    gnss_fix_info_pub_map_[GNSS1_ID].configureEventPublisher(node, topic, event_id);
  else if (data_topic == GNSS1_AIDING_STATUS_TOPIC)
    gnss_aiding_status_pub_map_[GNSS1_ID].configureEventPublisher(node, topic, event_id);
  
  // GNSS2 events
  else if (data_topic == GNSS2_NAVSATFIX_TOPIC)
    gnss_pub_map_[GNSS2_ID].configureEventPublisher(node, topic, event_id);
  else if (data_topic == GNSS2_ODOM_TOPIC)
    gnss_odom_pub_map_[GNSS2_ID].configureEventPublisher(node, topic, event_id);
  else if (data_topic == GNSS2_TIME_REF_TOPIC)
    gnss_time_pub_map_[GNSS2_ID].configureEventPublisher(node, topic, event_id);
  else if (data_topic == GNSS2_FIX_INFO_TOPIC)
    gnss_fix_info_pub_map_[GNSS2_ID].configureEventPublisher(node, topic, event_id);
  else if (data_topic == GNSS2_AIDING_STATUS_TOPIC)
    gnss_aiding_status_pub_map_[GNSS2_ID].configureEventPublisher(node, topic, event_id);
  
  // RTK events
  else if (data_topic == RTK_STATUS_TOPIC)
    rtk_pub_map_.configureEventPublisher(node, topic, event_id);
  else if (data_topic == RTK_STATUS_V1_TOPIC)
    rtk_pub_map_v1_.configureEventPublisher(node, topic, event_id);
  
  // Filter events
  else if (data_topic == FILTER_STATUS_TOPIC)
    filter_status_pub_map_.configureEventPublisher(node, topic, event_id);
  else if (data_topic == FILTER_HEADING_TOPIC)
    filter_heading_pub_map_.configureEventPublisher(node, topic, event_id);
  else if (data_topic == FILTER_HEADING_STATE_TOPIC)
    filter_heading_state_pub_map_.configureEventPublisher(node, topic, event_id);
  else if (data_topic == FILTER_ODOM_TOPIC)
    filter_pub_map_.configureEventPublisher(node, topic, event_id);
  else if (data_topic == FILTER_IMU_DATA_TOPIC)
    filtered_imu_pub_map_.configureEventPublisher(node, topic, event_id);
  else if (data_topic == FILTER_RELATIVE_ODOM_TOPIC)
    filter_relative_pos_pub_map_.configureEventPublisher(node, topic, event_id);
  else if (data_topic == FILTER_DUAL_ANTENNA_STATUS_TOPIC)
    gnss_dual_antenna_status_pub_map_.configureEventPublisher(node, topic, event_id);
  else if (data_topic == FILTER_AIDING_SUMMARY_TOPIC)
    filter_aiding_measurement_summary_pub_map_.configureEventPublisher(node, topic, event_id);

  // If the event is not valid, return an error
  else
    return false;
  */
  return true;
}

bool MicrostrainPublishers::activate()
{
  // IMU publishers
  imu_pub_map_.activate();
  imu_time_pub_map_.activate();
  mag_pub_map_.activate();
  gps_corr_pub_map_.activate();

  // GNSS 1 and 2 publishers
  for (uint8_t i = 0; i < NUM_GNSS; i++)
  {
    gnss_pub_map_[i].activate();
    gnss_odom_pub_map_[i].activate();
    gnss_time_pub_map_[i].activate();
    gnss_fix_info_pub_map_[i].activate();
    gnss_aiding_status_pub_map_[i].activate();
  }

  // RTK publishers
  rtk_pub_map_.activate();

  // Filter publishers
  filter_status_pub_map_.activate();
  filter_heading_pub_map_.activate();
  filter_heading_state_pub_map_.activate();
  filter_pub_map_.activate();
  filtered_imu_pub_map_.activate();
  filter_relative_pos_pub_map_.activate();
  gnss_dual_antenna_status_pub_map_.activate();
  filter_aiding_measurement_summary_pub_map_.activate();
  return true;
}

bool MicrostrainPublishers::deactivate()
{
  // IMU publishers
  imu_pub_map_.deactivate();
  imu_time_pub_map_.deactivate();
  mag_pub_map_.deactivate();
  gps_corr_pub_map_.deactivate();

  // GNSS 1 and 2 publishers
  for (uint8_t i = 0; i < NUM_GNSS; i++)
  {
    gnss_pub_map_[i].deactivate();
    gnss_odom_pub_map_[i].deactivate();
    gnss_time_pub_map_[i].deactivate();
    gnss_fix_info_pub_map_[i].deactivate();
    gnss_aiding_status_pub_map_[i].deactivate();
  }

  // RTK publishers
  rtk_pub_map_.deactivate();

  // Filter publishers
  filter_status_pub_map_.deactivate();
  filter_heading_pub_map_.deactivate();
  filter_heading_state_pub_map_.deactivate();
  filter_pub_map_.deactivate();
  filtered_imu_pub_map_.deactivate();
  filter_relative_pos_pub_map_.deactivate();
  gnss_dual_antenna_status_pub_map_.deactivate();
  filter_aiding_measurement_summary_pub_map_.deactivate();
  return true;
}

bool MicrostrainPublishers::reset()
{
  // IMU publishers
  imu_pub_map_.reset();
  imu_time_pub_map_.reset();
  mag_pub_map_.reset();
  gps_corr_pub_map_.reset();

  // GNSS 1 and 2 publishers
  for (uint8_t i = 0; i < NUM_GNSS; i++)
  {
    gnss_pub_map_[i].reset();
    gnss_odom_pub_map_[i].reset();
    gnss_time_pub_map_[i].reset();
    gnss_fix_info_pub_map_[i].reset();
    gnss_aiding_status_pub_map_[i].reset();
  }

  // RTK publishers
  rtk_pub_map_.reset();

  // Filter publishers
  filter_status_pub_map_.reset();
  filter_heading_pub_map_.reset();
  filter_heading_state_pub_map_.reset();
  filter_pub_map_.reset();
  filtered_imu_pub_map_.reset();
  filter_relative_pos_pub_map_.reset();
  gnss_dual_antenna_status_pub_map_.reset();
  filter_aiding_measurement_summary_pub_map_.reset();
  return true;
}

/*
void MicrostrainPublishers::publishDeviceStatus()
{
  if (!config_->inertial_device_)
  {
    return;
  }

  if (config_->inertial_device_->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    if (config_->inertial_device_->features().supportedStatusSelectors().size() > 1)
    {
      mscl::DeviceStatusData statusData = config_->inertial_device_->getDiagnosticDeviceStatus();
      mscl::DeviceStatusMap status = statusData.asMap();

      device_status_msg_.system_timer_ms = statusData.systemTimerInMS;

      mscl::DeviceStatusMap::iterator it;

      for (it = status.begin(); it != status.end(); it++)
      {
        switch (it->first)
        {
          case mscl::DeviceStatusValues::ModelNumber:
            device_status_msg_.device_model = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::StatusStructure_Value:
            device_status_msg_.status_selector = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::SystemState_Value:
            device_status_msg_.system_state = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ImuStreamInfo_Enabled:
            device_status_msg_.imu_stream_enabled = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssStreamInfo_Enabled:
            device_status_msg_.gps_stream_enabled = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ImuStreamInfo_PacketsDropped:
            device_status_msg_.imu_dropped_packets = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::EstimationFilterStreamInfo_Enabled:
            device_status_msg_.filter_stream_enabled = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::EstimationFilterStreamInfo_PacketsDropped:
            device_status_msg_.filter_dropped_packets = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ComPortInfo_BytesWritten:
            device_status_msg_.com1_port_bytes_written = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ComPortInfo_BytesRead:
            device_status_msg_.com1_port_bytes_read = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnWrite:
            device_status_msg_.com1_port_write_overruns = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnRead:
            device_status_msg_.com1_port_read_overruns = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ImuMessageInfo_MessageParsingErrors:
            device_status_msg_.imu_parser_errors = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ImuMessageInfo_MessagesRead:
            device_status_msg_.imu_message_count = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::ImuMessageInfo_LastMessageReadinMS:
            device_status_msg_.imu_last_message_ms = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssStreamInfo_PacketsDropped:
            device_status_msg_.gps_dropped_packets = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssMessageInfo_MessageParsingErrors:
            device_status_msg_.gps_parser_errors = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssMessageInfo_MessagesRead:
            device_status_msg_.gps_message_count = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssMessageInfo_LastMessageReadinMS:
            device_status_msg_.gps_last_message_ms = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::GnssPowerStateOn:
            device_status_msg_.gps_power_on = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::gnss1PpsPulseInfo_Count:
            device_status_msg_.num_gps_pps_triggers = atoi(it->second.c_str());
            break;

          case mscl::DeviceStatusValues::gnss1PpsPulseInfo_LastTimeinMS:
            device_status_msg_.last_gps_pps_trigger_ms = atoi(it->second.c_str());
            break;

          default:
            break;
        }
      }

      device_status_pub_->publish(device_status_msg_);
    }
  }
}
*/

}  // namespace microstrain
