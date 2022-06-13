/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord GX5-Series Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_PUBLISHERS_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_PUBLISHERS_H

#include "microstrain_inertial_driver_common/microstrain_defs.h"
#include "microstrain_inertial_driver_common/microstrain_ros_funcs.h"
#include "microstrain_inertial_driver_common/microstrain_config.h"

namespace microstrain
{

// Type alias for the publisher map
template<class T>
using PublisherMap = std::map<uint8_t, T>;

static constexpr uint8_t PUBLISHER_INDEX_NO_EVENT = 0;

/**
 * Contains ROS messages and the publishers that will publish them
 */
class MicrostrainPublishers
{
public:
  /**
   * \brief Default Constructor
   */
  MicrostrainPublishers() = default;

  /**
   * \brief Constructs this class with a reference to the node, and a config object
   * \param node  Reference to a node that will be saved to this class and used to log and interact with ROS
   * \param config Reference to the config object that will be saved to this class and used to determine whether or not to publish
   */
  MicrostrainPublishers(RosNodeType* node, MicrostrainConfig* config);

  /**
   * \brief Configures the publishers. After this function is called, the publishers will be created, but (ROS2 only) will not be activated
   * \return true if configuration was successful and false if configuration failed
   */
  bool configure();

  /**
   * \brief Publishes device status. This is useful as it happens at a different rate than the other publishers
   */
  void publishDeviceStatus();

  // IMU Publishers
  ImuPubType imu_pub_ = nullptr;
  TimeReferencePubType imu_time_pub_ = nullptr;
  MagneticFieldPubType mag_pub_ = nullptr;
  GPSCorrelationTimestampStampedPubType gps_corr_pub_ = nullptr;

  // GNSS Publishers
  NavSatFixPubType gnss_pub_[NUM_GNSS] = { nullptr };
  OdometryPubType gnss_odom_pub_[NUM_GNSS] = { nullptr };
  TimeReferencePubType gnss_time_pub_[NUM_GNSS] = { nullptr };
  GNSSAidingStatusPubType gnss_aiding_status_pub_[NUM_GNSS] = { nullptr };
  GNSSFixInfoPubType gnss_fix_info_pub_[NUM_GNSS] = { nullptr };

  // RTK Data publisher
  RTKStatusPubType rtk_pub_ = nullptr;
  RTKStatusPubTypeV1 rtk_pub_v1_ = nullptr;

  // Filter Publishers
  FilterStatusPubType filter_status_pub_ = nullptr;
  FilterHeadingPubType filter_heading_pub_ = nullptr;
  FilterHeadingStatePubType filter_heading_state_pub_ = nullptr;
  FilterAidingMeasurementSummaryPubType filter_aiding_measurement_summary_pub_ = nullptr;
  OdometryPubType filter_pub_ = nullptr;
  ImuPubType filtered_imu_pub_ = nullptr;
  OdometryPubType filter_relative_pos_pub_ = nullptr;
  GNSSDualAntennaStatusPubType gnss_dual_antenna_status_pub_ = nullptr;

  // Device Status Publisher
  StatusPubType device_status_pub_;

  // NMEA Sentence Publisher
  NMEASentencePubType nmea_sentence_pub_;

  // Transform Broadcaster
  TransformBroadcasterType transform_broadcaster_ = nullptr;

private:
  RosNodeType* node_;
  MicrostrainConfig* config_;
};  // struct MicrostrainPublishers

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_MICROSTRAIN_PUBLISHERS_H
