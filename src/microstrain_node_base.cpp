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
#include <algorithm>
#include "microstrain_inertial_driver_common/microstrain_node_base.h"

namespace microstrain
{

void MicrostrainNodeBase::parseAndPublishMain()
{
  mscl::MipDataPackets packets = config_.inertial_device_->getDataPackets(1000);
  for (mscl::MipDataPacket packet : packets)
  {
    parser_.parseMIPPacket(packet);
  }

  // Save raw data, if enabled
  if (config_.raw_file_enable_)
  {
    mscl::ConnectionDebugDataVec raw_packets = config_.inertial_device_->connection().getDebugData();

    for (mscl::ConnectionDebugData raw_packet : raw_packets)
    {
      const mscl::Bytes& raw_packet_bytes = raw_packet.data();
      config_.raw_file_.write(reinterpret_cast<const char*>(raw_packet_bytes.data()), raw_packet_bytes.size());
    }
  }
}

void MicrostrainNodeBase::parseAndPublishAux()
{
  parser_.parseAuxString(config_.aux_connection_->getRawBytesStr());
}

bool MicrostrainNodeBase::initialize(RosNodeType* init_node)
{
  node_ = init_node;
  config_ = MicrostrainConfig(node_);
  publishers_ = MicrostrainPublishers(node_, &config_);
  subscribers_ = MicrostrainSubscribers(node_, &config_);
  services_ = MicrostrainServices(node_, &config_);
  parser_ = MicrostrainParser(node_, &config_, &publishers_);
  return true;
}

bool MicrostrainNodeBase::configure(RosNodeType* config_node)
{
  if (!node_)
    return false;

  MICROSTRAIN_DEBUG(node_, "Reading config");
  if (!config_.configure(config_node))
  {
    MICROSTRAIN_ERROR(node_, "Failed to read configuration for node");
    return false;
  }
  MICROSTRAIN_DEBUG(node_, "Configuring Publishers");
  if (!publishers_.configure())
  {
    MICROSTRAIN_ERROR(node_, "Failed to configure publishers");
    return false;
  }
  MICROSTRAIN_DEBUG(node_, "Configuring Services");
  if (!services_.configure())
  {
    MICROSTRAIN_ERROR(node_, "Failed to setup services");
    return false;
  }

  // Determine loop rate as 2*(max update rate), but abs. max of 1kHz
  timer_update_rate_hz_ = std::min(2 * config_.topic_mapping_.getMaxDataRate(), 1000);
  MICROSTRAIN_INFO(node_, "Setting spin rate to <%f> hz", timer_update_rate_hz_);
  return true;
}

bool MicrostrainNodeBase::activate()
{
  if (!node_)
    return false;

  // Activate the publishers
  MICROSTRAIN_DEBUG(node_, "Activating publishers");
  if (!publishers_.activate())
  {
    MICROSTRAIN_ERROR(node_, "Failed to activate publishers");
    return false;
  }

  // Activate the subscribers
  MICROSTRAIN_DEBUG(node_, "Activating Subscribers");
  if (!subscribers_.activate())
  {
    MICROSTRAIN_ERROR(node_, "Failed to activate subscribers");
    return false;
  }

  // Start the device streaming
  try
  {
    config_.topic_mapping_.startStreaming();
  }
  catch(const mscl::Error& e)
  {
    MICROSTRAIN_ERROR(node_, "Failed to start device streaming");
    return false;
  }

  return true;
}

bool MicrostrainNodeBase::deactivate()
{
  // Stop the timers.
  stop_timer(main_parsing_timer_);
  stop_timer(aux_parsing_timer_);
  stop_timer(device_status_timer_);

  // Set the device to idle
  if (config_.inertial_device_)
  {
    try
    {
      config_.inertial_device_->setToIdle();
    }
    catch (const mscl::Error& e)
    {
      // Not much we can actually do at this point, so just log the error
      MICROSTRAIN_ERROR(node_, "Unable to set node to idle: %s", e.what());
    }
  }

  // Deactivate the publishers
  MICROSTRAIN_DEBUG(node_, "Deactivating publishers");
  if (!publishers_.deactivate())
  {
    MICROSTRAIN_ERROR(node_, "Failed to deactivate publishers");
    return false;
  }

  return true;
}

bool MicrostrainNodeBase::shutdown()
{
  // Reset the timers
  main_parsing_timer_.reset();
  aux_parsing_timer_.reset();
  device_status_timer_.reset();

  // Disconnect the device
  if (config_.inertial_device_)
  {
    try
    {
      config_.inertial_device_->connection().disconnect();
    }
    catch (const mscl::Error& e)
    {
      // Not much we can actually do at this point, so just log the error
      MICROSTRAIN_ERROR(node_, "Unable to disconnect node: %s", e.what());
    }
  }

  // Disconnect the aux device
  if (config_.aux_connection_)
  {
    try
    {
      config_.aux_connection_->disconnect();
    }
    catch (const mscl::Error& e)
    {
      // Not much we can actually do at this point, so just log the error
      MICROSTRAIN_ERROR(node_, "Unable to disconnect aux port: %s", e.what());
    }
  }

  // Close the raw data file if enabled
  if (config_.raw_file_enable_)
    config_.raw_file_.close();

  // Reset the publishers
  MICROSTRAIN_DEBUG(node_, "Resetting publishers");
  if (!publishers_.reset())
  {
    MICROSTRAIN_ERROR(node_, "Failed to reset publishers");
    return false;
  }

  return true;
}

}  // namespace microstrain
