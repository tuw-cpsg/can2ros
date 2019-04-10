/**
 * @file
 * @author Denise Ratasich
 * @date 27.01.2016
 *
 * @brief ROS to CAN interface.
 *
 * Sends frames from ROS environment to CAN bus.
 **/

#include <stdexcept>
#include <linux/can.h>

#include "ros/ros.h"
#include "can/CANInterface.h"
#include "can/CANFrame.h"
using namespace can;

/** CAN interface. */
static CANInterface *cani = NULL;

/** Callback for CAN frame messages from ROS system. */
void msgCallback(const CANFrame::ConstPtr& msg)
{
  try {
    canid_t id;
    uint8_t data[8];

    id = (canid_t) msg->id;
    for (int i = 0; i < 8; i++)
      data[i] = msg->data[i];

    ROS_DEBUG("CAN out: Send CAN message: ID = 0x%X, data = %02X %02X %02X %02X %02X %02X %02X %02X.",
	      id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

    cani->sendFrame(id, data, sizeof(data));
  } catch(const std::exception& e) {
    ROS_ERROR("Sending CAN message failed. %s", e.what());
  }
}

/** Node entry point. */
int main(int argc, char **argv)
{
  // set up ROS
  ros::init(argc, argv, "can_interface_out");
  ros::NodeHandle n("~");

  /** CAN frame subscriber. */
  ros::Subscriber sub;

  try {
    // get node params
    std::string ifname; // interface name, e.g., "can0"
    if (n.hasParam("ifname")) {
      n.getParam("ifname", ifname);
      ROS_INFO_STREAM("CAN out: Interface is set to: " << ifname);
    } else {
      ifname = "can0";
      ROS_INFO_STREAM("CAN out: Interface is set to (default): " << ifname);
    }

    // set up CAN interface
    cani = new CANInterface(ifname);

    // subscribe to CAN frames
    sub = n.subscribe<CANFrame>("frames", 1, msgCallback);

    ROS_INFO("CAN out: Initialization done.");
  } catch(const std::exception& e) {
    ROS_ERROR("CAN out: Initialization failed. %s", e.what());
    ros::shutdown();
    delete cani;
  }

  try {
    // loop (checking message queues)
    ros::spin();
  } catch(const std::exception& e) {
    ROS_ERROR("CAN out: Node failed. %s", e.what());
  }

  // cleanup
  delete cani;

  return 0;
} // end main()
