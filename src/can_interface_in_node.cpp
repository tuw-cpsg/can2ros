/**
 * @file
 * @author Denise Ratasich
 * @date 27.01.2016
 *
 * @brief CAN to ROS interface.
 *
 * Publishes all frames received from CAN bus to ROS environment.
 **/

#include <stdexcept>
#include <signal.h>
#include <errno.h>

#include "ros/ros.h"
#include "can/CANInterface.h"
#include "can/CANFrame.h"
using namespace can;


/** Node entry point. */
int main(int argc, char **argv)
{
  // set up ROS
  ros::init(argc, argv, "can_interface_in");
  ros::NodeHandle n("~");

  /** CAN interface. */
  CANInterface *can = NULL;

  /** CAN frame publisher. */
  ros::Publisher pub;

  try {
    // get node params
    std::string ifname; // interface name, e.g., "can0"
    if (n.hasParam("ifname")) {
      n.getParam("ifname", ifname);
      ROS_INFO_STREAM("CAN in: Interface is set to: " << ifname);
    } else {
      ifname = "can0";
      ROS_INFO_STREAM("CAN in: Interface is set to (default): " << ifname);
    }

    // set up CAN interface
    can = new CANInterface(ifname);

    // setup publisher of CAN frames
    pub = n.advertise<CANFrame>("frames", 1);

    ROS_INFO("CAN in: Initialization done.");
  } catch(const std::exception& e) {
    ROS_ERROR("CAN in: Initialization failed. %s", e.what());
    ros::shutdown();
  }

  try {
    canid_t id;
    uint8_t data[8];
    int res;
    CANFrame msg;

    // loop
    while (ros::ok()) {
      ROS_DEBUG("CAN in: Receive CAN message (block)...");

      // receive a CAN message (blocking)
      res = can->recvFrame(&id, data);

      // handle special errors
      if (res == -1) {
	if (errno == EAGAIN  ||  errno == EWOULDBLOCK) // timeout
	  ROS_WARN("CAN in: Receive timeout occured.");
	else if (errno == EINTR) { // interrupted by signal
	  ROS_INFO("CAN in: Receive interrupted by signal.");
	  ros::shutdown();
	}
      }

      // returns the number of bytes received
      if (res > 0) {
	ROS_DEBUG("CAN in: Frame: ID = 0x%X, data = %02X %02X %02X %02X %02X %02X %02X %02X.",
		  id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

	// publish data received via CAN
	msg.id = id;
	for (int i = 0; i < 8; i++)
	  msg.data[i] = data[i];
	msg.header.frame_id = "can-in";
	msg.header.stamp = ros::Time::now();
	pub.publish(msg);
      }

      // check for received ROS messages, actually no need here, we don't
      // receive anything, but for convinience...
      ros::spinOnce();
    }
  } catch(const std::exception& e) {
    ROS_ERROR("CAN in: Node failed. %s", e.what());
  }

  // cleanup
  delete can;

  return 0;
} // end main()
