/**
 * @file
 * @author Denise Ratasich
 * @date 26.04.2016
 *
 * @brief CAN frames to ROS topics gateway.
 *
 * A CAN frame usually contains several signals (e.g., engine speed), i.e.,
 * various signals are collected in a single CAN frame. A CAN signal maps 1:1
 * to a ROS topic. This node subscribes to CAN frames (provided by the CAN-in
 * interface node) and publishes relevant signals as individual
 * topics. Furthermore, this node publishes frames to send via the CAN-out
 * interface node. In particular, this node sends frames containing substitued
 * signals or a reconfiguration message for the application (notification that
 * a CAN message has been substitued).
 *
 * @note Processing/substituing frames MUST be faster than the period of the
 * CAN frame. Otherwise we should add a sequence number to the messages
 * (CANFrame, Float32Stamped).
 **/

#include <stdexcept>
#include <string>
#include <cstring>
#include <vector>
#include <map>
#include <algorithm>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "can/CANFrame.h"
#include "can/Signal.h"
using namespace can;

/** Replace source address in CAN ID. */
#define NEW_ID(old_id)	(old_id & 0x0FF)

/** CAN ID of reconfiguration message (default, can be set via launch file). */
static uint32_t can_id_reconf = 0x140;

/** CAN frame publisher. */
static ros::Publisher pub_can;

/** CAN IDs of frames to resend due to reconfiguration. */
static std::vector<uint32_t> ids2send;
/** Number of signals to await per CAN ID. */
static std::map<uint32_t, int> signalsPerID;
/** Signal names that will be substitued. */
static std::vector<std::string> signals2substitute;
/** TODO (for performance reasons): Signals to forward (from CAN to ROS). All
 * signals of interest except the substituted ones. */
static std::vector<Signal*> signals2forward;

/** CAN frame awaiting a substitued signal, per ID (last frame received). */
static std::map<uint32_t, CANFrame *> framePerID;
/** Pending flag, indicating if the last frame received has already been substituted. */
static std::map<uint32_t, bool> framePending;
/** Remaining number of signals to await per frame. */
static std::map<CANFrame *, int> signalsPerFrame;

/** Signals of interest, i.e., topics to publish or subscribe. */
static std::vector<Signal> signals;

/** Callback for incoming CAN frames. */
void msgCallbackCAN(const CANFrame msg)
{
  try {
    // check if received frame has to be resent
    for (int i = 0; i < ids2send.size(); i++) {
      if (ids2send[i] == msg.id) {
        // check if last frame has been subsituted (just to throw a warning if not)
        if (framePending[msg.id] == true)
          ROS_WARN("Gateway: Frames (ID 0x%X) arrive faster than substitution is performed.", msg.id);

        // delete last frame / free resources and reset pending frame
	if (framePerID[msg.id] != NULL) {
          delete framePerID[msg.id];
          framePerID[msg.id] = NULL;
        }

	// save this message to await the substitute value
	CANFrame *pmsg = new CANFrame(msg);
	framePerID[pmsg->id] = pmsg;
        framePending[pmsg->id] = true;
        signalsPerFrame[pmsg] = signalsPerID[pmsg->id];

	ROS_DEBUG("Gateway: Set frame with ID 0x%X to substitute with %d signal(s).", pmsg->id, signalsPerFrame[pmsg]);
	break;
      }
    }

    // send all relevant signals of this frame
    for (int i = 0; i < signals.size(); i++) {
      // signal is reconfigured (will be substituted)?
      if ( std::find(signals2substitute.begin(), signals2substitute.end(), signals[i].getTopic()) != signals2substitute.end() ) {
        // This is necessary to avoid triggering the monitor again, e.g., if it
        // is a wrong value. Hence the old source of this signal is ignored.
        continue; // skip forwarding
      }

      // check if signal is part of this frame
      if (msg.id == signals[i].getID()) {
	signals[i].publish(msg);
      }
    }
  } catch(const std::exception& e) {
    ROS_ERROR("Gateway: Callback of incoming CAN frames failed. %s", e.what());
  }
}

/** Callback for reconfiguration messages. */
void msgCallback(const std_msgs::String::ConstPtr& msg)
{
  try {
    uint32_t old_id = -1;
    ROS_DEBUG("Gateway: Monitor reported signal '%s' failed.", msg->data.c_str());

    // remember signals to substitute (in case reconfiguration message is sent
    // more than once)
    if ( std::find(signals2substitute.begin(), signals2substitute.end(), msg->data) != signals2substitute.end() )
      return; // already added
    signals2substitute.push_back(msg->data);

    // remove signals from those to forward
    //TODO

    // find CAN ID containing reconfigured signal/topic
    for (int i = 0; i < signals.size(); i++) {
      if (msg->data == signals[i].getTopic()) {
	old_id = signals[i].getID();
      }
    }

    if (old_id == -1) {
      ROS_WARN("Gateway: Signal '%s' not found.", msg->data.c_str());
      return;
    }

    // save the ID of the frame that should be resent
    ids2send.push_back(old_id);
    // reset frame for this ID (no frame pending)
    framePerID[old_id] = NULL;
    framePending[old_id] = false;
    // increase number of signals to substitute in this CAN ID
    signalsPerID[old_id]++;
    ROS_DEBUG("Gateway: Add CAN ID 0x%X to resend IDs (#signals to substitute = %d).",
	      old_id, signalsPerID[old_id]);

    // adapt sender ID (part of CAN ID represents the sender; see
    // J1939/ISO11992)
    uint32_t new_id = NEW_ID(old_id);

    // send CAN reconfiguration message
    CANFrame rmsg;
    rmsg.id = can_id_reconf;
    std::memcpy(&rmsg.data[4], &new_id, 4); // new_id (substitute)
    std::memcpy(&rmsg.data[0], &old_id, 4); // old_id (original)
    rmsg.header.frame_id = "can-gateway";
    rmsg.header.stamp = ros::Time::now();
    pub_can.publish(rmsg);
  } catch(const std::exception& e) {
    ROS_ERROR("Gateway: Sending reconfiguration message failed. %s", e.what());
  }
}

/** Callback for signals. */
void signalCallback(Signal *s)
{
  try {
    bool found = false;

    // check if this frame should be resent
    for (int i = 0; i < ids2send.size(); i++) {
      if (ids2send[i] == s->getID()) {
	found = true;
	ROS_DEBUG("Gateway: Signal '%s' will be substitued in frame with ID 0x%X.", s->getTopic().c_str(), s->getID());
	break;
      }
    }

    if (!found) {
      ROS_DEBUG("Gateway: Signal '%s' shall not be substitued. Ignore.", s->getTopic().c_str());
      return;
    }

    // first frame for this ID received?
    if (framePerID[s->getID()] != NULL) {
      // substitute signal in frame
      CANFrame *frame = framePerID[s->getID()];
      frame->id = NEW_ID(frame->id);
      s->fillFrameWithSignal(frame);

      // decrease number of signals to await
      signalsPerFrame[frame]--;

      // other signals to await?
      if (signalsPerFrame[frame] <= 0) {
	// no more signals to substitute -> send frame
        framePending[s->getID()] = false;
	pub_can.publish(*frame);
	ROS_DEBUG("Gateway: Send frame with ID 0x%X.", frame->id);
      } else {
	ROS_DEBUG("Gateway: Frame with ID 0x%X awaits another signal.", frame->id);
      }

      return;
    }

    // should never reach this line, as framePerID must not get NULL (but may
    // happen at the beginning - before the first frame arrives)
    ROS_ERROR("Gateway: No frame to fill with substituted signal '%s'.", s->getTopic().c_str());
  } catch(const std::exception& e) {
    ROS_ERROR("Gateway: Signal callback. %s", e.what());
  }
}

/** Node entry point. */
int main(int argc, char **argv)
{
  bool debug = false; // if true, ROS debug outputs are enabled

  // set up ROS
  ros::init(argc, argv, "can_gateway");
  ros::NodeHandle n("~");

  /** CAN frame subscriber. */
  ros::Subscriber sub_can;
  /** Reconfiguration message subscriber. */
  ros::Subscriber sub_reconf;

  /** Timer deleting superfluous frames periodically. */
  ros::Timer timer;

  try {
    // get node params
    // debug flag
    if (n.hasParam("debug")) {
      n.getParam("debug", debug);
      if (debug) {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
          ros::console::notifyLoggerLevelsChanged();
        }
        ROS_INFO("Gateway: Enable DEBUG outputs.");
      }
    }

    // read signal configuration
    if (n.hasParam("/signals")) {
      ROS_INFO("Gateway: Read signals configuration ...");

      // get signal names (all parameter are nested below a signal name)
      std::vector<std::string> signal_names;
      if (n.hasParam("/signals/names")) {
	n.getParam("/signals/names", signal_names);
      } else {
	throw std::runtime_error("Missing ROS parameter signal/names.");
      }

      // get parameters of each signal
      for (int i = 0; i < signal_names.size(); i++) {
	try {
	  std::string topic, dir_str;
	  int id, pos, len;
	  float res, offset;
          bool isSigned;
	  Signal::Direction dir;

	  // ROS parameters
	  n.getParam("/signals/" + signal_names[i] + "/topic", topic);
	  // direction has to be converted from string to enum
	  n.getParam("/signals/" + signal_names[i] + "/direction", dir_str);
	  if (dir_str.compare("both") == 0)
	    dir = Signal::D_BOTH;
	  else if (dir_str.compare("publish") == 0)
	    dir = Signal::D_PUBLISH;
	  else if (dir_str.compare("subscribe") == 0)
	    dir = Signal::D_SUBSCRIBE;
	  else
	    throw std::runtime_error("Wrong value for direction (" + dir_str +
				     "). Use [publish|subscribe|both].");
	  // CAN parameters
	  n.getParam("/signals/" + signal_names[i] + "/id", id);
	  n.getParam("/signals/" + signal_names[i] + "/position", pos);
	  n.getParam("/signals/" + signal_names[i] + "/length", len);
	  n.getParam("/signals/" + signal_names[i] + "/resolution", res);
	  n.getParam("/signals/" + signal_names[i] + "/offset", offset);
	  n.getParam("/signals/" + signal_names[i] + "/isSigned", isSigned);

	  signals.push_back(new Signal(&n, topic, id, dir, signalCallback,
				       pos, len, res, offset, isSigned));
          // TODO: save reference in signals2forward
          // signals2forward.push_back(&signals.back());
	} catch(const std::exception& e) {
	  throw std::runtime_error("Error while configuring signal " +
				   signal_names[i] + ". " + e.what());
	}
      }
    } else {
      throw std::runtime_error("No signal configuration found.");
    }

    // create and clear variables (for every CAN ID used)
    for (int i = 0; i < signals.size(); i++)
      signalsPerID[signals[i].getID()] = 0;

    // setup publisher
    pub_can = n.advertise<CANFrame>("frames_out", 1);
    // subscribe to incoming CAN frames
    sub_can = n.subscribe<CANFrame>("frames_in", 1, msgCallbackCAN);
    // subscribe to reconfiguration message (notification)
    sub_reconf = n.subscribe<std_msgs::String>("reconfiguration_message", 1, msgCallback);

    ROS_INFO("Gateway: Initialization done.");
  } catch(const std::exception& e) {
    ROS_ERROR("Gateway: Initialization failed. %s", e.what());
    ros::shutdown();
  }

  try {
    // loop (checking message queues)
    ros::spin();
  } catch(const std::exception& e) {
    ROS_ERROR("Gateway: Node failed. %s", e.what());
  }

  // cleanup
  for (std::map<uint32_t,CANFrame*>::iterator it=framePerID.begin(); it!=framePerID.end(); ++it) {
    std::cout << it->first << " => " << it->second << '\n';
    if (it->second != NULL)
      delete it->second;
  }

  return 0;
} // end main()
