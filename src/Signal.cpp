/**
 * @file
 * @author Denise Ratasich
 * @date 26.04.2016
 *
 * @brief Implementation of a CAN signal.
 *
 * This class is used to send and receive CAN signals that are part of CAN
 * frames. Instatiate this class for signals that should be forwarded to ROS
 * topics (and vice versa).
 **/

#include <stdexcept>
#include <cmath>

#include "can/Signal.h"

namespace can
{
  Signal::Signal()
  {
    // empty constructor
  }

  Signal::Signal(Signal *other)
  {
    // copy members
    id_ = other->id_;
    callback_ = other->callback_;
    n_ = other->n_;
    topic_ = other->topic_;
    msg_ = other->msg_;
    subscriber_ = other->subscriber_;
    publisher_ = other->publisher_;
    direction_ = other->direction_;
    position_ = other->position_;
    length_ = other->length_;
    resolution_ = other->resolution_;
    offset_ = other->offset_;
    isSigned_ = other->isSigned_;
    mask_ = other->mask_;
  }

  Signal::Signal(ros::NodeHandle *n, std::string topic, uint32_t id, enum Direction direction,
		 callbackfct callback, uint16_t pos, uint8_t len, float resolution, float offset, bool isSigned)
  {
    if (len > 64)
      throw std::runtime_error("Unsupported length of signal. Maximum length is 64 bits (8 bytes).");

    // init variables
    n_ = n;
    topic_ = topic;
    id_ = id;
    direction_ = direction;
    callback_ = callback;
    position_ = pos;
    length_ = len;
    resolution_ = resolution;
    offset_ = offset;
    isSigned_ = isSigned;

    // create bit mask for signal
    mask_ = 0;
    for (int i = 0; i < length_; i++)
      mask_ |= (1 << i);

    if (direction == D_PUBLISH  ||  direction == D_BOTH)
      publisher_ = n_->advertise<can::Float32Stamped>(topic_, 1);
    if (direction == D_SUBSCRIBE  ||  direction == D_BOTH)
      subscriber_ = n_->subscribe<can::Float32Stamped>(topic_, 1, &Signal::msgCallback, this);
    ROS_INFO("Signal '%s': initialized (CAN ID: 0x%X).", topic_.c_str(), id_);
  }

  Signal::~Signal(void)
  {
    // nothing to be done
  }

  std::string Signal::getTopic(void)
  {
    return topic_;
  }

  uint32_t Signal::getID(void)
  {
    return id_;
  }

  void Signal::fillFrameWithSignal(CANFrame *frame)
  {
    // copy relevant bytes, i.e., bytes to change
    uint64_t candata = 0;
    std::memcpy(&candata, &(frame->data[position_/8]), (int)((length_-1)/8+1)); // copy bytewise
    candata &= ~mask_; // clear bits where the signal should be put into

    // convert signal
    float value = (msg_.data - offset_) / resolution_;

    // round to integer (1 bit)
    if (value < 0.0)
      value = std::ceil(value - 0.5);
    else
      value = std::floor(value + 0.5);

    // boundary checks
    bool cut = false;
    float cutvalue = 0;
    if (!isSigned_ && value >= (1<<length_)) {
      cutvalue = (1<<length_)-1;
      cut = true;
    } else if (!isSigned_ && value < 0) {
      cutvalue = 0;
      cut = true;
    } else if (isSigned_ && value < -(1<<(length_-1))) {
      cutvalue = -(1<<(length_-1));
      cut = true;
    } else if (isSigned_ && value >= (1<<(length_-1))) {
      cutvalue = (1<<(length_-1)) - 1;
      cut = true;
    }
    if (cut) {
      if (isSigned_)
        ROS_ERROR("Signal '%s': Rounded value (%.0f) is out of bounds [%d,%d].", topic_.c_str(), value, -(1<<(length_-1)), (1<<(length_-1))-1);
      else
        ROS_ERROR("Signal '%s': Rounded value (%.0f) is out of bounds [%d,%d].", topic_.c_str(), value, 0, (1<<(length_))-1);
      value = cutvalue;
      ROS_WARN("Signal '%s': Value has been cut to %.0f.", topic_.c_str(), value);
    }

    // cast signal (float to bytes)
    uint64_t signal;
    if (isSigned_)
      signal = (int64_t)value;
    else
      signal = (uint64_t)value;

    // write substitute signal back to frame
    candata |= (signal & mask_);
    std::memcpy(&frame->data[position_/8], &candata, (int)((length_-1)/8+1)); // copy bytewise

    ROS_DEBUG("Signal '%s': Fill frame with value (%.2f -> %.2f -> 0x%08X%08X).", topic_.c_str(), msg_.data, value, (uint32_t)(signal>>32), (uint32_t)signal);
    return;
  }

  void Signal::publish(CANFrame frame)
  {
    if (frame.id != id_) {
      ROS_WARN("Signal '%s': Not contained in frame with ID 0x%X.", topic_.c_str(), frame.id);
      return;
    }

    // signal is part of the frame, extract it
    uint64_t candata = 0;
    std::memcpy(&candata, &frame.data[position_/8], (int)((length_-1)/8+1)); // copy bytewise
    candata &= mask_; // mask the number of bits

    // fill the rest of the bits with '1's when signed and negative
    int valueIsNegative = candata & (1<<(length_-1));
    if (isSigned_ && valueIsNegative)
      candata |= ~mask_;

    // convert value of signal (signed/unsigned int to float)
    can::Float32Stamped msg;
    if (isSigned_) {
      msg.data = (float)((int64_t)candata) * resolution_ + offset_;
    } else
      msg.data = (float)candata * resolution_ + offset_;

    // reuse message header (this signal gateway is just forwarding)
    msg.header.stamp = frame.header.stamp;
    msg.header.frame_id = "can-gateway-signal";

    publisher_.publish(msg);
    ROS_DEBUG("Signal '%s': Publish message (data = %.2f).", topic_.c_str(), msg.data);
  }

  void Signal::msgCallback(can::Float32Stamped msg)
  {
    try {
      // ignore this message if it has been published from here
      if (msg.header.frame_id == "can-gateway-signal") {
	ROS_DEBUG("Signal '%s': Received message from CAN gateway. Ignore.", topic_.c_str());
	return;
      }

      msg_ = msg; // save the message (the callback will use this variable, fillFrameWithSignal)
      ROS_DEBUG("Signal '%s': Received message (data = %.2f)", topic_.c_str(), msg.data);

      if (callback_ != NULL)
	callback_(this);
      else
	throw std::runtime_error("No callback function installed.");
    } catch(const std::exception& e) {
      ROS_ERROR("Signal '%s': Error in message callback. %s", topic_.c_str(), e.what());
    }
  }
}
