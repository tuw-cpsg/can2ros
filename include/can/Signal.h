/**
 * @file
 * @author Denise Ratasich
 * @date 26.04.2016
 *
 * @brief Header for CAN signals.
 **/

#ifndef __SIGNAL_H__
#define __SIGNAL_H__

#include <string>

#include "ros/ros.h"
#include "can/CANFrame.h"
#include "can/Float32Stamped.h"

namespace can
{
  class Signal
  {
  public:
    /** Callback function type. */
    typedef void (*callbackfct)(Signal*);

    /** Direction of signal. */
    enum Direction { D_PUBLISH = 0, D_SUBSCRIBE, D_BOTH, D_NONE };

  protected:
    /** CAN ID of the frame the signal resides in. */
    uint32_t id_;
    /** Callback function, that should be called when a ROS message is received. */
    callbackfct callback_;

    /** ROS node handle. */
    ros::NodeHandle *n_;
    /** ROS topic (name of the signal) that should be subscribed. */
    std::string topic_;
    /** Last received message. */
    can::Float32Stamped msg_;
    /** Publisher to send ROS messages to the topic. */
    ros::Publisher publisher_;
    /** Subscriber to receive ROS messages from the topic. */
    ros::Subscriber subscriber_;

    /** Direction of this signal (publish and/or subscribe). */
    enum Direction direction_;

    /** Position of signal in CAN frame in bits. */
    uint16_t position_;
    /** Number of bits occupied by signal in CAN frame. Must be <= 64. */
    uint8_t length_;
    /** Resolution per bit. */
    float resolution_;
    /** Offset. */
    float offset_;
    /** Signed/unsigned flag. */
    bool isSigned_;
    /** Bitmask for length of signal (apply when already shifted by position). */
    uint64_t mask_;

  public:

    /**
     * @brief Empty base constructor.
     **/
    Signal();

    /**
     * @brief Copy constructor.
     *
     * @param other Other signal to copy from.
     **/
    Signal(Signal *other);

    /**
     * @brief Initializes CAN gateway, i.e., creates a subscriber for the given
     * topic.
     *
     * @param n ROS node handle.
     * @param topic ROS topic name.
     * @param id CAN ID.
     * @param callback Callback function, called when ROS message received.
     **/
    Signal(ros::NodeHandle *n, std::string topic, uint32_t id, enum Direction direction,
	   callbackfct callback, uint16_t pos, uint8_t len, float resolution, float offset, bool isSigned);

    /**
     * @brief Destructor.
     **/
    virtual ~Signal();

    /**
     * @brief Returns name of the signal (topic).
     **/
    std::string getTopic(void);

    /**
     * @brief Returns the CAN ID of the frame that this signal is part of.
     **/
    uint32_t getID(void);

    /**
     * @brief Fill a CAN frame with the signal's last received message.
     **/
    void fillFrameWithSignal(CANFrame *frame);

    /**
     * @brief Publishes a ROS message based on the received CAN frame (if the
     * CAN ID of the frame matches id_).
     **/
    void publish(CANFrame frame);

    /**
     * @brief ROS message callback.
     *
     * Receives a ROS message. The message corresponds to a signal, that is
     * packed into its CAN frame and forwarded to the callback function (if
     * any).
     **/
    void msgCallback(can::Float32Stamped msg);
  };
}

#endif
