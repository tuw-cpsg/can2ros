/**
 * @file
 * @author Denise Ratasich
 * @date 27.01.2016
 *
 * @brief Header CAN interface.
 **/

#ifndef __CAN_INTERFACE_H__
#define __CAN_INTERFACE_H__

#include <string>
#include <stdint.h>
#include <linux/can.h>

namespace can
{
  class CANInterface
  {
  protected:
    /** CAN socket file descriptor. */
    int fd_;

  public:
    /**
     * @brief Initializes CAN interface.
     *
     * @param ifname Interface name, e.g., "can0".
     *
     * Throws exception if fails.
     **/
    CANInterface(std::string ifname);

    /**
     * @brief Destructor.
     **/
    virtual ~CANInterface();

    /**
     * @brief Reads CAN frames from socket.
     *
     * @param id CAN id of received message.
     * @param data Value of received message.
     * @return 0 if there is nothing to read, number of bytes read.
     *
     * On error an exception is thrown.
     **/
    int recvFrame(canid_t *id, uint8_t data[8]);

    /**
     * @brief Send CAN frame.
     *
     * @param id CAN id of message to send.
     * @param data Data array to fill into frame.
     * @param len Length of data (max 8 Bytes).
     *
     * Throws exception on error.
     **/
    void sendFrame(canid_t id, uint8_t data[], uint8_t len);

    /**
     * @brief Send CAN frame.
     *
     * @param id CAN id of message to send.
     * @param data Value to send.
     *
     * Throws exception on error.
     **/
    void sendSingleValue(canid_t id, uint8_t data);
  };
}

#endif
