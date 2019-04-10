/**
 * @file
 * @author Denise Ratasich
 * @date 27.01.2016
 *
 * @brief Implementation of a very simple CAN interface.
 **/

#include "can/CANInterface.h"

#include <stdexcept>

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can/raw.h>

namespace can
{
  CANInterface::CANInterface(std::string ifname)
  {
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct timeval recv_timeout;

    // create CAN socket
    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(fd_ == -1)
      throw std::runtime_error("CANInterface: Cannot open CAN socket. " + std::string(strerror(errno)));

    // set socket option (timeout on recv, with this option recv will not
    // restart when interrupted)
    recv_timeout.tv_sec = 60;
    recv_timeout.tv_usec = 0;
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout));

    // get interface index (e.g., from can0)
    strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) == -1)
      throw std::runtime_error("CANInterface: Getting interface index failed. " + std::string(strerror(errno)));

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // bind socket to a CAN interface
    if(bind(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
      throw std::runtime_error("CANInterface: Getting interface index failed. " + std::string(strerror(errno)));
  }

  CANInterface::~CANInterface(void)
  {
    if (close(fd_) == -1)
      throw std::runtime_error("CANInterface: Error while closing socket. " + std::string(strerror(errno)));
  }

  int CANInterface::recvFrame(canid_t *id, uint8_t data[8])
  {
    // saves received CAN frame
    int nbytes;
    struct can_frame frame;

    // receive CAN message (wait on whole frame)
    nbytes = recv(fd_, &frame, sizeof(struct can_frame), MSG_WAITALL);

    if (nbytes == 0) // nothing received and no error
      return 0;

    if (nbytes == -1) {
      if (errno == EAGAIN  ||  errno == EWOULDBLOCK) // timeout, no message available
	return -1;
      else if (errno == EINTR) // interrupted by signal
	return -1;
      else
	throw std::runtime_error("CANInterface: Receive failed. " + std::string(strerror(errno)));
    }

    // all bytes for a frame received?
    if (nbytes < sizeof(struct can_frame))
      throw std::runtime_error("CANInterface: Incomplete CAN frame. " + std::string(strerror(errno)));

    // set result
    *id = frame.can_id & 0x1FFFFFFF; // clear EFF/RTR/ERR flags
    for (int i = 0; i < 8; i++)
      data[i] = frame.data[i];

    return nbytes;
  }

  void CANInterface::sendFrame(canid_t id, uint8_t data[], uint8_t len)
  {
    int nbytes;
    struct can_frame frame;

    if (len > 8)
      throw std::runtime_error("CANInterface: Invalid length of data (>8).");

    frame.can_id  = id;
    frame.can_dlc = len;
    while (len > 0) {
      len--;
      frame.data[len] = data[len];
    }

    nbytes = send(fd_, &frame, sizeof(struct can_frame), 0);

    if (nbytes == -1)
      throw std::runtime_error("CANInterface: Send failed. " + std::string(strerror(errno)));

    if (nbytes < sizeof(struct can_frame))
      throw std::runtime_error("CANInterface: Incomplete frame sent. " + std::string(strerror(errno)));
  }

  void CANInterface::sendSingleValue(canid_t id, uint8_t data)
  {
    int nbytes;
    struct can_frame frame;

    frame.can_id  = id;
    frame.can_dlc = 1;
    frame.data[0] = data;

    nbytes = send(fd_, &frame, sizeof(struct can_frame), 0);

    if (nbytes == -1)
      throw std::runtime_error("CANInterface: Send failed. " + std::string(strerror(errno)));

    if (nbytes < sizeof(struct can_frame))
      throw std::runtime_error("CANInterface: Incomplete frame sent. " + std::string(strerror(errno)));
  }
}
