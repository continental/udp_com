/// Copyright 2019 Continental AG
///
/// @file udp_com_nodelet.h
///
/// @brief Implements the create_socket and send ROS services
///
/*
 * Copyright (c) 2015, Hunter Laux
 * All rights reserved.
 */

#ifndef UDP_COM_NODELET_H
#define UDP_COM_NODELET_H

#include <udp_com.h>
#include <map>
#include <nodelet/nodelet.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <udp_com/UdpPacket.h>
#include <udp_com/UdpSend.h>
#include <udp_com/UdpSocket.h>

namespace udp_com
{
///
/// @brief Implements the udp_com create_socket and send services
///
class UdpComNodelet : public nodelet::Nodelet
{
public:
    ///
    /// Initializer constructor.
    ///
    UdpComNodelet() {}

    ///
    /// Initialize destructor
    ///
    ~UdpComNodelet() {}

    ///
    /// Nodelet initalization function
    ///
    /// @return void
    ///
    void onInit();

private:
    /// ROS send service
    ros::ServiceServer send_service_;

    /// ROS socket creation service
    ros::ServiceServer create_socket_service_;

    /// ROS node handle
    ros::NodeHandle node_handler_;

    /// Map of all the created UDP Sockets
    std::map<uint16_t, std::shared_ptr<UdpCom>> udp_sockets_;

    ///
    /// Send service function
    ///
    /// @param[in] request UDP service request
    /// @param[in] response UDP service response
    ///
    /// @return True if request was sent
    ///
    bool send(UdpSend::Request &request, UdpSend::Response &);

    ///
    /// Socket creation service function
    ///
    /// @param[in] request UDP service request
    /// @param[in] response UDP service response
    ///
    /// @return True if socket was created
    ///
    bool createSocket(UdpSocket::Request &request, UdpSocket::Response &response);
};

}  // namespace udp_com

#endif  // UDP_COM_NODELET_H
