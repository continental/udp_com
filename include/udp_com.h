/// Copyright 2019 Continental AG
///
/// @file udp_com.h
///
/// @brief Implements the UDP methods for recieving and sending
/// within the ROS ecosystem.
///
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright 2015 Hunter Laux
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of owner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#ifndef UDP_COM_H
#define UDP_COM_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
// WANT TO REMOVE:
#include <ros/ros.h>
#include <udp_com/UdpPacket.h>
#include <udp_com/UdpSend.h>
#include <udp_com/UdpSocket.h>

using boost::asio::ip::udp;

namespace udp_com
{

///
/// @brief Implements the UDP methods for recieving and sending
/// within the ROS ecosystem.
///
class UdpCom
{
public:
    ///
    /// Initializer constructor.
    ///
    /// @param[in] computer_address computer ip address
    /// @param[in] sensor_address camera ip address
    /// @param[in] port camera UDP port number
    /// @param[in] is_multicast multicast flag
    /// @param[in] publisher ROS publisher
    ///
    UdpCom(std::string computer_address, std::string sensor_address,
           uint16_t port, bool is_multicast, ros::Publisher publisher);

    ///
    /// Destructor.
    ///
    ~UdpCom();

    ///
    /// Recieves and publishes the UDP data.
    ///
    /// Formats the recieved UDP data and publishes it
    /// as a ROS message.
    ///
    /// @return void
    ///
    void doReceive();

    ///
    /// Sends data through UDP protocol.
    ///
    /// Service send function for UDP data sending.
    ///
    /// @param[in] data The data to be sent.
    /// @param[in] ip_address The devide IP address.
    /// @param[in] udp_port The device port.
    ///
    /// @return size_t the number of bytes sent
    ///
    size_t send(const std::vector<u_char> &data, const std::string &ip_address,
                const uint16_t udp_port);

    /// Returns the src IP for the created socket
    std::string getIp()
    {
       return computer_address_;
    }

private:
    /// UDP endpoint receiver
    udp::endpoint receiver_endpoint_;

    /// UDP endpoint sender
    udp::endpoint sender_endpoint_;

    /// ROS publisher
    ros::Publisher udp_packet_publisher_;

    /// UDP packet to be sent
    UdpPacket udp_packet_;

    /// UDP IO service
    boost::asio::io_service io_service_;

    /// UDP socket
    udp::socket socket_;

    /// Thread for service
    boost::thread thread_;

    /// Source (computer) IP Address
    std::string computer_address_;
};

}  // namespace udp_com

#endif  // UDP_COM_H
