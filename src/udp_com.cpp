/// Copyright 2019 Continental AG
///
/// @file udp_com.cpp
///
/// @brief This file implements the udp_com class methods.
///
/*
 * Copyright (c) 2015, Hunter Laux
 * All rights reserved.
 */
#include <string>
#include <udp_com.h>
#include <vector>

namespace udp_com
{

UdpCom::UdpCom(std::string computer_address, std::string sensor_address,
               uint16_t port, bool isMulticast, ros::Publisher publisher)
    : io_service_(), socket_(io_service_), udp_packet_publisher_(publisher)
{
    // Store Computer IP Address to compare to requested sockets
    computer_address_ = computer_address;
    // convert addresses & ports
    const boost::asio::ip::address &multicast_addr = boost::asio::ip::address::from_string(sensor_address);
    const boost::asio::ip::address &receiver_addr = boost::asio::ip::address::from_string(computer_address);
    uint16_t multicast_port = boost::lexical_cast<uint16_t>(port);

    // Initialize endpoint
    udp::endpoint receiver_endpoint_(receiver_addr, multicast_port);

    // Open Socket
    socket_.open(receiver_endpoint_.protocol());

    // Allow other processes to reuse the address
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));

    // Set socket buffer size
    boost::asio::socket_base::receive_buffer_size r_option(16000000);
    socket_.set_option(r_option);

    // try to bind socket to endpoint
    try
    {
        socket_.bind(receiver_endpoint_);
    }
    catch (std::exception &e)
    {
        ROS_ERROR("COULD NOT BIND SOCKET TO ENDPOINT");
        ROS_ERROR(
            "Please Make sure your Computer's IPv4 Settings are set correctly.");
    }

    // Try to join the multicast group if there is one
    if (isMulticast)
    {
        try
        {
            // join multicast group
            boost::asio::ip::multicast::join_group option(receiver_addr.to_v4(),
                                                          multicast_addr.to_v4());
            socket_.set_option(option);
            ROS_INFO("*****JOINED MULTICAST GROUP*****");
        }
        catch (std::exception &e)
        {
            ROS_ERROR("No multicast group to join...");
            // Now that we have isMulticast set, if the group is not joined we should
            // do something about the error
        }
    }

    // Calls doReceive() function
    doReceive();

    // Initialize thread
    thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));

    // Initialize header message
    udp_packet_.header.seq = 0;
}

UdpCom::~UdpCom()
{
    io_service_.stop();
    thread_.interrupt();
}

void UdpCom::doReceive()
{
    // Receive UDP socket data
    udp_packet_.data.resize(1500);
    socket_.async_receive_from(
        boost::asio::buffer(udp_packet_.data), sender_endpoint_,
        [this](boost::system::error_code ec, std::size_t bytes_received)
        {
            // Populate UDP packet ROS message
            udp_packet_.header.stamp = ros::Time::now();
            udp_packet_.header.seq++;
            udp_packet_.data.resize(bytes_received);
            udp_packet_.address = sender_endpoint_.address().to_string();
            udp_packet_.srcPort = sender_endpoint_.port();

            // Publilshes UDP packet ROS message
            udp_packet_publisher_.publish(udp_packet_);
            doReceive();
        }
);
    boost::this_thread::interruption_point();
}

size_t UdpCom::send(const std::vector<u_char> &data,
                    const std::string &ip_address, const uint16_t port)
{
    // Create the remote endpoint so send data too
    udp::endpoint remote(boost::asio::ip::address::from_string(ip_address), port);
    return socket_.send_to(boost::asio::buffer(data), remote);
}
}  // namespace udp_com
