/// Copyright 2019 Continental AG
///
/// @file udp_com_nodelet.cpp
///
/// @brief This file implements the udp_com_nodelet class methods.
///
/*
 * Copyright (c) 2015, Hunter Laux
 * All rights reserved.
 */

#include <memory>
#include <udp_com_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace udp_com
{


void UdpComNodelet::onInit()
{
    // Get private node handler
    node_handler_ = getPrivateNodeHandle();

    // Advertize "send" service
    create_socket_service_ = node_handler_.advertiseService(
        "create_socket", &UdpComNodelet::createSocket, this);
    send_service_ =
        node_handler_.advertiseService("send", &UdpComNodelet::send, this);
}

///
/// Socket creation service function
///
/// @param[in] request UDP service request
/// @param[in] response UDP service response
///
/// @return bool true if request done
///
bool UdpComNodelet::createSocket(UdpSocket::Request &request,
                                 UdpSocket::Response &)
{
    ROS_INFO("Creating a UDP Socket for port: %i", request.port);
    if (udp_sockets_.find(request.port) != udp_sockets_.end())
    {
        ROS_ERROR("Socket with port: %i already exists.. ", request.port);
        if (udp_sockets_[request.port]->getIp() == request.srcAddress)
        {
            return true;  // return true since socket exists
        }
        else
        {
            ROS_ERROR(
                "Sockets with the same port but different source IP Addresses are "
                "currently not supported. ");
        }
    }
    try
    {
        ros::Publisher temp_pub = node_handler_.advertise<udp_com::UdpPacket>(
            "p" + std::to_string(request.port), 1000);

        // Create the Socket
        udp_sockets_[request.port] =
            std::make_shared<UdpCom>(request.srcAddress, request.destAddress,
                                     request.port, request.isMulticast, temp_pub);
        return true;
    }
    catch (std::exception e)
    {
        return false;
    }
    catch (...)
    {
        return false;
    }
}

bool UdpComNodelet::send(UdpSend::Request &request,
                         UdpSend::Response &response)
{
    ROS_DEBUG("sending data...");
    size_t bytes_sent;
    try
    {
        bytes_sent = udp_sockets_.at(request.srcPort)
                         ->send(request.data, request.address, request.dstPort);
    }
    catch (std::out_of_range e)
    {
        ROS_ERROR("Socket Requested was not found.");
        response.socketCreated = false;
        return false;
    }
    // No out of range error. Socket created
    response.socketCreated = true;

    // Return true if data sent
    if (bytes_sent > 0)
        return true;
    else
        return false;
}
}  // namespace udp_com
PLUGINLIB_EXPORT_CLASS(udp_com::UdpComNodelet, nodelet::Nodelet);
