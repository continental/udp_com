/*
    Unit Test for the udp_com

* Copyright (c) 2019, Evan Flynn
* All rights reserved.
*/

#include <gtest/gtest.h>
#include <vector>
// Example of calling a header file from the driver
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "udp_com/UdpPacket.h"
#include "udp_com/UdpSend.h"
#include "udp_com/UdpSocket.h"

///
/// helper function to simulate a packet
///
auto createPacket = [](double length, uint8_t initialValue)
{
  std::vector<uint8_t> packet(length, initialValue);
  return packet;
};

///
/// ################################
///           Declare Tests
/// ################################
///

TEST(UdpTestSuite, doesCreateSocketServiceExist)
{
  ros::NodeHandle nh;
  // TODO(uia75303): the path /eno1/udp/create_socket is dependent on the
  // udp_com.test file if the udp_com node in that test file is launched in a
  // different namespace this test will fail
  ros::ServiceClient create_socket_client_ =
      nh.serviceClient<udp_com::UdpSocket>("udp/create_socket");
  // waits for 3 secs for service to exist
  ASSERT_EQ(create_socket_client_.waitForExistence(ros::Duration(3)), true);
}

TEST(UdpTestSuite, doesSendServiceExist)
{
  ros::NodeHandle nh;
  // TODO(uia75303): the path /eno1/udp/send is dependent on the udp_com.test
  // file if the udp_com node in that test file is launched in a different
  // namespace this test will fail
  ros::ServiceClient send_service_client_ =
      nh.serviceClient<udp_com::UdpSend>("udp/send");

  ASSERT_EQ(send_service_client_.waitForExistence(ros::Duration(3)), true);
}

TEST(UdpTestSuite, testCreateSocketService)
{
  ros::NodeHandle nh;
  // TODO(uia75303): the path /eno1/udp/send is dependent on the udp_com.test
  // file if the udp_com node in that test file is launched in a different
  // namespace this test will fail
  ros::ServiceClient create_socket_client_ =
      nh.serviceClient<udp_com::UdpSocket>("/eno1/udp/create_socket");
  udp_com::UdpSocket socket_request;

  // NOTE: this assumes the workstations IPv4 address is set to 127.0.0.1
  // and that port 6000 is available
  socket_request.request.srcAddress = "127.0.0.1";
  socket_request.request.destAddress = "127.0.0.2";
  socket_request.request.port = 54786;
  socket_request.request.isMulticast = false;

  if (create_socket_client_.waitForExistence(ros::Duration(3)) == true)
  {
    ASSERT_EQ(create_socket_client_.call(socket_request), true);
  }
}

TEST(UdpTestSuite, testSendService)
{
  ros::NodeHandle nh;
  // TODO(uia75303): the path /eno1/udp/send is dependent on the udp_com.test
  // file if the udp_com node in that test file is launched in a different
  // namespace this test will fail
  ros::ServiceClient send_service_client_ =
      nh.serviceClient<udp_com::UdpSend>("/eno1/udp/send");
  udp_com::UdpSend send_request;

  send_request.request.address = "127.0.0.2";
  send_request.request.srcPort = 54786;
  send_request.request.dstPort = 54786;
  send_request.request.data = createPacket(10, 1);

  if (send_service_client_.waitForExistence(ros::Duration(3)) == true)
  {
    ASSERT_EQ(send_service_client_.call(send_request), true);
  }
}
