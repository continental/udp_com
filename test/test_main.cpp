/*
* Unit Test for the imageprocessor

* Copyright (c) 2019, Evan Flynn
* All rights reserved.
*/

#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "UdpComTest");
    ros::AsyncSpinner spinner(1);

    // thread t([]{while(ros::ok()) ros::spin();});
    // auto res = RUN_ALL_TESTS();
    spinner.start();
    int ret = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();
    return ret;
}
