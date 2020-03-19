#define _CRT_SECURE_NO_WARNINGS

// C/C++ includes
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <vector>

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#include "win_serial_fcns.h"

#elif defined(__linux__)
#include <sys/ioctl.h>
#include "linux_serial_fcns.h"

#if defined(USE_ROS)
// ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#endif

#endif

// custom includes
#include "sleep_ms.h"
#include "get_current_time.h"
#include "get_platform.h"
#include "num2string.h"
#include "file_parser.h"

// Project includes
#include "dwm.h"

// -------------------------------GLOBALS--------------------------------------

int main(int argc, char** argv)
{

    uint32_t idx, jdx;

    serial_port sp;
    uint32_t wait_time;
    uint32_t baud_rate = 115200;


#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
    wait_time = 10;
    std::string port_name = "COM8";

#elif defined(__linux__)
    //struct termios options;
    wait_time = 5;
    std:string port_name = "/dev/ttyACM0";
    
    
#if defined(USE_ROS)
    // initialize the ros node
    ros::init(argc, argv, "dwm");

    // NodeHandle is the main access point to communications with the ROS system
    ros::NodeHandle dwm_node;
    
    // setup the publisher to send out the 
    ros::Publisher dwm_pub = dwm_node.advertise<std_msgs::String>("range", 1);

    // the rate at which the message is published in Hz
    ros::Rate loop_rate(1);
    
#endif

#endif

    // dwm variables
    std::vector<dwm_version> version;
    std::vector<anchor_pos> anchor;
    dwm_position tag_position;

    try{

        // open the serial port
        std::cout << "opening port..." << std::endl;
        sp.open_port(port_name, baud_rate, wait_time);

        // get the firmware/config/hardware versions
        get_fw(sp, version);

        // get the position of the tag, the position of the anchors and distance of anchors to the tag
        get_pos(sp, tag_position, anchor);

        std::cout << "tag " << tag_position << std::endl;

        sleep_ms(200);
        //std::cout << "bytes avail: " << sp.bytes_available() << std::endl;
        for (idx = 0; idx < anchor.size(); ++idx)
        {
            std::cout << anchor[idx] << std::endl;
        }

#if defined(USE_ROS)
        while (ros::ok())
        {
            std_msgs::String msg;
            
            std::string position_msg = "";
            
            get_pos(sp, tag_position, anchor);

            for (idx = 0; idx < anchor.size(); ++idx)
            {
                position_msg = position_msg + "0x" + num2str<uint16_t>(anchor[idx].address, "%04X:") + num2str<float>(anchor[idx].range, "%2.4f,");
            }
            
            position_msg = position_msg.substr(0, position_msg.length()-2);            
                        
            msg.data = position_msg;
            
            dwm_pub.publish(msg);

            ros::spinOnce();

            loop_rate.sleep();

        }
#else

        while (1)
        {
            std::string position_msg = "";
            
            get_pos(sp, tag_position, anchor);

            for (idx = 0; idx < anchor.size(); ++idx)
            {
                position_msg = position_msg + "0x" + num2str<uint16_t>(anchor[idx].address, "%04X:") + num2str<float>(anchor[idx].range, "%2.4f,");
                //std::cout << "0x" << num2str<uint16_t>(anchor[idx].address, "%04X") << ":" << anchor[idx].range << ", ";
            }
            
            position_msg = position_msg.substr(0, position_msg.length()-2);
            std::cout << position_msg << std::endl;
            sleep_ms(100);
        }

#endif                
        
        // close the port
        sp.close_port();


    }
    catch(std::exception e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }

    std::cout << "Program Complete!  Press Enter to Close..." << std::endl;
    std::cin.ignore();

} // end of main

