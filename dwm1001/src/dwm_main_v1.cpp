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
#include <list>

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#include "win_serial_fcns.h"

#elif defined(__linux__)
#include <sys/ioctl.h>
#include "linux_serial_fcns.h"

#if defined(USE_ROS)
// ROS includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include "dwm_wrapper/target_location.h"
#include "dwm_wrapper/target_location_list.h"

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
#include "target_locator.h"

// -------------------------------GLOBALS--------------------------------------
std::vector<float> current_location(2);


#if defined(USE_ROS)
void pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{

    current_location[0] = msg->pose.pose.position.x;
    current_location[1] = msg->pose.pose.position.y;
    //current_location[2] = msg->pose.pose.position.z;

    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
}
#endif


// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    uint32_t idx, jdx;

    serial_port sp;
    uint32_t wait_time;
    uint32_t baud_rate = 115200;

    // dwm variables
    dwm_object tag;
    std::vector<dwm_location> anchor;
    //std::vector<dwm_version> version;
    //std::vector<anchor_pos> anchor;
    //dwm_location tag_position;

    list<target_locator> target;

    std::string x_input, y_input;
    float x = 0, y = 0;
    bool match = false;
    int32_t stop_code;

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
    
    // setup the publisher to send out the target location messages
    ros::Publisher dwm_pub = dwm_node.advertise<std_msgs::String>("range", 1);

    // setup the subscriber to the odomotry ROS topic to get the platform [x, y, z] location
    ros::Subscriber location_sub = dwm_node.subscribe("odom", 1, pose_callback);

    // the rate at which the message is published in Hz
    ros::Rate loop_rate(1);
    
#endif

#endif

    // ----------------------------------------------------------------------------------------
    try{

        // open the serial port
        std::cout << "opening port..." << std::endl;
        sp.open_port(port_name, baud_rate, wait_time);

        // get the firmware/config/hardware versions
        //tag.get_version(sp, version);
        tag.get_version(sp);
        tag.print_versions();

        // get the position of the tag, the position of the anchors and distance of anchors to the tag
        //get_position(sp, tag_position, anchor);
        tag.get_anchor_locations(sp, anchor);
        std::cout << "tag " << tag.position << std::endl;
        sleep_ms(200);

        current_location[0] = x;
        current_location[1] = y;

        //std::cout << "bytes avail: " << sp.bytes_available() << std::endl;
        for (idx = 0; idx < anchor.size(); ++idx)
        {
            std::cout << "anchor " << anchor[idx] << std::endl;

            // add the anchors to the target list
            target.push_back(target_locator(anchor[idx].address, observation(anchor[idx].range, current_location), { anchor[idx].range + current_location[0], current_location[1] }));
        }

#if defined(USE_ROS)
        while (ros::ok())
        {
            std_msgs::String msg;
            
            std::string position_msg = "";
            
            // get the position of the platform from the correct ROS topic


            //get_pos(sp, tag_position, anchor);
            tag.get_anchor_locations(sp, anchor);

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
            
            // programtically this is where the position of the platform should be determined
            // get_platform location            
            std::cout << "enter x: ";
            std::getline(std::cin, x_input);
            std::cout << "enter y: ";
            std::getline(std::cin, y_input);            
            
            try{
                current_location[0] = std::stof(x_input);
                current_location[1] = std::stof(y_input);
            }
            catch(...)
            {
                std::cout << "nope!" << std::endl;
            }
            
            
            //get_pos(sp, tag_position, anchor);
            tag.get_anchor_locations(sp, anchor);
            

            for (idx = 0; idx < anchor.size(); ++idx)
            {
                position_msg = position_msg + "0x" + num2str<uint16_t>(anchor[idx].address, "%04X:") + num2str<float>(anchor[idx].range, "%2.4f,");
                
                // this is where each anchor detection should be checked
                // 1. check the unique ID for the anchor against the unique ID for the target_locator object
                // if it matches then add the new observation and move on
                // if there is no match then add a brand new target_locator object and add the new observation
                if (target.size() > 0)
                {
                    match = false;
                    for (auto &t : target)
                    {
                        if (t.id == anchor[idx].address)
                        {
                            t.add_observation(observation(anchor[idx].range, current_location));
                            match = true;
                            break;
                        }
                    }
                    
                    if (match == false)
                    {
                        target.push_back(target_locator(anchor[idx].address, observation(anchor[idx].range, current_location), { anchor[idx].range + current_location[0], current_location[1] }));
                    }
                }
                else
                {
                    // add the very first detect automatically
                    target.push_back(target_locator(anchor[idx].address, observation(anchor[idx].range, current_location), { anchor[idx].range + current_location[0], current_location[1] }));
                }
                
            }
            
            for (auto& t : target)
            {
                stop_code = t.get_position();
                if (t.valid_location)
                {
                    std::cout << "target id: " << num2str(t.id, "0x%04X") << std::endl;
                    std::cout << "  x=" << num2str(t.location[0], "%3.6f") << " y=" << num2str(t.location[1], "%3.6f") << std::endl;
                }
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

