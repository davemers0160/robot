#define _CRT_SECURE_NO_WARNINGS

// #if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
// #include <windows.h>
// #endif

#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <thread>
#include <sstream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <string>
#include <utility>
#include <stdexcept>


// Net Version
#include "obj_det_net_v10.h"
//#include "tfd_net_v03.h"


// Custom includes
#include "obj_det_run.h"
#include "get_platform.h"
#include "get_current_time.h"
#include "num2string.h"
#include "file_ops.h"
#include "sleep_ms.h"


// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// object detector library header
//#include "obj_det_lib.h"
//extern const uint32_t array_depth = 1;

// OpenCV Includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// -------------------------------GLOBALS--------------------------------------

// ROS publishers
// ros::Publisher image_det_pub;
// ros::Publisher boxes_pub;
// ros::Publisher razel_pub;


// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void print_usage(void)
{
    std::cout << "Enter the following as arguments into the program:" << std::endl;
    std::cout << "<input file name> " << std::endl;
    std::cout << endl;
}

// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint64_t idx = 0, jdx = 0;

    // timing variables
    typedef std::chrono::duration<double> d_sec;
    auto start_time = chrono::system_clock::now();
    auto stop_time = chrono::system_clock::now();
    auto elapsed_time = chrono::duration_cast<d_sec>(stop_time - start_time);
    std::string sdate, stime;

    // ROS subscriber topic names
    static std::string image_topic;
    static std::string depth_topic;
    static std::string cam_info_topic;

    // ROS publisher topics
    // static const std::string root_topic = "/obj_det/";
    // static const std::string image_det_topic = root_topic + "image";
    // static const std::string boxes_topic = root_topic + "boxes";
    // static const std::string razel_topic = root_topic + "target_razel";

    std::string cam_type;
    std::string net_file;

    // ----------------------------------------------------------------------------------------
    if (argc == 1)
    {
        print_usage();
        std::cin.ignore();
        return 0;
    }


    // ----------------------------------------------------------------------------------------
    // configure the basic ROS stuff

    //std::string pose_topic = "obj_det";

    // the message to be published
    //::dwm_wrapper::point_array point_array_msg;

    // initialize the ros node
    ros::init(argc, argv, "obj_det");

    // NodeHandle is the main access point to communications with the ROS system
    ros::NodeHandle obj_det_node;


    // get the required parameters /enemy_locations/max_observations
    obj_det_node.param<std::string>("/obj_det/cam_type", cam_type, "/zed/");
    obj_det_node.param<std::string>("/obj_det/net_file", net_file, "../nets/dc_3_v10_20_20_100_Laptop_final_net.dat");
    //dwm_node.param("obj_det/max_observations", max_obs, 20);

    image_topic = cam_type + "zed_node/rgb/image_rect_color";
    depth_topic = cam_type + "zed_node/depth/depth_registered";
    cam_info_topic = cam_type + "zed_node/rgb/camera_info";

    /*
    // setup the subscriber to the odomotry ROS topic to get the platform [x, y, z] location
    ros::Subscriber image_sub = obj_det_node.subscribe(image_topic, 1, run_net_callback);
    ros::Subscriber depth_sub = obj_det_node.subscribe(depth_topic, 1, run_net_callback);
    ros::Subscriber cam_info_sub = obj_det_node.subscribe(cam_info_topic, 1, run_net_callback);


    // setup the publisher to send out the target location messages
    image_det_pub = obj_det_node.advertise<sensor_msgs::Image>(image_det_topic, 1);
    boxes_pub = obj_det_node.advertise<std::string>(boxes_topic, 1);
    razel_pub = obj_det_node.advertise<::obj_det_wrapper::object_det_list>(razel_topic, 1);


    // the rate at which the message is published in Hz
    ros::Rate loop_rate(1);
    */

    try {

        // create the object detector class
        //const auto obj_det = std::make_shared<object_detector>(obj_det_node, image_topic, depth_topic, cam_info_topic);
        object_detector obj_det(obj_det_node, image_topic, depth_topic, cam_info_topic);

        // initialize the object detector network
        obj_det.init(net_file);

        // run the detections
        obj_det.run();

        std::cout << "End of Program." << std::endl;
        //std::cin.ignore();

    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;

        std::cout << "Press Enter to close..." << std::endl;
        std::cin.ignore();
    }

    return 0;

}   // end of main
