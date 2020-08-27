#define _CRT_SECURE_NO_WARNINGS

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
#include <mutex>
#include <vector>


// Custom includes
#include "zed_wrapper.h"
//#include "get_platform.h"
//#include "get_current_time.h"
#include "num2string.h"
#include "file_ops.h"
#include "sleep_ms.h"


#if defined(USE_ROS)

// ROS includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <image_transport/image_transport.h>

// custom message headers
//#include "object_detect/object_det.h"
//#include "object_detect/object_det_list.h"

#endif

// OpenCV Includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda.hpp>

// ZED Includes
#include <sl/Camera.hpp>


// -------------------------------GLOBALS--------------------------------------



// ----------------------------------------------------------------------------



// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint64_t idx = 0, jdx = 0;

    // timing variables
    typedef std::chrono::duration<double> d_sec;
    auto start_time = std::chrono::system_clock::now();
    auto stop_time = std::chrono::system_clock::now();
    auto elapsed_time = std::chrono::duration_cast<d_sec>(stop_time - start_time);
    std::string sdate, stime;

    // ROS publisher topics
    static const std::string root_topic = "/zed2/zed_node/";
    static const std::string image_topic = root_topic + "left/image_rect_color";
    static const std::string depth_topic = root_topic + "depth/depth_registered";
    static const std::string cam_info_topic = root_topic + "rgb/camera_info";
    
    // Create a ZED camera object
    sl::Camera zed;
    sl::ERROR_CODE state;
        
    unsigned long img_h;
    unsigned long img_w;
       
    double rate = 10.0;

#if defined(USE_ROS)

    sensor_msgs::CameraInfo cam_info;

    // ----------------------------------------------------------------------------------------
    // configure the basic ROS stuff

    // initialize the ros node
    ros::init(argc, argv, "zed2");

    // NodeHandle is the main access point to communications with the ROS system
    ros::NodeHandle zed_node;
    
    // get the rate at which the frames are grabbed
    zed_node.param<double>("/zed2/loop_rate", rate, 5);       
    
    // ----------------------------------------------------------------------------------------
    // the rate at which the message is published in Hz
    ros::Rate loop_rate(rate);
        
    // setup the publisher to send out the target location messages
    ros::Publisher image_pub = zed_node.advertise<sensor_msgs::Image>(image_topic, 1);
    ros::Publisher depth_pub = zed_node.advertise<sensor_msgs::Image>(depth_topic, 1);
    ros::Publisher cam_info_pub = zed_node.advertise<sensor_msgs::CameraInfo>(cam_info_topic, 1);

#endif

    // Set camera configuration parameters
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;      // Use HD720 video mode
    init_parameters.camera_fps = rate;                              // Set the frame rate
    init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;       // Use PERFORMANCE depth mode
    init_parameters.coordinate_units = sl::UNIT::METER;             // Use meters for depth measurements
    

    try {

        // Open the camera
        state = zed.open(init_parameters);
        if (state != sl::ERROR_CODE::SUCCESS) {
            std::cout << "Error " << state << ", exit program." << std::endl;
            return EXIT_FAILURE;
        }
    
        // Set sensing mode in FILL
        sl::RuntimeParameters runtime_parameters;
        runtime_parameters.sensing_mode = sl::SENSING_MODE::FILL;
        
        // get the camera information
        auto cam_info = zed.getCameraInformation();

        img_w = cam_info.camera_resolution.width;
        img_h = cam_info.camera_resolution.height;

        sl::Mat zed_image(img_w, img_h, sl::MAT_TYPE::U8_C4);
        sl::Mat zed_dm(img_w, img_h, sl::MAT_TYPE::F32_C1);

        cv::Mat cv_image = slMat2cvMat(zed_image);
        cv::Mat cv_dm = slMat2cvMat(zed_dm);

        std::cout << std::endl;

        std::cout << "------------------------------------------------------------------" << std::endl;
        std::cout << "Camera Info:" << std::endl;
        std::cout << "Image Size (h x w): " << img_h << " x " << img_w << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

#if defined(USE_ROS)

        // start the main loop
        while (ros::ok())
        {

            // Grab an image
            state = zed.grab(runtime_parameters);
            
            // A new image is available if grab() returns ERROR_CODE::SUCCESS
            if (state == sl::ERROR_CODE::SUCCESS) 
            {

                // Get the left image
                zed.retrieveImage(zed_image, sl::VIEW::LEFT);
            
                // Retrieve depth map. Depth is aligned on the left image
                zed.retrieveMeasure(zed_dm, sl::MEASURE::DEPTH);

                cam_info.width = zed_image.getWidth();
                cam_info.height = zed_image.getHeight();
                               
                // publish the topics
                               
            }
            
            ros::spinOnce();
            
            loop_rate.sleep();

        }   // end of while(ros::ok())
#else
     
        char key = ' ';
        while(key != 'q')
        {
            // Grab an image
            //state = zed.grab(runtime_parameters);
            state = zed.grab();

            // A new image is available if grab() returns ERROR_CODE::SUCCESS
            if (state == sl::ERROR_CODE::SUCCESS) 
            {

                // Get the left image
                zed.retrieveImage(zed_image, sl::VIEW::LEFT);
            
                // Retrieve depth map. Depth is aligned on the left image
                zed.retrieveMeasure(zed_dm, sl::MEASURE::DEPTH);

                // Display image and depth using cv:Mat which share sl:Mat data
                cv::imshow("Image", cv_image);
                cv::imshow("Depth", cv_dm*(1.0f/20.0f));

                // Handle key event
                key = cv::waitKey(10);
               
            }            
        }

        cv::destroyAllWindows();

#endif
        // Close the camera
        zed.close();
        
        std::cout << "End of Program." << std::endl;

    }
    // catch(ros::Exception& e)
    // {
        // ROS_ERROR("ROS exception %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);

        ////std::cout << "Press Enter to close..." << std::endl;
        ////std::cin.ignore();
    // }
    catch(cv::Exception &e)
    {
        //ROS_ERROR("OpenCV exception %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
        std::cout << e.what() << std::endl;
    }

    std::cout << "Press Enter to Close..." << std::endl;

    std::cin.ignore();
    return 0;

}   // end of main
