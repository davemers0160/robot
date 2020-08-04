#ifndef _ZED_WRAPPER_H_
#define _ZED_WRAPPER_H_

// C/C++ includes
#include <cstdint>
#include <cmath>
#include <string>
#include <mutex>
#include <vector>

// Custom includes
#include "num2string.h"

// Net Version
#include "obj_det_net_rgb_v10.h"
//#include "tfd_net_v03.h"
#include "overlay_bounding_box.h"
#include "prune_detects.h"

// dlib includes
#include <dlib/dnn.h>
#include <dlib/image_transforms.h>

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

#include "object_detect/object_det.h"
#include "object_detect/object_det_list.h"

// OpenCV Includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// ZED camera include
#include <sl/Camera.hpp>


class zed_wrapper
{
    
public:

    sl::Camera zed;
    sl::RuntimeParameters runtime_parameters;
    
    bool valid_data;
    
    
    zed_wrapper() = default;
    
    
    

    // ----------------------------------------------------------------------------
    void init_zed()
    {
        // Set configuration parameters
        sl::InitParameters init_parameters;
        init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE; // Use PERFORMANCE depth mode
        init_parameters.coordinate_units = sl::UNIT::METER; // Use millimeter units (for depth measurements)
        init_parameters.camera_resolution = sl::RESOLUTION::HD720; // Use HD720 video mode: sl::RESOLUTION::VGA, sl::RESOLUTION::HD720, sl::RESOLUTION::HD1080
        init_parameters.camera_fps = 15; // Set fps at 30
        init_parameters.sensors_required = false;

        
        
        // Open the camera
        sl::ERROR_CODE state = zed.open(init_parameters);
        if (state != sl::ERROR_CODE::SUCCESS) {
            std::cout << "Error " << state << ", exit program." << std::endl;
            return EXIT_FAILURE;
        }
        
        // Set runtime parameters after opening the camera
        runtime_parameters.sensing_mode = SENSING_MODE::STANDARD; // Use STANDARD sensing mode
        
        
        
    }   // end of init_zed



    // ----------------------------------------------------------------------------
    void get_zed_data(dlib::matrix<dlib::rgb_pixel> &img, dlib::matrix<float> &dm)
    {
        
         // A new image is available if grab() returns ERROR_CODE::SUCCESS
        if (zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS) 
        {
            // Retrieve left image
            zed.retrieveImage(image, sl::VIEW::LEFT);

            // Retrieve depth map. Depth is aligned on the left image
            zed.retrieveMeasure(depth, sl::MEASURE::DEPTH);

            // Convert sl::Mat to cv::Mat (share buffer)
            cv::Mat cv_image = cv::Mat((int)zed_image.getHeight(), (int)zed_image.getWidth(), CV_8UC3, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
            
            dlib::assign_image(image, dlib::cv_image<dlib::bgr_pixel>(cv_image));
            
            // Convert sl::Mat to cv::Mat (share buffer)
            cv::Mat cv_depth = cv::Mat((int)depth.getHeight(), (int)depth.getWidth(), CV_32FC1, depth.getPtr<sl::float1>(sl::MEM::CPU));
            
            dlib::assign_image(image, dlib::cv_image<dlib::bgr_pixel>(cv_image));
            
            
        }   

    }   // end of get_zed_data

private:

    sl::Mat zed_image, zed_depth;
    
    dlib::matrix<dlib::rgb_pixel> image;
    dlib::matrix<float> depth;
    
    
}; 



#endif  // _ZED_WRAPPER_H_
