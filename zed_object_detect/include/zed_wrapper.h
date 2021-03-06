#ifndef ZED_WRAPPER_H_
#define ZED_WRAPPER_H_


#include <cstdint>
#include <string>
#include <vector>

// Custom includes
#include "num2string.h"
#include "dlib_pixel_operations.h"

// dlib includes
#include <dlib/matrix.h>
#include <dlib/image_transforms.h>

/* 
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

#endif 
*/

// ZED Includes
#include <sl/Camera.hpp>

// OpenCV Includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// -------------------------------GLOBALS--------------------------------------
#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
// opencv 4 vs opencv 3 naming convention change
#define CV_BGRA2RGB cv::COLOR_BGRA2RGB 
#endif

// ----------------------------------------------------------------------------
template <typename T>
double nan_mean(cv::Mat& img)
{
    uint64_t count = 0;
    double mn = 0;

    cv::MatIterator_<T> it;

    for (it = img.begin<T>(); it != img.end<T>(); ++it)
    {
        if (!std::isnan(*it))
        {
            mn += (double)*it;
            ++count;
        }
    }

    return (mn / (double)count);
}   // end of nan_mean


// ----------------------------------------------------------------------------
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) 
    {
    case sl::MAT_TYPE::F32_C1: 
        cv_type = CV_32FC1; 
        break;
    case sl::MAT_TYPE::F32_C2: 
        cv_type = CV_32FC2; 
        break;
    case sl::MAT_TYPE::F32_C3: 
        cv_type = CV_32FC3; 
        break;
    case sl::MAT_TYPE::F32_C4: 
        cv_type = CV_32FC4; 
        break;
    case sl::MAT_TYPE::U8_C1: 
        cv_type = CV_8UC1; 
        break;
    case sl::MAT_TYPE::U8_C2: 
        cv_type = CV_8UC2; 
        break;
    case sl::MAT_TYPE::U8_C3: 
        cv_type = CV_8UC3; 
        break;
    case sl::MAT_TYPE::U8_C4: 
        cv_type = CV_8UC4; 
        break;
    default: 
        break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

// ----------------------------------------------------------------------------

#endif  // ZED_WRAPPER_H_
