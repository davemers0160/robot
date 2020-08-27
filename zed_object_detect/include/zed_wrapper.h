#ifndef ZED_WRAPPER_H_
#define ZED_WRAPPER_H_


#include <cstdint>
#include <cmath>
#include <string>
#include <mutex>
#include <vector>

// Custom includes
#include "num2string.h"
#include "dlib_pixel_operations.h"

// dlib includes
#include <dlib/matrix.h>
#include <dlib/image_transforms.h>

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

// ZED Includes
#include <sl/Camera.hpp>

// OpenCV Includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


// -------------------------------GLOBALS--------------------------------------
extern const uint32_t array_depth;

extern cv::Mat image;
extern cv::Mat depthmap;

extern bool valid_cam_info;
extern bool valid_images;


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
dlib::matrix<uint32_t, 1, 4> get_color_match(dlib::matrix<dlib::rgb_pixel>& img, dlib::mmod_rect& det)
{
    uint64_t r, c;

    dlib::hsi_pixel red_ll1(0, 0, 64);
    dlib::hsi_pixel red_ul1(15, 255, 255);
    dlib::hsi_pixel red_ll2(240, 0, 64);
    dlib::hsi_pixel red_ul2(255, 255, 255);

    dlib::hsi_pixel blue_ll(155, 0, 64);
    dlib::hsi_pixel blue_ul(185, 255, 255);

    //dlib::hsi_pixel black_ll(0, 0, 0);
    //dlib::hsi_pixel black_ul(255, 64, 48);
    dlib::rgb_pixel black_ll(0, 0, 0);
    dlib::rgb_pixel black_ul(48, 48, 48);


    dlib::hsi_pixel gray_ll(0, 0, 48);
    dlib::hsi_pixel gray_ul(255, 255, 128);
    //dlib::rgb_pixel gray_ll(65, 65, 65);
    //dlib::rgb_pixel gray_ul(128, 128, 128);

    const int w = 20, h = 20;

    dlib::matrix<uint16_t> red_mask = dlib::zeros_matrix<uint16_t>(h, w);
    dlib::matrix<uint16_t> blue_mask = dlib::zeros_matrix<uint16_t>(h, w);
    dlib::matrix<uint16_t> black_mask = dlib::zeros_matrix<uint16_t>(h, w);
    dlib::matrix<uint16_t> gray_mask = dlib::zeros_matrix<uint16_t>(h, w);

    // crop out the detection
    dlib::point ctr = dlib::center(det.rect);

    dlib::matrix<dlib::rgb_pixel> rgb_crop = dlib::subm(img, dlib::centered_rect(ctr, w, h));
    dlib::matrix<dlib::hsi_pixel> hsi_crop;
    dlib::assign_image(hsi_crop, rgb_crop);

    dlib::hsi_pixel p;
    dlib::rgb_pixel q;

    for (r = 0; r < hsi_crop.nr(); ++r)
    {
        for (c = 0; c < hsi_crop.nc(); ++c)
        {
            dlib::assign_pixel(p, hsi_crop(r, c));
            dlib::assign_pixel(q, rgb_crop(r, c));

            // test for red backpack
            if ((p >= red_ll1) && (p <= red_ul1))
            {
                red_mask(r, c) = 1;
            }
            else if ((p >= red_ll2) && (p <= red_ul2))
            {
                red_mask(r, c) = 1;
            }
            else if ((p >= blue_ll) && (p <= blue_ul))
            {
                blue_mask(r, c) = 1;
            }
            else if ((q >= black_ll) && (q <= black_ul))
            {
                black_mask(r, c) = 1;
            }
            //else if ((p >= gray_ll) && (p <= gray_ul))
            //{
            //    gray_mask(r, c) = 1;
            //}

        }
    }

    dlib::matrix<uint32_t, 1, 4> res;
    
    uint32_t mask_sum = (uint32_t)dlib::sum(red_mask) + (uint32_t)dlib::sum(blue_mask) + (uint32_t)dlib::sum(black_mask);
    //res = (uint32_t)dlib::sum(red_mask), (uint32_t)dlib::sum(blue_mask), (uint32_t)dlib::sum(black_mask), (uint32_t)dlib::sum(gray_mask);
    res = mask_sum, (uint32_t)(rgb_crop.size() - mask_sum), 0, 0;
    return res;

}   // end of get_color_match


#endif  // ZED_WRAPPER_H_
