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
#include <mutex>
#include <vector>


// object detector library header
//#include "obj_det_lib.h"

// Net Version
#include "obj_det_net_rgb_v10.h"
//#include "tfd_net_v03.h"
#include "overlay_bounding_box.h"
#include "prune_detects.h"

// Custom includes
#include "obj_det_run.h"
#include "get_platform.h"
#include "get_current_time.h"
#include "num2string.h"
#include "file_ops.h"
#include "sleep_ms.h"
#include "ocv_threshold_functions.h"

// dlib includes
#include <dlib/dnn.h>
#include <dlib/image_transforms.h>

// dlib-contrib includes
#include <array_image_operations.h>

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

// custum message headers
#include "object_detect/object_det.h"
#include "object_detect/object_det_list.h"


// OpenCV Includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda.hpp>

// -------------------------------GLOBALS--------------------------------------

//sensor_msgs::CameraInfo cam_info;

//extern bool valid_cam_info;
//extern bool valid_images;

//extern cv::Mat image;
//extern cv::Mat depthmap;

// ----------------------------------------------------------------------------
/*
void get_images_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dm)
{
        try
        {
            auto tmp_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            auto tmp_dm = cv_bridge::toCvCopy(dm, sensor_msgs::image_encodings::TYPE_32FC1);

            // it is very important to lock the below assignment operation.
            // remember that we are accessing it from another thread too.
            //std::lock_guard<std::mutex> lock(mtx);
            image = tmp_img->image;
            depthmap = tmp_dm->image;

            valid_images = true;
        }
        catch (cv_bridge::Exception& e)
        {
            // display the error at most once per 10 seconds
            ROS_ERROR_THROTTLE(10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
        }

    valid_images = true;
}   // end of get_images_callback
*/


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
    static const std::string root_topic = "/obj_det/";
    static const std::string image_det_topic = root_topic + "image";
    static const std::string boxes_topic = root_topic + "boxes";
    static const std::string razel_topic = root_topic + "target_razel";
    static const std::string dnn_input_topic = root_topic + "dnn_input";

    std::string cam_type;
    std::string net_file;

    //valid_cam_info = false;
    //valid_images = false;

    std::string box_string = "";

    anet_type net;

    int x_min, x_max, min_dim;
    int y_min, y_max;
    
    unsigned long img_h = 720;
    unsigned long img_w = 1280;
    
    int crop_x, crop_y, crop_w, crop_h;

    dlib::point center;

    double az, el, range;

    long nr, nc, r, c;

    double h_res = 0.1;
    double v_res = 0.1;
    
    double rate = 10.0;


    // ----------------------------------------------------------------------------------------
    // configure the basic ROS stuff

    // initialize the ros node
    ros::init(argc, argv, "obj_det");

    // NodeHandle is the main access point to communications with the ROS system
    ros::NodeHandle obj_det_node;
    
    obj_det_node.param<double>("/obj_det/loop_rate", rate, 5);       

    // get the required parameters /enemy_locations/max_observations
    obj_det_node.param<std::string>("/obj_det/img_topic", image_topic, "/zed/zed_node/left/image_rect_color");
    obj_det_node.param<std::string>("/obj_det/depth_topic", depth_topic, "/zed/zed_node/depth/depth_registered");
    obj_det_node.param<std::string>("/obj_det/cam_info_topic", cam_info_topic, "/zed/zed_node/rgb/camera_info");
    
    // get the network weights file
    obj_det_node.param<std::string>("/obj_det/net_file", net_file, "../src/robot/obj_det/nets/dc3_rgb_v10_40_40_100_HPC_final_net.dat");
    
    // get the cropping parameters
    obj_det_node.param<int>("/obj_det/crop_x", crop_x, 270);
    obj_det_node.param<int>("/obj_det/crop_y", crop_y, 0);
    obj_det_node.param<int>("/obj_det/crop_w", crop_w, 720);
    obj_det_node.param<int>("/obj_det/crop_h", crop_h, 720);
    
    dlib::rectangle crop_rect(crop_x, crop_y, crop_x + crop_w + 1, crop_y + crop_h - 1);
    
    // ----------------------------------------------------------------------------------------
    // initialize the network
    dlib::deserialize(net_file) >> net;

    // get the details about the loss layer -> the number and names of the classes
    dlib::mmod_options options = dlib::layer<0>(net).loss_details().get_options();

    std::set<std::string> tmp_names;
    std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
    for (uint64_t idx = 0; idx < options.detector_windows.size(); ++idx)
    {
        std::cout << "detector window (w x h): " << options.detector_windows[idx].label << " - " << options.detector_windows[idx].width 
                  << " x " << options.detector_windows[idx].height << std::endl;
        tmp_names.insert(options.detector_windows[idx].label);
    }
    std::cout << "------------------------------------------------------------------" << std::endl;

    // pull out the class names
    std::vector<std::string> class_names;
    for (const auto &it : tmp_names)
    {
        class_names.push_back(it);
    }

    //class_names.push_back("box");
    //class_names.push_back("backpack");

    //dlib::rand rnd(time(NULL));
    cv::RNG rng(time(NULL));
    std::vector<cv::Scalar> class_color;
    // for (uint64_t idx = 0; idx < class_names.size(); ++idx)
    // {
        // class_color.push_back(cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256)));
    // }

    class_color.push_back(cv::Scalar(0, 255, 0));
    class_color.push_back(cv::Scalar(0, 0, 255));
    // ----------------------------------------------------------------------------------------

    // the rate at which the message is published in Hz
    ros::Rate loop_rate(rate);
    ::object_detect::object_det_list detect_list;

    msg_listener ml;

    try {

        // setup the publisher to send out the target location messages
        ros::Publisher image_det_pub = obj_det_node.advertise<sensor_msgs::Image>(image_det_topic, 1);
        ros::Publisher boxes_pub = obj_det_node.advertise<std_msgs::String>(boxes_topic, 1);
        ros::Publisher razel_pub = obj_det_node.advertise<::object_detect::object_det_list>(razel_topic, 1);
        ros::Publisher dnn_input_pub = obj_det_node.advertise<sensor_msgs::Image>(dnn_input_topic, 1);

        ml.init(obj_det_node, image_topic, depth_topic);

        // setup the subscribers
        //ros::Subscriber cam_info_sub = obj_det_node.subscribe<sensor_msgs::CameraInfo>(cam_info_topic, 1, &object_detector::get_cam_info_cb, this);

        //message_filters::Subscriber<sensor_msgs::Image> image_sub(obj_det_node, image_topic, 1);
        //message_filters::Subscriber<sensor_msgs::Image> depth_sub(obj_det_node, depth_topic, 1);

        // create the synchronization policy
        //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> image_sync_policy;
        //message_filters::Synchronizer<image_sync_policy> sync(image_sync_policy(1), image_sub, depth_sub);
        //message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub, depth_sub, 1);
        //sync.registerCallback(boost::bind(&msg_listener::get_images_callback, ml, _1, _2));

        // get the image info
        //std::cout << std::endl << "Waiting for Camera Info...";
        boost::shared_ptr<sensor_msgs::CameraInfo const> cam_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic, obj_det_node, ros::Duration(5));
        sensor_msgs::CameraInfo cam_info;

        if(cam_info_ptr != NULL)
        {
            cam_info = *cam_info_ptr;
        }

        //while(!valid_cam_info)
        //{
        //    std::cout << ".";
            //ros::spinOnce();
        //}
        std::cout << std::endl;

        img_w = cam_info.width;
        img_h = cam_info.height;
        h_res = 90.0/(double)img_w;
        v_res = 60.0/(double)img_h;

        std::cout << "------------------------------------------------------------------" << std::endl;
        std::cout << "Camera Info:" << std::endl;
        std::cout << "Image Size (h x w): " << img_h << " x " << img_w << std::endl;
        std::cout << "Angular Resolution (AZ, EL): " << h_res << ", " << v_res << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

        ml.valid_images = false;

        //std::array<dlib::matrix<uint8_t>, array_depth> a_img;
        dlib::matrix<dlib::rgb_pixel> rgb_img;

        while (ros::ok())
        {


            detect_list.det.clear();

            box_string = "";

            if(ml.valid_images)
            {

                ml.mtx.lock();
                // check to make sure that the images are the correct sizes and have data
                if(ml.image.empty() || ml.depthmap.empty() || ml.image.rows != img_h || ml.image.cols != img_w)
                {
                    std::cout << "error processing image..." << std::endl;
                    continue;
                }
                
                // copy the image to a dlib array matrix for input into the dnn
                //unsigned char *img_ptr = ml.image.ptr<unsigned char>(0);

                //std::vector<cv::Mat> rgb(3);
                
                //cv::Mat cv_img = ml.image.clone();
                
                //cv::split(cv_img, rgb);

                // crop and copy the RGB images
                //dlib::assign_image(a_img[0], dlib::subm(dlib::mat(rgb[0].ptr<unsigned char>(0), rgb[0].rows, rgb[0].cols), crop_rect) );
                //dlib::assign_image(a_img[1], dlib::subm(dlib::mat(rgb[1].ptr<unsigned char>(0), rgb[1].rows, rgb[1].cols), crop_rect) );
                //dlib::assign_image(a_img[2], dlib::subm(dlib::mat(rgb[2].ptr<unsigned char>(0), rgb[2].rows, rgb[2].cols), crop_rect) );
                //dlib::assign_image(rgb_img, dlib::subm(dlib::cv_image<dlib::rgb_pixel>(cv_img), crop_rect) );
                dlib::assign_image(rgb_img, dlib::cv_image<dlib::rgb_pixel>(ml.image));

                rgb_img = dlib::subm(rgb_img, crop_rect);
                
                //run the detection
                start_time = chrono::system_clock::now();
                //dlib::equalize_histogram(rgb_img);
                std::vector<dlib::mmod_rect> d = net(rgb_img);
                stop_time = chrono::system_clock::now();
                elapsed_time = chrono::duration_cast<d_sec>(stop_time - start_time);

                std::cout << "Run Time (s): " << elapsed_time.count() << std::endl;

                // simulate a detection of each type
                //std::vector<dlib::mmod_rect> d;
                //d.push_back(dlib::mmod_rect(dlib::rectangle(20,20,100,100), 0.0, "box"));
                //d.push_back(dlib::mmod_rect(dlib::rectangle(100,100,200,200), 0.0, "backpack"));
                
                prune_detects(d, 0.3);
                
                overlay_bounding_box(ml.image, dlib2cv_rect(crop_rect), "crop", cv::Scalar(0, 255, 255), false);
                                    
                for (idx = 0; idx < d.size(); ++idx)
                {
                    auto class_index = std::find(class_names.begin(), class_names.end(), d[idx].label);
                    d[idx].rect = dlib::translate_rect(d[idx].rect, crop_x, crop_y);
                    overlay_bounding_box(ml.image, dlib2cv_rect(d[idx].rect), d[idx].label, class_color[std::distance(class_names.begin(), class_index)]);

                    // get the rect coordinates and make sure that they are within the image bounds
                    x_min = std::max((int)d[idx].rect.left(), min_dim);
                    x_max = std::min((int)d[idx].rect.right(), (int)img_w);
                    y_min = std::max((int)d[idx].rect.top(), min_dim);
                    y_max = std::min((int)d[idx].rect.bottom(), (int)img_h);

                    // fill in the box string message
                    box_string = box_string + "{Class=" + d[idx].label + "; xmin=" + num2str(x_min,"%d") + ", ymin=" + num2str(y_min,"%d") + \
                                 ", xmax=" + num2str(x_max,"%d") + ", ymax=" + num2str(y_max,"%d") + "},";

                    // crop the depthmap around the bounding box and get the range
                    cv::Range rows(y_min, y_max);
                    cv::Range cols(x_min, x_max);

                    cv::Mat tmp_dm = ml.depthmap(rows, cols);
                    cv::Mat sub_dm;
                    ranged_threshold<float>(tmp_dm, sub_dm, 0.0f, 25.0f);
                    range = nan_mean<float>(sub_dm);

                    center = dlib::center(d[idx].rect);

                    az = h_res*(center.x() - (int64_t)(img_w>>1));
                    el = v_res*((int64_t)(img_h>>1) - center.y());

                    ::object_detect::object_det obj_det;
                    obj_det.label = d[idx].label;
                    obj_det.range = range;
                    obj_det.az = az;
                    obj_det.el = el;
                    detect_list.det.push_back(obj_det);

                }

                // if the list is empty then there were no detects and we don't publish anything
                if(detect_list.det.size() > 0)
                {
                    box_string = box_string.substr(0, box_string.length()-1);
                    boxes_pub.publish(box_string);
                    
                    razel_pub.publish(detect_list);
                }
                
                
                // always publish the image topic
                image_det_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", ml.image).toImageMsg());
                dnn_input_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", dlib::toMat(rgb_img)).toImageMsg());
                
                ml.mtx.unlock();
                ml.valid_images = false;
            }

            ros::spinOnce();
            
            loop_rate.sleep();

        }

        std::cout << "End of Program." << std::endl;

    }
    catch(ros::Exception& e)
    {
        ROS_ERROR("ROS exception %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
        //std::cout << e.what() << std::endl;

        //std::cout << "Press Enter to close..." << std::endl;
        //std::cin.ignore();
    }
    catch(cv::Exception &cve)
    {
        ROS_ERROR("OpenCV exception %s at line number %d on function %s in file %s", cve.what(), __LINE__, __FUNCTION__, __FILE__);    
    }

    return 0;

}   // end of main
