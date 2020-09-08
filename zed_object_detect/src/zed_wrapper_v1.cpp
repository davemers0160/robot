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
#include "num2string.h"
#include "file_ops.h"
#include "sleep_ms.h"
#include "ocv_threshold_functions.h"
#include "color_match.h"

// Net Version
#include "obj_det_net_rgb_v10.h"
#include "overlay_bounding_box.h"

// dlib includes
#include <dlib/dnn.h>
#include <dlib/image_transforms.h>

// dlib-contrib includes
#include <dlib_pixel_operations.h>
#include <prune_detects.h>

#if defined(USE_ROS)

// ROS includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <zed_obj_det_project/zed_Config.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>

// custom message headers
#include "object_detect/object_det.h"
#include "object_detect/object_det_list.h"

#endif

// OpenCV Includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/core/cuda.hpp>

// ZED Includes
#include <sl/Camera.hpp>

// -------------------------------GLOBALS--------------------------------------
// global camera variable to allow dynamic reconfig of camera settings
sl::Camera zed;


// ----------------------Dynamic Reconfigure Callback--------------------------
#if defined(USE_ROS)

void reconfig_callback(zed_obj_det_project::zed_Config &config, uint32_t level)
{

    //frame_rate = config.pub_frame_rate;
/*    
    brightness = config.brightness;    
    contrast = config.contrast;
    hue = config.hue;
    saturation = config.saturation;
    sharpness = config.sharpness;
    gamma = config.gamma;
    auto_exposure_gain = config.auto_exposure_gain;
    gain = config.gain;
    exposure = config.exposure;
    auto_whitebalance = config.auto_whitebalance;
    whitebalance_temperature = config.whitebalance_temperature;
*/    
    switch(level)
    {
        case 4:
            zed.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, config.brightness);
            break;
            
        case 5:
            zed.setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, config.contrast);
            break;
            
        case 6:
            zed.setCameraSettings(sl::VIDEO_SETTINGS::HUE, config.hue);
            break;
            
        case 7:
            zed.setCameraSettings(sl::VIDEO_SETTINGS::SATURATION, config.saturation);
            break;
            
        case 8:
            zed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, config.sharpness);
            break;
            
        case 9:
            zed.setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, config.gamma);
            break;
            
        case 10:
            zed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, config.auto_exposure_gain);
            break;
            
        case 11:
            zed.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, config.gain);
            break;
            
        case 12:
            zed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, config.exposure);
            break;

        case 13:
            zed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, config.auto_whitebalance);
            break;
            
        case 14:
            zed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, config.whitebalance_temperature);
            break;                    

        default:
            break;
    
    }

}

#endif


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

    // ROS publisher topics - zed topics
    static const std::string zed_root_topic = "/zed2/zed_node/";
    static const std::string image_topic = zed_root_topic + "left/image_rect_color";
    static const std::string depth_topic = zed_root_topic + "depth/depth_registered";
    static const std::string cam_info_topic = zed_root_topic + "rgb/camera_info";
    // object detect topics
    static const std::string obj_root_topic = "/obj_det/";
    static const std::string image_det_topic = obj_root_topic + "image";
    static const std::string image_det_topic_raw = obj_root_topic + "image_raw";
    static const std::string razel_topic = obj_root_topic + "target_razel";
    static const std::string razel_topic_raw = obj_root_topic + "target_razel_raw";
    static const std::string dnn_input_topic = obj_root_topic + "dnn_input";
          
    // Create a ZED camera object
    //sl::Camera zed;
    sl::ERROR_CODE state;
        
    unsigned long img_h;
    unsigned long img_w;
    
    std::string net_file; 
    std::string box_string = "";

    anet_type net;

    int x_min, x_max;
    int y_min, y_max;
    int min_dim = 0;
    
    int crop_x = 128;
    int crop_y = 196;
    int crop_w = 1024;
    int crop_h = 512;
    
    dlib::point center;

    double az, el, range;
    
    double h_res = 0.1;
    double v_res = 0.1;
    
    double rate = 10.0;

#if defined(USE_ROS)

    sensor_msgs::CameraInfo cam_info;

    // ----------------------------------------------------------------------------------------
    // configure the basic ROS stuff

    // initialize the ros node
    ros::init(argc, argv, "zed_node");

    // NodeHandle is the main access point to communications with the ROS system
    ros::NodeHandle zed_node;
      
    // ----------------------------------------------------------------------------------------
    // setup the publisher to send out the target location messages
    //image_transport::ImageTransport image_it(zed_node);
    //image_transport::Publisher image_pub = image_it.advertise(image_topic, 1);
    //sensor_msgs::ImagePtr image_msg;
    
    //image_transport::Publisher depth_pub = image_it.advertise(depth_topic, 1);
    
    ros::Publisher image_det_pub = zed_node.advertise<sensor_msgs::Image>(image_det_topic, 1);
    ros::Publisher image_det_raw_pub = zed_node.advertise<sensor_msgs::Image>(image_det_topic_raw, 1);
    ros::Publisher depth_pub = zed_node.advertise<sensor_msgs::Image>(depth_topic, 1);
    
    ros::Publisher cam_info_pub = zed_node.advertise<sensor_msgs::CameraInfo>(cam_info_topic, 1);

    ros::Publisher razel_pub = zed_node.advertise<::object_detect::object_det_list>(razel_topic, 1);
    ros::Publisher razel_raw_pub = zed_node.advertise<::object_detect::object_det_list>(razel_topic_raw, 1);
        
    // get the required parameters 
    // get the rate at which the frames are grabbed
    zed_node.param<double>("/zed2/loop_rate", rate, 10);
    
    // get the network weights file
    zed_node.param<std::string>("/zed_node/net_file", net_file, "/home/jax/catkin_ws/src/robot/common/nets/dc3_rgb_v10e_035_035_100_90_HPC_final_net.dat");
    
    // get the cropping parameters
    zed_node.param<int>("/zed_node/crop_x", crop_x, 270);
    zed_node.param<int>("/zed_node/crop_y", crop_y, 200);
    zed_node.param<int>("/zed_node/crop_w", crop_w, 720);
    zed_node.param<int>("/zed_node/crop_h", crop_h, 420);
    
    // the rate at which the message is published in Hz
    ros::Rate loop_rate(rate);
    
    ::object_detect::object_det_list detect_list;
    ::object_detect::object_det_list detect_list_filtered;


    // setup the dynamic reconfigure
    dynamic_reconfigure::Server<zed_obj_det_project::zed_Config> server;
    dynamic_reconfigure::Server<zed_obj_det_project::zed_Config>::CallbackType f;

    f = boost::bind(&reconfig_callback, _1, _2);
    server.setCallback(f);
    
    
    //zed_node.param<int>("/brightness", brightness, 6);

#else
    net_file = "D:/Projects/robot/common/nets/dc3_rgb_v10e_035_035_100_90_HPC_final_net.dat";
#endif    

    dlib::rectangle crop_rect(crop_x, crop_y, crop_x + crop_w - 1, crop_y + crop_h - 1);

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

    cv::RNG rng(time(NULL));
    std::vector<cv::Scalar> class_color;

    class_color.push_back(cv::Scalar(0, 255, 0));
    class_color.push_back(cv::Scalar(0, 0, 255));
    // ----------------------------------------------------------------------------------------
    
    // Set camera configuration parameters
    CUcontext zed_cuda_context;
    cuCtxGetCurrent(&zed_cuda_context);

    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;      // Use HD720 video mode
    init_parameters.camera_fps = 15;                                // Set the frame rate
    init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;       // Use PERFORMANCE depth mode
    init_parameters.coordinate_units = sl::UNIT::METER;             // Use meters for depth measurements
    init_parameters.sdk_cuda_ctx = zed_cuda_context;                // set
    init_parameters.sdk_gpu_id = 0;                                 // set the gpu id to the first gpu

    try {

        // Open the camera
        state = zed.open(init_parameters);
        if (state != sl::ERROR_CODE::SUCCESS) 
        {
            std::cout << "Error " << state << ", exit program." << std::endl;
            return EXIT_FAILURE;
        }
    
        // Set sensing mode in FILL
        sl::RuntimeParameters runtime_parameters;
        runtime_parameters.sensing_mode = sl::SENSING_MODE::STANDARD;
        
        // get the camera information
        sl::CameraInformation camera_info = zed.getCameraInformation();
        
        img_w = camera_info.camera_configuration.resolution.width;
        img_h = camera_info.camera_configuration.resolution.height;
        h_res = 90.0/(double)img_w;
        v_res = 60.0/(double)img_h;
        
        sl::Mat zed_image(img_w, img_h, sl::MAT_TYPE::U8_C4);
        sl::Mat zed_dm(img_w, img_h, sl::MAT_TYPE::F32_C1);

        cv::Mat cv_image = slMat2cvMat(zed_image);
        cv::Mat cv_dm = slMat2cvMat(zed_dm);
        cv::Mat cv_raw;
        cv::Mat cv_filtered;

        std::cout << std::endl;

        std::cout << "------------------------------------------------------------------" << std::endl;
        std::cout << "Camera Info:" << std::endl;
        std::cout << "Image Size (h x w): " << img_h << " x " << img_w << std::endl;
        std::cout << "Angular Resolution (AZ, EL): " << h_res << ", " << v_res << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;
        
        dlib::matrix<dlib::rgb_pixel> rgb_img;
        dlib::matrix<uint32_t, 1, 2> cm;
                
        std::cout << "Running..." << std::endl << std::endl;
                
        bool valid_detect;
        char key = ' ';

        // start the main loop       
#if defined(USE_ROS)
        while (ros::ok())
        {
        
            detect_list.det.clear();
            detect_list_filtered.det.clear();
            
            // reconfigure the camera 
            //zed.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, brightness);
            
#else
        while (key != 'q')
        {
#endif


            
            // Grab an image
            state = zed.grab(runtime_parameters);
            
            // A new image is available if grab() returns ERROR_CODE::SUCCESS
            if (state == sl::ERROR_CODE::SUCCESS) 
            {
                // Get the left image
                zed.retrieveImage(zed_image, sl::VIEW::LEFT);
            
                // Retrieve depth map. Depth is aligned on the left image
                zed.retrieveMeasure(zed_dm, sl::MEASURE::DEPTH);

                // convert the zed image from BGRA into an RGB image
                cv::cvtColor(cv_image, cv_raw, CV_BGRA2RGB);
                cv::cvtColor(cv_image, cv_filtered, CV_BGRA2RGB);
                
                // crop and copy the RGB images for input into the dnn
                dlib::assign_image(rgb_img, dlib::cv_image<dlib::rgb_pixel>(cv_raw));
                rgb_img = dlib::subm(rgb_img, crop_rect);
                
                //run the detection
                start_time = std::chrono::system_clock::now();
                //dlib::equalize_histogram(rgb_img);
                std::vector<dlib::mmod_rect> d = net(rgb_img);
                stop_time = std::chrono::system_clock::now();
                elapsed_time = std::chrono::duration_cast<d_sec>(stop_time - start_time);

                //std::cout << "Run Time (s): " << elapsed_time.count() << std::endl;
                       
                prune_detects(d, 0.3);
                
                overlay_bounding_box(cv_filtered, dlib2cv_rect(crop_rect), "crop", cv::Scalar(0, 255, 255), false);
                               
                for (idx = 0; idx < d.size(); ++idx)
                {
                    auto class_index = std::find(class_names.begin(), class_names.end(), d[idx].label);
                    
                    valid_detect = false;
                    if (d[idx].label == "backpack")
                    {
                        cm = get_color_match(rgb_img, d[idx]);
                        //long index = dlib::index_of_max(cm);

                        //if (index != 1)
                        if(cm(0,0) >= cm(0,1))
                        {
                            valid_detect = true;
                        }

                    }
                    else
                    {
                        valid_detect = true;
                    }
                    
                    d[idx].rect = dlib::translate_rect(d[idx].rect, crop_x, crop_y);
                    overlay_bounding_box(cv_raw, dlib2cv_rect(d[idx].rect), d[idx].label, class_color[std::distance(class_names.begin(), class_index)]);

                    // get the center of the detection box
                    center = dlib::center(d[idx].rect);
                    
                    // get the rect coordinates and make sure that they are within the image bounds
                    //x_min = std::max((int)d[idx].rect.left(), min_dim);
                    //x_max = std::min((int)d[idx].rect.right(), (int)img_w);
                    //y_min = std::max((int)d[idx].rect.top(), min_dim);
                    //y_max = std::min((int)d[idx].rect.bottom(), (int)img_h);
                    x_min = std::max((int)(center.x()-10), min_dim);
                    x_max = std::min((int)(center.x()+10), (int)img_w);
                    y_min = std::max((int)(center.y()-10), min_dim);
                    y_max = std::min((int)(center.y()+10), (int)img_h);
                    
                    // fill in the box string message
                    //box_string = box_string + "{Class=" + d[idx].label + "; xmin=" + num2str(x_min,"%d") + ", ymin=" + num2str(y_min,"%d") + \
                    //             ", xmax=" + num2str(x_max,"%d") + ", ymax=" + num2str(y_max,"%d") + "},";

                    // crop the depthmap around the bounding box and get the range
                    cv::Range rows(y_min, y_max);
                    cv::Range cols(x_min, x_max);

                    cv::Mat tmp_dm = cv_dm(rows, cols);
                    cv::Mat sub_dm;
                    ranged_threshold<float>(tmp_dm, sub_dm, 0.0f, 25.0f);
                    range = nan_mean<float>(sub_dm);

                    az = h_res*(center.x() - (int64_t)(img_w>>1));
                    el = v_res*((int64_t)(img_h>>1) - center.y());
#if defined(USE_ROS)            
                    ::object_detect::object_det obj_det;
                    obj_det.label = d[idx].label;
                    obj_det.range = range;
                    obj_det.az = az;
                    obj_det.el = el;
                    detect_list.det.push_back(obj_det);
                    
                    if(valid_detect)
                    {
                        detect_list_filtered.det.push_back(obj_det);
                        overlay_bounding_box(cv_filtered, dlib2cv_rect(d[idx].rect), d[idx].label, class_color[std::distance(class_names.begin(), class_index)]);
                    }                 
#endif
                }

#if defined(USE_ROS)
                // header for dc_tracker
                detect_list.header.stamp = ros::Time::now();
                detect_list_filtered.header.stamp = ros::Time::now();
                
                // if the list is empty then there were no detects and we don't publish anything
                if(detect_list.det.size() > 0)
                {
                    //box_string = box_string.substr(0, box_string.length()-1);
                    //boxes_pub.publish(box_string);
                    
                    razel_raw_pub.publish(detect_list);
                }
                
                if(detect_list_filtered.det.size() > 0)
                {
                    razel_pub.publish(detect_list_filtered);
                }                

                // publish the topics
                image_det_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_filtered).toImageMsg());
                image_det_raw_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_raw).toImageMsg());

                depth_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "32FC1", cv_dm).toImageMsg());
#else
                // Display image and depth using cv:Mat which share sl:Mat data
                cv::imshow("Image", cv_image);
                cv::imshow("Depth", cv_dm * (1.0f / 20.0f));

                // wait for 1ms to display images
                key = cv::waitKey(1);
#endif                                               
            }

#if defined(USE_ROS)
            ros::spinOnce();            
            loop_rate.sleep();
#endif

        }   // end of while loop

        cv::destroyAllWindows();

        // Close the camera
        zed.close();
        


    }

#if defined(USE_ROS)
    catch(ros::Exception& e)
    {
        ROS_ERROR("ROS exception %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
    catch (cv::Exception& e)
    {
        ROS_ERROR("OpenCV exception %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
    }

#else
    catch (std::exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }

#endif

    std::cout << "End of Program." << std::endl;    

    std::cin.ignore();
    return 0;

}   // end of main
