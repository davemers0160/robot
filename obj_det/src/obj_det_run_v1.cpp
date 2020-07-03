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

//sensor_msgs::CameraInfo cam_info;

bool valid_cam_info = false;
bool valid_images = false;

cv::Mat image;
cv::Mat depthmap;

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
    static const std::string root_topic = "/obj_det/";
    static const std::string image_det_topic = root_topic + "image";
    static const std::string boxes_topic = root_topic + "boxes";
    static const std::string razel_topic = root_topic + "target_razel";

    std::string cam_type;
    std::string net_file;
    
    std::string box_string = "";

    anet_type net;

    uint64_t x_min, x_max;
    uint64_t y_min, y_max;
    uint64_t img_h = 720;
    uint64_t img_w = 1280;

    dlib::point center;

    double az, el, range;

    long nr, nc, r, c;
    
    double h_res = 0.1;
    double v_res = 0.1;

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

    */
    
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
    ros::Rate loop_rate(1);
    ::object_detect::object_det_list detect_list;

    try {

        // setup the publisher to send out the target location messages
        ros::Publisher image_det_pub = obj_det_node.advertise<sensor_msgs::Image>(image_det_topic, 1);
        ros::Publisher boxes_pub = obj_det_node.advertise<std_msgs::String>(boxes_topic, 1);
        ros::Publisher razel_pub = obj_det_node.advertise<::object_detect::object_det_list>(razel_topic, 1);

        // setup the subscribers
        //ros::Subscriber cam_info_sub = obj_det_node.subscribe<sensor_msgs::CameraInfo>(cam_info_topic, 1, &object_detector::get_cam_info_cb, this);

        message_filters::Subscriber<sensor_msgs::Image> image_sub(obj_det_node, image_topic, 1);
        message_filters::Subscriber<sensor_msgs::Image> depth_sub(obj_det_node, depth_topic, 1);

        message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub, depth_sub, 1);
        sync.registerCallback(boost::bind(&get_images_callback, _1, _2));

        // get the image info
        std::cout << "Waiting for Camera Info...";
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
        std::cout << std::endl << "Camera Info:" << std::endl;
        std::cout << "Image Size (h x w): " << img_h << " x " << img_w << std::endl;
        std::cout << "Angular Resolution (AZ, EL): " << h_res << ", " << v_res << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl;

/*        
        while (ros::ok())
        {
            detect_list.det.clear();
            
            box_string = "";
           
            if(valid_images)
            {
                

                // copy the image to a dlib array matrix for input into the dnn
                unsigned char *img_ptr = img.ptr<unsigned char>(0);

                r = 0;
                c = 0;
                
                // for (idx = 0; idx < img_w*img_h*3; idx+=3)
                // {

                    // a_img[0](r, c) = *(img_ptr + idx + 2);  //*test_img.ptr<unsigned char>(idx);
                    // a_img[1](r, c) = *(img_ptr + idx + 1);  //*test_img.ptr<unsigned char>(idx+1);
                    // a_img[2](r, c) = *(img_ptr + idx);      //*test_img.ptr<unsigned char>(idx+2);

                    // ++c;

                    // if (c >= img_w)
                    // {
                        // c = 0;
                        // ++r;
                    // }

                // }                

                ////run the detection
                // std::vector<dlib::mmod_rect> d = net(a_img);
                // prune_detects(d, 0.3);

                // simulate a detection of each type
                std::vector<dlib::mmod_rect> d;
                d.push_back(dlib::mmod_rect(dlib::rectangle(20,20,100,100), 0.0, "box"));
                d.push_back(dlib::mmod_rect(dlib::rectangle(100,100,200,200), 0.0, "backpack"));

                for (idx = 0; idx < d.size(); ++idx)
                {
                    auto class_index = std::find(class_names.begin(), class_names.end(), d[idx].label);
                    overlay_bounding_box(img, dlib2cv_rect(d[idx].rect), d[idx].label, class_color[std::distance(class_names.begin(), class_index)]);

                    x_min = d[idx].rect.left();
                    x_max = d[idx].rect.right();
                    y_min = d[idx].rect.top();
                    y_max = d[idx].rect.bottom();

                    // fill in the box string
                    box_string = box_string + "{Class=" + d[idx].label + "; xmin=" + num2str(x_min,"%d") + ", ymin=" + num2str(y_min,"%d") + ", xmax=" + num2str(x_max,"%d") + ", ymax=" + num2str(y_max,"%d") + "},";

                    // crop the depthmap around the bounding box and get the range
                    cv::Range rows(y_min, y_max);
                    cv::Range cols(x_min, x_max);

                    cv::Mat bp_image = dm(rows, cols);
                    range = nan_mean<float>(bp_image);

                    center = dlib::center(d[idx].rect);

                    az = h_res*(center.x() - (int64_t)(img.cols>>1));
                    el = v_res*((int64_t)(img.rows>>1) - center.y());

                    ::object_detect::object_det obj_det;
                    obj_det.label = d[idx].label;
                    obj_det.range = range;
                    obj_det.az = az;
                    obj_det.el = el;
                    detect_list.det.push_back(obj_det);

                }

                box_string = box_string.substr(0, box_string.length()-1);

                boxes_pub.publish(box_string);
                razel_pub.publish(detect_list);
                image_det_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg());                
                
                
                valid_images = false;
            }
            
            
            ros::spinOnce();
            loop_rate.sleep();
            
        }
*/


        // create the object detector class
        //const auto obj_det = std::make_shared<object_detector>(obj_det_node, image_topic, depth_topic, cam_info_topic);
        //object_detector obj_det(obj_det_node, image_topic, depth_topic, cam_info_topic);

        // initialize the object detector network
        //obj_det.init(net_file);

        // run the detections
        //obj_det.run();

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
