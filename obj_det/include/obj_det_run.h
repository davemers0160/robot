#ifndef OBJ_DET_RUN_H_
#define OBJ_DET_RUN_H_


#include <cstdint>
#include <cmath>
#include <string>
#include <mutex>
#include <vector>


// Custom includes
#include "num2string.h"

// Net Version
#include "obj_det_net_v10.h"
//#include "tfd_net_v03.h"
#include "overlay_bounding_box.h"
#include "prune_detects.h"

// ROS includes
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "object_detect/object_det.h"
#include "object_detect/object_det_list.h"

#include <dlib/dnn.h>
#include <dlib/image_transforms.h>

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

// ----------------------------------------------------------------------------
/*
template <typename T>
void copy_image(std::array<dlib::matrix<T>, array_depth> &dest, unsigned char *src)
{
    long r, c;

    for (r = 0; r < nr; ++r)
    {
        for (c = 0; c < nc*3; c+=3)
        {
            long index = r*c + c;
            dest[0](r, c) = src[index + 0];
            dest[1](r, c) = src[index + 1];
            dest[2](r, c) = src[index + 2];
        }
    }

}   // end of copy_image
*/

// ----------------------------------------------------------------------------
void get_images_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dm)
{
        try
        {
            auto tmp_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            auto tmp_dm = cv_bridge::toCvCopy(dm, sensor_msgs::image_encodings::TYPE_32FC1);

            // it is very important to lock the below assignment operation.
            // remember that we are accessing it from another thread too.
            //std::lock_guard<std::mutex> lock(mtx);
            //image = tmp_img->image;
            //depthmap = tmp_dm->image;

            //valid_images = true;
        }
        catch (cv_bridge::Exception& e)
        {
            // display the error at most once per 10 seconds
            ROS_ERROR_THROTTLE(10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
        }

    valid_images = true;
}   // end of get_images_callback


// ----------------------------------------------------------------------------
/*
void cam_info_callback(const sensor_msgs::CameraInfoConstPtr& cam_msg)
{
    cam_info = *cam_msg;
    
        // since the calibration don't change we stop the subscriber once we receive the parameters
        if (cam_info != nullptr)
        {
            //img_w = cam_info->width;
            //img_h = cam_info->height;
            //h_res = 90.0/(double)img_w;
            //v_res = 60.0/(double)img_h;

            //std::cout << std::endl << "cam info:" << std::endl;
            //std::cout << "Image Size (h x w): " << img_h << " x " << img_w << std::endl;
            //std::cout << "Angular Resolution (AZ, EL): " << h_res << ", " << v_res << std::endl;
            
            valid_cam_info = true;

            cam_info_sub.shutdown();
        }    
    
    
}   // end of cam_info_callback
*/
// ----------------------------------------------------------------------------
class object_detector
{

public:


    object_detector(ros::NodeHandle& obj_det_node_,
        const std::string& image_topic_,
        const std::string& depth_topic_,
        const std::string& cam_info_topic_
    ) : obj_det_node(obj_det_node_), image_topic(image_topic_), depth_topic(depth_topic_), cam_info_topic(cam_info_topic_)
    {
        // create the subscribers to read the camera parameters from ROS
        //image_sub = obj_det_node.subscribe<sensor_msgs::Image>(image_topic, 1, &object_detector::get_image_cb, this);
        //depth_sub = obj_det_node.subscribe<sensor_msgs::Image>(depth_topic, 1, &object_detector::get_depth_cb, this);
        cam_info_sub = obj_det_node.subscribe<sensor_msgs::CameraInfo>(cam_info_topic, 1, &object_detector::get_cam_info_cb, this);
        
        message_filters::Subscriber<sensor_msgs::Image> image_sub(obj_det_node, image_topic, 1);
        message_filters::Subscriber<sensor_msgs::Image> depth_sub(obj_det_node, depth_topic, 1);

    }

    ~object_detector() = default;

    // ----------------------------------------------------------------------------
    inline void init(std::string net_file)
    {

        // initialize the network
        dlib::deserialize(net_file) >> net;

        // get the details about the loss layer -> the number and names of the classes
        dlib::mmod_options options = dlib::layer<0>(net).loss_details().get_options();

        std::set<std::string> tmp_names;
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        for (uint64_t idx = 0; idx < options.detector_windows.size(); ++idx)
        {
            std::cout << "detector window (w x h): " << options.detector_windows[idx].label << " - " << options.detector_windows[idx].width << " x " << options.detector_windows[idx].height << std::endl;

            tmp_names.insert(options.detector_windows[idx].label);
        }
        std::cout << "------------------------------------------------------------------" << std::endl;

        // pull out the class names
        class_names.clear();
        for (const auto &it : tmp_names)
        {
            class_names.push_back(it);
        }

        
        //class_names.push_back("box");
        //class_names.push_back("backpack");

        //dlib::rand rnd(time(NULL));
        cv::RNG rng(time(NULL));
        class_color.clear();
        // for (uint64_t idx = 0; idx < class_names.size(); ++idx)
        // {
            // class_color.push_back(cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256)));
        // }

        class_color.push_back(cv::Scalar(0, 255, 0));
        class_color.push_back(cv::Scalar(0, 0, 255));
        
    }   // end of init

    // ----------------------------------------------------------------------------
    void get_image_cb(const sensor_msgs::ImageConstPtr& img)
    {
        try
        {
            auto tmp_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

            // it is very important to lock the below assignment operation.
            // remember that we are accessing it from another thread too.
            std::lock_guard<std::mutex> lock(mtx);
            image = tmp_img->image;
        }
        catch (cv_bridge::Exception& e)
        {
            // display the error at most once per 10 seconds
            ROS_ERROR_THROTTLE(10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void get_depth_cb(const sensor_msgs::ImageConstPtr& dm)
    {
        try
        {
            auto tmp_dm = cv_bridge::toCvCopy(dm, sensor_msgs::image_encodings::TYPE_32FC1);

            // it is very important to lock the below assignment operation.
            // remember that we are accessing it from another thread too.
            std::lock_guard<std::mutex> lock(mtx);
            depth_map = tmp_dm->image;
        }
        catch (cv_bridge::Exception& e)
        {
            // display the error at most once per 10 seconds
            ROS_ERROR_THROTTLE(10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void get_cam_info_cb(const sensor_msgs::CameraInfoConstPtr& cam_msg)
    {
        cam_info = std::make_shared<sensor_msgs::CameraInfo>(*cam_msg);

        // since the calibration don't change we stop the subscriber once we receive the parameters
        if (cam_info != nullptr)
        {
            img_w = cam_info->width;
            img_h = cam_info->height;
            h_res = 90.0/(double)img_w;
            v_res = 60.0/(double)img_h;

            std::cout << std::endl << "cam info:" << std::endl;
            std::cout << "Image Size (h x w): " << img_h << " x " << img_w << std::endl;
            std::cout << "Angular Resolution (AZ, EL): " << h_res << ", " << v_res << std::endl;

            cam_info_sub.shutdown();
        }
    }

    // ----------------------------------------------------------------------------
    void run()
    {
        uint64_t idx;
        std::string box_string = "";

        uint64_t x_min, x_max;
        uint64_t y_min, y_max;

        dlib::point center;

        double az, el, range;

        long nr, nc, r, c;
        
        std::array<dlib::matrix<uint8_t>, array_depth> a_img;

        for (idx = 0; idx < array_depth; ++idx)
        {
            a_img[idx].set_size(img_h, img_w);
        }
        
        ::object_detect::object_det_list detect_list;
        ros::Rate loop_rate(4);

        while (ros::ok())
        {
            // get the image and depthmap from the camera
            mtx.lock();
            cv::Mat img = image;
            cv::Mat dm  = depth_map;
            mtx.unlock();

            detect_list.det.clear();

            if (!img.empty() && !dm.empty())
            {
                box_string = "";

                // copy the image to a dlib array matrix for input into the dnn
                unsigned char *img_ptr = img.ptr<unsigned char>(0);

                r = 0;
                c = 0;
                
                for (idx = 0; idx < img_w*img_h*3; idx+=3)
                {

                    a_img[0](r, c) = *(img_ptr + idx + 2);  //*test_img.ptr<unsigned char>(idx);
                    a_img[1](r, c) = *(img_ptr + idx + 1);  //*test_img.ptr<unsigned char>(idx+1);
                    a_img[2](r, c) = *(img_ptr + idx);      //*test_img.ptr<unsigned char>(idx+2);

                    ++c;

                    if (c >= img_w)
                    {
                        c = 0;
                        ++r;
                    }

                }                

                // run the detection
                std::vector<dlib::mmod_rect> d = net(a_img);
                prune_detects(d, 0.3);

                // simulate a detection of each type
                //std::vector<dlib::mmod_rect> d;
                //d.push_back(dlib::mmod_rect(dlib::rectangle(20,20,100,100), 0.0, "box"));
                //d.push_back(dlib::mmod_rect(dlib::rectangle(100,100,200,200), 0.0, "backpack"));

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

            }
            else
            {
                // display the error at most once per 10 seconds
                ROS_WARN_THROTTLE(10, "Empty depth image frame detected. Ignoring...");
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:

    ros::NodeHandle obj_det_node;
    std::shared_ptr<sensor_msgs::CameraInfo> cam_info;

    // ROS subscriber topics
    std::string image_topic;
    std::string depth_topic;
    std::string cam_info_topic;
    ros::Subscriber image_sub;
    ros::Subscriber depth_sub;
    ros::Subscriber cam_info_sub;


    // ROS publisher topics
    const std::string root_topic = "/obj_det/";
    const std::string image_det_topic = root_topic + "image";
    const std::string boxes_topic = root_topic + "boxes";
    const std::string razel_topic = root_topic + "target_razel";

    // setup the publisher to send out the target location messages
    ros::Publisher image_det_pub = obj_det_node.advertise<sensor_msgs::Image>(image_det_topic, 1);
    ros::Publisher boxes_pub = obj_det_node.advertise<std_msgs::String>(boxes_topic, 1);
    ros::Publisher razel_pub = obj_det_node.advertise<::object_detect::object_det_list>(razel_topic, 1);

    cv::Mat image, depth_map;
    uint64_t img_w, img_h;
    double h_res, v_res;

    anet_type net;
    std::vector<std::string> class_names;
    std::vector<cv::Scalar> class_color;

    std::mutex mtx;

};


#endif  // OBJ_DET_RUN_H_
