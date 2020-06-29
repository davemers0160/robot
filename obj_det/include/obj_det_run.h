#ifndef OBJ_DET_RUN_H_
#define OBJ_DET_RUN_H_


#include <cstdint>
#include <string>
#include <mutex>
#include <vector>

// Net Version
#include "tfd_net_v03.h"


// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <dlib/dnn.h>
#include <dlib/image_transforms.h>


extern const uint32_t array_depth;

// ----------------------------------------------------------------------------
template <typename T>
double nan_mean(cv::Mat& img)
{
    uint64_t count = 0;
    double mn = 0;

    cv::MatIterator_<T> it;

    for (it = img.begin<T>(); it != img.end<T>(); ++it)
    {
        if (!isnan(*it))
        {
            mn += (double)*it;
            ++count;
        }
    }

    return (mn / (double)count);
}   // end of nan_mean

// ----------------------------------------------------------------------------
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



// ----------------------------------------------------------------------------
class object_detector
{

public:


    object_detector(ros::NodeHandle& nh, 
        const std::string& image_topic_, 
        const std::string& depth_topic_,
        const std::string& cam_info_topic_
    ) : image_topic(image_topic_), depth_topic(depth_topic_), cam_info_topic(cam_info_topic_)
    {
        // create the subscribers to read the camera parameters from ROS
        image_sub = obj_det_node.subscribe<sensor_msgs::Image>(image_topic, 1, &get_image_cb, this);
        depth_sub = obj_det_node.subscribe<sensor_msgs::Image>(depth_topic, 1, &get_depth_cb, this);
        cam_info_sub = obj_det_node.subscribe<sensor_msgs::CameraInfo>(cam_info_topic, 1, &get_cam_info_cb, this);        
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
        for (idx = 0; idx < options.detector_windows.size(); ++idx)
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
            
        dlib::rand rnd(time(NULL));
        class_color.clear();
        for (idx = 0; idx < *num_classes; ++idx)
        {
            class_color.push_back(dlib::rgb_pixel(rnd.get_random_8bit_number(), rnd.get_random_8bit_number(), rnd.get_random_8bit_number()));
        }
        
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
            
            //++frame_number;
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
            auto tmp_dm = cv_bridge::toCvCopy(dm, sensor_msgs::image_encodings::BGR8);
            
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
            get_cam_info_cb.shutdown();        
    }
    
    
    // ----------------------------------------------------------------------------
    void run()
    {
        uint64_t idx;
        std::string box_string = "";
        
        uint64_t x_min, x_max;
        uint64_t y_min, y_max;
        
        uint64_t det_x, det_y;
        
        double az, el, range;
        
        long nr;
        long nc;
        
        ::obj_det_wrapper::object_det_list detect_list;
        
        while (ros::ok())
        {
            // get the image and depthmap from the camera
            mtx.lock();
            cv::Mat img = image;
            cv::Mat dm  = depthmap;
            mtx.unlock();
            
            detect_list.clear();
            
            if (!img.empty() && !dm.empty())
            {
                box_string = "";
                
                cv::imshow("color image", img);
                
                
                // copy the image over
                
                
                // run the detection
                //std::vector<dlib::mmod_rect> d = net(d_img);
                //prune_detects(d, 0.3);
                
                // simulate a detection of each type
                std::vector<dlib::mmod_rect> d;
                d.push_back(dlib::mmod_rect(dlib::rectangle r(20,20,100,100), 0.0, "box"));
                d.push_back(dlib::mmod_rect(dlib::rectangle r(100,100,200,200), 0.0, "backpack"));
                
                
                for (idx = 0; idx < d.size(); ++idx)
                {
                    auto class_index = std::find(class_names.begin(), class_names.end(), d[idx].label);
                    overlay_bounding_box(img, d[idx], class_color[std::distance(class_names.begin(), class_index)]);
                    
                    x_min = d[idx].rect.left();
                    x_max = d[idx].rect.right();
                    y_min = d[idx].rect. top();
                    y_max = d[idx].rect.bottom();
                    
                    // fill in the box string
                    box_string = box_string + "{Class=" + d[idx].label + "; xmin=" + num2str(x_min,"%d") + ", ymin=" + num2str(y_min,"%d") + ", xmax=" + num2str(x_max,"%d") + ", ymax=" + num2str(y_max,"%d") + "},"
                    
                    // crop the depthmap around the bounding box and get the range
                    cv::Range rows(y_min, y_max);
                    cv::Range cols(x_min, x_max);
                    
                    cv::Mat bp_image = dm(rows, cols);                    
                    range = nan_mean(bp_image);
                    
                    det_x = (uint64_t)(x_min + (x_max-x_min)/2.0);
                    det_y = (uint64_t)(y_min + (y_max-y_min)/2.0);
                    az = self.h_res*(det_x - (uint64_t)(img_w/2.0));
                    el = self.v_res*((uint64_t)(img_h/2.0) - det_y);
                    
                    ::obj_det_wrapper::object_det obj_det;
                    obj_det.label = d[idx].label;
                    obj_det.range = range;
                    obj_det.az = az;
                    obj_det.el = el;
                    detect_list.det.push_back(obj_det);
                    
                }
                
                box_string = box_string.substr(0, box_string.length()-2);
                
                boxes_pub.publish(box_string);
                // razel_pub.publish(detect_list);
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
    static std::string image_topic;
    static std::string depth_topic;
    static std::string cam_info_topic;
    ros::Subscriber image_sub;          // = obj_det_node.subscribe(image_topic, 1, run_net_callback);
    ros::Subscriber depth_sub;          // = obj_det_node.subscribe(depth_topic, 1, run_net_callback);
    ros::Subscriber cam_info_sub;       // = obj_det_node.subscribe(cam_info_topic, 1, run_net_callback);


    // ROS publisher topics
    static const std::string root_topic = "/obj_det/";
    static const std::string image_det_topic = root_topic + "image";
    static const std::string boxes_topic = root_topic + "boxes";
    static const std::string razel_topic = root_topic + "target_razel";

    // setup the publisher to send out the target location messages
    image_det_pub = obj_det_node.advertise<sensor_msgs::Image>(image_det_topic, 1);
    boxes_pub = obj_det_node.advertise<std_msgs::String>(boxes_topic, 1);
    razel_pub = obj_det_node.advertise<::obj_det_wrapper::object_det_list>(razel_topic, 1);

    cv::Mat image, depth_map;

    anet_type net;
    std::vector<std::string> class_names;
    std::vector<dlib::rgb_pixel> class_color;

    std::mutex mtx;
    
    ros::Rate loop_rate(1);
    
};


#endif  // OBJ_DET_RUN_H_
