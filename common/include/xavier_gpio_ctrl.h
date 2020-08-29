/*
 * NVIDIA Jetson Xavier Carrier board GPIO Control
 *
 * Inspiration from:
 * MengHuanLee: https://github.com/MengHuanLee/jetsonTX2GPIO
 * gaosiy: https://github.com/gaosiy/JetsonXavierGPIO
 */

#ifndef JETSON_XAVIER_GPIO_CTRL_H_
#define JETSON_XAVIER_GPIO_CTRL_H_

#include <cstdint>
#include <string>

#include "num2str.h"
 
const std::string SYSFS_GPIO_DIR = "/sys/class/gpio";
const double POLL_TIMEOUT = 3000; // 3 seconds
const int MAX_BUF = 64;

//typedef unsigned int gpio_pin;
//typedef unsigned int pinDirection;
//typedef unsigned int pinValue;

// ----------------------------------------------------------------------------
enum pin_dir : unsigned int 
{
    output_pin = 0,
    input_pin  = 1
};

enum pin_value : unsigned int
{
    low     = 0,
    off     = 0,
    high    = 1,
    on      = 1
} ;

enum gpio_number : unsigned int
{
    gpio422 = 422,      // Pin  7
    gpio428 = 428,      // Pin 11
    gpio351 = 351,      // Pin 12
    gpio424 = 424,      // Pin 13
    gpio393 = 393,      // Pin 15
    gpio256 = 256,      // Pin 16
    gpio344 = 344,      // Pin 21
    gpio417 = 417,      // Pin 22
    gpio251 = 251,      // Pin 29
    gpio250 = 250,      // Pin 31
    gpio257 = 257,      // Pin 32
    gpio248 = 248,      // Pin 33
    gpio354 = 354,      // Pin 35
    gpio429 = 429,      // Pin 36
    gpio249 = 249,      // Pin 37
    gpio353 = 353,      // Pin 38
    gpio352 = 352,      // Pin 40
};



// ----------------------------------------------------------------------------
class gpio_pin
{
    
public:

    unsigned int pin;

    gpio_pin() = default;
    
    
    gpio_pin( unsigned int pin_) : pin(pin_) 
    {
        init();
    }
    
    
    int init()
    {
        std::ofstream fs;
        fs.open(SYSFS_GPIO_DIR + "/export", std::ofstream::out);
        
        if(!fs.is_open())
        {
            std::cout << "error exporting pin" << std::endl;
            return -1;
        }
        
        fs << pin;
        
        if(fs.fail())
        {
            std::cout << "error writing value" << std::endl;
            fs.close();
            return -2;
        }

        fs.close();
        return 1;
    }
    
    
    // ----------------------------------------------------------------------------
    int set_direction(pin_dir pd)
    {
        std::ofstream fs;
        fs.open(SYSFS_GPIO_DIR + "/gpio" + num2str(pin, "%d") + "/direction", std::ofstream::out);        
        
        if(!fs.is_open())
        {
            std::cout << "unable to open gpio pin" << std::endl;
            return -1;
        }

        if(pd)
        {
            fs << "in";
        }
        else
        {
            fs << "out";
        }
        
        if(fs.fail())
        {
            std::cout << "error writing value" << std::endl;
            fs.close();
            return -2;
        }

        fs.close();
        return 1;
        
    }   // end of set_direction
    
    // ----------------------------------------------------------------------------
    int set_value(unsigned int value)
    {
        std::ofstream fs;
        fs.open(SYSFS_GPIO_DIR + "/gpio" + num2str(pin, "%d") + "/value", std::ofstream::out);        
        
        if(!fs.is_open())
        {
            std::cout << "unable to open gpio pin" << std::endl;
            return -1;
        }
        
        fs << num2str(value, "%d");
        
        if(fs.fail())
        {
            std::cout << "error writing value" << std::endl;
            fs.close();
            return -2;
        }

        fs.close();
        return 1;        
        
        
    }

    // ----------------------------------------------------------------------------
    int get_value(unsigned int &value)
    {
        
        
    }


private:
    
    
    
};



#endif // JETSON_XAVIER_GPIO_CTRL_H_
