#define _CRT_SECURE_NO_WARNINGS

// C/C++ includes
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <vector>

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#include "win_serial_fcns.h"

#else defined(__linux__)
//#include <string.h>  /* String function definitions */
//#include <unistd.h>  /* UNIX standard function definitions */
//#include <fcntl.h>   /* File control definitions */
//#include <errno.h>   /* Error number definitions */
//#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include "linux_serial_fcns.h"
#endif

// custom includes
#include "sleep_ms.h"
#include "get_current_time.h"
#include "get_platform.h"
#include "num2string.h"
#include "file_parser.h"

// Project includes
#include "dwm.h"

// -------------------------------GLOBALS--------------------------------------

int main(int argc, char** argv)
{

    uint32_t idx;

    serial_port sp;

    uint32_t baud_rate = 115200;
    uint32_t wait_time = 10;

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
    std::string port_name = "COM8";

#else defined(__linux__)
    //struct termios options;
    std:string port_name = "/dev/ttyACM0";
#endif

    //uint64_t bytes_read = 0;
    //uint64_t bytes_written = 0;

    // dwm variables
    std::vector<dwm_version> version;
    std::vector<anchor_pos> anchor;
    dwm_position tag_position;


    try{

        // open the serial port
        std::cout << "opening port..." << std::endl;
        sp.open_port(port_name, baud_rate, wait_time);

        // get the firmware/config/hardware versions
        get_fw(sp, version);

        // get the position of the tag, the position of the anchors and distance of anchors to the tag
        get_pos(sp, tag_position, anchor);

        std::cout << "tag " << tag_position << std::endl;

        for (idx = 0; idx < anchor.size(); ++idx)
        {
            std::cout << anchor[idx] << std::endl;
        }

        // close the port
        sp.close_port();


    }
    catch(std::exception e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }

    std::cout << "Program Complete!  Press Enter to Close..." << std::endl;
    std::cin.ignore();

} // end of main

