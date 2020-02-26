
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
    std:string port_name = "COM8";

#else defined(__linux__)
    struct termios options;
    std:string port_name = "/dev/ttyACM0";
#endif

    std::string position = "";
    std::string read_data = "";
    uint64_t bytes_read = 0;
    uint64_t bytes_written = 0;

    std::vector<std::string> params;



    char buffer[255];
    char *bufptr;      /* Current char in buffer */
    int  nbytes;       /* Number of bytes read */
    int  tries;        /* Number of tries so far */

    std::vector<uint8_t> v_buff(255);
    std::vector<uint8_t> dwm_init = {10, 10, 10};
    std::vector<uint8_t> dwm_lec = {'l', 'e', 'c', 10, 10};

    std::vector<uint8_t> rx_buff;
    std::vector<uint8_t> rx_buff2;

    std::vector<anchor_pos> anchor;

    try{
/*
        std::cout << "writing \\r\\n" << std::endl;
        n = write(fd, "\n", 1);
        std::cout << n << std::endl;

        std::cout << "writing lec" << std::endl;
        n = write(fd, "lec\n", 4);
        std::cout << n << std::endl;

//        std::cout << "reading buffer" << std::endl;
//        n = read(fd, buffer, 66);
//        std::cout << n << std::endl;

        // wait 1s before trying to read the port
        sleep_ms(1000);

        std::cout << "reading v_buffer" << std::endl;
        n = read(fd, &v_buff[0], 66);
        std::cout << n << std::endl;


        position.assign(v_buff.begin(), v_buff.end());
        std::cout << "res: " << position << std::endl;

        for(int idx=0; idx<n; ++idx)
        {
            std::cout << (int)v_buff[idx] << " ";
        }

        std::cout << std::endl;
        close(fd);
*/



        // open the serial port
        std::cout << "opening port..." << std::endl;
        sp.open_port(port_name, baud_rate, wait_time);

        // this is the intial write to bring the DWM1001 into console mode
//        sp.write_port("\n\n");
//        std::cout << "1" << std::endl;

        std::vector<char> fw = {0x15, 0x00};
        bytes_written = sp.write_port(fw);

        // do the intial check to see if the tag has been configured previously
        bytes_read = sp.read_port(rx_buff, 50);
        std::cout << "br: " << bytes_read << std::endl;
        //std::cout << rx_buff << std::endl;
/*
        std::vector<char> pos = { 0x0C, 0x00 };

        bytes_written = sp.write_port(pos);
        bytes_read = sp.read_port(rx_buff, 200);
        std::cout << "br: " << bytes_read << std::endl;

        //bytes_read = sp.read_port(rx_buff2, 200);
        //std::cout << "br: " << bytes_read << std::endl;
*/
        if(bytes_read > 0)
        {
            // parse through the data to see if we've received at packet that contains the "DIST" indicator
            //params.clear();
            //parse_csv_line(read_data, params);

//            for(idx=0; idx<params.size(); ++idx)
            for(idx=0; idx<bytes_read; ++idx)
            {
                std::cout << (uint32_t)rx_buff[idx] << "," ;
            }
            std::cout << std::endl;
        }
        else
        {
            // sent the init commands
            bytes_written = sp.write_port(dwm_init);
            //sleep_ms(2);
            //bytes_written = write(sp.port, "\n", 1);
            std::cout << "bw: " << bytes_written << std::endl;

            // now set it to start printing out data
            bytes_written = sp.write_port(dwm_lec);
            //bytes_written = write(sp.port, "\r\n\r\n", 4);
            std::cout << "sent lec" << std::endl;
            std::cout << "bw: " << bytes_written << std::endl;

            // now try to read in a string
            //std::vector<uint8_t> test;
            bytes_read = sp.read_port(read_data, 66);
            std::cout << "br: " << bytes_read <<  std::endl;

            //std::string test2;
	    //test2.assign(test.begin(), test.end());

            //std::cout << "string: " << test2 << std::endl;


            // parse through the data to see if we've received at packet that contains the "DIST" indicator
            params.clear();
            parse_csv_line(read_data, params);

            for(idx=0; idx<params.size(); ++idx)
            {
                std::cout << params[idx] << "," ;
            }
            std::cout << std::endl;


        }

        get_pos(sp, anchor);

        for (idx = 0; idx < anchor.size(); ++idx)
            std::cout << anchor[idx] << std::endl;

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

