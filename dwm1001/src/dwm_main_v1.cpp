
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

//#include <string.h>  /* String function definitions */
//#include <unistd.h>  /* UNIX standard function definitions */
//#include <fcntl.h>   /* File control definitions */
//#include <errno.h>   /* Error number definitions */
//#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>

// custom includes
#include "sleep_ms.h"
#include "get_current_time.h"
#include "get_platform.h"
#include "num2string.h"
#include "file_parser.h"

#include "linux_serial_fcns.h"


// -------------------------------GLOBALS--------------------------------------

int main(int argc, char** argv)
{

    uint32_t idx;

    serial_port sp;

    std:string port_name = "/dev/ttyACM0";

    uint32_t baud_rate = 115200;
    uint32_t wait_time = 10;

    struct termios options;

    std::string position = "";
    std::string read_data = "";
    int64_t bytes_read = 0;
    int64_t bytes_written = 0;

    std::vector<std::string> params;



    char buffer[255];
    char *bufptr;      /* Current char in buffer */
    int  nbytes;       /* Number of bytes read */
    int  tries;        /* Number of tries so far */

    std::vector<uint8_t> v_buff(100);

    try{
/*
        int fd;

        fd = open(port_name.c_str(), O_RDWR | O_NOCTTY); // | O_NDELAY);


        tcgetattr(fd, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);

        options.c_cflag &= ~PARENB;        // No Parity
        options.c_cflag &= ~CSTOPB;        // Stop bits = 1 
        options.c_cflag &= ~CSIZE;            // Clears the Mask
        options.c_cflag |=  CS8;            // Set the data bits = 8

        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |= (CLOCAL | CREAD);

        // set wait time
        options.c_cc[VTIME] = wait_time;    // Wait for up to 1s (100ms increments), returning as soon as any data is received.
        options.c_cc[VMIN] = 0;



        tcsetattr(fd, TCSANOW, &options);

        int n = 0;
        std::cout << "writing lec" << std::endl;
        n = write(fd, "\r\n", 2);
        n = write(fd, "lec\r\n", 5);
*/









//while(1)
//{

//        position = "";
/*
        //ioctl(fd, FIONREAD, &n);
        //std::cout << n << std::endl;

        //std::cout << "reading v_buffer" << std::endl;
        n = read(fd, &v_buff[0], v_buff.size());
        //std::cout << n << std::endl;

        position.assign(v_buff.begin(), v_buff.end());
        std::cout << position << std::endl; 
*/


/*
        // read characters into our string buffer until we get a CR or NL
        bufptr = buffer;
        while ((nbytes = read(fd, bufptr, buffer + sizeof(buffer) - bufptr - 1)) > 0)
        {
            bufptr += nbytes;
            if (bufptr[-1] == '\n' || bufptr[-1] == '\r')
                break;
        }


        position = std::string(buffer);
        std::cout << position << std::endl; 

        //std::cout << std::string(buffer) << std::endl;
*/
//}

//        std::cout << std::endl;
//        close(fd);

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

        idx = 0;
        position = "";
        bytes_read = sp.read_port(read_data, 1);
        while(bytes_read>0)
        {

            if((read_data == "\r") || (read_data == "\n"))
                break;
 
            position = position + read_data;
            ++idx;
   
            if(idx >= 200)
                break;
        }

        std::cout << "P: " << position << std::endl;

        // do the intial check to see if the tag has been configured previously
        bytes_read = sp.read_port(read_data, 66);
        std::cout << "br: " << bytes_read << std::endl;

        if(bytes_read > 0)
        {
            // parse through the data to see if we've received at packet that contains the "DIST" indicator
            params.clear();
            parse_csv_line(read_data, params);

            for(idx=0; idx<params.size(); ++idx)
            {
                std::cout << params[idx] << "," ;
            }
            std::cout << std::endl;
        }
        else
        {
            // sent the init commands
            //bytes_written = sp.write_port("\r\n\r\n");
            bytes_written = write(sp.port, "\r", 1);
            sleep_ms(2);
            bytes_written = write(sp.port, "\n", 1);
            std::cout << "1" << std::endl;

            // now set it to start printing out data
            bytes_written = sp.write_port("lec");
            bytes_written = write(sp.port, "\r\n\r\n", 4);
            std::cout << "sent lec" << std::endl;

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

        // close the port
        sp.close_port();
        std::cout << "1" << std::endl;


    }
    catch(std::exception e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }

} // end of main

