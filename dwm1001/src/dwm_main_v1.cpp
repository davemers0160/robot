
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

    serial_port sp;

    std:string port_name = "/dev/ttyACM0";

    uint32_t baud_rate = 115200;
    uint32_t wait_time = 50;

    struct termios options;

    std::string position = "";
    char buffer[255];
    char *bufptr;      /* Current char in buffer */
    int  nbytes;       /* Number of bytes read */
    int  tries;        /* Number of tries so far */

    std::vector<uint8_t> v_buff(100);

    try{

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

while(1)
{

        position = "";
/*
        //ioctl(fd, FIONREAD, &n);
        //std::cout << n << std::endl;

        //std::cout << "reading v_buffer" << std::endl;
        n = read(fd, &v_buff[0], v_buff.size());
        //std::cout << n << std::endl;

        position.assign(v_buff.begin(), v_buff.end());
        std::cout << position << std::endl; 
*/



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
}


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
*/
        std::cout << std::endl;

        close(fd);

/*

        // open the serial port
        sp.open_port(port_name, baud_rate, wait_time);
        std::cout << "1" << std::endl;

        // this is the intial write to bring the DWM1001 into console mode
        sp.write_port("\n\n");
        std::cout << "1" << std::endl;

        // now set it to start printing out data
        sp.write_port("lec\n\n");
        std::cout << "1" << std::endl;

        // now try to read in a string
        std::vector<uint8_t> test;
        sp.read_port(test, 66);
        std::cout << "1" << std::endl;

        std::string test2;
	test2.assign(test.begin(), test.end());

        std::cout << "string: " << test2 << std::endl;


        // close the port
        sp.close_port();
        std::cout << "1" << std::endl;
*/

    }
    catch(std::exception e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }

} // end of main

