#ifndef _DWM_H_
#define _DWM_H_

#include <cstdint>
#include <vector>
#include <iostream>
#include <iomanip>

#include "num2string.h"


#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#include "win_serial_fcns.h"
#else defined(__linux__)
#include "linux_serial_fcns.h"
#endif
//-----------------------------------------------------------------------------

const uint8_t anchor_data_size = 20;

typedef struct dwm_error
{
    uint8_t type;
    uint8_t error;

} dwm_error;

//-----------------------------------------------------------------------------
typedef struct dwm_fw
{
    uint8_t error;
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t var;

    dwm_fw() {}

    dwm_fw(std::vector<uint8_t> data)
    {
        error = data[0];
        major = data[];
        minor = data[];
        patch = data[];
        var = data[1];



    }

} dwm_fw;

//-----------------------------------------------------------------------------
typedef struct anchor_pos
{
    uint16_t address;
    float range;
    uint8_t qf;

    float x;
    float y;
    float z;

    anchor_pos() {}

    anchor_pos(uint16_t add_, float r_, uint8_t qf_, float x_, float y_, float z_)
    {
        address = add_;
        range = r_;
        x = x_;
        y = y_;
        z = z_;
        qf = qf_;
    }

    anchor_pos(std::vector<uint8_t> data)
    {
        if (data.size() == anchor_data_size)
        {
            address = (data[1] << 8) | data[0];
            range = (float)((data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]) / 1000.0;
            qf = data[6];
            x = (float)((data[10] << 24) | (data[9] << 16) | (data[8] << 8) | data[7]) / 1000.0;
            y = (float)((data[14] << 24) | (data[13] << 16) | (data[12] << 8) | data[11]) / 1000.0;
            z = (float)((data[18] << 24) | (data[17] << 16) | (data[16] << 8) | data[15]) / 1000.0;
        }
        else
        {
            address = 0;
            range = 0.0;
            qf = 0;
            x = 0.0;
            y = 0.0;
            z = 0.0;
        }

    }

    inline friend std::ostream& operator<< (
        std::ostream& out,
        const anchor_pos& item
        )
    {
        out << "anchor data" << std::endl;
        out << "  address: 0x" << num2str<uint16_t>(item.address, "%04X") << std::endl;
        out << "  range:   " << item.range << std::endl;
        out << "  x:       " << item.x << std::endl;
        out << "  y:       " << item.y << std::endl;
        out << "  z:       " << item.z << std::endl;
        out << "  qf:      " << (uint16_t)item.qf << std::endl;
        return out;
    }

} anchor_pos;

//-----------------------------------------------------------------------------
void get_fw(serial_port& sp)
{

}   // end of get_fw

//-----------------------------------------------------------------------------
void get_pos(serial_port& sp, std::vector<anchor_pos> &anchor)
{
    uint8_t idx;
    const uint64_t frame_size = 21;
    uint8_t anchor_count = 0;
    
    uint64_t bytes_read = 0;
    uint64_t bytes_written = 0;
    std::vector<char> pos = { 0x0C, 0x00 };
    std::vector<uint8_t> frame_data;
    std::vector<uint8_t> anchor_data;

    bytes_written = sp.write_port(pos);

    // read in the error code, the position block and info which shows how many anchors are in range
    bytes_read = sp.read_port(frame_data, frame_size);
    anchor_count = frame_data[frame_data.size() - 1];

    anchor.clear();

    // read in the remaining anchor data
    for (idx = 0; idx < anchor_count; ++idx)
    {
        bytes_read = sp.read_port(anchor_data, anchor_data_size);
        anchor.push_back(std::move(anchor_pos(anchor_data)));        
    }

}


//-----------------------------------------------------------------------------

#endif  // _DWM_H_
