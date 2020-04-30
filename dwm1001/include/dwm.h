#ifndef _DWM_H_
#define _DWM_H_

#include <cstdint>
#include <vector>
#include <iostream>
#include <iomanip>

#include "num2string.h"


#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#include "win_serial_fcns.h"
#elif defined(__linux__)
#include "linux_serial_fcns.h"
#endif
//-----------------------------------------------------------------------------

const uint16_t error_packet_size = 3;
const uint16_t version_packet_size = 6;
const uint16_t position_packet_size = 15;
const uint16_t anchor_packet_size = 20;


typedef struct dwm_error
{
    uint8_t type;
    uint8_t error;

    dwm_error() {}

    dwm_error(std::vector<uint8_t> data)
    {
        type = data[0];
        error = data[2];
    }

    inline friend std::ostream& operator<< (
        std::ostream& out,
        const dwm_error& item
        )
    {
        out << "dwm error type: ";
        out << (uint32_t)item.type << ", error code: " << (uint32_t)item.error << std::endl;
        return out;
    }
} dwm_error;

//-----------------------------------------------------------------------------
typedef struct dwm_version
{
    uint8_t type;
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t var;

    dwm_version() {}

    dwm_version(std::vector<uint8_t> data)
    {
        type = data[0];
        if (data[1] == 4)
        {
            major = data[5];
            minor = data[4];
            patch = data[3];
            var = data[2];
        }
        else
        {
            std::cout << "Not enough bytes received to decode version packet!" << std::endl;
        }
    }

    inline friend std::ostream& operator<< (
        std::ostream& out,
        const dwm_version& item
        )
    {
        //out << "dwm firmware version: ";
        out << (uint32_t)item.major << "." << (uint32_t)item.minor << ".";
        out << (uint32_t)item.patch << "." << (uint32_t)item.var;
        return out;
    }

} dwm_version;

//-----------------------------------------------------------------------------
typedef struct dwm_position
{
    uint8_t type;
    float x;
    float y;
    float z;
    uint8_t qf;

    dwm_position() {}

    dwm_position(uint8_t t_, float x_, float y_, float z_, uint8_t qf_)
    {
        type = t_;
        x = x_;
        y = y_;
        z = z_;
        qf = qf_;
    }

    dwm_position(std::vector<uint8_t> data)
    {
        type = data[0];
        if (data[1] == 13)
        {
            x = (float)((data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]) / 1000.0f;
            y = (float)((data[9] << 24) | (data[8] << 16) | (data[7] << 8) | data[6]) / 1000.0f;
            z = (float)((data[13] << 24) | (data[12] << 16) | (data[11] << 8) | data[10]) / 1000.0f;
            qf = data[14];
        }
        else
        {
            std::cout << "Not enough bytes received to decode position packet!" << std::endl;
            std::cout << "packet length: " << (uint16_t)data[1] << std::endl;
            for(uint32_t idx=0; idx<data.size(); ++idx)
                std::cout << "0x" << num2str(data[idx], "%02X") << " ";
                
            std::cout << std::endl;

        }
    }

    inline friend std::ostream& operator<< (
        std::ostream& out,
        const dwm_position& item
        )
    {
        out << "position" << std::endl;
        out << "  x:       " << item.x << std::endl;
        out << "  y:       " << item.y << std::endl;
        out << "  z:       " << item.z << std::endl;
        out << "  qf:      " << (uint16_t)item.qf << std::endl;
        return out;
    }

} dwm_position;

//-----------------------------------------------------------------------------
typedef struct anchor_pos
{
    uint16_t address;
    float range;

    float x;
    float y;
    float z;
    uint8_t qf;
    bool valid;

    anchor_pos() 
    {
        address = 0;
        range = 0;
        x = 0;
        y = 0;
        z = 0;
        qf = 0;
        valid = false;
    }

    anchor_pos(uint16_t add_, float r_, uint8_t qf_, float x_, float y_, float z_)
    {
        address = add_;
        range = r_;
        x = x_;
        y = y_;
        z = z_;
        qf = qf_;
        valid = true;
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
anchor_pos build_anchor_pos(std::vector<uint8_t> data)
{
    anchor_pos ap;
    ap.valid = false;
    if (data.size() == anchor_packet_size)
    {
        ap.qf = data[6];
        if (ap.qf > 5)
        {
            ap.address = (data[1] << 8) | data[0];
            ap.range = (float)((data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]) / 1000.0f;
            ap.x = (float)((data[10] << 24) | (data[9] << 16) | (data[8] << 8) | data[7]) / 1000.0f;
            ap.y = (float)((data[14] << 24) | (data[13] << 16) | (data[12] << 8) | data[11]) / 1000.0f;
            ap.z = (float)((data[18] << 24) | (data[17] << 16) | (data[16] << 8) | data[15]) / 1000.0f;
            ap.valid = true;                
        }
    }

    return ap;
}   // end of build_anchor_pos

//-----------------------------------------------------------------------------
void get_fw(serial_port& sp, std::vector<dwm_version> &version)
{
    std::vector<uint8_t> packet_data;

    uint64_t bytes_read = 0;
    uint64_t bytes_written = 0;
    std::vector<char> pkt = { 0x15, 0x00 };

    version.clear();

    bytes_written = sp.write_port(pkt);

    // get the error packet 
    bytes_read = sp.read_port(packet_data, error_packet_size);
    dwm_error err(packet_data);

    if (err.error != 0)
    {
        std::cout << "Error: " << err << std::endl;
        return;
    }

    // read in the remaining data
    bytes_read = sp.read_port(packet_data, version_packet_size);
    //dwm_version fw(packet_data);
    version.push_back(std::move(dwm_version(packet_data)));

    bytes_read = sp.read_port(packet_data, version_packet_size);
    //dwm_version cfg(packet_data);
    version.push_back(std::move(dwm_version(packet_data)));

    bytes_read = sp.read_port(packet_data, version_packet_size);
    //dwm_version hw(packet_data);
    version.push_back(std::move(dwm_version(packet_data)));

    std::cout << "dwm version" << std::endl;
    std::cout << "fw: " << version[0] << ", cfg: " << version[1] << ", hw: " << version[2] << std::endl;

    //std::cout << "bytes avail: " << sp.bytes_available() << std::endl;
    //sp.flush_port();
    
}   // end of get_fw

//-----------------------------------------------------------------------------
void get_pos(serial_port& sp, dwm_position &position, std::vector<anchor_pos> &anchor)
{
    uint32_t idx;
    //const uint64_t frame_size = 21;
    uint8_t anchor_count = 0;
    
    uint64_t bytes_read = 0;
    uint64_t bytes_written = 0;
    std::vector<char> pkt = { 0x0C, 0x00 };
    std::vector<uint8_t> packet_data;
    std::vector<uint8_t> anchor_data;

    anchor.clear();

    //std::cout << "bytes avail: " << sp.bytes_available() << std::endl;
    
    bytes_written = sp.write_port(pkt);

    // read in the error code and check
    bytes_read = sp.read_port(packet_data, error_packet_size);
    dwm_error err(packet_data);

    if (err.error != 0)
    {
        std::cout << "Bytes Expected: " << error_packet_size << ", Bytes Received: " << bytes_read << std::endl;
        std::cout << "Error: " << err << std::endl;
        for(uint32_t idx=0; idx<packet_data.size(); ++idx)
            std::cout << "0x" << num2str(packet_data[idx], "%02X") << " ";
                
        std::cout << std::endl;
        return;
    }

    // read in the tag position
    bytes_read = sp.read_port(packet_data, position_packet_size);
    position = dwm_position(packet_data);

    // read in the initial packet header for the anchors to determine how many anchors are detected
    bytes_read = sp.read_port(packet_data, 3);
    if (bytes_read == 3)
    {
        anchor_count = packet_data[packet_data.size() - 1];
    }
    else
    {
        std::cout << "Bytes Expected: " << error_packet_size << ", Bytes Received: " << bytes_read << std::endl;
        std::cout << "Not enough bytes received to decode anchor position packet!" << std::endl;
        return;
    }

    // read in the remaining anchor data
    for (idx = 0; idx < anchor_count; ++idx)
    {
        bytes_read = sp.read_port(packet_data, anchor_packet_size);
        anchor_pos ap = build_anchor_pos(packet_data);
        if(ap.valid)
            anchor.push_back(std::move(ap));
    }
    
    //sp.flush_port();
    //std::cout << "bytes avail: " << sp.bytes_available() << std::endl;
    
    //bytes_read = sp.read_port(packet_data, 30);
    //for(uint32_t idx=0; idx<packet_data.size(); ++idx)
    //    std::cout << "0x" << num2str(packet_data[idx], "%02X") << " ";
            
    //std::cout << std::endl;    

}   // end of get_pos

//-----------------------------------------------------------------------------

#endif  // _DWM_H_
