#pragma once
#include <vector>
#include <ros/ros.h>
#include <vscan_usbcan_api/vs_can_api.h>

namespace vscan_api
{

//! Class for holding all functions, structures and variables for establishing connection with USB-CAN adapter and working with CAN-channel. 
class usbcan_handle
{

public:

    usbcan_handle(); 
    ~usbcan_handle(); 

    bool open(CHAR * device, DWORD mode, void * speed); 

    void close();

    bool setSpeed(void * speed); 

    bool getErrorFlag();

    char * getStatusString();
    
    bool readRequest(VSCAN_MSG * read_buffer, DWORD read_buffer_size);
    bool writeRequest(VSCAN_MSG * write_buffer, DWORD write_buffer_size);
    bool Flush();

    unsigned long getActualWriteNum();

    unsigned long getActualReadNum();

    bool noError(bool show_error_str);

    void wrapMsgData(VSCAN_MSG &msg, int16_t val, size_t byte_offset = 0);
    void wrapMsgData(VSCAN_MSG &msg, float val, size_t byte_offset = 0);

    int16_t getDatafromMsg(VSCAN_MSG &msg, size_t byte_offset = 0);
    float getFloatDatafromMsg(VSCAN_MSG &msg, size_t byte_offset = 0);

// not defined functions (
    DWORD getMode(); 

    VSCAN_STATUS enableTimeStamp();
    VSCAN_STATUS disableTimeStamp();
    VSCAN_STATUS enableReadBlockingMode();
    VSCAN_STATUS disableReadBlockingMode();
// ) not defined functions 

private:

    // DWORD write_buffer_size_;
    // DWORD read_buffer_size_;

    VSCAN_HANDLE vscan_handle_; // Addres of serial port, or actual status code.
    VSCAN_STATUS vscan_status_; // Actual status code, use getStatusString() to print it in human-readable form.

    DWORD vscan_flags_; // Flags for coding errors and warnings given by USB-CAN adapter.

    CHAR * error_string_; // Error string with fixed size.

    DWORD actual_read_frame_number_; // Actual number of read CAN-fames.
    DWORD actual_write_frame_number_; // Actual number of written CAN-fames.

}; //class

} //namespace



/* 

TO-DO:
- check array issues
- private members??

*/