#include <vscan_usbcan_api/usbcan.h>


namespace vscan_api
{

// An usbcan_handle object constructor. 
usbcan_handle::usbcan_handle() :
actual_read_frame_number_(0), actual_write_frame_number_(0),
vscan_handle_(-1), vscan_status_(-1), vscan_flags_(0),
error_string_(new CHAR[33])
{}

// An usbcan_handle object destructor. 
usbcan_handle::~usbcan_handle()
{
    // need to check if device is ready, to close it???
    // if(isReady()) 
    // {
        
    // }
}


/*!
 Creating socket for USB-CAN adapter connection and open CAN-channel.
 Use "sudo chmod 777 /dev/ttyUSB0" in terminal, if couldn't connect to USB-CAN adapter.
    \param device The name of device, which the USB-CAN adapter is plugged to (e.g. "/dev/ttyUSB0").
    \param mode USB-CAN functioning mode (e.g. listen-only).
    \param speed CAN baudrate.
    \return The connection status boolean (true/false).
*/
bool usbcan_handle::open(CHAR * device, DWORD mode, void * speed)
{
    vscan_handle_ = VSCAN_Open(device, mode);   
    vscan_status_ = vscan_handle_;
    if(vscan_handle_>0)
    {
        ROS_INFO("[USB-CAN adapter] Successfuly connected on port %s",device);

        if(mode==VSCAN_MODE_NORMAL){ROS_INFO("[USB-CAN adapter] Mode: normal");}
        else if(mode==VSCAN_MODE_LISTEN_ONLY){ROS_INFO("[USB-CAN adapter] Mode: listen only");}
        else if(mode==VSCAN_MODE_SELF_RECEPTION){ROS_INFO("[USB-CAN adapter] Mode: self-reception");}

        setSpeed(speed);
        return true;
    }else{
        ROS_ERROR("[USB-CAN adapter] Failed to connect.");
        return false;
    }
}

// Closing CAN-channel.
void usbcan_handle::close()
{
    ROS_INFO("[USB-CAN adapter] Shutting down connection...");
    vscan_status_ = VSCAN_Close(vscan_handle_);
} 

/*!
Getting status of USB-CAN adapter and CAN-channel in human-readable form.
    \return An error_string_ for terminal printing.
*/
char * usbcan_handle::getStatusString()
{
    VSCAN_GetErrorString(vscan_status_, error_string_, sizeof(error_string_));
    return error_string_;
}


/*!
Taking atempt to read CAN-frames from USB-CAN buffer.
Modifies given read CAN-frames container object, if there is new frames arrived and stored in USB-CAN buffer.
    \param read_buffer Pointer to CAN-frames container object (e.g. std::vector<VSCAN_MSG>).
    \param read_buffer_size Size of buffer.
    \return The no-error-status boolean (true if no error).
*/
bool  usbcan_handle::readRequest(VSCAN_MSG * read_buffer, DWORD read_buffer_size)
{
    vscan_status_ = VSCAN_Read(vscan_handle_, read_buffer, read_buffer_size, &actual_read_frame_number_);
    return noError();
}


/*!
Taking atempt to write CAN-frames.
    \param write_buffer Pointer to CAN-frames container object (e.g. std::vector<VSCAN_MSG>) or pointer to VSCAN_MSG structer itself.
    \param write_buffer_size Size of buffer (number of frames) to write.
    \return The no-error-status boolean (true if no error).
*/
bool  usbcan_handle::writeRequest(VSCAN_MSG * write_buffer, DWORD write_buffer_size)
{
    vscan_status_ = VSCAN_Write(vscan_handle_, write_buffer, write_buffer_size, &actual_write_frame_number_);
    return Flush();
}


/*!
Taking atempt to send out all frames to CAN immediately.
    \return The no-error-status boolean (true if no error).
*/
bool  usbcan_handle::Flush()
{
    vscan_status_ = VSCAN_Flush(vscan_handle_);
    return noError();
}


/*!
Setting CAN baudrate.
    \return The no-error-status boolean (true if no error).
*/
bool usbcan_handle::setSpeed(void * speed)
{
    vscan_status_ = VSCAN_Ioctl(vscan_handle_, VSCAN_IOCTL_SET_SPEED, speed);
    if(noError())
    {
        int speed_val = 0;
        if(speed==VSCAN_SPEED_100K){speed_val = 100000;}
        else if(speed==VSCAN_SPEED_125K){speed_val = 125000;}
        else if(speed==VSCAN_SPEED_1M){speed_val = 1000000;}
        else if(speed==VSCAN_SPEED_20K){speed_val = 20000;}
        else if(speed==VSCAN_SPEED_250K){speed_val = 250000;}
        else if(speed==VSCAN_SPEED_500K){speed_val = 500000;}
        else if(speed==VSCAN_SPEED_50K){speed_val = 50000;}
        else if(speed==VSCAN_SPEED_800K){speed_val = 800000;}
        ROS_INFO("[USB-CAN adapter] CAN baudrate set: %i",speed_val);
        return true;
    }else{
        return false;
    }
}


/*!
Printing info about error and warning flags in terminal.
    \return The no-error-status for this operation (true if no errors and warnings).
*/
bool usbcan_handle::getErrorFlag()
{
    vscan_status_ = VSCAN_Ioctl(vscan_handle_, VSCAN_IOCTL_GET_FLAGS, &vscan_flags_);
    if(vscan_flags_!=0)
    {
        ROS_WARN("[USB-CAN adapter] Error flags: %lx",vscan_flags_);

        if(vscan_flags_&VSCAN_IOCTL_FLAG_RX_FIFO_FULL != 0)
        {
            ROS_INFO("[USB-CAN adapter]     RX FIFO FULL: %li",vscan_flags_&VSCAN_IOCTL_FLAG_RX_FIFO_FULL);
        }

        if((vscan_flags_ >> 1)&(VSCAN_IOCTL_FLAG_TX_FIFO_FULL >> 1))
        {
            ROS_INFO("[USB-CAN adapter]     TX FIFO FULL: %li",(vscan_flags_ >> 1)&(VSCAN_IOCTL_FLAG_TX_FIFO_FULL >> 1));
        }

        if((vscan_flags_ >> 2)&(VSCAN_IOCTL_FLAG_ERR_WARNING >> 2) !=0)
        {
            ROS_INFO("[USB-CAN adapter]      ERR WARNING: %li",(vscan_flags_ >> 2)&(VSCAN_IOCTL_FLAG_ERR_WARNING >> 2));
        }

        if((vscan_flags_ >> 3)&(VSCAN_IOCTL_FLAG_DATA_OVERRUN >> 3) != 0)
        {
            ROS_INFO("[USB-CAN adapter]     DATA OVERRUN: %li",(vscan_flags_ >> 3)&(VSCAN_IOCTL_FLAG_DATA_OVERRUN >> 3));
        }

        if((vscan_flags_ >> 4)&(VSCAN_IOCTL_FLAG_UNUSED >> 4) != 0)
        {
            ROS_INFO("[USB-CAN adapter]           UNUSED: %li",(vscan_flags_ >> 4)&(VSCAN_IOCTL_FLAG_UNUSED >> 4));
        }

        if((vscan_flags_ >> 5)&(VSCAN_IOCTL_FLAG_ERR_PASSIVE >> 5) != 0)
        {
            ROS_INFO("[USB-CAN adapter]      ERR PASSIVE: %li",(vscan_flags_ >> 5)&(VSCAN_IOCTL_FLAG_ERR_PASSIVE >> 5));
        }

        if((vscan_flags_ >> 6)&(VSCAN_IOCTL_FLAG_ARBIT_LOST >> 6) != 0)
        {
            ROS_INFO("[USB-CAN adapter]       ARBIT LOST: %li",(vscan_flags_ >> 6)&(VSCAN_IOCTL_FLAG_ARBIT_LOST >> 6));
        }

        if((vscan_flags_ >> 7)&(VSCAN_IOCTL_FLAG_BUS_ERROR >> 7) != 0)
        {
            ROS_INFO("[USB-CAN adapter]        BUS ERROR: %li",(vscan_flags_ >> 7)&(VSCAN_IOCTL_FLAG_BUS_ERROR >> 7));
        }

        if((vscan_flags_ >> 16)&(VSCAN_IOCTL_FLAG_API_RX_FIFO_FULL >> 16) != 0)
        {
            ROS_INFO("[USB-CAN adapter] API RX FIFO FULL: %li",(vscan_flags_ >> 16)&(VSCAN_IOCTL_FLAG_API_RX_FIFO_FULL >> 16));
        }

        return false;
    }
    return true;
}


// Getter for actual number of written frames.
unsigned long usbcan_handle::getActualWriteNum(){ return actual_write_frame_number_;}


// Getter for actual number of frames, read from CAN-channel.
unsigned long usbcan_handle::getActualReadNum(){ return actual_read_frame_number_;}


/*!
Checking no-error-status.
    \return True if no errors, and false if there are some errors with USB-CAN adapter or CAN-channel.
*/
bool usbcan_handle::noError(bool show_error_str = true)
{
    if(show_error_str){getErrorFlag();}
    if(vscan_status_ == VSCAN_ERR_OK)
    {
        return true;
    }else{
        ROS_ERROR("[USB-CAN adapter] Error: %s",getStatusString());
        return false;
    }
}


/*!
Inserting given number (int16 or float) in data field of given CAN-frame. 
    \param msg Pointer to VSCAN_MSG structure.
    \param val A number to pack into data field of CAN-frame
    \param byte_offset A byte offset of given number in data field of given CAN-frame (0-7 bytes).
*/
void usbcan_handle::wrapMsgData(VSCAN_MSG &msg, int16_t val, size_t byte_offset)
{
    msg.Data[byte_offset]   = val >> 8;
    msg.Data[byte_offset+1] = val;
}

void usbcan_handle::wrapMsgData(VSCAN_MSG &msg, float val, size_t byte_offset)
{
    unsigned char * data_addr = (unsigned char *)&val; // float ptr --> uchar ptr
    msg.Data[byte_offset] = *data_addr; 
    msg.Data[byte_offset+1] = *++data_addr;
    msg.Data[byte_offset+2] = *++data_addr;
    msg.Data[byte_offset+3] = *++data_addr;
    
}


/*!
Getting number (int16) from data field of given CAN-frame. 
    \param msg Pointer to VSCAN_MSG structure.
    \param byte_offset A byte offset of number in data field of given CAN-frame (0-7 bytes).
*/
int16_t usbcan_handle::getDatafromMsg(VSCAN_MSG &msg, size_t byte_offset)
{
    return msg.Data[byte_offset]<<8 | msg.Data[byte_offset+1];
}


/*!
Getting number (float) from data field of given CAN-frame. 
    \param msg Pointer to VSCAN_MSG structure.
    \param byte_offset A byte offset of number in data field of given CAN-frame (0-4 bytes).
*/
float usbcan_handle::getFloatDatafromMsg(VSCAN_MSG &msg, size_t byte_offset)
{
    unsigned char data_field[] = {msg.Data[0], msg.Data[1], msg.Data[2], msg.Data[3]};
    float val_out = *(float *)&data_field;
    return val_out;
}


} // namespace



