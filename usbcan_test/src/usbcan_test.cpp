#include <vscan_usbcan_api/usbcan.h>

// run in terminal: rosrun usbcan_test usbcan_test /dev/ttyUSB0
// or run in terminal: roslaunch usbcan_test usbcan_test.launch
int main(int argc, char **argv)
{

    std::string n_name = "usbcan_test"; // node name
    std::string devname = "/dev/ttyUSB0"; // default device name
    DWORD mode = VSCAN_MODE_NORMAL; // default USB-CAN mode: normal (read and write CAN-frames from/to CAN-channel)
    auto can_baudrate = VSCAN_SPEED_1M; // default CAN baudrate: 1 Mbps



// parsing arguments, given from terminal
    // char tty[] = "/dev/ttyUSB0";
    char * tty; // device name
    if(argc>1)
    {
        tty = argv[1];
        if(argc>2)
        {
            if(!strcmp(argv[2],"listen")) // only-listen mode
            {
                mode = VSCAN_MODE_LISTEN_ONLY;
            }else if(!strcmp(argv[2],"self")){ // self-reception mode
                mode = VSCAN_MODE_SELF_RECEPTION;
            }
        }
    }else{
        // choosing default device name
        tty = new char[devname.length()+1];
        strcpy(tty,devname.c_str());
    }





// initializing node
    ros::init(argc, argv, n_name);
    ros::NodeHandle nh;





// init USB-CAN handle
    vscan_api::usbcan_handle usbcan_handle; 

// open CAN port
// you can use VSCAN_FIRST_FOUND instead tty
    usbcan_handle.open(tty,mode,can_baudrate);
    



// buffers' sizes
    int read_buff_size = 10,
        write_buff_size = 2;

// define read buffer
    std::vector<VSCAN_MSG> test_read_buffer;
    test_read_buffer.resize(read_buff_size);

// define write buffer (CAN-frames to send)
    std::array<VSCAN_MSG, write_buff_size> test_write_buffer;





// examples of editing buffer directly
    // test_write_buffer.resize(write_buff_size); // fixing size of buffer <-- if buffer is std::vector<VSCAN_MSG>

    // test_write_buffer[0].Id = 0x123;
    // test_write_buffer[0].Size = 4;
    // test_write_buffer[0].Flags = VSCAN_FLAGS_STANDARD;
    // test_write_buffer[0].Data[0] = 0x00;
    // test_write_buffer[0].Data[1] = 0x01;
    // test_write_buffer[0].Data[2] = 0x02;
    // test_write_buffer[0].Data[3] = 0x03;

    // test_write_buffer[1].Id = 0x321;
    // test_write_buffer[1].Size = 2;
    // test_write_buffer[1].Flags = VSCAN_FLAGS_STANDARD;
    // test_write_buffer[1].Data[0] = 0x01;
    // test_write_buffer[1].Data[1] = 0x02;




    unsigned char some_uint8_number = 45; // 0 - 255
    // UINT8 some_uint8_number = 45; // - equal to previous
    int16_t some_int16_number = 234;
    float some_float_number = 0.34;

// VSCAN_MSG structure instance: define some CAN-frame here
    VSCAN_MSG test_frame;
    test_frame.Id = 0x12fff; // CAN ID
    test_frame.Size = 4; // length of data field (in bytes)
    test_frame.Flags = VSCAN_FLAGS_STANDARD; // standart or extended CAN ID
    test_frame.Data[0] = some_uint8_number; // inserting some number (length of 8-bit or 1 byte) in data field

// inserting int16 number (2 bytes) in data field with 1 byte offset (first byte is already occupied by some_uint8_number)
    usbcan_handle.wrapMsgData(test_frame,some_int16_number,1);
// inserting float number (float32, 4 bytes) in data field with 3 byte offset (first 3 bytes are already occupied)
    usbcan_handle.wrapMsgData(test_frame,some_int16_number,3);



// defining another CAN-frame
    VSCAN_MSG test_frame_two;
    test_frame_two.Id = 0x10; 
    test_frame_two.Size = 1; 
    test_frame_two.Flags = VSCAN_FLAGS_STANDARD;
    test_frame_two.Data[0] = some_uint8_number;




// FOR test_write_buffer --> std::vector<VSCAN_MSG>
// pushing created CAN-frame to write buffer (RECOMMENDING NOT to use it in realtime! This function performs memory allocation.)
    // test_write_buffer.push_back(test_frame);

// inserting created CAN-frame to write buffer
    test_write_buffer[0] = test_frame
    test_write_buffer[1] = test_frame_two





// pre-allocating memory for some numbers
    unsigned int32_t some_read_id = 0x01;

    unsigned char got_some_uint8_number = 0;
    int16_t got_some_int16_number = 0;
    float got_some_float_number = 0.0;

// defining ROS-cycle rate (not neccessary!?)
    ros::Rate rate(200);



// MAIN ROS-CYCLE
    while (ros::ok())
    {
    
    // check if no error with CAN-channel
        if(usbcan_handle.noError())
        {
        
        // read request
            if(usbcan_handle.readRequest(test_read_buffer.data(),test_read_buffer.size()))
            {
            
            // if read request SUCCEEDS --> frames, read from CAN, are stored in read buffer
                if(usbcan_handle.getActualReadNum()>0)
                {
                    // ROS_INFO_STREAM("Read " << usbcan_handle.getActualReadNum() << " CAN-frames.");
                    for(VSCAN_MSG read_msg : test_read_buffer) // iterating over read buffer
                    {
                        // ROS_INFO("Got CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", read_msg.Id, read_msg.Data[0], read_msg.Data[1], read_msg.Data[2], read_msg.Data[3], read_msg.Data[4], read_msg.Data[5], read_msg.Data[6], read_msg.Data[7]);
                        if(read_msg.Id==some_read_id)
                        {
                            got_some_int8_number = read_msg[0];
                            got_some_int16_number = usbcan_handle.getDatafromMsg(read_msg,1);
                            got_some_float_number = usbcan_handle.getFloatDatafromMsg(read_msg,3);

                            // test_publisher.publish(got_some_int16_number);
                        }
                    }
                }
            }

        // write request
            usbcan_handle.writeRequest(test_write_buffer.data(),test_write_buffer.size()) // sending buffer of messages
            usbcan_handle.writeRequest(&test_frame,1) // sending one frame
            usbcan_handle.Flush() // sending to CAN all frames, which are not sent yet, immediately
            usbcan_handle.writeRequest(&test_frame_two,1) // sending another CAN-frame
            usbcan_handle.Flush() 

            usbcan_handle_.getErrorFlag(); // check for errors on CAN-bus and print errors in terminal

            rate.sleep(); // some delay between cycles


        }else{
        // if there's some error, try to open CAN-channel again every 5 seconds
            ROS_WARN_STREAM("Reconnecting to USB-CAN adapter and opening port...");
            usbcan_handle.open(VSCAN_FIRST_FOUND,mode,can_baudrate);
            sleep(5);
        }

        ros::spinOnce();
    }


    usbcan_handle_.close(); // closing CAN-channel 

    return 0;
}


