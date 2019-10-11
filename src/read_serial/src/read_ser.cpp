#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <serial/serial.h>
#include <read_serial/serial_msg.h>

#include <vector>

//#define rBUFFERSIZE 
serial::Serial serial_set;


int index_ = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_listen_test");
    ros::NodeHandle nh;
    std::vector<uint8_t> Data_received;

    ros::Publisher ser_data_pub;
    ser_data_pub = nh.advertise<read_serial::serial_msg>("ser_data_pub", 10);

    read_serial::serial_msg sonar_data;

    int temp;

    //发布主题，需要定义消息的类型
    //ros::Publisher read_pub = nh.advertise<std_msgs::String>("read1",1000);
    try
    {
        serial_set.setPort("/dev/ttyUSB0");
        serial_set.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_set.setTimeout(to);
        serial_set.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
    }

    if(serial_set.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(50);
    while(ros::ok())

    {
        //ROS_INFO_STREAM("Reading from serial port");
        //std_msgs::String result;
        //result.data = ser.read(ser.available());
        //ROS_INFO_STREAM("Read: "<<result.data);

        while(Data_received.size()-index_<19)
        {
            while(serial_set.available()<19)
            {            
                //ROS_INFO("Read ing...");
            };
            serial_set.read(Data_received,serial_set.available());                
            //ROS_INFO("Read...");

        }
        while(true)
        {
            if(Data_received[index_] == 0xA5 && Data_received[index_+1] == 0x5A && Data_received[index_+2] == 0x13)
            {
                //ROS_INFO("Read: %x", Data_received[index_]);

                std::vector<uint8_t>::iterator temp = Data_received.begin()+3;
                    //ROS_INFO("%x ", *temp);
                sonar_data.front_center = (*(temp+1)<<8)|*(temp);
                ROS_INFO("front center: %d ", sonar_data.front_center);

                temp = temp+2;
                sonar_data.front_left = (*(temp+1)<<8)|*(temp);
                ROS_INFO("front left: %d ", sonar_data.front_left);

                temp = temp+2;
                sonar_data.front_right = (*(temp+1)<<8)|*(temp);
                ROS_INFO("front right: %d ", sonar_data.front_right);

                temp = temp+2;
                sonar_data.side_left = (*(temp+1)<<8)|*(temp);
                ROS_INFO("side left: %d ", sonar_data.side_left);

                temp = temp+2;
                sonar_data.side_right = (*(temp+1)<<8)|*(temp);
                ROS_INFO("side right: %d ", sonar_data.side_right);

                temp = temp+2;
                sonar_data.to_back = (*(temp+1)<<8)|*(temp);
                ROS_INFO("back: %d ", sonar_data.to_back);
                
                temp = temp+2;
                sonar_data.to_ground = (*(temp+1)<<8)|*(temp);
                ROS_INFO("ground: %d ", sonar_data.to_ground);

                //ROS_INFO("正前: %d", )

                serial_set.flushInput();
                Data_received.clear();
                //break;
            }
            else
            {
                index_++;
            }
            while(Data_received.size()-index_-18<19)
            {
                while(serial_set.available()<19)
                {
                };
                serial_set.read(Data_received,serial_set.available());  
            }
        }
        ser_data_pub.publish(sonar_data);

        ros::spinOnce();
        loop_rate.sleep();
    }
}