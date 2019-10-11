#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>


#include <serial/serial.h>
#include <listen_serial/serial_msgs.h>

#include <vector>

//#define rBUFFERSIZE 

serial::Serial ser;
int index_ = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_listen_test");
    ros::NodeHandle nh;
    std::vector<uint8_t> Data_received;

    ros::Publisher ser_data_pub;
    ser_data_pub = nh.advertise<listen_serial::serial_msgs>("ser_data_pub", 10);

    listen_serial::serial_msgs sonar_data;

    int temp;

    //发布主题，需要定义消息的类型
    //ros::Publisher read_pub = nh.advertise<std_msgs::String>("read1",1000);
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
    }

    if(ser.isOpen())
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
            while(ser.available()<19)
            {            
                //ROS_INFO("Read ing...");
            };
            ser.read(Data_received,ser.available());                
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

                ser.flushInput();
                Data_received.clear();
                //break;
            }
            else
            {
                index_++;
            }
            while(Data_received.size()-index_-18<19)
            {
                while(ser.available()<19)
                {
                };
                ser.read(Data_received,ser.available());  
            }
        }
        ser_data_pub.publish(sonar_data);

        ros::spinOnce();
        loop_rate.sleep();
    }
}