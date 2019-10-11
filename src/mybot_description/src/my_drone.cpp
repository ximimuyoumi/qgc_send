#include "ros/ros.h"
#include </home/caosu/ros_learning/mybot_ws/src/mybot_description/include/mybot_description/my_drone.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dronenode");
    //ros::NodeHandle car;


    my_drone little_drone; 

    //ROS_INFO("obstacle: %d", mylittlecar.obstacle_detect_result);


    //sonar_sub = car.subscribe("mybot/sonar",1000,&mycar::listen_sonar, &mylittlecar);

    //orient_sub = car.subscribe<gazebo_msgs::ModelState>("gazebo/model_states",1000,orient_callback);

    //modelstate_sub = car.subscribe("gazebo/model_states",1000,model_state_CB);

    //vel_pub = car.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

    //ros::spin();
    return 0;
}
            