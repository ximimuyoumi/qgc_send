#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sensor_msgs/Range.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}


geometry_msgs::PoseStamped pose;
ros::Publisher local_pos_pub;
char getch();
bool offboard_ok = 0;
bool arm_ok = 0;

void right_listen(const sensor_msgs::Range::ConstPtr& msg_right)
{
    ROS_INFO("hello!\n");
    std_msgs::Float64 right_range;
    right_range.data = msg_right->range;
    ROS_INFO("right distance: %f \n", right_range.data);
    
    //geometry_msgs::PoseStamped pose;

    int c = getch();
        if (c != EOF) {
            switch (c) {
            case 65:    // key up
                pose.pose.position.z += 1;
                break;
            case 66:    // key down
                pose.pose.position.z += -1;
                break;
            case 67:    // key right
                pose.pose.position.y += 1;
                break;
            case 68:    // key left
                pose.pose.position.y -= 1;
                break;
            case 63:
                //return 0;
                break;
            }
        }

    local_pos_pub.publish(pose);

}
/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}


/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_main");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("mavros/set_mode");

    ros::Subscriber right_sonar_sub;

    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(20.0);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request(0);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    
    right_sonar_sub = nh.subscribe("right_sonar", 10000, right_listen);   

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                
                ROS_INFO("Offboard enabled");
                offboard_ok = true;
            }
            last_request = ros::Time::now();
        } else {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                    arm_ok = true;
                }
                last_request = ros::Time::now();
            }
        }        
        
        //if(offboard_ok&&arm_ok)
        //{
        //}
        //else
        //{
            //local_pos_pub.publish(pose);
        //}
            ros::spinOnce();
            rate.sleep();
        }




        //ROS_INFO("setpoint: %.1f, %.1f, %.1f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);


        //local_pos_pub.publish(pose);


    return 0;
}


