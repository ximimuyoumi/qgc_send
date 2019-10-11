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


ros::Time left_last_request(0);
ros::Time right_last_request(0);

bool left_bound_reach = 0;
bool right_bound_reach = 0;

int wall_passed_num = 0;



class my_drone
{
public:
	my_drone();
	//virtual ~my_drone();

	ros::Subscriber state_sub; 
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    ros::Subscriber front_right_sonar_sub;
	ros::Subscriber front_center_sonar_sub;
    ros::Subscriber front_left_sonar_sub;
    ros::Subscriber rear_right_sonar_sub;
    ros::Subscriber rear_left_sonar_sub;

	geometry_msgs::PoseStamped pose;
	ros::Publisher local_pos_pub;	
	mavros_msgs::State current_state;

	ros::NodeHandle drone_little;

	char getch();
	void front_right_listen(const sensor_msgs::Range::ConstPtr& msg_right);
	void front_center_listen(const sensor_msgs::Range::ConstPtr& msg_center); 
	void front_left_listen(const sensor_msgs::Range::ConstPtr& msg_left);
	void rear_right_listen(const sensor_msgs::Range::ConstPtr& msg_right);
	void rear_left_listen(const sensor_msgs::Range::ConstPtr& msg_left);

	double left_distance;
	double right_distance;
	void state_cb(const mavros_msgs::State::ConstPtr& msg);
	double range_limit;

	bool left_obstacle;
	bool center_obstacle;
	bool right_obstacle;





};

my_drone::my_drone()
{
	local_pos_pub = drone_little.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 10);
	state_sub = drone_little.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, &my_drone::state_cb, this);
	arming_client = drone_little.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");
	set_mode_client = drone_little.serviceClient<mavros_msgs::SetMode>
                                         ("mavros/set_mode");
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

    ros::Time last_request = ros::Time::now();

	pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;

	range_limit = 0.5; //给定距离的限制
	left_obstacle = 0;
	center_obstacle = 0;
	right_obstacle = 0;
    ROS_INFO("test1");

	front_right_sonar_sub = drone_little.subscribe("front_right_sonar", 10000, &my_drone::front_right_listen, this); 
	front_center_sonar_sub = drone_little.subscribe("front_center_sonar", 10000, &my_drone::front_center_listen, this);  
	front_left_sonar_sub = drone_little.subscribe("front_left_sonar", 10000, &my_drone::front_left_listen, this);	
    rear_right_sonar_sub = drone_little.subscribe("rear_right_sonar", 10000, &my_drone::rear_right_listen, this);   
	rear_left_sonar_sub = drone_little.subscribe("rear_left_sonar", 10000, &my_drone::rear_left_listen, this);      

 	while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                
                ROS_INFO("Offboard enabled");                
                //ROS_INFO("wait1-1");

                //offboard_ok = true;
            }
                last_request = ros::Time::now();
                //ROS_INFO("wait1-2");
        } else {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                    //arm_ok = true;
                }
                last_request = ros::Time::now();
                //ROS_INFO("wait2-1");

            }
                //ROS_INFO("wait2-2");



        }        
            ros::spinOnce();               
            rate.sleep();
        }

}

char my_drone::getch()
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

void my_drone::front_right_listen(const sensor_msgs::Range::ConstPtr& msg_right)
{
    //ROS_INFO("hello!\n");
    std_msgs::Float64 right_range;
    right_range.data = msg_right->range;
    //ROS_INFO("right distance: %f \n", right_range.data);
    
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
			case 119:
				pose.pose.position.x += 1;
				break;
			case 115:
				pose.pose.position.x -= 1;
		
            case 63:
                //return 0;
                //break;
			default:
				break;
				//ROS_INFO("right now: %d", c);
            }
        }


	//ros::Time last_request(0);
	//right_last_request = ros::Time::now();



	if(right_range.data<range_limit)
	{
		//if((ros::Time::now() - right_last_request > ros::Duration(0.2))&&right_range.data<range_limit)
		//{
			right_obstacle = true;
			//ROS_INFO("right obstacle detected!");
			//ROS_INFO("right range: %f  range_limit: %f", right_range.data, range_limit);
		//	right_last_request = ros::Time::now();
	}
	else
	{
			//ROS_INFO("left range: %f  range_limit: %f", right_range.data, range_limit);
		    right_obstacle = false;
	}
//}

	//检测是否通过这面墙
	










	//开始避障的几个case

	if(right_obstacle == true && left_obstacle == true)
	{
        if(right_bound_reach==true && left_bound_reach==false)
		    pose.pose.position.y -= 0.003;
        else if(right_bound_reach==false && left_bound_reach==true)
        	pose.pose.position.y += 0.003;
        else
        	pose.pose.position.y -= 0.003;
	}
	else if(right_obstacle == true && left_obstacle == false)
	{
		pose.pose.position.y -= 0.003;
	}
	else if(right_obstacle == false && left_obstacle == true)
	{
		pose.pose.position.y += 0.003;
	}
	else if(right_obstacle == false && left_obstacle == false)
	{
		pose.pose.position.x += 0.003;
	}

        local_pos_pub.publish(pose);





}

void my_drone::front_center_listen(const sensor_msgs::Range::ConstPtr& msg_center)
{
	std_msgs::Float64 center_range;
	center_range.data = msg_center->range;
	//ROS_INFO("left distance: %f \n", left_range.data);

	//ros::Time last_request(0);
	//left_last_request = ros::Time::now();


	if(center_range.data<range_limit)
	{
		//if((ros::Time::now() - left_last_request > ros::Duration(0.2))&&left_range.data<range_limit)
		//{
			center_obstacle = true;
			//ROS_INFO("left obstacle detected!");
			//ROS_INFO("left range: %f  range_limit: %f", left_range.data, range_limit);
		//	left_last_request = ros::Time::now();
	}
	else
	{
			//ROS_INFO("left range: %f  range_limit: %f", left_range.data, range_limit);
		    center_obstacle = false;
	}
	//}
	
}

void my_drone::front_left_listen(const sensor_msgs::Range::ConstPtr& msg_left)
{
	std_msgs::Float64 left_range;
	left_range.data = msg_left->range;
	//ROS_INFO("left distance: %f \n", left_range.data);

	//ros::Time last_request(0);
	//left_last_request = ros::Time::now();


	if(left_range.data<range_limit)
	{
		//if((ros::Time::now() - left_last_request > ros::Duration(0.2))&&left_range.data<range_limit)
		//{
			left_obstacle = true;
			//ROS_INFO("left obstacle detected!");
			//ROS_INFO("left range: %f  range_limit: %f", left_range.data, range_limit);
		//	left_last_request = ros::Time::now();
	}
	else
	{
			//ROS_INFO("left range: %f  range_limit: %f", left_range.data, range_limit);
		    left_obstacle = false;
	}
	//}
	
}




void my_drone::rear_left_listen(const sensor_msgs::Range::ConstPtr& msg_left)
{
	std_msgs::Float64 left_range;
	left_range.data = msg_left->range;
	//ROS_INFO("left distance: %f \n", left_range.data);

	//ros::Time last_request(0);
	//left_last_request = ros::Time::now();


	if(left_range.data<range_limit)
	{
		//if((ros::Time::now() - left_last_request > ros::Duration(0.2))&&left_range.data<range_limit)
		//{
			right_bound_reach = false;
            left_bound_reach = true;
			ROS_INFO("left bound detected!");
			//ROS_INFO("left range: %f  range_limit: %f", left_range.data, range_limit);
		//	left_last_request = ros::Time::now();
	}
	else
	{
			//ROS_INFO("left range: %f  range_limit: %f", left_range.data, range_limit);
		    //left_bound_reach = false;
	}
	//}
	
}




void my_drone::rear_right_listen(const sensor_msgs::Range::ConstPtr& msg_right)
{
	std_msgs::Float64 right_range;
	right_range.data = msg_right->range;
	//ROS_INFO("left distance: %f \n", left_range.data);

	//ros::Time last_request(0);
	//left_last_request = ros::Time::now();


	if(right_range.data<range_limit)
	{
		//if((ros::Time::now() - left_last_request > ros::Duration(0.2))&&left_range.data<range_limit)
		//{

			right_bound_reach = true;
            left_bound_reach = false;
			ROS_INFO("left bound detected!");
			//ROS_INFO("left range: %f  range_limit: %f", left_range.data, range_limit);
		//	left_last_request = ros::Time::now();
	}	
}













void my_drone::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}