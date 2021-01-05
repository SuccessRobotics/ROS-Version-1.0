#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void SetPose(double x, double y, double theta)
{
	//ros::init(argc, argv, "Set_Pose");
    ros::NodeHandle p_nav;
    ros::Publisher pubi = p_nav.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);
    //ros::Rate loop_rate(10); // 10HZ = 0.1s

    std::string fixed_frame = "map";
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = fixed_frame;
    pose.header.stamp = ros::Time::now();

    // set x,y coord
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = 0.0;

    // set theta
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);
    tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
    pose.pose.covariance[6*0+0] = 0.5 * 0.5;
    pose.pose.covariance[6*1+1] = 0.5 * 0.5;
    pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;

    // publish
    ROS_INFO("initial pose : x: %f, y: %f, z: 0.0, theta: %f",x,y,theta);
    pubi.publish(pose);
}

void SendGoal(double x, double y, double theta)
{
        //ros::init(argc, argv, "Send_Goal");
    ros::NodeHandle p_nav;
    ros::Publisher pubg = p_nav.advertise<geometry_msgs::PoseStamped> ("move_base_simple/goal", 1);
    //ros::Rate loop_rate(10); // 10HZ = 0.1s

    std::string fixed_frame = "map";
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = fixed_frame;
    goal.header.stamp = ros::Time::now();

    // set x,y coord
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    // set theta
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);
    tf::quaternionTFToMsg(quat, goal.pose.orientation);

    // publish
    ROS_INFO("Send Goal : x: %f, y: %f, z: 0.0, theta: %f",x,y,theta);
    pubg.publish(goal);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "send_position");
	 //ros::NodeHandle p_nav;
	ros::start();

	ros::NodeHandle p_nav;
	ros::Publisher pubi = p_nav.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);
    	ros::Publisher pubg = p_nav.advertise<geometry_msgs::PoseStamped> ("move_base_simple/goal", 1);

    	 ros::Rate loop_rate(10);
while (ros::ok())
{
ros::spinOnce();
  int c = getch();   // call your non-blocking input function
  if (c == '0'){
	SetPose(-2.614, -2.674,  1.534);
	SendGoal(-2.614, -2.674,  1.534);
	}
  else if (c == '1')
	SendGoal(-2.974, -0.785, 0.524);
  else if (c == '2')
        SendGoal(-3.476, -0.905, -3.080);
  else if (c == '3')
        SendGoal(-2.614, -2.674,  1.534);
  else ROS_INFO("Sorry not in command");
loop_rate.sleep();
}
return 0;
}

