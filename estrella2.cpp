#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<turtlesim/Pose.h>
#include<sstream>
#include<math.h>

ros::Publisher vel_pub;
ros::Subscriber pose_sub;
turtlesim::Pose turtlesim_pose;

const double PI = acos(-1);

void move(double , double, bool);
void rotate(double, double, bool);
double deg2rad(double);
void dibujar();
void poseCallback(const turtlesim::Pose::ConstPtr &);

using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "mover");
	ros::NodeHandle n;
	vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_sub = n.subscribe("/turtle1/pose", 10, &poseCallback);
	dibujar();
}

void move( double vy, double dx, bool isf){
	geometry_msgs::Twist vel_msg;
	double t0, t1, cur_dis=0, vx = 2;
	if(isf){
		vel_msg.linear.x = abs(vx);
		vel_msg.linear.y = abs(vy);
	}
	else{
		vel_msg.linear.x = -abs(vx);
		vel_msg.linear.y = -abs(vy);
	}
	ros::Rate rate(10);
	t0 = ros::Time::now().toSec();
	while(cur_dis != dx){
		vel_pub.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		cur_dis = vel_msg.linear.x * (t1 - t0);
		ros::spinOnce();
		rate.sleep();
	}
}

void rotate(double ang_speed, double rel_ang, bool clockwise){
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	double cur_ang = 0.0;
	double t0, t1;
	ros::Rate rate(10);
	if (clockwise)	vel_msg.angular.z = -abs(ang_speed);
	else	vel_msg.angular.z = abs(ang_speed);
	t0 = ros::Time::now().toSec();
	while(cur_ang < rel_ang){
		vel_pub.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		cur_ang = ang_speed * (t1 - t0);
		ros::spinOnce();
		rate.sleep();
	}
	vel_msg.angular.z = 0;
	vel_pub.publish(vel_msg);
}

double deg2rad(double input){
	return input * PI / 180.0;
}

void dibujar(){
	int i, l=1;
	double a_speed = 30, ang; 
	double px, py, vx, vy;
	double p[5][2];
	a_speed = deg2rad(a_speed);
	ang = 36;
	ang = deg2rad(ang);
	p[0][0] = turtlesim_pose.x;
	cout << p[0][0] << endl;
	p[0][1] = turtlesim_pose.y;
	p[1][0] = p[0][0] + 2*l*cos(ang);
	p[1][1] = p[0][1];
	p[2][0] = p[0][0] - l*sin(ang);
	p[2][1] = p[0][1] - l*cos(ang);
	p[3][0] = p[0][0] + l*cos(ang);
	p[3][1] = p[0][1] + l*sin(ang);
	p[4][0] = p[0][0] + 2*l*cos(ang) - l*sin(36);
	p[4][1] = p[0][1] - l*cos(ang);
	for(i=0; i< 4; i++){
		px = p[i+1][0] - p[i][0];
		py = p[i+1][1] - p[i][1];
		vy = py / px * vx;
		if(i != 0){
			rotate(a_speed, ang, 0);
		}
		if(i %2 == 0){
			move(vy, px,1);
		}
		else{
			move(vy, px, 0);
		}
	}
	rotate(a_speed, ang, 0);
	px = p[0][0] - p[4][0];
	py = p[0][1] - p[4][1];
	vy = py / px * vx;
	move(vy, px, 1);
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_msg){
	turtlesim_pose.x = pose_msg -> x;
	turtlesim_pose.y = pose_msg -> y;
	turtlesim_pose.theta = pose_msg -> theta;
}