#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>

#include <chrono>

float angular = 0.0;
float linear = 0.0;

float posX = 0.0, posY = 0.0, yaw = 0.0;

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#define RATE 10

uint8_t bumper[N_BUMPER] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//fill with your code
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position:(%f,%f) Orientation:%frad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odomCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);


    // rate in Hz
    ros::Rate loop_rate(RATE);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;

    // 0 --> linear
    // 1 --> angular
    float movements[] = {(0,0,5), (1,90)};


    int movementIndex = 0;
    int iterationCount = 0;
    int iterationTarget = 0;

    int movement_loops(float movememnt){
        int count = 0;
        if (movement[0] == 0){
            count = RATE*(movement[1]/linear);
        }
        else{
            count = RATE*(DEG2RAD(movement[1])/angular);
        }
        return count;
    }

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        //fill with your code

        if (iterationCount == iterationTarget){
            movementIndex++;
            iterationCount = 0;
            iterationTarget = 0;
        }

        if (movementIndex==sizeof(movements)/sizeof(movements[0])){
            break;
        }

        if (iterationTarget == 0){
            iterationTarget = movement_loops(movements[movementIndex]);
        }

        if (movements[movementIndex][0] == 0){
            vel.angular.z = 0.0;
            vel.linear.x = movements[movementIndex][1];    
        } else {
            vel.angular.z = movements[movementIndex][1];
            vel.linear.x = 0.0;     
        }
        vel.angular.z = movements[movementIndex];
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();

        iterationCount++;
        loop_rate.sleep();
    }

    return 0;
}
