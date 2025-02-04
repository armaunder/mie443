#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI/180.)


#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

float angular = 0.0;
float linear = 0.0;
float posX = 0.0, posY=0.0, yaw = 0.0;
float angle_min = 0.0;
float angle_max = 0.0;
float angle_increment = 0.0;
float ranges = 0;

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t leftstate = bumper[kobuki_msgs::BumperEvent::LEFT];
uint8_t frontstate = bumper [kobuki_msgs::BumperEvent::CENTER];
uint8_t rightstate = bumper [kobuki_msgs::BumperEvent::RIGHT];

float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers = 0, desiredNLasers=0, desiredAngle = 10;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//fill with your code
    minLaserDist= std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min)/msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    ROS_INFO("Size of laser scan array: %i and size of offset %i", nLasers, desiredNLasers);

    if(desiredAngle*M_PI/180 < msg->angle_max && -desiredAngle*M_PI/180>msg->angle_min){
        for(uint32_t laser_idx = nLasers/2 - desiredNLasers; laser_idx<nLasers/2+desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist,msg->ranges[laser_idx]);
        }
    }
    else{
        for(uint32_t laser_idx = 0; laser_idx<nLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist,msg->ranges[laser_idx]);
        }
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Subscriber odom = nh.subscribe("odom",1,odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;
   

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
       

       //check if any of the bumpers were pressed
       bool any_bumper_pressed=false;
       for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
       }
        //Control logic after bumpers are being pressed
        ROS_INFO("Position: (%f,%f) Orientation:%f degrees and distance away:%f units.",posX,posY,RAD2DEG(yaw),minLaserDist);
        
        float oldyaw = 0.0;
        
        if(minLaserDist < 0.5 && !any_bumper_pressed){
            oldyaw = yaw;
            while (abs(yaw - oldyaw) < M_PI/5) {
                
                angular = M_PI/4;
                linear = 0.0;
                ROS_INFO("Condition 0");
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                ROS_INFO ("Yaw%f oldyaw:%f", yaw, oldyaw);
                ros::spinOnce();
                loop_rate.sleep();
            }

                
        } 
        else if(!any_bumper_pressed && minLaserDist > 0.7) {
            angular = 0.0;
            linear = 0.25;
            ROS_INFO("Condition 1");
        }
        else if(!any_bumper_pressed && minLaserDist > 0.5) {
            angular = 0.0;
            linear = 0.1;
            ROS_INFO("Condition 2");
        }
            
            /*while(yaw-oldyaw < M_PI/2) {
                linear = 0.0;
                angular = M_PI/12;
                ROS_INFO("Condition 3");
            }*/
          
        else {
            angular = 0.0;
            linear = 0.0;
            ROS_INFO("STOPPED...");
            break;
        }

        // ROS_INFO("")

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}