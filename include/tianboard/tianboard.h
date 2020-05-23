#ifndef __TIANBOARD_H__
#define __TIANBOARD_H__

#include "ros/ros.h"
#include "serial.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include <tf/transform_broadcaster.h>

#define DEFAULT_SERIAL_DEVICE "/dev/ttyUSB0"

using namespace std;
using namespace boost;

class Tianboard {
public:
    Tianboard(ros::NodeHandle *nh);
    //~Tianboard();
private:
    ros::Publisher odom_pub_;
    ros::Publisher uwb_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Timer heart_timer_;
    geometry_msgs::TransformStamped odom_tf_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::NodeHandle nh_;
    Serial serial_;
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void serialDataProc(uint8_t *data, unsigned int data_len);
    void tianboardDataProc(unsigned char *buf, int len);
    void heartCallback(const ros::TimerEvent&);
};

#endif
