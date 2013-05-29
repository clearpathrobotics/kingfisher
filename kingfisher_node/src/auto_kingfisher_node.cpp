#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <kingfisher_node/TwistConfig.h>
#include <sensor_msgs/Imu.h>
#include <control_toolbox/pid.h>
#include <string.h>
#include <kingfisher_msgs/Drive.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>

class AutoTwistCompensator
{
    public:
        AutoTwistCompensator(ros::NodeHandle &n): node_(n) {
            node_.param<double>("imu_data_timeout", imu_data_timeout,0.5); 
            node_.param<double>("pid_data_timeout", pid_timeout,1/20.0); 
            node_.param<double>("yaw_compensator/kp", kp,2.0); 
            node_.param<double>("yaw_compensator/kd", kd,1.0);
            node_.param<double>("yaw_compensator/ki", ki,0.0);
            node_.param<double>("yaw_compensator/i1", i1,0.0);
            node_.param<double>("yaw_compensator/i2", i2,-0.0);
	    node_.param<double>("rotation_scale",rotation_scale,0.2);
	    node_.param<double>("fwd_speed_scale",fwd_speed_scale,1);
	    node_.param<double>("rev_speed_scale",rev_speed_scale,1);
	    node_.param<double>("left_max",left_max,1);
	    node_.param<double>("right_max",right_max,1);
	    node_.param<bool>("auto_control",auto_control,false);

	    ROS_INFO("Subscribing to cmd_vel");

            cmd_pub = node_.advertise<kingfisher_msgs::Drive>("cmd_drive",1000);

	    yaw_rate_pid.reset();
	    yaw_rate_pid.initPid(kp,ki,kd,i1,i2);
        }
        
        void reconfigure(kingfisher_node::TwistConfig &config, uint32_t level){
            rotation_scale = config.rotation_scale;
            fwd_speed_scale = config.fwd_speed_scale;
            rev_speed_scale = config.rev_speed_scale;
            left_max = config.left_max;
            right_max = config.right_max;
	

        }
        void twist_callback(const geometry_msgs::Twist msg);
        void imu_callback(const sensor_msgs::Imu msg);

    private:
        ros::NodeHandle node_;
        ros::Publisher cmd_pub;
        //teleop constants
        double rotation_scale,fwd_speed_scale,rev_speed_scale,left_max,right_max;
        //yaw controller constants
        double imu_data_timeout,pid_timeout, kp, kd, ki, i1, i2;
        double curr_yaw_reading, yaw_reading_time;
	control_toolbox::Pid yaw_rate_pid;
	double yaw_rate_l_error;
	bool auto_control;

};


void AutoTwistCompensator::imu_callback(const sensor_msgs::Imu msg) {
    curr_yaw_reading = msg.angular_velocity.z;
    yaw_reading_time = ros::Time::now().toSec();
}

void AutoTwistCompensator::twist_callback(const geometry_msgs::Twist msg)
{
    kingfisher_msgs::Drive drive_msg;         

    if (auto_control) {
	

    }
    else {
	    double speed_scale = 1.0;
	    if (msg.linear.x > 0.0)
		speed_scale = fwd_speed_scale;
	    else if (msg.linear.x < 0.0)
		speed_scale = rev_speed_scale;

	    drive_msg.left = (msg.linear.x * speed_scale) - (msg.angular.z * rotation_scale);
	    drive_msg.right = (msg.linear.x * speed_scale) - (msg.angular.z * rotation_scale);

	    if (fabs(drive_msg.left) > 1.0) {
		drive_msg.right = drive_msg.right * 1.0 / fabs(drive_msg.left);
		drive_msg.left = copysign(1.0, drive_msg.left);
	    }
	    if (fabs(drive_msg.right) > 1.0) {
		drive_msg.left = drive_msg.left * 1.0 / fabs(drive_msg.right);
		drive_msg.right = copysign(1.0, drive_msg.right);
	    }
    }

    drive_msg.left *= left_max;
    drive_msg.right *= right_max;
    cmd_pub.publish(drive_msg);

}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "kingfisher_node");
    ros::NodeHandle nh;

    AutoTwistCompensator kingfisher_twist_compensator(nh);

    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel",1,&AutoTwistCompensator::twist_callback, &kingfisher_twist_compensator);
    ros::Subscriber imu_sub = nh.subscribe("imu/data",1,&AutoTwistCompensator::imu_callback, &kingfisher_twist_compensator);

    ros::spin();
    return 0;
}
