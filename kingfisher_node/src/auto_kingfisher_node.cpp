#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <kingfisher_node/TwistConfig.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <control_toolbox/pid.h>
#include <string.h>
#include <kingfisher_msgs/Drive.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>


class AutoTwistCompensator
{
    public:
        AutoTwistCompensator(ros::NodeHandle &n): node_(n) {
            node_.param<double>("imu_data_timeout", imu_data_timeout,0.5); 
            node_.param<double>("pid_data_timeout", pid_timeout,1/10.0); 
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

            cmd_pub = node_.advertise<kingfisher_msgs::Drive>("cmd_drive",1000);
            dbg_pub = node_.advertise<geometry_msgs::Vector3>("yaw_rate_debug",1000);
    	    yaw_rate_pid.reset();
	        yaw_rate_pid.initPid(kp,ki,kd,i1,i2);
            last_pid_time = ros::Time::now().toSec();

            ROS_INFO("Yaw Rate Params: P,I,D:%f, %f, %f:",kp,ki,kd);

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
        void joy_callback(const sensor_msgs::Joy msg);

    private:
        ros::NodeHandle node_;
        ros::Publisher cmd_pub,dbg_pub;
        //teleop constants
        double rotation_scale,fwd_speed_scale,rev_speed_scale,left_max,right_max;
        //yaw controller constants
        double imu_data_timeout,pid_timeout, kp, kd, ki, i1, i2;
        double curr_yaw_reading, yaw_reading_time;
	    control_toolbox::Pid yaw_rate_pid;
    	double yaw_rate_l_error,last_pid_time;
	    bool auto_control;
    	int last_button;


        void xaxis_thrust (geometry_msgs::Twist cmd_vel, kingfisher_msgs::Drive* drive_msg);
        void rc_control (geometry_msgs::Twist cmd_vel, kingfisher_msgs::Drive* drive_msg);
        void compensate (geometry_msgs::Twist cmd_vel, kingfisher_msgs::Drive* drive_msg);

};


void AutoTwistCompensator::imu_callback(const sensor_msgs::Imu msg) {
    curr_yaw_reading = msg.angular_velocity.z;
    yaw_reading_time = ros::Time::now().toSec();
}

void AutoTwistCompensator::joy_callback(const sensor_msgs::Joy msg) 
{
   int button_sensor = msg.buttons[13];
   if (button_sensor==1 && last_button==0) { //catch button transition from 0 to 1
       auto_control = !auto_control;
       if (auto_control) 
           ROS_INFO("Boat in Autonomous mode");
       else
           ROS_INFO("Boat in R/C mode");
   }
   last_button = button_sensor;
}

   

void AutoTwistCompensator::twist_callback(const geometry_msgs::Twist msg)
{
    kingfisher_msgs::Drive drive_msg;         

    //Calculate required forward thrust
    xaxis_thrust (msg,&drive_msg);

    //Handle autonomous features
    if (auto_control) {
        if (ros::Time::now().toSec() - yaw_reading_time > imu_data_timeout) {
            ROS_WARN("IMU Data timeout, Yaw Compensator deactivated. IMU data not being received");
            rc_control (msg,&drive_msg);
        }
        else {
            //calculate pid differential
            compensate(msg, &drive_msg);
        }
    } 
    //Go into standard RC mode
    else {
        rc_control (msg,&drive_msg);
    }

    drive_msg.left *= left_max;
    drive_msg.right *= right_max;
    cmd_pub.publish(drive_msg);

}

void AutoTwistCompensator::xaxis_thrust (geometry_msgs::Twist cmd_vel, kingfisher_msgs::Drive* drive_msg)
{
    double speed_scale = 1.0;
    if (cmd_vel.linear.x > 0.0)
    	speed_scale = fwd_speed_scale;
    else if (cmd_vel.linear.x < 0.0)
	    speed_scale = rev_speed_scale;

    drive_msg->left = (cmd_vel.linear.x * speed_scale); 
    drive_msg->right = (cmd_vel.linear.x * speed_scale);
}
 
void AutoTwistCompensator::rc_control(geometry_msgs::Twist cmd_vel, kingfisher_msgs::Drive* drive_msg)
{
    drive_msg->left -= (cmd_vel.angular.z * rotation_scale);
    drive_msg->right +=  (cmd_vel.angular.z * rotation_scale);

    if (fabs(drive_msg->left) > 1.0) {
    	drive_msg->right = drive_msg->right * 1.0 / fabs(drive_msg->left);
	    drive_msg->left = copysign(1.0, drive_msg->left);
    }
    if (fabs(drive_msg->right) > 1.0) {
    	drive_msg->left = drive_msg->left * 1.0 / fabs(drive_msg->right);
	    drive_msg->right = copysign(1.0, drive_msg->right);
    }
}

void AutoTwistCompensator::compensate(geometry_msgs::Twist cmd_vel, kingfisher_msgs::Drive* drive_msg)
{
    double dt = ros::Time::now().toSec() - last_pid_time;  
    if (dt > pid_timeout)
        dt = pid_timeout;

    last_pid_time = ros::Time::now().toSec();
    double error = curr_yaw_reading - (cmd_vel.angular.z*0.5);
    double output = yaw_rate_pid.updatePid(error, ros::Duration(pid_timeout));

    drive_msg->left -= (output * rotation_scale);
    drive_msg->right += (output * rotation_scale);

    geometry_msgs::Vector3 dbg_info;             
    dbg_info.x = cmd_vel.angular.z*0.5;
    dbg_info.y = curr_yaw_reading;
    dbg_info.z = output;
    dbg_pub.publish(dbg_info);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "kingfisher_node");
    ros::NodeHandle nh;


    AutoTwistCompensator kingfisher_twist_compensator(nh);

    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel",1,&AutoTwistCompensator::twist_callback, &kingfisher_twist_compensator);
    ros::Subscriber imu_sub = nh.subscribe("imu/data",1,&AutoTwistCompensator::imu_callback, &kingfisher_twist_compensator);

    ros::Subscriber joy_sub = nh.subscribe("joy",1,&AutoTwistCompensator::joy_callback, &kingfisher_twist_compensator);


    ros::spin();
    return 0;
}
