#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <control_toolbox/pid.h>
#include <string.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float32.h>
#include <kingfisher_node/kf_constants.h>
#include <kingfisher_node/force_compensator.h>


class Controller {
    private:
        ros::NodeHandle node_;
        ForceCompensator *force_compensator_;
        geometry_msgs::Wrench force_output_;
        double imu_data_timeout_;



        //Yaw Rate Controller Details        
        control_toolbox::Pid yr_pid_;
        ros::Publisher yr_dbg_pub_;
        double yr_kp_, yr_ki_, yr_kd_,yr_imax_,yr_imin_;
        double yr_cmd_,yr_cmd_time_,last_yr_cmd_time_;
        double yr_meas_,yr_meas_time_;
        bool new_yr_cmd_;


        //Yaw Control Details        
        control_toolbox::Pid y_pid_;
        ros::Publisher y_dbg_pub_;
        double y_kp_, y_ki_, y_kd_,y_imax_,y_imin_;
        double y_cmd_,y_cmd_time_,last_y_cmd_time_;
        double y_meas_,y_meas_time_;
        bool new_yaw_cmd_;

        //Speed Control details
        double spd_cmd_;


    public:
        Controller(ros::NodeHandle &n);
        ~Controller() {
            delete force_compensator_;
        }

        double yr_compensator();
        double y_compensator();


        //TODO:Fix yaw command input, should be Odom or Pose
        void yaw_callback(const std_msgs::Float32 msg) {
            y_cmd_ = msg.data;
            y_cmd_time_ = ros::Time::now().toSec();
            new_yaw_cmd_ = true;
        }

        void wrench_callback(const geometry_msgs::Wrench msg) { 
            force_output_.force.x = msg.force.x;
            force_output_.torque.z = msg.torque.z;
        }

        void twist_callback(const geometry_msgs::Twist msg) { 
            //Autonomous twist control
            yr_cmd_ = msg.angular.z;
            spd_cmd_ = msg.linear.x;
            yr_cmd_time_ = ros::Time::now().toSec();
            new_yr_cmd_ = true;
        }

        void imu_callback(const sensor_msgs::Imu msg) {
            yr_meas_ = msg.angular_velocity.z;
            yr_meas_time_ = ros::Time::now().toSec();
            //TODO: Is this the best way to handle this? Theoretically it would be best if forces are implemented the moment new IMU data comes in. However, that does not sync well because there are other controllers (like speed) that are running in conjunction
            if (new_yr_cmd_) {
                force_output_.torque.z  = yr_compensator(); 
                new_yr_cmd_ = false;
            }
        }

        void control_update(const ros::TimerEvent& event) {
            force_compensator_->update_forces(force_output_);
        }

};

             






    
