#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <kingfisher_node/TwistConfig.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <control_toolbox/pid.h>
#include <string.h>
#include <kingfisher_msgs/Drive.h>
#include <kingfisher_msgs/Sense.h>


#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>

#define BOAT_WIDTH 0.381 //m, ~15inches
#define MAX_FWD_THRUST 35.0 //Newtons
#define MAX_BCK_THRUST 20.0 //Newtons
#define MAX_OUTPUT 1

#define MAX_YAW_RATE 0.4 //rad/s

        
class ForceCompensator
{
    private:
        ros::NodeHandle node_;
        ros::Publisher cmd_pub;
        ros::Publisher eff_pub;

    public:

        ForceCompensator(ros::NodeHandle &n): node_(n) {
            cmd_pub = node_.advertise<kingfisher_msgs::Drive>("cmd_drive",1000);
            eff_pub = node_.advertise<geometry_msgs::Wrench>("eff_wrench",1000);
        }
        ~ForceCompensator() {
        }

        double get_output (double thrust) {
            //saturate

            double output = 0;
            if (thrust > 0)
                output = thrust * (MAX_OUTPUT/MAX_FWD_THRUST);
            else if (thrust < 0)
                output = thrust * (MAX_OUTPUT/MAX_BCK_THRUST);
            return output; 
        }

        double saturate_thrusters (double thrust) {
            thrust = std::min(MAX_FWD_THRUST,thrust);
            thrust = std::max(-1*MAX_BCK_THRUST,thrust);
            return thrust;
        }


        void update_forces (geometry_msgs::Wrench output) {
            kingfisher_msgs::Drive cmd_output;
            double fx = output.force.x;
            double tauz = output.torque.z;

            //yaw torque maxed out at max torque achievable with the help of reverse thrust
            double max_tauz = MAX_BCK_THRUST*2*BOAT_WIDTH;
            tauz = std::min(tauz, max_tauz);
            tauz = std::max(tauz, -max_tauz);

            //Guarantee atleast max yaw torque
            double left_thrust = tauz/(2*BOAT_WIDTH); 
            double right_thrust = -tauz/(2*BOAT_WIDTH);

            //Provide maximum allowable thrust after yaw torque is guaranteed 
            double max_fx = 0;
            if (tauz >= 0) {
                if (fx >= 0) { //forward thrust on the left thruster will be limiting factor 
                    max_fx = (MAX_FWD_THRUST - left_thrust) * 2;
                    fx = std::min(max_fx,fx);
                }
                else { //backward thrust on the right thruster will be limiting factor
                    max_fx = (-MAX_BCK_THRUST - right_thrust) * 2;
                    fx = std::max(max_fx,fx);
                }
            }
            else { 
                if (fx >= 0 ) {
                    max_fx = (MAX_FWD_THRUST - right_thrust) * 2;
                    fx = std::min(max_fx,fx);
                }
                else {
                    max_fx = (-MAX_BCK_THRUST - left_thrust) * 2;
                    fx = std::max(max_fx,fx);
                }
            }

            left_thrust += fx /2.0;
            right_thrust += fx/2.0;
            
            left_thrust = saturate_thrusters (left_thrust);
            right_thrust = saturate_thrusters (right_thrust);


            ROS_INFO("FX:%f,TAUZ:%f,LTHR:%f,RTHR:%f",fx,tauz,left_thrust,right_thrust);

            cmd_output.left = get_output (left_thrust);
            cmd_output.right = get_output (right_thrust);
            cmd_pub.publish(cmd_output);

            pub_effective_wrench(left_thrust, right_thrust);
        }

        void pub_effective_wrench(double left_thrust,double right_thrust) {
            geometry_msgs::Wrench effective_output;
            effective_output.force.x = left_thrust + right_thrust;
            effective_output.torque.z = (left_thrust - right_thrust)*BOAT_WIDTH;
            eff_pub.publish(effective_output);  
        }
};

class KingfisherController {
    private:
        ros::NodeHandle node_;
        ForceCompensator *force_compensator;
        geometry_msgs::Wrench force_output;

        control_toolbox::Pid yr_pid;
        double yr_kp, yr_ki, yr_kd,yr_i1,yr_i2;
        double yr_cmd,yr_cmd_time,last_yr_cmd_time;
        double spd_cmd;
        double cmd_vel_time;
        double yr_meas,yr_meas_time;
        double imu_data_timeout;

    public:

        KingfisherController(ros::NodeHandle &n):node_(n) {
            force_compensator = new ForceCompensator(node_);

            node_.param<double>("yaw_rate/kp", yr_kp,2.0); 
            node_.param<double>("yaw_rate/kd", yr_kd,1.0);
            node_.param<double>("yaw_rate/ki", yr_ki,0.0);
            node_.param<double>("yaw_rate/i1", yr_i1,MAX_BCK_THRUST*2*BOAT_WIDTH); //clamp integral output at max yaw yorque
            node_.param<double>("yaw_rate/i2", yr_i2,-MAX_BCK_THRUST*2*BOAT_WIDTH);

            node_.param<double>("yaw_rate/i1", yr_i1,0.0); //clamp integral output at max yaw yorque
            node_.param<double>("yaw_rate/i2", yr_i2,0.0);

            node_.param<double>("imu_data_timeout",imu_data_timeout,1/10.0);

            yr_pid.reset();
	        yr_pid.initPid(yr_kp,yr_ki,yr_kd,yr_i1,yr_i2);
            yr_cmd = 0;
            spd_cmd = 0;
            yr_cmd_time = ros::Time::now().toSec();
            last_yr_cmd_time = ros::Time::now().toSec();

            yr_meas = 0;
            yr_meas_time=0;


        }
        ~KingfisherController() {
            delete force_compensator;
        }

        void wrench_callback(const geometry_msgs::Wrench msg) { 
            force_output.force.x = msg.force.x;
            force_output.torque.z = msg.torque.z;
        }

        void twist_callback(const geometry_msgs::Twist msg) { 
            //Autonomous twist control
            yr_cmd = msg.angular.z;
            spd_cmd = msg.linear.x;
            yr_cmd_time = ros::Time::now().toSec();
            force_output.torque.z  = yr_compensator();
        }

        double yr_compensator() {
            if (ros::Time::now().toSec() - yr_meas_time > imu_data_timeout) {
                ROS_ERROR("IMU Data timeout, controller deactivated. IMU data not being received");
                return 0.0;
            }
            else {
                //calculate pid torque z
                double dt = yr_cmd_time - last_yr_cmd_time; 
                double yr_error = yr_cmd - yr_meas;   
                return yr_pid.updatePid(yr_error, ros::Duration(dt));
            }
        }

        void imu_callback(const sensor_msgs::Imu msg) {
           yr_meas = msg.angular_velocity.z;
           yr_meas_time = ros::Time::now().toSec();
        }

        void control_update(const ros::TimerEvent& event) {
            force_compensator->update_forces(force_output);
        }

};

             






    
