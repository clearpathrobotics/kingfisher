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
#define MAX_BCK_THRUST 8.0 //Newtons
#define MAX_OUTPUT 1
        
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


            ROS_INFO("Left thrust:%f,Right thrust:%f",left_thrust,right_thrust);

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
        int last_button;
        bool auto_control;

        double curr_yawrate_reading;
        double yawrate_reading_time;

    public:

        KingfisherController(ros::NodeHandle &n):node_(n) {
            force_compensator = new ForceCompensator(node_);
            last_button=0;
            auto_control = false;
        }
        ~KingfisherController() {
            delete force_compensator;
        }

        void joy_callback(const sensor_msgs::Joy msg) { 
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

        void twist_callback(const geometry_msgs::Twist msg) { 

            if (auto_control) {
                //Autonomous Control
            }
            else {
                //RC Control
                if (msg.linear.x >= 0) 
                    force_output.force.x = msg.linear.x * 40;
                else 
                    force_output.force.x = msg.linear.x * 16;

                force_output.torque.z = msg.angular.z * MAX_BCK_THRUST*2*BOAT_WIDTH;
            }
        }

        void imu_callback(const sensor_msgs::Imu msg) {
           curr_yawrate_reading = msg.angular_velocity.z;
           yawrate_reading_time = ros::Time::now().toSec();
        }

        void control_update(const ros::TimerEvent& event) {
            force_compensator->update_forces(force_output);
        }

};

             






    
