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

    public:

        ForceCompensator(ros::NodeHandle &n): node_(n) {
            cmd_pub = node_.advertise<kingfisher_msgs::Drive>("cmd_drive",1000);
        }
        ~ForceCompensator() {
        }

        double get_output (double thrust) {
            //saturate
            thrust = std::min(MAX_FWD_THRUST,thrust);
            thrust = std::max(-1*MAX_BCK_THRUST,thrust);

            double output = 0;
            if (thrust > 0)
                output = thrust * (MAX_OUTPUT/MAX_FWD_THRUST);
            else if (thrust < 0)
                output = -1 * thrust * (MAX_OUTPUT/MAX_BCK_THRUST);
            return output;    
        }


        void update_forces (geometry_msgs::Wrench output) {
            kingfisher_msgs::Drive cmd_output;
            double forward = output.force.x;
            double yaw = output.force.z;

            double left_thrust = forward/2.0;
            double right_thrust = forward/2.0;
            
            left_thrust-=yaw/(2*BOAT_WIDTH); 
            right_thrust += yaw/(2*BOAT_WIDTH); 

            cmd_output.left = get_output (left_thrust);
            cmd_output.right = get_output (right_thrust);
        }
};


class KingfisherController {
    private:
        ros::NodeHandle node_;
        ForceCompensator *force_compensator;
        geometry_msgs::Wrench force_output;
        int last_button;
        bool auto_control;

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
            force_output.force.x = msg.linear.x * 4;
            force_output.torque.z = msg.angular.z * 1;
        }

        void imu_callback(const sensor_msgs::Imu msg) {
            
        }

        void control_update(const ros::TimerEvent& event) {
            force_compensator->update_forces(force_output);
        }

};

             






    
