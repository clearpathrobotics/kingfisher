#include <ros/ros.h>
#include <kingfisher_msgs/Drive.h>
#include <geometry_msgs/Wrench.h>
#include <kingfisher_node/kf_constants.h>

class ForceCompensator
{
    private:
        ros::NodeHandle node_;
        ros::Publisher cmd_pub_;
        ros::Publisher eff_pub_;

    public:

        ForceCompensator(ros::NodeHandle &n): node_(n) {
            cmd_pub_ = node_.advertise<kingfisher_msgs::Drive>("cmd_drive",1000);
            eff_pub_ = node_.advertise<geometry_msgs::Wrench>("eff_wrench",1000);
        }
        ~ForceCompensator() {
        }

        double get_output (double thrust);
        double saturate_thrusters (double thrust);
        void update_forces (geometry_msgs::Wrench output);
        void pub_effective_wrench(double left_thrust,double right_thrust);

};

