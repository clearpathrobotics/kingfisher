#include <kingfisher_node/kingfisher_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv, "kingfisher_controller");
    ros::NodeHandle nh;
    KingfisherController kf_control(nh);
    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel",1,&KingfisherController::twist_callback, &kf_control);
    ros::Subscriber imu_sub = nh.subscribe("imu/data",1,&KingfisherController::imu_callback, &kf_control);
    ros::Subscriber joy_sub = nh.subscribe("joy",1,&KingfisherController::joy_callback, &kf_control);
    ros::Timer control_output = nh.createTimer(ros::Duration(0.1), &KingfisherController::control_update,&kf_control); 
    ros::spin();

    return 0;
}
