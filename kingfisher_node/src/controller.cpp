#include <kingfisher_node/controller.h>

Controller::Controller(ros::NodeHandle &n):node_(n) {
    force_compensator_ = new ForceCompensator(node_);
    node_.param<double>("imu_data_timeout",imu_data_timeout_,1/10.0);

    //Setup Yaw Rate Controller
    yr_dbg_pub_ = node_.advertise<geometry_msgs::Vector3>("yaw_rate_debug",1000);
    y_dbg_pub_ = node_.advertise<geometry_msgs::Vector3>("yaw_debug",1000);

    node_.param<double>("yaw_rate/kf", yr_kf_,10); //Feedforward Gain
    node_.param<double>("yaw_rate/kp", yr_kp_,2.0);  //Proportional Gain
    node_.param<double>("yaw_rate/kd", yr_kd_,1.0); //Derivative Gain
    node_.param<double>("yaw_rate/ki", yr_ki_,0.0); //Integral Gain
    node_.param<double>("yaw_rate/imax", yr_imax_,0.0); //Clamp Integral Outputs
    node_.param<double>("yaw_rate/imin", yr_imin_,0.0);
    node_.param<double>("yaw_rate/cmd_timeout", yr_cmd_timeout_,0.5); //If the commands dont show up in this much time don't send out drive commans
    yr_meas_ = 0;
    yr_meas_time_=0;

    node_.param<double>("yaw/kp", y_kf_,5.0); 
    node_.param<double>("yaw/kp", y_kp_,5.0); 
    node_.param<double>("yaw/kd", y_kd_,1.0);
    node_.param<double>("yaw/ki", y_ki_,0.5);
    node_.param<double>("yaw/imax", y_imax_,0.0); //clamp integral output at max yaw yorque
    node_.param<double>("yaw/imin", y_imin_,0.0);
    node_.param<double>("yaw/cmd_timeout",y_cmd_timeout_,0.5);
    y_meas_ = 0;
    y_meas_time_=0;

    ROS_INFO("Yaw Rate Params (F,P,I,D,Max,Min):%f,%f,%f,%f,%f,%f",yr_kf_,yr_kp_, yr_ki_,yr_kd_,yr_imax_,yr_imin_);
    ROS_INFO("Yaw Params (F,P,I,D,Max,Min):%f,%f,%f,%f,%f,%f",y_kf_,y_kp_, y_ki_,y_kd_,y_imax_,y_imin_);

    yr_pid_.reset();
    yr_pid_.initPid(yr_kp_,yr_ki_,yr_kd_,yr_imax_,yr_imin_);
    yr_cmd_ = 0;
    yr_cmd_time_ = 0;
    last_yr_cmd_time_ = 0;

    //Setup Yaw Controller
    y_pid_.reset();
    y_pid_.initPid(y_kp_,y_ki_,y_kd_,y_imax_,y_imin_);
    y_cmd_ = 0;
    y_cmd_time_ = 0;
    last_y_cmd_time_ = 0;
 
    //Setup Speed Control (linear mapping)
    spd_cmd_ = 0;
    node_.param<double>("max/fwd_vel", max_fwd_vel_,MAX_FWD_VEL); 
    node_.param<double>("max/fwd_force", max_fwd_force_,MAX_FWD_THRUST); 
    node_.param<double>("max/bck_vel",max_bck_vel_,MAX_BCK_VEL);
    node_.param<double>("max/bck_force",max_bck_force_,MAX_BCK_THRUST);
}

double Controller::yr_compensator() {
    //calculate pid torque z
    double dt = yr_cmd_time_ - last_yr_cmd_time_; 
    double yr_error = yr_cmd_ - yr_meas_;   
    double yr_comp_output = yr_pid_.updatePid(-yr_error, ros::Duration(1/20.0));
    yr_comp_output = yr_comp_output + yr_kf_*yr_cmd_; //feedforward
    geometry_msgs::Vector3 dbg_info;             
    dbg_info.x = yr_cmd_;
    dbg_info.y = yr_meas_;
    dbg_info.z = yr_comp_output;
    yr_dbg_pub_.publish(dbg_info);
    return yr_comp_output;
}

double Controller::y_compensator() {
    //calculate pid torque z
    double dt = y_cmd_time_ - last_y_cmd_time_; 
    double y_error = yr_cmd_ - y_meas_;   
    double y_comp_output = y_pid_.updatePid(y_error, ros::Duration(dt));
    geometry_msgs::Vector3 dbg_info;             
    dbg_info.x = y_cmd_;
    dbg_info.y = y_meas_;
    dbg_info.z = y_comp_output;
    y_dbg_pub_.publish(dbg_info);
    return y_comp_output;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "controller");
    ros::NodeHandle nh;
    Controller kf_control(nh);
    //ros::Subscriber vel_sub = nh.subscribe("cmd_vel",1,&Controller::twist_callback, &kf_control);
    ros::Subscriber wrench_sub = nh.subscribe("cmd_wrench",1, &Controller::wrench_callback, &kf_control);
    ros::Subscriber helm_sub = nh.subscribe("cmd_helm",1, &Controller::helm_callback,&kf_control);
    ros::Subscriber heading_sub = nh.subscribe("cmd_heading",1, &Controller::heading_callback,&kf_control);
    ros::Subscriber imu_sub = nh.subscribe("imu/data",1, &Controller::imu_callback, &kf_control);
    ros::Timer control_output = nh.createTimer(ros::Duration(1/50.0), &Controller::control_update,&kf_control); 
    ros::spin();

    return 0;
}
