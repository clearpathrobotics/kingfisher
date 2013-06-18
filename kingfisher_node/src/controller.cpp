#include <kingfisher_node/controller.h>

Controller::Controller(ros::NodeHandle &n):node_(n) {
    force_compensator_ = new ForceCompensator(node_);
    node_.param<double>("imu_data_timeout",imu_data_timeout_,1/10.0);



    //Setup Yaw Rate Controller
    yr_dbg_pub_ = node_.advertise<geometry_msgs::Vector3>("yaw_rate_debug",1000);

    node_.param<double>("yaw_rate/kp", yr_kp_,2.0); 
    node_.param<double>("yaw_rate/kd", yr_kd_,1.0);
    node_.param<double>("yaw_rate/ki", yr_ki_,0.0);
    //TODO:Clean up integral clamps
    //  node_.param<double>("yaw_rate/imax", yr_imax_,MAX_BCK_THRUST*2*BOAT_WIDTH); //clamp integral output at max yaw yorque
    //  node_.param<double>("yaw_rate/imin", yr_imin_,-MAX_BCK_THRUST*2*BOAT_WIDTH);
    node_.param<double>("yaw_rate/imax", yr_imax_,0.0); //clamp integral output at max yaw yorque
    node_.param<double>("yaw_rate/imin", yr_imin_,0.0);


    yr_pid_.reset();
    yr_pid_.initPid(yr_kp_,yr_ki_,yr_kd_,yr_imax_,yr_imin_);
    yr_cmd_ = 0;
    yr_cmd_time_ = ros::Time::now().toSec();
    last_yr_cmd_time_ = ros::Time::now().toSec();
    new_yr_cmd_ = false;


    yr_meas_ = 0;
    yr_meas_time_=0;



    //Setup Yaw Controller
    y_dbg_pub_ = node_.advertise<geometry_msgs::Vector3>("yaw_rate_debug",1000);
    y_pid_.reset();
    y_pid_.initPid(y_kp_,y_ki_,y_kd_,y_imax_,y_imin_);
    y_cmd_ = 0;
    y_cmd_time_ = ros::Time::now().toSec();
    last_y_cmd_time_ = ros::Time::now().toSec();

    node_.param<double>("yaw/kp", y_kp_,2.0); 
    node_.param<double>("yaw/kd", y_kd_,1.0);
    node_.param<double>("yaw/ki", y_ki_,0.0);
    //TODO:Clean up integral clamps
    //  node_.param<double>("yaw_rate/imax", y_imax_,MAX_YAW_RATE); //clamp integral output at max yaw yorque
    //  node_.param<double>("yaw_rate/imin", y_imin_,-MAX_YAW_RATE);
    node_.param<double>("yaw/imax", y_imax_,0.0); //clamp integral output at max yaw yorque
    node_.param<double>("yaw/imin", y_imin_,0.0);
    y_meas_ = 0;
    y_meas_time_=0;
    new_yaw_cmd_ = false;
 
    //Setup Speed Control
    spd_cmd_ = 0;
    
}

double Controller::yr_compensator() {
    if (ros::Time::now().toSec() - yr_meas_time_ > imu_data_timeout_) {
        ROS_ERROR("IMU Data timeout, controller deactivated. IMU data not being received");
        return 0.0;
    }
    else {
        //calculate pid torque z
        double dt = yr_cmd_time_ - last_yr_cmd_time_; 
        double yr_error = yr_cmd_ - yr_meas_;   
        double yr_comp_output = yr_pid_.updatePid(yr_error, ros::Duration(dt));

        geometry_msgs::Vector3 dbg_info;             
        dbg_info.x = yr_cmd_;
        dbg_info.y = yr_meas_;
        dbg_info.z = yr_comp_output;
        yr_dbg_pub_.publish(dbg_info);
  
        return yr_comp_output;
        
    }
}

double Controller::y_compensator() {
    if (ros::Time::now().toSec() - y_meas_time_ > imu_data_timeout_) {
        ROS_ERROR("IMU Data timeout, controller deactivated. IMU data not being received");
        return 0.0;
    }
    else {
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
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "controller");
    ros::NodeHandle nh;
    Controller kf_control(nh);
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel",1,&Controller::twist_callback, &kf_control);
    ros::Subscriber wrench_sub = nh.subscribe("cmd_wrench",1,&Controller::wrench_callback, &kf_control);

    ros::Subscriber yaw_sub = nh.subscribe("cmd_yaw",1,&Controller::yaw_callback, &kf_control);


    ros::Subscriber imu_sub = nh.subscribe("imu/data",1,&Controller::imu_callback, &kf_control);
    ros::Timer control_output = nh.createTimer(ros::Duration(0.1), &Controller::control_update,&kf_control); 
    ros::spin();

    return 0;
}
