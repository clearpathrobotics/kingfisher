#include <kingfisher_node/force_compensator.h>

double ForceCompensator::get_output (double thrust) {
    //saturate

    double output = 0;
    if (thrust > 0)
        output = thrust * (MAX_OUTPUT/MAX_FWD_THRUST);
    else if (thrust < 0)
        output = thrust * (MAX_OUTPUT/MAX_BCK_THRUST);
    return output; 
}

double ForceCompensator::saturate_thrusters (double thrust) {
    thrust = std::min(MAX_FWD_THRUST,thrust);
    thrust = std::max(-1*MAX_BCK_THRUST,thrust);
    return thrust;
}


void ForceCompensator::update_forces (geometry_msgs::Wrench output) {
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

    left_thrust += fx/2.0;
    right_thrust += fx/2.0;
    
    left_thrust = saturate_thrusters (left_thrust);
    right_thrust = saturate_thrusters (right_thrust);


    //ROS_INFO("FX:%f,TAUZ:%f,LTHR:%f,RTHR:%f",fx,tauz,left_thrust,right_thrust);

    cmd_output.left = get_output (left_thrust);
    cmd_output.right = get_output (right_thrust);
    cmd_pub_.publish(cmd_output);

    pub_effective_wrench(left_thrust, right_thrust);
}

void ForceCompensator::pub_effective_wrench(double left_thrust,double right_thrust) {
    geometry_msgs::Wrench effective_output;
    effective_output.force.x = left_thrust + right_thrust;
    effective_output.torque.z = (left_thrust - right_thrust)*BOAT_WIDTH;
    eff_pub_.publish(effective_output);  
}

