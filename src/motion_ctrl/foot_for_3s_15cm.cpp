 if(flag == 1){
        cycle_duration = parameters.t_ssp;
    }
    else{
        cycle_duration = parameters.step_duration;
    }
    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory
    double q_f = 0*pi/180;

    std::vector<double> t_posz(8), t_posx(6), t_theta(7), theta(7), x(6), z(8);
    // Calculate x
    double step_length = x_swing_foot_end-x_swing_foot_start;
    double t0,t1,t2,t3,t4,t5,t6,t7,zk0,zk1,zk2,zk3,zk4,zk5,zk6, zk7;
    double xk0,xk1,xk2,xk3,xk4,xk5,xk6;
    t0 = 0;
    t1 = 0.01;
    t2 = 0.3;
    t3 = 0.45;
    t4 = 0.6;
    t5 = 0.8;
    t6 = 0.99;
    t7 = 1;

    zk0 = 0;
    zk1 = 0;
    zk2 = 1;
    zk3 = 0.2;
    zk4 = 0;
    zk5 = 0;
    zk6 = 0;
    zk7 = 0;


    t_posz[0] = t0*cycle_duration;
    t_posz[1] = t1*cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posz[2] = t2*cycle_duration;
    t_posz[3] = t3*cycle_duration;
    t_posz[4] = t4*cycle_duration;
    t_posz[5] = t5*cycle_duration;
    t_posz[6] = t6*cycle_duration;
    t_posz[7] = t7*cycle_duration;

    z[0] = parameters.robot_ankle_to_foot+zk0 * swing_foot_z_peak;
    z[1] = parameters.robot_ankle_to_foot+zk1 * swing_foot_z_peak;
    z[2] = parameters.robot_ankle_to_foot+zk2 * swing_foot_z_peak;
    z[3] = parameters.robot_ankle_to_foot+zk3 * swing_foot_z_peak;
    z[4] = parameters.robot_ankle_to_foot+zk4 * swing_foot_z_peak;
    z[5] = parameters.robot_ankle_to_foot+zk5 * swing_foot_z_peak;
    z[6] = parameters.robot_ankle_to_foot+zk6 * swing_foot_z_peak;
    z[7] = parameters.robot_ankle_to_foot+zk7 * swing_foot_z_peak;

    // t_posx[0] = 0.0*cycle_duration;
    // t_posx[1] = 0.4*cycle_duration;
    // t_posx[2] = 0.6*cycle_duration;
    // t_posx[3] = 0.3*cycle_duration;//cycle_duration - parameters.t_dsp;
    // t_posx[4] = 0.7*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    // t_posx[5] = 0.99*cycle_duration;
    // t_posx[6] = 1*cycle_duration;
    t_posx[0] = t0*cycle_duration;
    // t_posx[1] = (t3-0.2)*cycle_duration;
    t_posx[1] = (t3-0.1)*cycle_duration;
    t_posx[2] = t3*cycle_duration;
    t_posx[3] = t4*cycle_duration;
    t_posx[4] = t5*cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posx[5] = t6*cycle_duration;//cycle_duration - parameters.t_dsp/2;


    double swinging_distance = x_swing_foot_end - x_swing_foot_start;

    //if(x_swing_foot_start ! = x_swing_foot_end){
    // x[0]                      = x_swing_foot_start;
    // x[1]                      = x_swing_foot_start;
    // x[2]                      = x_swing_foot_start + swing_foot_x_peak;
    // x[3]                      = x_swing_foot_start + swinging_distance * 0.5;//x_swing_foot_end * 0.8 - parameters.robot_ankle_to_foot*sin(q_f)-parameters.foot_length_back*(1-cos(q_f));
    // x[4]                      = x_swing_foot_start + swinging_distance * 0.8;
    // x[5]                      = x_swing_foot_end;
    // x[6]                      = x_swing_foot_end;
    
    xk0 = 0;
    xk1 = 1;
    xk2 = 1;
    xk3 = 1;
    xk4 = 1;
    xk5 = 1;

    x[0]                      = x_swing_foot_start + swinging_distance * xk0;
    x[1]                      = x_swing_foot_start + swinging_distance * xk1;
    x[2]                      = x_swing_foot_start + swinging_distance * xk2;
    x[3]                      = x_swing_foot_start + swinging_distance * xk3;
    x[4]                      = x_swing_foot_start + swinging_distance * xk4;
    x[5]                      = x_swing_foot_start + swinging_distance * xk5;
    