
    t_posz[0] = 0.0*cycle_duration;
    t_posz[1] = 0.01*cycle_duration;
    // t_posz[2] = 0.1*cycle_duration;
    t_posz[2] = 0.2*cycle_duration;
    t_posz[3] = 0.3 * cycle_duration;
    t_posz[4] = 0.4 * cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posz[5] = 0.5 * cycle_duration;
    t_posz[6] = 0.6 * cycle_duration;;//cycle_duration - parameters.t_dsp / 2 ;
    t_posz[7] = 0.65*cycle_duration;
    t_posz[8] = 0.8*cycle_duration;
    t_posz[9] = 1*cycle_duration;

    z[0] = parameters.robot_ankle_to_foot+0.0 * swing_foot_z_peak;
    z[1] = parameters.robot_ankle_to_foot+0.001 * swing_foot_z_peak;
    z[2] = parameters.robot_ankle_to_foot+0.3 * swing_foot_z_peak;

    z[3] = parameters.robot_ankle_to_foot+0.6 * swing_foot_z_peak;
    z[4] = parameters.robot_ankle_to_foot+0.8 * swing_foot_z_peak;
    
    //z[3] = parameters.robot_ankle_to_foot*cos(q_f)+parameters.foot_length_back*sin(q_f);
    z[5] = parameters.robot_ankle_to_foot+0.4 * swing_foot_z_peak;
    z[6] = parameters.robot_ankle_to_foot+0.02 * swing_foot_z_peak;
    z[7] = parameters.robot_ankle_to_foot+0.01 * swing_foot_z_peak;
    z[8] = parameters.robot_ankle_to_foot+0.0 * swing_foot_z_peak;
    z[9] = parameters.robot_ankle_to_foot+0.0 * swing_foot_z_peak;

    t_posx[0] = 0.0*cycle_duration;
    t_posx[1] = 0.4*cycle_duration;
    // t_posx[2] = 0.6*cycle_duration;
    // t_posx[2] = 0.3*cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posx[2] = 0.7*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    t_posx[3] = 0.99*cycle_duration;
    t_posx[4] = 1*cycle_duration;

    //if(x_swing_foot_start ! = x_swing_foot_end){
    x[0]                      = x_swing_foot_start;
    // x[1]                      = x_swing_foot_start;
    // x[2]                      = x_swing_foot_start + swing_foot_x_peak;
    x[1]                      = x_swing_foot_start + step_length * 0.7;//x_swing_foot_end * 0.8 - parameters.robot_ankle_to_foot*sin(q_f)-parameters.foot_length_back*(1-cos(q_f));
    x[2]                      = x_swing_foot_start + step_length * 1.3;
    x[3]                      = x_swing_foot_end;
    x[4]                      = x_swing_foot_end;
    