#include <P1MC.h>

//===================================
//=========Utility functions=========
//===================================

// Sign returns the sign of x (-1, 1 or 0). 
int sign(double x){return (x > 0) - (x < 0);}

// Takes a 3xn vector and extracts the first row into a 1xn vector
void extract_pos_of_vector(std::vector<dlib::matrix<double,3,1>> vector_array, std::vector<double> * pos){
    for(std::vector<dlib::matrix<double,3,1>>::iterator it = vector_array.begin(); it != vector_array.end(); ++it){
        dlib::matrix<double,3,1> state_vector = *it;
        pos->push_back(state_vector(0));
    }
}

// Measures the time between the two cliptime function. Usage: cliptime(&time passed, &t_initial), some calculation, cliptime(&time passed, &t_initial)
void clipTime(double * time_passed, double * t_initial){
    struct timeval tv;
    struct timezone tz;
    gettimeofday (&tv , &tz);
    if (*t_initial == 0){
        *time_passed = 0;
        *t_initial = tv.tv_usec * 0.001 + tv.tv_sec * 1000;
    }
    else{
        *time_passed = (tv.tv_usec * 0.001 + tv.tv_sec * 1000) - *t_initial;
        //*time_passed = *time_passed/ 1000.0;
        // std::cout << "Time passed: " << *time_passed << "ms" << std::endl;
        *t_initial   = 0;
    }

}



/*===============================================*/
/*================ ROBOT CLASS ==================*/
/*===============================================*/


// Sets all parameters for the gait, robot and calculation
void Robot::set_parameters(){

    parameters.step_duration           = 0.5;     //* Duration for one step. This step consists of 80% SSP and 20% DSP. NOT gait cycle duration (which is 2*step_duration)
    parameters.steps_real              = 1000;   // Amount of steps the robot takes. The first and last step count as 1 step (in terms of step length) because it's only half a step.
    parameters.z_com_ctm               = 0.6;   //* The CoM height used for CTM and calculating the CoM trajectory of the MBM
    parameters.z_com_mbm               = 0.6;   //* 55-60cm The CoM height used for the MBM and the actual height of the robot CoM (how to follow)
    parameters.zmp_offset_x            = -0.03;  // Offset in x direction for the ZMP. Use this variable to compensate the LIPM and MBM model error (ca 2-4 cm depending on walking speed)
    parameters.zmp_offset_y            = 0.049;  // Offset in x direction for the ZMP. Use this variable to compensate the LIPM and MBM model error (ca 2-4 cm depending on walking speed)

    parameters.zmp_offset_y_left            = 0.035;  // Offset in x direction for the ZMP. Use this variable to compensate the LIPM and MBM model error (ca 2-4 cm depending on walking speed)
    parameters.zmp_offset_y_right            = 0.05;  // Offset in x direction for the ZMP. Use this variable to compensate the LIPM and MBM model error (ca 2-4 cm depending on walking speed)

    parameters.step_length             = 0.15;   //* Length of one step
    parameters.step_width             = 0.1;   //* Length of one step

    parameters.pz_com                  = 0.00;  // Distance between the middle of both hips and the COM of the robot. By design the COM was put into the middle of the hips. But can be higher (the higher the better) later on.
    parameters.Tipping_Scale_left      = 0.9; //88     //* 0.7-1 This value modifies the hip roll in case of soft joints. Due to control schemes, this value can be kept at 1. 
    parameters.Tipping_Scale_right     = 0.9; //88    //* This value modifies the hip roll in case of soft joints. Due to control schemes, this value can be kept at 1. 
    // parameters.Tipping_Scale_left_support      = 1;     // This value modifies the hip roll in case of soft joints. Due to control schemes, this value can be kept at 1. 
    // parameters.Tipping_Scale_right_support     = 1;     // This value modifies the hip roll in case of soft joints. Due to control schemes, this value can be kept at 1. 

    // Fixed, these parameters are "fixed" and no DoF and shouldnt be used for tuning
    parameters.Ts                  = 5e-3;  // Sample time between each cycle. Better not change this value because the sample time is hard coded for the LQR calculation (during the discretization step)
    parameters.dsp_percentage      = 20.0;  // Ratio between DSP and SSP (20:80) is set. Usually does not need to be changed
    parameters.robot_width         = 0.18;  // Width of the robot (from z-axis of the hip to z-axis of the other hip)
    parameters.robot_knee_length   = 0.275; // From hip-y to knee-y
    parameters.robot_shin_length   = 0.275; // From knee-y to ankle-y
    parameters.robot_ankle_to_foot = 0.109; // From ankle-y to ground
    parameters.foot_length_front   = 0.130; // From ankle-z to tip of the foot
    parameters.foot_length_back    = 0.095; // From ankle-z to rear of the foot
    parameters.foot_width          = 0.147; // Width of the foot
    parameters.N_T                 = 400;   // Prediction horizon of the MPC
    parameters.g                   = 9.81;  // Gravity constant

    parameters.steps               = parameters.steps_real - 1;                                 // Actual amount of steps (because the first and last step is only half a step)
    parameters.t_sim               = parameters.steps_real*parameters.step_duration;            // Time for the whole gait
    parameters.N                   = parameters.t_sim/parameters.Ts;                            // Amount of samples
    parameters.t_dsp               = parameters.step_duration*parameters.dsp_percentage/100;    // Time for DSP
    parameters.t_ssp               = parameters.step_duration - parameters.t_dsp;               // Time for SSP

    // Safety parameters for ZMP. If the ATI is flawed and calculates the ZMP to inf or large, then some saftey mechanism needs to be active. 
    parameters.ZMP_x_min = -0.5; 
    parameters.ZMP_x_max =  0.5;
    parameters.ZMP_y_max =  0.5;
    parameters.ZMP_y_min = -0.5;

    // Estimation parameters for the Kalman filter
    parameters.Q_x                 = 1e-8; // Process noise
    parameters.R_pos_x             = 1e-3; // Measurement noise of the encoder (position is gained from FK)
    parameters.R_cop_x             = 1e-3; // Measurement noise of the ATI (ZMP is calculated from F/T information)
    parameters.Q_y                 = 1e-8;
    parameters.R_pos_y             = 1e-3;
    parameters.R_cop_y             = 1e-3;

    parameters.Fx_ref = 0;      // The desired force in x direction during foot landing control
    parameters.Fz_ref = 150;    // The desired force in z direction during foot landing control

    parameters.debug_msg            = 0;        // Display debug messages
    parameters.control_leg_length   = 0;        // Control the leg length (equivalent to using foot landing control)
    parameters.use_feedback_x       = 0;
    parameters.use_feedback_y       = 0;

    parameters.control_pitch        = 0;        // Control the pitch of the upper body (using gyro information)
    parameters.control_roll         = 0;        // Control the roll of the upper body (using gyro information)

    psi1 = 0;
    psi2 = 0;
    theta1 = 0;
    theta2 = 0;
    phi1 = 0;
    phi2 = 0;

}

// Constructor for a robot. The input the mode (walking mode). Currently there are 3 modes: SSP balance, walking and squatting. 
Robot::Robot(int mode){

    this->set_parameters(); // Set the robot parameters
    this->set_LQR_gains();  // Calculate the LQR gains with the parameters fo r MPC
    // Initiate some variables
    body_theta_fk                   = 0;      
    in_foot_landing_control_phase   = 0;

    // dlib::matrix<double, 3, 1> x_init, y_init;
    // x_init = 0, 0, 0;
    // y_init = 0, 0, 0;
    // y_init = parameters.robot_width/2, 0, 0;

    // std::cout << "Y init set to " << y_init << std::endl;
    // X_ref.push_back(x_init); 
    X_est = 0,0,0,0;
    // Y_ref.push_back(y_init); 
    Y_est = 0,0,0,0;
    walking_mode = mode; 
    // Calculate the walking trajectories based on the mode
    if(mode == 1){
        // Stand on one leg and balance on one leg
        this->calc_trajectories_stand_ssp();
    }
    else if(mode == 2 ){
        // Move in a sine wave on one leg
        this->calc_trajectories_stand_ssp_sine();
    }
    else if(mode == 3 ){
        // Move in a sine wave on two legs
        this->calc_trajectories_stand_dsp_sine();
    }
    else if(mode == 4){
        // // // This mode tests all different pieces
        int forward  = 1;
        int leftFootStart = 1;
        int step_amount = 20;
        double step_length = 0.15;

        this->calc_trajectories_walk_straight(forward, leftFootStart, step_amount, step_length);
    }
    else if(mode == 0){
        // Walk straight
        this->calc_trajectories_walk();   
    }  
    else{
        this->calc_trajectories_walk();   
    }
    this->initKalman(); // Initialize Kalman Filter

    left_support_foot_state.p   = 0, parameters.robot_width/2, parameters.robot_ankle_to_foot;
    right_support_foot_state.p  = 0, -parameters.robot_width/2, parameters.robot_ankle_to_foot;
};

// This constructor is unused atm. Can be initialized with some other parameters
Robot::Robot(Parameters robot_parameters, int mode){

    this->set_LQR_gains();

    parameters = robot_parameters;
    in_foot_landing_control_phase = 0;
    // parameters.control_leg_length            = 0;
    // dlib::matrix<double, 3, 1> x_init, y_init;
    // x_init = 0, 0, 0;
    // y_init = 0, 0, 0;

    // X_ref.push_back(x_init); 
    X_est = 0,0,0,0;

    // Y_ref.push_back(y_init); 
    Y_est = 0,0,0,0;

    walking_mode = mode; 

    if(mode == 1){
        this->calc_trajectories_stand_ssp();
    }
    else{
        this->calc_trajectories_walk();   
    }
    this->initKalman();
};

// This function is used to move all angles to zero at startup
bool Robot::goto_zero(dxl_Actuator *dxl_actuator){

    if(dxl_actuator->get_present_theta() > 0.1)
    {
        dxl_actuator->traj_theta = dxl_actuator->traj_theta - 12.0/200;
        dxl_actuator->set_goal_theta(dxl_actuator->traj_theta);
    }

    if(dxl_actuator->get_present_theta() < -0.1)
    {
        dxl_actuator->traj_theta = dxl_actuator->traj_theta + 12.0/200;
        dxl_actuator->set_goal_theta(dxl_actuator->traj_theta);
    }

    if(abs(dxl_actuator->get_present_theta()) <= 0.1)
    {
        dxl_actuator->set_goal_position(0);
        return true;
    }
    return false;   
}
void Robot::print_ref_length(){
    std::cout << "Support: " << std::endl;
    std::cout << "x: " <<ref_x_support_foot_trajectory.size()<< std::endl;
    std::cout << "y: " <<ref_y_support_foot_trajectory.size() << std::endl;
    std::cout << "z: " << ref_z_support_foot_trajectory.size()<< std::endl;
    std::cout << "psi: " <<ref_psi_support_foot_trajectory.size() << std::endl;

    std::cout << "Swing: " << std::endl;
    std::cout << "x: " <<ref_x_swing_foot_trajectory.size() << std::endl;
    std::cout << "y: " << ref_y_swing_foot_trajectory.size()<< std::endl;
    std::cout << "z: " << ref_z_swing_foot_trajectory.size()<< std::endl;
    std::cout << "theta: " << ref_theta_swing_foot_trajectory.size()<< std::endl;
    std::cout << "psi: " << ref_psi_swing_foot_trajectory.size() << std::endl;


    std::cout << "ZMP: " << std::endl;
    std::cout << "x: " << ref_x_zmp_trajectory.size()<< std::endl;
    std::cout << "y: " <<ref_y_zmp_trajectory.size()<< std::endl;
    
    std::cout << "Body psi: " << ref_psi_body.size() << std::endl;

}

void Robot::save_last_step(double * support_x_last, double * support_y_last, double * support_z_last, double * support_psi_last,
    double * swing_x_last, double * swing_y_last, double * swing_z_last, double * swing_theta_last, double * swing_psi_last,
    double * x_zmp_last, double * y_zmp_last){

    *support_x_last = ref_x_support_foot_trajectory.back()-x_ref.back();
    *support_y_last = ref_y_support_foot_trajectory.back()-y_ref.back();
    *support_z_last = ref_z_support_foot_trajectory.back();
    *support_psi_last = ref_psi_support_foot_trajectory.back()-ref_psi_body.back();
    
    *swing_x_last = ref_x_swing_foot_trajectory.back()-x_ref.back();
    *swing_y_last = ref_y_swing_foot_trajectory.back()-y_ref.back();
    *swing_z_last = ref_z_swing_foot_trajectory.back();
    *swing_theta_last = ref_theta_swing_foot_trajectory.back();
    *swing_psi_last = ref_psi_swing_foot_trajectory.back()-ref_psi_body.back();
    
    *x_zmp_last = ref_x_zmp_trajectory.back()-x_ref.back();
    *y_zmp_last = ref_y_zmp_trajectory.back()-y_ref.back();

}

void Robot::erase_trajectories(){

    ref_x_support_foot_trajectory.erase(ref_x_support_foot_trajectory.begin(), ref_x_support_foot_trajectory.end());
    ref_y_support_foot_trajectory.erase(ref_y_support_foot_trajectory.begin(), ref_y_support_foot_trajectory.end());
    ref_z_support_foot_trajectory.erase(ref_z_support_foot_trajectory.begin(), ref_z_support_foot_trajectory.end());
    ref_psi_support_foot_trajectory.erase(ref_psi_support_foot_trajectory.begin(), ref_psi_support_foot_trajectory.end());

    ref_x_swing_foot_trajectory.erase(ref_x_swing_foot_trajectory.begin(), ref_x_swing_foot_trajectory.end());
    ref_y_swing_foot_trajectory.erase(ref_y_swing_foot_trajectory.begin(), ref_y_swing_foot_trajectory.end());
    ref_z_swing_foot_trajectory.erase(ref_z_swing_foot_trajectory.begin(), ref_z_swing_foot_trajectory.end());
    ref_theta_swing_foot_trajectory.erase(ref_theta_swing_foot_trajectory.begin(), ref_theta_swing_foot_trajectory.end());
    ref_psi_swing_foot_trajectory.erase(ref_psi_swing_foot_trajectory.begin(), ref_psi_swing_foot_trajectory.end());

    ref_x_zmp_trajectory.erase(ref_x_zmp_trajectory.begin(), ref_x_zmp_trajectory.end());
    ref_y_zmp_trajectory.erase(ref_y_zmp_trajectory.begin(), ref_y_zmp_trajectory.end());

    x_footprint_on_spline.erase(x_footprint_on_spline.begin(), x_footprint_on_spline.end());
    y_footprint_on_spline.erase(y_footprint_on_spline.begin(), y_footprint_on_spline.end());
    yaw_footprint_on_spline.erase(yaw_footprint_on_spline.begin(), yaw_footprint_on_spline.end());
    x_footprint_on_circle.erase(x_footprint_on_circle.begin(), x_footprint_on_circle.end()); 
    y_footprint_on_circle.erase(y_footprint_on_circle.begin(), y_footprint_on_circle.end()); 
    yaw_footprint_on_circle.erase(yaw_footprint_on_circle.begin(), yaw_footprint_on_circle.end());

    
    ref_psi_body.erase(ref_psi_body.begin(), ref_psi_body.end());
    
    u_x.erase(u_x.begin(), u_x.end());
    u_y.erase(u_y.begin(), u_y.end());
    
    q1.erase(q1.begin(), q1.end());
    q2.erase(q2.begin(), q2.end());
    q3.erase(q3.begin(), q3.end());
    q4.erase(q4.begin(), q4.end());
    q5.erase(q5.begin(), q5.end());
    q6.erase(q6.begin(), q6.end());
    q7.erase(q7.begin(), q7.end());
    q8.erase(q8.begin(), q8.end());
    q9.erase(q9.begin(), q9.end());
    q10.erase(q10.begin(), q10.end());
    q11.erase(q11.begin(), q11.end());
    q12.erase(q12.begin(), q12.end());
    
    x_ref.erase(x_ref.begin(), x_ref.end());
    
    y_ref.erase(y_ref.begin(), y_ref.end());
    
    z_ref.erase(z_ref.begin(), z_ref.end()); 
    
    z_ref_swing_foot_complete_unmodified.erase(z_ref_swing_foot_complete_unmodified.begin(), z_ref_swing_foot_complete_unmodified.end());

    time_step = 0;
    print_ref_length();
}

void Robot::change_Tipping_Scale(){
    double adjustment_time = parameters.t_dsp;
    if(leftIsSupportLeg){
        if(in_foot_landing_control_phase == 0){
            if((Tipping_Scale_left_ref-parameters.Tipping_Scale_left_swing) < 0.01){
                Tipping_Scale_left_ref 
                -= (parameters.Tipping_Scale_left_support-parameters.Tipping_Scale_left_swing)/adjustment_time;
            }  
            if((Tipping_Scale_right_ref-parameters.Tipping_Scale_right_support) < 0.01){
                Tipping_Scale_right_ref 
                += (parameters.Tipping_Scale_right_support-parameters.Tipping_Scale_right_swing)/adjustment_time;
            }
        }
        // else{
        //     Tipping_Scale_left_ref = parameters.Tipping_Scale_left_support;
        // }
    }
    else{
        if(in_foot_landing_control_phase == 0){
            if((Tipping_Scale_right_ref-parameters.Tipping_Scale_right_swing) < 0.01){
                Tipping_Scale_right_ref 
                -= (parameters.Tipping_Scale_right_support-parameters.Tipping_Scale_right_swing)/adjustment_time;
            }
            if((Tipping_Scale_left_ref-parameters.Tipping_Scale_left_support) < 0.01){
                Tipping_Scale_left_ref 
                += (parameters.Tipping_Scale_left_support-parameters.Tipping_Scale_left_swing)/adjustment_time;
            }
        }

    }

}


// After standing straight (all angles are zero), the robot gets into the starting position (squatting with the COM at the given height, bent knees)
void Robot::stand(dxl_Actuator *dxl_actuator, int nNum){

    float stand[12]={0,0,0,0,0,0,
        0,0,0,0,0,0};

        left_foot.p = 0,parameters.robot_width/2, parameters.robot_ankle_to_foot;
     //left_foot.R    =   Ryaw(10*pi/180)*Rpitch(6*pi/180)*Rroll(5*pi/180);

        right_foot.p = 0, -parameters.robot_width/2, parameters.robot_ankle_to_foot;
    // right_foot.R    =   Ryaw(-10*pi/180)*Rpitch(6*pi/180)*Rroll(5*pi/180);

        body.p = 0,0,parameters.z_com_ctm;

        calculate_walking_pattern_ankle();

        stand[0] = joint_angles(0)*180/pi;
        stand[1] = joint_angles(1)*180/pi;
        stand[2] = joint_angles(2)*180/pi;
        stand[3] = joint_angles(3)*180/pi;
        stand[4] = joint_angles(4)*180/pi;
        stand[5] = joint_angles(5)*180/pi;

        stand[6] = joint_angles(6)*180/pi;
        stand[7] = joint_angles(7)*180/pi;
        stand[8] = joint_angles(8)*180/pi;
        stand[9] = joint_angles(9)*180/pi;
        stand[10] = joint_angles(10)*180/pi;
        stand[11] = joint_angles(11)*180/pi;
    // stand[0] = 0;
    // stand[1] = 22.5;
    // stand[2] = -31.95;
    // stand[3] = 63.79;
    // stand[4] = -31.95;
    // stand[5] = -22.56;

    // stand[6] = 0;
    // stand[7] = 20;
    // stand[8] = -18.15;
    // stand[9] = 36;
    // stand[10] = -18.15;
    // stand[11] = -20;

    // stand[6]    = 4;
    // stand[7]    = 5;
    // stand[8]    = 6;
    // stand[9]    = 0;
    // stand[10]   = 0;
    // stand[11]   = 0;

    // stand[0] = 4;
    // stand[1] = 5;
    // stand[2] = 6;
    // stand[3] = 0;
    // stand[4] = 0;
    // // stand[5] = 0;
    // std::cout << stand[0] << ","  << stand[1] << "," << stand[2] << "," << stand[3] << "," << stand[4] << "," << stand[5] << ","
    //              << stand[6] << "," << stand[7] << "," << stand[8] << "," << stand[9] << "," << stand[10] << "," << stand[11] << std::endl;

        for(int i=0;i<nNum;i++)
        {  
            if(stand[i] - dxl_actuator[i].get_present_theta() > 0.5)
            {
                stand_zero[i] += 15.0/200;
                dxl_actuator[i].set_goal_theta(stand_zero[i]);    
            }
            else if(stand[i] - dxl_actuator[i].get_present_theta() < -0.5)
            {
                stand_zero[i] -= 15.0/200;
                dxl_actuator[i].set_goal_theta(stand_zero[i]); 
            }
            else
            {
                dxl_actuator[i].set_goal_theta(stand[i]);    
            }
        } 

        // return 0;
    // q_swing =   dxl_actuator[6].get_present_theta(),
    //            dxl_actuator[7].get_present_theta()/parameters.Tipping_Scale,
    //                    dxl_actuator[8].get_present_theta(),
    //                    dxl_actuator[9].get_present_theta(),
    //                    dxl_actuator[10].get_present_theta(),
    //                    dxl_actuator[11].get_present_theta();

    // std::cout << dxl_actuator[6].get_present_theta() << "," << dxl_actuator[7].get_present_theta() << "," << dxl_actuator[8].get_present_theta() << "," 
    // << dxl_actuator[9].get_present_theta() << "," << dxl_actuator[10].get_present_theta() << "," << dxl_actuator[11].get_present_theta() << std::endl;

    // ForwardKinematic(0, q_swing, &swing_foot_state_fk);
    //  q_swing   = dxl_actuator[0].get_present_theta(),
    //                   dxl_actuator[1].get_present_theta()/parameters.Tipping_Scale,
    //                   dxl_actuator[2].get_present_theta(),
    //                   dxl_actuator[3].get_present_theta(),
    //                   dxl_actuator[4].get_present_theta(),
    //                   dxl_actuator[5].get_present_theta();

    // ForwardKinematic(1, q_swing, &swing_foot_state_fk); 
    // std::cout << "phi:" << swing_foot_state_fk(3)*180/pi<< " theta:" << swing_foot_state_fk(4)*180/pi<< " psi:" << swing_foot_state_fk(5)*180/pi<<std::endl; 
    }

    int Robot::get_time_step(){return time_step;}

    int Robot::is_touching_ground(){
        if(leftIsSupportLeg){
            std::cout << "fz:" << m_gati_data[1].Fz << std::endl;
            return m_gati_data[1].Fz < -20;
        }
        else{
            std::cout << "fz:" << m_gati_data[0].Fz << std::endl;
            return m_gati_data[0].Fz < -20;
        }
    }


// Updates the ati data and saves it. 
    void  Robot::update_ati(gAti_Data *_ati){
        for(int i=0;i<2;i++)
        {
            m_gati_data[i].Fx = _ati[i].Fx;
            m_gati_data[i].Fy = -_ati[i].Fy;

        // If the value is too small, the ZMP tends to get calculated to large. Therefore if the value Fz is smaller then 1e-4, it will be set to 1e-2 to prevent the ZMP calculated being too big.
            if( fabs(_ati[i].Fz) < 1e-4 ){
                m_gati_data[i].Fz = 1e-2;
                if (parameters.debug_msg == 1){
                    std::cout << "The absolute value is smaller than 1e-4" << std::endl;
                }
            }
            else{
                m_gati_data[i].Fz = _ati[i].Fz;
            }

            m_gati_data[i].Tx = -_ati[i].Tx;
            m_gati_data[i].Ty = _ati[i].Ty;
            m_gati_data[i].Tz = _ati[i].Tz;
        }
    // Check for ati flaws
    // if ( (m_gati_data[0].Fz < -1000) || (m_gati_data[0].Fz > 50) ){
    //     left_ati_fault = 1;
    //     std::cout << "Left flaw: " << (m_gati_data[0].Fz < -1000) << ", " << (m_gati_data[0].Fz > 50) << std::endl;
    // }
    // else{
    //     left_ati_fault = 0;
    // }

    // if ( (m_gati_data[1].Fz < -1000)  || (m_gati_data[1].Fz > 50) ){
    //     right_ati_fault = 1;
    //     std::cout << "Right flaw: " << (m_gati_data[1].Fz < -1000) << ", " << (m_gati_data[1].Fz > 50) << std::endl;

    // }
    // else{
    //     right_ati_fault = 0;
    // }
    }

// Check if the ati is working. In P1MC.cpp the gait will stop if the ati is flawed
    int Robot::ati_not_working(){
        if (left_ati_fault){
            std::cout << "Left ati flawed" << std::endl;
            return 1;
        }
        else if(right_ati_fault){
            std::cout << "Right ati flawed" << std::endl;
            return 1;
        }
        else{
            return 0;
        }
    }


// Updates the MTI data
    void  Robot::update_mti(gMti_Data _mti){
        m_gmti_data.eulerangles.Roll = _mti.eulerangles.Roll;
        m_gmti_data.eulerangles.Pitch = _mti.eulerangles.Pitch;
        m_gmti_data.eulerangles.Yaw = _mti.eulerangles.Yaw;

        m_gmti_data.acceleration.accX = _mti.acceleration.accX;
        m_gmti_data.acceleration.accY = _mti.acceleration.accY;
        m_gmti_data.acceleration.accZ = _mti.acceleration.accZ;

        m_gmti_data.rateofturn.gyrX = _mti.rateofturn.gyrX;
        m_gmti_data.rateofturn.gyrY = _mti.rateofturn.gyrY;
        m_gmti_data.rateofturn.gyrZ = _mti.rateofturn.gyrZ;
    }


// Stop the gait and remain at the last joint angles
    void Robot::stop(dxl_Actuator *dxl_actuator, float gait_packet[]){
    // When key hit, stop at time_step

        dxl_actuator[0].set_goal_theta(joint_angles(0)*180/pi);           
        dxl_actuator[1].set_goal_theta(joint_angles(1)*180/pi*parameters.Tipping_Scale_left);           
        dxl_actuator[2].set_goal_theta(joint_angles(2)*180/pi);  
        dxl_actuator[3].set_goal_theta(joint_angles(3)*180/pi); 
        dxl_actuator[4].set_goal_theta(joint_angles(4)*180/pi);   
        dxl_actuator[5].set_goal_theta(joint_angles(5)*180/pi);  

        dxl_actuator[6].set_goal_theta(joint_angles(6)*180/pi);          
        dxl_actuator[7].set_goal_theta(joint_angles(7)*180/pi*parameters.Tipping_Scale_right);           
        dxl_actuator[8].set_goal_theta(joint_angles(8)*180/pi);  
        dxl_actuator[9].set_goal_theta(joint_angles(9)*180/pi); 
        dxl_actuator[10].set_goal_theta(joint_angles(10)*180/pi);    
        dxl_actuator[11].set_goal_theta(joint_angles(11)*180/pi);  

        push_into_gait_packet(gait_packet);
    }



// Control the pitch by having a very weak P controller.
    void Robot::controlPitch(){

    body_theta = (double) (m_gmti_data.eulerangles.Pitch*pi/180); // Information from ATI

    // If the inclination is too much, stop the gait
    if(body_theta < -10*pi/180 ){
        body_theta = -10*pi/180;
        std::cout << "body_theta exceeding -10. setting to -10 " << std::endl;
    }
    else if(body_theta > 10*pi/180){
        body_theta = 10*pi/180;
        std::cout << "body_theta exceeding 10. setting to 10 " << std::endl;
    }

    // P controller. 1 degree per second correction (200Hz)
    double incre = (double) (body_theta/200);
    body_theta_fk = body_theta_fk - incre; // body_theta_fk is the pitch angle of the COM of the robot for the IK.
}

// Control the roll by having a very weak P controller. Same a controlPitch()
void Robot::controlRoll(){

    body_phi = (double) (m_gmti_data.eulerangles.Roll*pi/180);

    if(body_phi < -10*pi/180 ){
        body_phi = -10*pi/180;
        std::cout << "body_theta exceeding -10. setting to -10 " << std::endl;

    }
    else if(body_phi > 10*pi/180){
        body_phi = 10*pi/180;
        std::cout << "body_theta exceeding 10. setting to 10 " << std::endl;
    }

    // P controller. 1 degree per second correction (200Hz)
    double incre = (double) (body_phi/200);
    body_phi_fk = body_phi_fk - incre;       
}



// Use MPC to determine the COM position (for IK)
void Robot::calc_next_COM_position(){

    // Calculate the CoM trajectory for x
    calculate_X_ref_integrated(ZMP_x, ref_x_zmp_trajectory, parameters.use_feedback_x, &X_ref, dlib::rowm(X_est, dlib::range(0,2)), &u_x, &x_mpc);
    calculate_X_ref_integrated(ZMP_y, ref_y_zmp_trajectory, parameters.use_feedback_y, &Y_ref, dlib::rowm(Y_est, dlib::range(0,2)), &u_y, &y_mpc);       

    dlib::matrix<double,3,1> state_vector = X_ref.back();
    x_ref.push_back(state_vector(0));
    // Calculate the CoM trajectory for y
    dlib::matrix<double,3,1> ystate_vector = Y_ref.back();
    y_ref.push_back(ystate_vector(0));

    // Calculate the CoM trajectory for z
    dlib::matrix<double, 3, 1> Z_ref_vector;
    Z_ref_vector = parameters.z_com_mbm, 0, 0;
    Z_ref.assign(1, Z_ref_vector);
    extract_pos_of_vector(Z_ref, &z_ref);
}

// Calculate support foot, swing foot and zmp trajectory for walking straight
void Robot::calc_trajectories_walk(){    
    // Calculate the support foot trajectory
    calc_p_ref_heel();
    ref_z_support_foot_trajectory.assign(ref_x_support_foot_trajectory.size(), parameters.robot_ankle_to_foot);

    // Calculate the reference zmp trajectory
    calc_p_ref_transition();

    // Calculate swing foot piece
    std::vector<double> p_ref_x_heel, p_ref_y_heel;
    calc_p_ref(&p_ref_x_heel, &p_ref_y_heel); // Calculating a trajectory for the swing leg as reference
    calculate_swing_foot_complete_trajectory_heel(p_ref_x_heel, p_ref_y_heel, 
        &ref_x_swing_foot_trajectory, &ref_y_swing_foot_trajectory, &ref_z_swing_foot_trajectory, &ref_theta_swing_foot_trajectory);
    z_ref_swing_foot_complete_unmodified = ref_z_swing_foot_trajectory;
    ref_psi_support_foot_trajectory.assign(ref_x_support_foot_trajectory.size(),0);
    ref_psi_swing_foot_trajectory = ref_psi_support_foot_trajectory;
    ref_psi_body = ref_psi_support_foot_trajectory;
    
    set_MPC_initial_position(ref_x_zmp_trajectory.at(0), ref_y_zmp_trajectory.at(0));
    print_ref_length();
};
int Robot::get_walking_state(){
    return this->walking_state;
}
void Robot::calc_trajectories_walk_side(int leftside, int leftFootStart, int step_amount, double step_width){    
    if(leftside == 1){
        walking_state = 3;
    } 
    else{
        walking_state = 4;
    }
  parameters.steps_real   = step_amount;
  parameters.t_sim        = parameters.steps_real * parameters.step_duration;
  parameters.N            = parameters.t_sim / parameters.Ts;

    // Calculate the support foot trajectory

  calc_p_ref_heel_side(leftside, leftFootStart,  step_amount,  step_width);

  ref_z_support_foot_trajectory.assign(ref_x_support_foot_trajectory.size(), parameters.robot_ankle_to_foot);

    // Calculate the reference zmp trajectory
  calc_p_ref_transition_side(leftside,  leftFootStart,  step_amount,  step_width);

    // Calculate swing foot piece
  std::vector<double> p_ref_x_heel, p_ref_y_heel;
    calc_p_ref_side(&p_ref_x_heel, &p_ref_y_heel, leftside,  leftFootStart,  step_amount, step_width); // Calculating a trajectory for the swing leg as reference
    calculate_swing_foot_complete_trajectory_heel_side(p_ref_x_heel, p_ref_y_heel, 
        &ref_x_swing_foot_trajectory, &ref_y_swing_foot_trajectory, &ref_z_swing_foot_trajectory, &ref_theta_swing_foot_trajectory,  leftside,leftFootStart, step_amount, step_width);
    z_ref_swing_foot_complete_unmodified = ref_z_swing_foot_trajectory;
    ref_psi_support_foot_trajectory.assign(ref_y_zmp_trajectory.size(),0);
    ref_psi_swing_foot_trajectory = ref_psi_support_foot_trajectory;
    ref_psi_body = ref_psi_support_foot_trajectory;
    set_MPC_initial_position(ref_x_zmp_trajectory.at(0), ref_y_zmp_trajectory.at(0));
    print_ref_length();
};


void Robot::calc_trajectories_walk_circling(int clockwise, double radius, double step_width, double psi_all)
{
    if(clockwise == 1){
        walking_state = 6;
    }
    else{
        walking_state = 7;
    }
    double psi_max = step_width / radius;
    int step_amount_circling = ceil(psi_all / psi_max);
    double step_psi = psi_all / step_amount_circling;
    int step_amount = step_amount_circling * 2 + 1; //2 steps moves one.
    parameters.steps_real = step_amount;    // Send the data to the Obj.Robot.steps_real
    parameters.t_sim               = parameters.steps_real*parameters.step_duration;            // Time for the whole gait
    parameters.N                   = parameters.t_sim/parameters.Ts;  

    calc_p_ref_heel_circling(clockwise, radius, step_amount, step_psi);

    ref_z_support_foot_trajectory.assign(ref_x_support_foot_trajectory.size(), parameters.robot_ankle_to_foot);
    
  
    // Calculate the reference zmp trajectory & the Yaw of Body.
    calc_p_ref_transition_circling(clockwise, radius, step_amount, step_psi);


    std::vector<double> p_ref_x_heel, p_ref_y_heel;
    calc_p_ref_circling(&p_ref_x_heel, &p_ref_y_heel, clockwise, radius, step_amount, step_psi);
    calculate_swing_foot_complete_trajectory_heel_circling(p_ref_x_heel, p_ref_y_heel, 
        &ref_x_swing_foot_trajectory, &ref_y_swing_foot_trajectory, &ref_z_swing_foot_trajectory, 
        &ref_theta_swing_foot_trajectory, &ref_psi_swing_foot_trajectory, clockwise,step_amount,step_psi);
    // calculate_yaw_for_body_orthogonal(&ref_psi_body);
    set_MPC_initial_position(ref_x_zmp_trajectory.at(0), ref_y_zmp_trajectory.at(0));

    std::cout << "P_ref_x_heel"     << p_ref_y_heel.size()                      << std::endl;
    std::cout << "Pref length: "    << ref_y_zmp_trajectory.size()              << std::endl;
    std::cout << "Swing length: "   << ref_y_swing_foot_trajectory.size()       << std::endl;
    std::cout << "Body Psi Len"     << ref_psi_body.size()                      << std::endl;
    std::cout << "Sup length: "     << ref_y_support_foot_trajectory.size()     << std::endl;
    std::cout << "Sup Psi length: " << ref_psi_support_foot_trajectory.size()   << std::endl;
    std::cout << "Swing Psi leng:"  << ref_psi_swing_foot_trajectory.size()     << std::endl;
}

// Calculate support foot, swing foot and zmp trajectory for walking straight for the given setup (backwards, left right, etc)
void Robot::calc_trajectories_walk_straight(int forward, int leftFootStart, int step_amount, double step_length){    
    // Calculate the support foot trajectory
    if(forward == 1){
        walking_state = 1;
    } 
    else{
        walking_state = 2;
    }
    parameters.steps_real = step_amount;
    parameters.t_sim      = parameters.steps_real*parameters.step_duration;            // Time for the whole gait
    parameters.N          = parameters.t_sim/parameters.Ts;                            // Amount of samples

    calc_p_ref_heel_straight(forward, leftFootStart,  step_amount,  step_length);

    ref_z_support_foot_trajectory.assign(ref_x_support_foot_trajectory.size(), parameters.robot_ankle_to_foot);

    // Calculate the reference zmp trajectory
    calc_p_ref_transition_straight(forward,  leftFootStart,  step_amount,  step_length);

    // Calculate swing foot piece
    std::vector<double> p_ref_x_heel, p_ref_y_heel;
    calc_p_ref_straight(&p_ref_x_heel, &p_ref_y_heel, forward,  leftFootStart,  step_amount, step_length); // Calculating a trajectory for the swing leg as reference
    calculate_swing_foot_complete_trajectory_heel_straight(p_ref_x_heel, p_ref_y_heel, 
        &ref_x_swing_foot_trajectory, &ref_y_swing_foot_trajectory, &ref_z_swing_foot_trajectory, &ref_theta_swing_foot_trajectory,  forward,leftFootStart, step_amount, step_length);
    z_ref_swing_foot_complete_unmodified = ref_z_swing_foot_trajectory;
    ref_psi_support_foot_trajectory.assign(ref_x_support_foot_trajectory.size(),0);
    ref_psi_swing_foot_trajectory = ref_psi_support_foot_trajectory;
    ref_psi_body = ref_psi_support_foot_trajectory;
    set_MPC_initial_position(ref_x_zmp_trajectory.at(0), ref_y_zmp_trajectory.at(0));   
    print_ref_length();
};

// Calculate support foot, swing foot and zmp trajectory for walking straight for the given setup (backwards, left right, etc)
void Robot::calc_trajectories_walk_curve(int forward, int leftFootStart, double x_goal, double y_goal, double psi_goal){
    if(forward == 1){
        walking_state = 5;
    }    
    left_foot.p = 0,parameters.robot_width/2, parameters.robot_ankle_to_foot;
     //left_foot.R    =   Ryaw(10*pi/180)*Rpitch(6*pi/180)*Rroll(5*pi/180);

    right_foot.p = 0, -parameters.robot_width/2, parameters.robot_ankle_to_foot;
    // right_foot.R    =   Ryaw(-10*pi/180)*Rpitch(6*pi/180)*Rroll(5*pi/180);

    body.p = 0,0,parameters.z_com_ctm;    

    int samples;
    double straight_dist = sqrt(pow(x_goal,2)+pow(y_goal,2));
    double estimatedSteps = straight_dist/parameters.step_length+10;

    samples = estimatedSteps*parameters.step_duration/parameters.Ts;
    std::vector<double> xSpline, ySpline;

    generate_orientated_spline(x_goal,y_goal,psi_goal,samples, &xSpline, &ySpline);
    // std::cout << "ok" <<std::endl;
    generate_foot_print_on_spline(xSpline, ySpline, &x_footprint_on_spline, &y_footprint_on_spline, &yaw_footprint_on_spline);
    // std::cout << "ok" <<std::endl;
    while(x_footprint_on_spline.size() > estimatedSteps){
        std::cout << "the estimation was bad" << std::endl;
        estimatedSteps = x_footprint_on_spline.size() + 10;
        samples = estimatedSteps*parameters.step_duration/parameters.Ts;
        generate_orientated_spline(x_goal,y_goal,psi_goal,samples, &xSpline, &ySpline);
        // generate_foot_print_on_spline(xSpline, ySpline, &x_footprint_on_spline, &y_footprint_on_spline, &yaw_footprint_on_spline);
    }


    parameters.steps_real = x_footprint_on_spline.size()+2;                 /// One step bugs @ 2017-07-11 also plus one more to make the feet parallel
    parameters.t_sim      = parameters.steps_real*parameters.step_duration;            // Time for the whole gait
    parameters.N          = parameters.t_sim/parameters.Ts;                            // Amount of samples
    


    // Calculate the support foot trajectory

    //std::cout << "calc:" << std::endl;
    calc_p_ref_heel_curve(forward, leftFootStart, x_footprint_on_spline, y_footprint_on_spline, yaw_footprint_on_spline);

    ref_z_support_foot_trajectory.assign(ref_x_support_foot_trajectory.size(), parameters.robot_ankle_to_foot);

    //// Calculate the reference zmp trajectory
    calc_p_ref_transition_curve(forward, leftFootStart, x_footprint_on_spline, y_footprint_on_spline, yaw_footprint_on_spline);
    //// Calculate swing foot piece

    calculate_swing_foot_complete_trajectory_heel_curve(x_footprint_on_spline, y_footprint_on_spline, yaw_footprint_on_spline,
        &ref_x_swing_foot_trajectory, &ref_y_swing_foot_trajectory, &ref_z_swing_foot_trajectory, &ref_theta_swing_foot_trajectory, &ref_psi_swing_foot_trajectory,
        forward, leftFootStart);
    z_ref_swing_foot_complete_unmodified = ref_z_swing_foot_trajectory;

    // connect_steps();

    calculate_yaw_for_body(yaw_footprint_on_spline, &ref_psi_body);
    print_ref_length();


    // std::cout << "Pref length: " << ref_y_zmp_trajectory.size() << std::endl;
    // std::cout << "Swing length: " << ref_y_swing_foot_trajectory.size() << std::endl;
    // std::cout << "Ref Psi body length: " << ref_psi_body.size() << std::endl;
    // std::cout << "Ref Psi support foot length: " << ref_psi_support_foot_trajectory.size() << std::endl;
    // std::cout << "Ref Psi swing foot length: " << ref_psi_swing_foot_trajectory.size() << std::endl;

    // std::cout << "ref_x_zmp_trajectory.at(0)" << ref_x_zmp_trajectory.at(0) << "," << ref_y_zmp_trajectory.at(0) << std::endl;
    set_MPC_initial_position(ref_x_zmp_trajectory.at(0), ref_y_zmp_trajectory.at(0));
    print_ref_length();
};

    /* 2017-06-16
    void Robot::stopGait(){
        double x_goal_left, y_goal_left, z_goal_left, psi_goal_left, x_goal_right, y_goal_right, z_goal_right, psi_goal_right, psi_body_start, psi_body_end;
        double pointsTillDsp;
        int step_duration = (parameters.step_duration/parameters.Ts);
        double x_support_new, y_support_new, z_support_new, psi_support_new;
        double x_swing_new, y_swing_new, z_swing_new, psi_swing_new, theta_swing_new;
        double x_swing_end, y_swing_end, z_swing_end, psi_swing_end, theta_swing_end;
      
        //PointsTillDSP Means till the end.
        pointsTillDsp = step_duration - time_step%step_duration;

        // y_swing_new = ref_y_support_foot_trajectory.at(time_step+pointsTillDsp+1);

        // Erase all points after DSP
        // for (int i = 0; i < ref_y_swing_foot_trajectory.size(); i++){
        //     std::cout << "i: " << i << ", " << ref_y_support_foot_trajectory.at(i) << std::endl;
        // }

        ref_x_support_foot_trajectory.erase(ref_x_support_foot_trajectory.begin()+time_step+pointsTillDsp, ref_x_support_foot_trajectory.end());
        ref_y_support_foot_trajectory.erase(ref_y_support_foot_trajectory.begin()+time_step+pointsTillDsp, ref_y_support_foot_trajectory.end());
        ref_z_support_foot_trajectory.erase(ref_z_support_foot_trajectory.begin()+time_step+pointsTillDsp, ref_z_support_foot_trajectory.end());
        ref_psi_support_foot_trajectory.erase(ref_psi_support_foot_trajectory.begin()+time_step+pointsTillDsp, ref_psi_support_foot_trajectory.end());

        ref_x_swing_foot_trajectory.erase(ref_x_swing_foot_trajectory.begin()+time_step+pointsTillDsp, ref_x_swing_foot_trajectory.end());
        ref_y_swing_foot_trajectory.erase(ref_y_swing_foot_trajectory.begin()+time_step+pointsTillDsp, ref_y_swing_foot_trajectory.end());
        ref_z_swing_foot_trajectory.erase(ref_z_swing_foot_trajectory.begin()+time_step+pointsTillDsp, ref_z_swing_foot_trajectory.end());
        ref_theta_swing_foot_trajectory.erase(ref_theta_swing_foot_trajectory.begin()+time_step+pointsTillDsp, ref_theta_swing_foot_trajectory.end());
        ref_psi_swing_foot_trajectory.erase(ref_psi_swing_foot_trajectory.begin()+time_step+pointsTillDsp, ref_psi_swing_foot_trajectory.end());

        ref_x_zmp_trajectory.erase(ref_x_zmp_trajectory.begin()+time_step+pointsTillDsp, ref_x_zmp_trajectory.end());
        ref_y_zmp_trajectory.erase(ref_y_zmp_trajectory.begin()+time_step+pointsTillDsp, ref_y_zmp_trajectory.end());

        ref_psi_body.erase(ref_psi_body.begin()+time_step+pointsTillDsp, ref_psi_body.end());
        // x_footprint_on_spline.erase(x_footprint_on_spline.begin()+time_step+pointsTillDsp, x_footprint_on_spline.end());
        // y_footprint_on_spline.erase(y_footprint_on_spline.begin()+time_step+pointsTillDsp, y_footprint_on_spline.end());
        // yaw_footprint_on_spline.erase(yaw_footprint_on_spline.begin()+time_step+pointsTillDsp, yaw_footprint_on_spline.end());
        // x_footprint_on_circle.erase(x_footprint_on_circle.begin()+time_step+pointsTillDsp, x_footprint_on_circle.end()); 
        // y_footprint_on_circle.erase(y_footprint_on_circle.begin()+time_step+pointsTillDsp, y_footprint_on_circle.end()); 
        // yaw_footprint_on_circle.erase(yaw_footprint_on_circle.begin()+time_step+pointsTillDsp, yaw_footprint_on_circle.end());
        
        int leftIsSupporting;
        
            if( (ref_y_support_foot_trajectory.back()-ref_y_swing_foot_trajectory.back()) > 0){
                leftIsSupporting = 1;
            }
            else{
                leftIsSupporting = 0;
            }
        




        x_support_new = ref_x_swing_foot_trajectory.back();
        y_support_new = ref_y_swing_foot_trajectory.back();
        z_support_new = ref_z_swing_foot_trajectory.back();
        psi_support_new = ref_psi_swing_foot_trajectory.back();

        x_swing_new = ref_x_support_foot_trajectory.back();
        y_swing_new = ref_y_support_foot_trajectory.back();
        z_swing_new = ref_z_support_foot_trajectory.back();
        psi_swing_new = ref_psi_support_foot_trajectory.back();
        psi_swing_end = psi_support_new;
        theta_swing_new = 0;
        theta_swing_end = 0;

        psi_body_start = ref_psi_body.back();
        psi_body_end   = psi_support_new;

        calculate_yaw_for_body_stop( psi_body_start,psi_body_end, &ref_psi_body);

        if(leftIsSupporting == 1){
            x_swing_end = x_support_new - sin(psi_support_new)*parameters.robot_width;
            y_swing_end = y_support_new + cos(psi_support_new)*parameters.robot_width;
        }
        else{
            x_swing_end = x_support_new + sin(psi_support_new)*parameters.robot_width;
            y_swing_end = y_support_new - cos(psi_support_new)*parameters.robot_width;   
        }





        ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), 2*step_duration, x_support_new);
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), 2*step_duration, y_support_new);
        ref_z_support_foot_trajectory.insert(ref_z_support_foot_trajectory.end(), 2*step_duration, z_support_new);
        ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.end(), 2*step_duration, psi_support_new);

        ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, x_support_new+parameters.zmp_offset_x*sin(psi_support_new));

        if(leftIsSupporting == 1){
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, y_support_new-parameters.zmp_offset_y*cos(psi_support_new));
        }
        else{
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, y_support_new+parameters.zmp_offset_y*cos(psi_support_new));
        }

        ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, (x_swing_end+x_support_new)/2);
        ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, (y_swing_end+y_support_new)/2);

        double x_swing_foot_start = x_swing_new;
        double x_swing_foot_end   = x_swing_end;
        double y_swing_foot_start = y_swing_new;
        double y_swing_foot_end   = y_swing_end;
        double psi_start          = psi_swing_new;
        double psi_stop           = psi_swing_end;
        // std::cout << "y start: " << y_swing_foot_start << std::endl;
        // std::cout << "y stop: " << y_swing_foot_stop << std::endl;

        double swing_foot_x_peak = (x_swing_foot_end - x_swing_foot_start)/2;
        double swing_foot_z_peak = 0.06;
        int forward = 1;

        calculate_swing_foot_piece_trajectory_heel_curve(0, x_swing_foot_start, x_swing_foot_end, 
                                                        swing_foot_z_peak, swing_foot_x_peak, y_swing_foot_start, y_swing_foot_end, psi_start, psi_stop,
                                                        &ref_x_swing_foot_trajectory, &ref_y_swing_foot_trajectory, &ref_z_swing_foot_trajectory, 
                                                        &ref_theta_swing_foot_trajectory, &ref_psi_swing_foot_trajectory, forward);

        ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(),step_duration, ref_x_swing_foot_trajectory.back());
        ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(),step_duration, ref_y_swing_foot_trajectory.back());
        ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(),step_duration, ref_z_swing_foot_trajectory.back());
        ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(),step_duration, ref_theta_swing_foot_trajectory.back());
        ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(),step_duration, ref_psi_swing_foot_trajectory.back());

        parameters.N          = time_step + pointsTillDsp-1 + 2*step_duration; // + parameters.step_duration/parameters.Ts;                            // Amount of samples
        parameters.t_sim      = parameters.N*parameters.Ts;            // Time for the whole gait
    
        // std::cout << "time_step: " << time_step << ", pointsTillDsp: " << pointsTillDsp << ", N:" << parameters.N << std::endl;

        
        print_ref_length();

        // ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.back(), ref_x_sup.front()+time_step, ref_x_sup.front()+time_step+pointsTillDsp);
    }
    */
    void Robot::stopGait(){
        double x_goal_left, y_goal_left, z_goal_left, psi_goal_left, x_goal_right, y_goal_right, z_goal_right, psi_goal_right, psi_body_start, psi_body_end;
        double pointsTillDsp;
        int step_duration = (parameters.step_duration/parameters.Ts);
        double x_support_new, y_support_new, z_support_new, psi_support_new;
        double x_swing_new, y_swing_new, z_swing_new, psi_swing_new, theta_swing_new;
        double x_swing_end, y_swing_end, z_swing_end, psi_swing_end, theta_swing_end;
      
        //PointsTillDSP Means till the end.
        pointsTillDsp = step_duration - time_step%step_duration;

        // y_swing_new = ref_y_support_foot_trajectory.at(time_step+pointsTillDsp+1);

        // Erase all points after DSP
        // for (int i = 0; i < ref_y_swing_foot_trajectory.size(); i++){
        //     std::cout << "i: " << i << ", " << ref_y_support_foot_trajectory.at(i) << std::endl;
        // }

        ref_x_support_foot_trajectory.erase(ref_x_support_foot_trajectory.begin()+time_step+pointsTillDsp, ref_x_support_foot_trajectory.end());
        ref_y_support_foot_trajectory.erase(ref_y_support_foot_trajectory.begin()+time_step+pointsTillDsp, ref_y_support_foot_trajectory.end());
        ref_z_support_foot_trajectory.erase(ref_z_support_foot_trajectory.begin()+time_step+pointsTillDsp, ref_z_support_foot_trajectory.end());
        ref_psi_support_foot_trajectory.erase(ref_psi_support_foot_trajectory.begin()+time_step+pointsTillDsp, ref_psi_support_foot_trajectory.end());

        ref_x_swing_foot_trajectory.erase(ref_x_swing_foot_trajectory.begin()+time_step+pointsTillDsp, ref_x_swing_foot_trajectory.end());
        ref_y_swing_foot_trajectory.erase(ref_y_swing_foot_trajectory.begin()+time_step+pointsTillDsp, ref_y_swing_foot_trajectory.end());
        ref_z_swing_foot_trajectory.erase(ref_z_swing_foot_trajectory.begin()+time_step+pointsTillDsp, ref_z_swing_foot_trajectory.end());
        ref_theta_swing_foot_trajectory.erase(ref_theta_swing_foot_trajectory.begin()+time_step+pointsTillDsp, ref_theta_swing_foot_trajectory.end());
        ref_psi_swing_foot_trajectory.erase(ref_psi_swing_foot_trajectory.begin()+time_step+pointsTillDsp, ref_psi_swing_foot_trajectory.end());

        ref_x_zmp_trajectory.erase(ref_x_zmp_trajectory.begin()+time_step+pointsTillDsp, ref_x_zmp_trajectory.end());
        ref_y_zmp_trajectory.erase(ref_y_zmp_trajectory.begin()+time_step+pointsTillDsp, ref_y_zmp_trajectory.end());

        ref_psi_body.erase(ref_psi_body.begin()+time_step+pointsTillDsp, ref_psi_body.end());
        // x_footprint_on_spline.erase(x_footprint_on_spline.begin()+time_step+pointsTillDsp, x_footprint_on_spline.end());
        // y_footprint_on_spline.erase(y_footprint_on_spline.begin()+time_step+pointsTillDsp, y_footprint_on_spline.end());
        // yaw_footprint_on_spline.erase(yaw_footprint_on_spline.begin()+time_step+pointsTillDsp, yaw_footprint_on_spline.end());
        // x_footprint_on_circle.erase(x_footprint_on_circle.begin()+time_step+pointsTillDsp, x_footprint_on_circle.end()); 
        // y_footprint_on_circle.erase(y_footprint_on_circle.begin()+time_step+pointsTillDsp, y_footprint_on_circle.end()); 
        // yaw_footprint_on_circle.erase(yaw_footprint_on_circle.begin()+time_step+pointsTillDsp, yaw_footprint_on_circle.end());
        
        int leftIsSupporting;
        
            if( (ref_y_support_foot_trajectory.back()-ref_y_swing_foot_trajectory.back()) > 0){
                leftIsSupporting = 1;
            }
            else{
                leftIsSupporting = 0;
            }
        




        x_support_new = ref_x_swing_foot_trajectory.back();
        y_support_new = ref_y_swing_foot_trajectory.back();
        z_support_new = ref_z_swing_foot_trajectory.back();
        psi_support_new = ref_psi_swing_foot_trajectory.back();

        x_swing_new = ref_x_support_foot_trajectory.back();
        y_swing_new = ref_y_support_foot_trajectory.back();
        z_swing_new = ref_z_support_foot_trajectory.back();
        psi_swing_new = ref_psi_support_foot_trajectory.back();
        psi_swing_end = psi_support_new;
        theta_swing_new = 0;
        theta_swing_end = 0;

        psi_body_start = ref_psi_body.back();
        psi_body_end   = psi_support_new;

        calculate_yaw_for_body_stop( psi_body_start,psi_body_end, &ref_psi_body);

        if(leftIsSupporting == 1){
            x_swing_end = x_support_new - sin(psi_support_new)*parameters.robot_width;
            y_swing_end = y_support_new + cos(psi_support_new)*parameters.robot_width;
        }
        else{            x_swing_end = x_support_new + sin(psi_support_new)*parameters.robot_width;
            y_swing_end = y_support_new - cos(psi_support_new)*parameters.robot_width;   
        }

        int step_points = step_duration*parameters.dsp_percentage/100;


        ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), 0.2*step_duration, ref_x_support_foot_trajectory.back());
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), 0.2*step_duration, ref_y_support_foot_trajectory.back());
        ref_z_support_foot_trajectory.insert(ref_z_support_foot_trajectory.end(), 0.2*step_duration, ref_z_support_foot_trajectory.back());
        ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.end(), 0.2*step_duration, ref_psi_support_foot_trajectory.back());
        


        ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), 1.8*step_duration, x_support_new);
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), 1.8*step_duration, y_support_new);
        ref_z_support_foot_trajectory.insert(ref_z_support_foot_trajectory.end(), 1.8*step_duration, z_support_new);
        ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.end(), 1.8*step_duration, psi_support_new);
        
        double x_zmp_increment, y_zmp_increment;

        double x_zmp_start = x_support_new+parameters.zmp_offset_x*sin(psi_support_new);
        double y_zmp_start;

        if(leftIsSupportLeg == 1){
            y_zmp_start = y_support_new-parameters.zmp_offset_y*cos(psi_support_new);
        }
        else{
            y_zmp_start = y_support_new+parameters.zmp_offset_y*cos(psi_support_new);
        }

        x_zmp_increment = (ref_x_zmp_trajectory.back()-x_zmp_start)/step_points;
        y_zmp_increment = (ref_y_zmp_trajectory.back()-y_zmp_start)/step_points;
        
        for(int i = 0; i < step_points; i++){
            ref_x_zmp_trajectory.push_back(ref_x_zmp_trajectory.back()-x_zmp_increment);
            ref_y_zmp_trajectory.push_back(ref_y_zmp_trajectory.back()-y_zmp_increment);
        }

     
        ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration*(1-parameters.dsp_percentage/100), x_zmp_start);
        ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration*(1-parameters.dsp_percentage/100), y_zmp_start);


        x_zmp_increment = ((x_swing_end+x_support_new)/2-x_zmp_start)/step_points;
        y_zmp_increment = ((y_swing_end+y_support_new)/2-y_zmp_start)/step_points;
        
        for(int i = 0; i < step_points; i++){
            ref_x_zmp_trajectory.push_back(ref_x_zmp_trajectory.back()+x_zmp_increment);
            ref_y_zmp_trajectory.push_back(ref_y_zmp_trajectory.back()+y_zmp_increment);
        }


        ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration*(1-parameters.dsp_percentage/100), (x_swing_end+x_support_new)/2);
        ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration*(1-parameters.dsp_percentage/100), (y_swing_end+y_support_new)/2);

        double x_swing_foot_start = x_swing_new;
        double x_swing_foot_end   = x_swing_end;
        double y_swing_foot_start = y_swing_new;
        double y_swing_foot_end   = y_swing_end;
        double psi_start          = psi_swing_new;
        double psi_stop           = psi_swing_end;
        // std::cout << "y start: " << y_swing_foot_start << std::endl;
        // std::cout << "y stop: " << y_swing_foot_stop << std::endl;

        double swing_foot_x_peak = (x_swing_foot_end - x_swing_foot_start)/2;
        double swing_foot_z_peak = 0.06;
        int forward = 1;
        
        ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(),step_duration*(parameters.dsp_percentage/100), ref_x_swing_foot_trajectory.back());
        ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(),step_duration*(parameters.dsp_percentage/100), ref_y_swing_foot_trajectory.back());
        ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(),step_duration*(parameters.dsp_percentage/100), ref_z_swing_foot_trajectory.back());
        ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(),step_duration*(parameters.dsp_percentage/100), ref_theta_swing_foot_trajectory.back());
        ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(),step_duration*(parameters.dsp_percentage/100), ref_psi_swing_foot_trajectory.back());

        calculate_swing_foot_piece_trajectory_heel_curve(0, x_swing_foot_start, x_swing_foot_end, 
                                                        swing_foot_z_peak, swing_foot_x_peak, y_swing_foot_start, y_swing_foot_end, psi_start, psi_stop,
                                                        &ref_x_swing_foot_trajectory, &ref_y_swing_foot_trajectory, &ref_z_swing_foot_trajectory, 
                                                        &ref_theta_swing_foot_trajectory, &ref_psi_swing_foot_trajectory, forward);

        ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(),step_duration*(1-parameters.dsp_percentage/100), ref_x_swing_foot_trajectory.back());
        ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(),step_duration*(1-parameters.dsp_percentage/100), ref_y_swing_foot_trajectory.back());
        ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(),step_duration*(1-parameters.dsp_percentage/100), ref_z_swing_foot_trajectory.back());
        ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(),step_duration*(1-parameters.dsp_percentage/100), ref_theta_swing_foot_trajectory.back());
        ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(),step_duration*(1-parameters.dsp_percentage/100), ref_psi_swing_foot_trajectory.back());

        parameters.N          = time_step + pointsTillDsp-1 + 2*step_duration; // + parameters.step_duration/parameters.Ts;                            // Amount of samples
        parameters.t_sim      = parameters.N*parameters.Ts;            // Time for the whole gait
    
        // std::cout << "time_step: " << time_step << ", pointsTillDsp: " << pointsTillDsp << ", N:" << parameters.N << std::endl;

        
        print_ref_length();

        // ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.back(), ref_x_sup.front()+time_step, ref_x_sup.front()+time_step+pointsTillDsp);

    }
// Calculate support foot, swing foot and zmp trajectory for standing on a single leg
void Robot::calc_trajectories_stand_ssp(){
    // Calculate the support foot trajectory
    calc_p_ref_only_ssp();
    ref_z_support_foot_trajectory.assign(ref_x_support_foot_trajectory.size(), parameters.robot_ankle_to_foot);

    // Calculate the reference zmp trajectory
    calc_p_ref_transition_only_ssp();

    // Calculate swing foot piece
    std::vector<double> p_ref_x_ssp, p_ref_y_ssp;
    calc_p_ref(&p_ref_x_ssp, &p_ref_y_ssp);
    calculate_swing_foot_complete_trajectory_only_ssp(p_ref_x_ssp, p_ref_y_ssp,
        &ref_x_swing_foot_trajectory, &ref_y_swing_foot_trajectory, &ref_z_swing_foot_trajectory);
    z_ref_swing_foot_complete_unmodified = ref_z_swing_foot_trajectory;
    set_MPC_initial_position(ref_x_zmp_trajectory.at(0), ref_y_zmp_trajectory.at(0));
};

// Calculate support foot, swing foot and zmp trajectory for standing on a single leg and moving the COM in a sine wave
void Robot::calc_trajectories_stand_ssp_sine(){
    // Calculate the support foot trajectory
    calc_p_ref_only_ssp();
    ref_z_support_foot_trajectory.assign(ref_x_support_foot_trajectory.size(), parameters.robot_ankle_to_foot);

    // Calculate the reference zmp trajectory
    calc_p_ref_transition_only_ssp_sine();
    // Calculate swing foot piece
    std::vector<double> p_ref_x_ssp, p_ref_y_ssp;
    calc_p_ref(&p_ref_x_ssp, &p_ref_y_ssp);

    calculate_swing_foot_complete_trajectory_only_ssp(p_ref_x_ssp, p_ref_y_ssp,
        &ref_x_swing_foot_trajectory, &ref_y_swing_foot_trajectory, &ref_z_swing_foot_trajectory);
    z_ref_swing_foot_complete_unmodified = ref_z_swing_foot_trajectory;
    set_MPC_initial_position(ref_x_zmp_trajectory.at(0), ref_y_zmp_trajectory.at(0));
};

// Calculate support foot, swing foot and zmp trajectory for standing on both legs and moving the COM in a sine wave
void Robot::calc_trajectories_stand_dsp_sine(){
    // Calculate the support foot trajectory
    calc_p_ref_only_ssp();
    ref_z_support_foot_trajectory.assign(ref_x_support_foot_trajectory.size(), parameters.robot_ankle_to_foot);

    // Calculate the reference zmp trajectory
    calc_p_ref_transition_only_dsp_sine();
    // Calculate swing foot piece
    std::vector<double> p_ref_x_ssp, p_ref_y_ssp;
    calc_p_ref(&p_ref_x_ssp, &p_ref_y_ssp);

    calculate_swing_foot_complete_trajectory_only_dsp(p_ref_x_ssp, p_ref_y_ssp,
        &ref_x_swing_foot_trajectory, &ref_y_swing_foot_trajectory, &ref_z_swing_foot_trajectory);
    z_ref_swing_foot_complete_unmodified = ref_z_swing_foot_trajectory;
    set_MPC_initial_position(ref_x_zmp_trajectory.at(0), ref_y_zmp_trajectory.at(0));

};

// To be implemented
void Robot::calc_trajectories_ssp_squat(){};

// Initialize the Kalman Filter with the given parameters 
void Robot::initKalman(){
    double Q_x;
    Q_x = parameters.Q_x; //8
    dlib::matrix<double> R_x;
    R_x = dlib::identity_matrix<double>(2);
    R_x =   parameters.R_pos_x,0, //5
        0,  parameters.R_cop_x; //4
        kfx.initKalmanFilter();
        kfx.setQ(Q_x);
        kfx.setR(R_x);

        double Q_y;
        Q_y = parameters.Q_y;
        dlib::matrix<double> R_y;
        R_y = dlib::identity_matrix<double>(2);
        R_y =   parameters.R_pos_y,0,
        0,  parameters.R_cop_y;
        kfy.initKalmanFilter();
        kfy.setQ(Q_y);
        kfy.setR(R_y);
    }

// Estimate the state of the COM via Kalman Filter
    void Robot::estimateState(){

        dlib::matrix<double,2,1> x_measurement;
        x_measurement = COM_state_fk(0), ZMP_x;
        kfx.estimateState(u_x.back(), x_measurement, &X_est, &x_output_estimation);         

        dlib::matrix<double,2,1> y_measurement;
        y_measurement = COM_state_fk(1), ZMP_y;
        kfy.estimateState(u_y.back(), y_measurement, &Y_est, &y_output_estimation);         
    }

 void Robot::calculate_yaw_for_body_stop(double yaw_start, double yaw_end, std::vector<double> * ref_psi_body){
        // ref_psi_body->push_back(yaw_footprint_on_spline.front());
        int amount_of_points = 3;
        std::vector<double> t_points(3), psi_points(3);
        // std::cout << "t_sim: " << parameters.t_sim << std::endl;
        // std::cout << "amount: " << amount_of_points << std::endl;

        t_points[0] = 0;
        t_points[1] = 1*parameters.step_duration;
        t_points[2] = 2*parameters.step_duration;

        psi_points[0] = yaw_start;
        psi_points[1] = (yaw_end+yaw_start)/2;
        psi_points[2] = yaw_end;

        tk::spline yaw_ref;
        yaw_ref.set_points(t_points,psi_points);

        for( int i = 0; i < (parameters.step_duration*2)/parameters.Ts; i++){
            ref_psi_body->push_back(yaw_ref(parameters.Ts*i));
        }

    }

    void Robot::calculate_yaw_for_body(std::vector<double> yaw_footprint_on_spline, std::vector<double> * ref_psi_body){
        // ref_psi_body->push_back(yaw_footprint_on_spline.front());
        int amount_of_points = yaw_footprint_on_spline.size()-1;
        std::vector<double> t_points(amount_of_points);
        // std::cout << "t_sim: " << parameters.t_sim << std::endl;
        // std::cout << "amount: " << amount_of_points << std::endl;
        std::vector<double> yaw_tmp;

        for (int i = 0; i < amount_of_points; i++){
            t_points[i] = (parameters.t_sim)/amount_of_points*i;
            // std::cout << "t points: " << t_points[i] << std::endl;
            yaw_tmp.push_back(yaw_footprint_on_spline.at(i));
        }

        tk::spline yaw_ref;
        yaw_ref.set_points(t_points,yaw_tmp);

        for( int i = 0; i < ref_psi_support_foot_trajectory.size(); i++){
            ref_psi_body->push_back(yaw_ref(parameters.Ts*i));
        }

    }
    void Robot::calculate_yaw_for_body_orthogonal(std::vector<double> * ref_psi_body){


    // ref_psi_body->push_back(yaw_footprint_on_spline.front());
        int amount_of_points = 2;
        std::vector<double> t_points(amount_of_points);
        std::cout << "t_sim: " << parameters.t_sim << std::endl;
        std::cout << "amount: " << amount_of_points << std::endl;
        std::vector<double> yaw_tmp;
        // yaw_tmp.push_back(0);
        t_points[0] = 0;
        t_points[1] = parameters.t_sim;
        // for (int i = 0; i < amount_of_points; i++){
            // t_points[i] = (parameters.t_sim)/amount_of_points*i;
            // if(i != 0){

                // yaw_tmp.push_back(ref_psi_body->at(i));
            // }
        // std::cout << "t points: " << t_points[i] << std::endl;
        // }
        yaw_tmp.push_back(ref_psi_body->front());
        yaw_tmp.push_back(ref_psi_body->back());
        // for(int i = 0; i<amount_of_points;i++){
        // }

        tk::spline yaw_ref;
        yaw_ref.set_points(t_points,yaw_tmp);

        // for(int i = 0; i<yaw_tmp.size(); i++){
            // std::cout << yaw_tmp.at(i) << std::endl;;
            // std::cout << t_points[i] << std::endl;;
        // }
        ref_psi_body->erase(ref_psi_body->begin(), ref_psi_body->end());


        for( int i = 0; i < ref_psi_support_foot_trajectory.size(); i++){
            ref_psi_body->push_back(yaw_ref(parameters.Ts*i));
        }

        FILE *fp;
        if( ( fp = fopen( "./data.txt", "w" ) ) == NULL )
        {
            printf( "error opening data file ./data.txt\n" );
        }

        for(int i; i < ref_psi_body->size(); i++){
            // std::cout << "ok" << std::endl;
            fprintf(fp, "%f\n", ref_psi_body->at(i));
                // printf("%f, %f\n", xCircle.at(i), yCircle.at(i));

        }

    }
    // void Robot::calculate_yaw_for_body_orthogonal(std::vector<double> xCircle, std::vector<double> yCircle, std::vector<double> * ref_psi_body){
    //     ref_psi_body->push_back(0);
    // // std::cout << "ping" << std::endl;

    //     double step_duration = parameters.step_duration/parameters.Ts;

    //     ref_psi_body->insert(ref_psi_body->end(), step_duration*2, pi/2);
    //     for(int i = 1; i < xCircle.size(); i++){    
    //         ref_psi_body->push_back(atan((yCircle.at(i)-yCircle.at(i-1))/(xCircle.at(i)-xCircle.at(i-1)))+pi/2);
    //     }

    //     // std::cout << "problemhere. length different. first orientation needs to be switched" << std::endl;
    // // std::cout << "pong" << std::endl;
    // }

    void Robot::calc_p_ref_heel(){
        int step_amount = parameters.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < parameters.steps_real; i++){
        s[i] = i*parameters.step_length;
    }

    double step_duration = ceil(parameters.N/parameters.steps_real);
    // Every step in the lateral direction is made of this piece
    // Calculating the time that is needed for one step

    //First step
    ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), ceil(step_duration+step_duration*parameters.dsp_percentage/100), s[0]);
    ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), ceil(step_duration+step_duration*parameters.dsp_percentage/100), -parameters.robot_width/2);

    // Middle steps
    for (int i = 1; i < parameters.steps_real-1; i++){
        ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, s[i]);
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, pow(-1,i+1)*(parameters.robot_width/2));
    }
    //Last step
    ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), s[step_amount-1]);
    ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), pow(-1, parameters.steps_real)*parameters.robot_width/2);
}
void Robot::calc_p_ref_heel_side(int leftside, int leftFootStart, int step_amount/*Useless*/, double step_width){
    step_amount = parameters.steps_real;  // Special
    double sy[step_amount]; 
    double sx[step_amount];
    for (int i = 0; i < step_amount; i++){  //bug fixed.
        sy[i] = (i - floor(i / 2))*step_width;
        sx[i] = 0;
    }
    
    int  invert_y;
    
    if(leftFootStart == 0){
        invert_y = 1;
    }
    else{
        invert_y = -1;
    }
    
    double step_duration = parameters.step_duration/parameters.Ts;
    // Every step in the lateral direction is made of this piece
    //First step
    ref_x_support_foot_trajectory.insert(
        ref_x_support_foot_trajectory.end(), ceil(step_duration+step_duration*parameters.dsp_percentage/100), sx[0]);
    ref_y_support_foot_trajectory.insert(
        ref_y_support_foot_trajectory.end(), ceil(step_duration+step_duration*parameters.dsp_percentage/100), sy[0] + invert_y*(-parameters.robot_width/2));
    // Middle steps
    for (int i = 1; i < parameters.steps_real-1; i++){
        ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, sx[i]);
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, sy[i] + invert_y*pow(-1,i+1)*(parameters.robot_width/2));
    }
    //Last step
    //Have some trick!
    ref_x_support_foot_trajectory.insert(
        ref_x_support_foot_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), sx[step_amount - 1]);
    ref_y_support_foot_trajectory.insert(
        ref_y_support_foot_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), sy[step_amount - 1] + invert_y*pow(-1,step_amount)*(parameters.robot_width/2));
}

void Robot::calc_p_ref_heel_circling(int clockwise, double radius, int step_amount, double step_psi)
{
    //Sx, Sy, So means the Virtual CoM's coordinate position
    double sy[step_amount];
    double sx[step_amount];
    double so[step_amount];

    

    int  invert_y;
    
    if(clockwise == 0){
        invert_y = -1;
    }
    else{
        invert_y = 1;
    }

    for (int i = 0; i < step_amount; i++){  //bug fixed.
        so[i] =  (i - floor(i / 2))*step_psi;       //Absolut Angle
        sy[i] =  invert_y * radius * sin(so[i]);
        sx[i] =  (-1) * radius * (cos(so[i]) -1);
        printf("%f %f %f\n",so[i],sx[i],sy[i]);
    }    
    double step_duration = parameters.step_duration/parameters.Ts;
    // Every step in the lateral direction is made of this piece
    //First step
    ref_x_support_foot_trajectory.insert(
        ref_x_support_foot_trajectory.end(), ceil(step_duration+step_duration*parameters.dsp_percentage/100), sx[0]);
    ref_y_support_foot_trajectory.insert(
        ref_y_support_foot_trajectory.end(), ceil(step_duration+step_duration*parameters.dsp_percentage/100), sy[0] + invert_y*(-parameters.robot_width/2));
    ref_psi_support_foot_trajectory.insert(
        ref_psi_support_foot_trajectory.end(),ceil(step_duration+step_duration*parameters.dsp_percentage/100), -invert_y*so[0]);

    // Middle steps
    for (int i = 1; i < step_amount - 1; i++){
        ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, sx[i] + (-1) * pow(-1,i) * sin(so[i]) * ( parameters.robot_width/2));
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, sy[i] + invert_y * pow(-1,i) * cos(so[i]) * (-parameters.robot_width/2));
        ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.end(),step_duration, -invert_y*so[i]);
    }    
    
    ref_x_support_foot_trajectory.insert(
        ref_x_support_foot_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), sx[step_amount - 1] + (-1) * sin(so[step_amount-1]) * ( parameters.robot_width/2));
    ref_y_support_foot_trajectory.insert(
        ref_y_support_foot_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), sy[step_amount - 1] + invert_y * cos(so[step_amount-1]) * (-parameters.robot_width/2));
    ref_psi_support_foot_trajectory.insert(
        ref_psi_support_foot_trajectory.end(),floor(step_duration*(1-parameters.dsp_percentage/100)), -invert_y*so[step_amount - 1]);
}


void Robot::calc_p_ref_heel_straight(int forward, int leftFootStart, int step_amount, double step_length){
    step_amount = parameters.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually

    for (int i = 0; i < step_amount+1; i++){
        s[i] = i*step_length;
    }

    int invert_x, invert_y;


    if(forward == 0){
        invert_x = -1;
    }
    else{
        invert_x = 1;
    }
    if(leftFootStart == 0){
        invert_y = 1;
    }
    else{
        invert_y = -1;
    }

    double step_duration = parameters.step_duration/parameters.Ts;
    // Every step in the lateral direction is made of this piece


    //First step
    ref_x_support_foot_trajectory.insert(
        ref_x_support_foot_trajectory.end(), ceil(step_duration+step_duration*parameters.dsp_percentage/100), invert_x*s[0]);
    ref_y_support_foot_trajectory.insert(
        ref_y_support_foot_trajectory.end(), ceil(step_duration+step_duration*parameters.dsp_percentage/100), invert_y*-parameters.robot_width/2);
    // Middle steps
    for (int i = 1; i < parameters.steps_real-1; i++){
        ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, invert_x*s[i]);
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, invert_y*pow(-1,i+1)*(parameters.robot_width/2));
    }


    //Last step
    ref_x_support_foot_trajectory.insert(
        ref_x_support_foot_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), invert_x*s[step_amount-1]);
    ref_y_support_foot_trajectory.insert(
        ref_y_support_foot_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), invert_y*pow(-1, parameters.steps_real)*parameters.robot_width/2);
}

void Robot::calc_p_ref_heel_curve(int forward, int leftFootStart, std::vector<double> x_footprint_on_spline, std::vector<double> y_footprint_on_spline, std::vector<double> yaw_footprint_on_spline){

    double invert_y;
    if(leftFootStart){
        invert_y = -1;
    }
    else{
        invert_y = 1;
    }
    double step_duration = parameters.step_duration/parameters.Ts;
    // Every step in the lateral direction is made of this piece

    ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, 0);
    if(leftFootStart == 0){
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, -parameters.robot_width/2);
    }
    else{
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, parameters.robot_width/2);
    }
    ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.end(), step_duration, 0);

    for (int i = 0; i < x_footprint_on_spline.size(); i++){
        ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, x_footprint_on_spline.at(i)+invert_y*pow(-1,i)*parameters.robot_width/2*sin(yaw_footprint_on_spline.at(i)));
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, y_footprint_on_spline.at(i)-invert_y*pow(-1,i)*parameters.robot_width/2*cos(yaw_footprint_on_spline.at(i)));
        ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.end(), step_duration, yaw_footprint_on_spline.at(i));
    }

    // ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, 
    //     x_footprint_on_spline.at(x_footprint_on_spline.size()-2)
    //     +invert_y*pow(-1,x_footprint_on_spline.size()-2)*parameters.robot_width/2*sin(yaw_footprint_on_spline.at(x_footprint_on_spline.size()-2)));
    // ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, 
    //     y_footprint_on_spline.at(x_footprint_on_spline.size()-2)
    //     -invert_y*pow(-1,x_footprint_on_spline.size()-2)*parameters.robot_width/2*cos(yaw_footprint_on_spline.at(x_footprint_on_spline.size()-2)));
    // ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.end(), step_duration, yaw_footprint_on_spline.at(x_footprint_on_spline.size()-2));

    ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, 
        ref_x_support_foot_trajectory.back());
    ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, ref_y_support_foot_trajectory.back());
    ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.end(), step_duration,ref_psi_support_foot_trajectory.back());

}


void Robot::calc_p_ref( std::vector<double> *p_ref_x, std::vector<double> *p_ref_y ){

    int step_amount = parameters.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < parameters.steps_real; i++){
        s[i] = i*parameters.step_length;
    }

    double step_duration = ceil(parameters.N/parameters.steps_real);
    // Every step in the lateral direction is made of this piece
    // Calculating the time that is needed for one step


    p_ref_x->insert(p_ref_x->end(), ceil(step_duration*parameters.dsp_percentage/100), s[0]);
    p_ref_y->insert(p_ref_y->end(), ceil(step_duration*parameters.dsp_percentage/100), -parameters.robot_width/2);

    // Middle steps
    for (int i = 0; i < parameters.steps_real; i++){
        if (i == parameters.steps_real-1){
            p_ref_x->insert(p_ref_x->end(), step_duration, s[i-1]);
        }
        else{
            p_ref_x->insert(p_ref_x->end(), step_duration, s[i]);
        }
        p_ref_y->insert(p_ref_y->end(), step_duration, pow(-1,i+1)*(parameters.robot_width/2));

    }
}

void Robot::calc_p_ref_side( std::vector<double> *p_ref_x, std::vector<double> *p_ref_y, int leftside, int leftFootStart, int step_amount, double step_width ){

    step_amount = parameters.steps_real; 

    int  invert_y;

    if(leftFootStart == 0){
        invert_y = 1;
    }
    else{
        invert_y = -1;
    }

    double sx[step_amount]; 
    double sy[step_amount]; 
    for (int i = 0; i < step_amount; i++){
        sy[i] = (i - floor(i / 2))*step_width;
        sx[i] = 0;
    }

    double step_duration = ceil(parameters.N/parameters.steps_real);
    // Every step in the lateral direction is made of this piece
    // Calculating the time that is needed for one step

    p_ref_x->insert(p_ref_x->end(), ceil(step_duration*parameters.dsp_percentage/100), sx[0]);
    p_ref_y->insert(p_ref_y->end(), ceil(step_duration*parameters.dsp_percentage/100), sy[0] - invert_y*(parameters.robot_width/2));
    // std::cout << "length: " << p_ref_x->size() << std::endl;

    // Middle steps
    for (int i = 0; i < parameters.steps_real; i++){
        p_ref_x->insert(p_ref_x->end(), step_duration, sx[i]);
        p_ref_y->insert(p_ref_y->end(), step_duration, sy[i] + invert_y*pow(-1,i+1)*(parameters.robot_width/2));
    }
    // std::cout << "length: " << p_ref_x->size() << std::endl;
}

void Robot::calc_p_ref_circling( std::vector<double> *p_ref_x, std::vector<double> *p_ref_y, int clockwise, 
                                double radius, int step_amount, double step_psi ){

    //Sx, Sy, So means the Virtual CoM's coordinate position
    double sy[step_amount];
    double sx[step_amount];
    double so[step_amount];
    int  invert_y;
    
    if(clockwise == 0){
        invert_y = -1;
    }
    else{
        invert_y = 1;
    }
    for (int i = 0; i < step_amount; i++){  //bug fixed.
        so[i] =  (i - floor(i / 2))*step_psi;
        sy[i] =  invert_y * radius * sin(so[i]);
        //sx[i] =  invert_y * radius * (cos(so[i]) -1); // Bug fixed in clockwise.
        sx[i] =     (-1)     * radius * (cos(so[i]) -1);
        //printf("%f %f %f\n",so[i],sx[i],sy[i]);
    }     


    
    double step_duration = parameters.step_duration/parameters.Ts;
    // Every step in the lateral direction is made of this piece
    // Calculating the time that is needed for one step

    p_ref_x->insert(p_ref_x->end(), ceil(step_duration*parameters.dsp_percentage/100), sx[0]);
    p_ref_y->insert(p_ref_y->end(), ceil(step_duration*parameters.dsp_percentage/100), sy[0] + invert_y*(-parameters.robot_width/2));
    // std::cout << "length: " << p_ref_x->size() << std::endl;

    // Middle steps
    for (int i = 0; i < step_amount -1; i++){
        p_ref_x->insert(p_ref_x->end(), step_duration, sx[i] + (-1) * pow(-1,i) * sin(so[i]) * ( parameters.robot_width/2));
        p_ref_y->insert(p_ref_y->end(), step_duration, sy[i] + invert_y * pow(-1,i) * cos(so[i]) * (-parameters.robot_width/2));
    }
    // Last Steps
    p_ref_x->insert(p_ref_x->end(), step_duration * (1 - parameters.dsp_percentage/100), sx[step_amount-1] + (-1) * pow(-1,step_amount-1) * sin(so[step_amount-1]) * ( parameters.robot_width/2));
    p_ref_y->insert(p_ref_y->end(), step_duration * (1 - parameters.dsp_percentage/100), sy[step_amount-1] + invert_y * pow(-1,step_amount-1) * cos(so[step_amount-1]) * (-parameters.robot_width/2));
    // std::cout << "length: " << p_ref_x->size() << std::endl;
}

void Robot::calc_p_ref_straight( std::vector<double> *p_ref_x, std::vector<double> *p_ref_y, int forward, int leftFootStart, int step_amount, double step_length ){

    step_amount = parameters.steps_real - 1; 

    int invert_x, invert_y;
    if(forward == 0){
        invert_x = -1;
    }
    else{
        invert_x = 1;
    }
    if(leftFootStart == 0){
        invert_y = 1;
    }
    else{
        invert_y = -1;
    }

    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < step_amount; i++){
        s[i] = i*step_length;
    }

    double step_duration = ceil(parameters.N/parameters.steps_real);
    // Every step in the lateral direction is made of this piece
    // Calculating the time that is needed for one step

    p_ref_x->insert(p_ref_x->end(), ceil(step_duration*parameters.dsp_percentage/100), s[0]*invert_x);
    p_ref_y->insert(p_ref_y->end(), ceil(step_duration*parameters.dsp_percentage/100), -parameters.robot_width/2*invert_y);
    // std::cout << "length: " << p_ref_x->size() << std::endl;

    // Middle steps
    for (int i = 0; i < parameters.steps_real; i++){
        if (i == parameters.steps_real-1){
            p_ref_x->insert(p_ref_x->end(), step_duration, s[i-1]*invert_x);
        }
        else{
            p_ref_x->insert(p_ref_x->end(), step_duration, s[i]*invert_x);
        }
        p_ref_y->insert(p_ref_y->end(), step_duration, pow(-1,i+1)*(parameters.robot_width/2)*invert_y);

    }
    // std::cout << "length: " << p_ref_x->size() << std::endl;


}

void Robot::calc_p_ref_only_ssp(){

    int step_amount = parameters.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < parameters.steps_real; i++){
        s[i] = i*parameters.step_length;
    }

    double step_duration = ceil(parameters.N/parameters.steps_real);
    // Every step in the lateral direction is made of this piece
    // Calculating the time that is needed for one step

    //First step



    // Middle steps
    for (int i = 0; i < parameters.steps_real; i++){
        if (i == 0){
            ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, 0);
            ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, -(parameters.robot_width/2));

        }
        else if (i == 1){
            ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, 0);
            ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, (parameters.robot_width/2));
        }
        else{
            ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, 0);
            ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, ref_y_support_foot_trajectory.back());

        }
    }
}
void Robot::calc_p_ref_transition_side(int leftside, int leftFootStart, int step_amount, double step_width){
    step_amount  = parameters.steps_real - 1;
    
    double sx[step_amount]; 
    double sy[step_amount]; 
    for (int i = 0; i < step_amount; i++){
        sy[i] = (i - floor(i / 2))*step_width;
        sx[i] = 0;
    }

    double step_duration = parameters.step_duration/parameters.Ts;

    int  invert_y;

    if(leftFootStart == 0){
        invert_y = 1;
    }
    else{
        invert_y = -1;
    }
    double zmp_offset_y;

    //First step
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ceil(step_duration*parameters.dsp_percentage/100), (sx[0]+parameters.zmp_offset_x));
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ceil(step_duration*parameters.dsp_percentage/100), 0);

    for (int i = 0; i < step_amount; i++){

        if(i == 0){
            if(leftside == 1){
                zmp_offset_y = parameters.zmp_offset_y_right;
            }
            else{
                zmp_offset_y = parameters.zmp_offset_y_left;
            }
            ref_x_zmp_trajectory.insert(
                ref_x_zmp_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), (sx[i]+parameters.zmp_offset_x));
            ref_y_zmp_trajectory.insert(
                ref_y_zmp_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), sy[i] + invert_y*pow(-1,i+1)*(parameters.robot_width/2 + zmp_offset_y));
        }
        else{
            if(i%2==1){
                zmp_offset_y = parameters.zmp_offset_y_left ;
            }
            else{
                zmp_offset_y = parameters.zmp_offset_y_right;
            }
            // Calculate the line of the last and this point
            double x1 = ref_x_zmp_trajectory.back();
            double y1 = ref_y_zmp_trajectory.back();
            double x2 = sx[i] + parameters.zmp_offset_x;
            double y2 = sy[i] + invert_y*pow(-1,i+1)*(parameters.robot_width/2 + zmp_offset_y);
            double cx = x1;
            double cy = y1;
            double mx = (x2-cx)/parameters.t_dsp;   //slope 
            double my = (y2-cy)/parameters.t_dsp;
            double dsp_duration = ceil(step_duration*parameters.dsp_percentage/100);
            double increment = parameters.t_dsp/dsp_duration;
            for (int k = 0; k < dsp_duration; k++){
                double x_increment = mx*k*increment + cx;
                double y_increment = my*k*increment + cy;
                ref_x_zmp_trajectory.push_back(x_increment);
                ref_y_zmp_trajectory.push_back(y_increment);
            }
            double ssp_duration = step_duration - dsp_duration;

            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ssp_duration, x2);
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ssp_duration, y2);
        }
    }

    //Last step
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, (sx[step_amount-1]+parameters.zmp_offset_x));
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, sy[step_amount-1]);
}


void Robot::calc_p_ref_transition_curve(int forward, int leftFootStart, std::vector<double> x_footprint_on_spline, std::vector<double> y_footprint_on_spline, std::vector<double> yaw_footprint_on_spline){

    double step_duration = parameters.step_duration/parameters.Ts;
    double zmp_offset_y;
    int invert_y;

    if(leftFootStart == 0){
        invert_y = 1;
    }
    else{
        invert_y = -1;
    }
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, 0);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, 0);

    // Every step in the lateral direction is made of this piece
    for (int i = 0; i < x_footprint_on_spline.size()-1; i++){
        // ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(), step_duration, x_footprint_on_spline.at(i)+invert_y*pow(-1,i)*parameters.robot_width/2*sin(yaw_footprint_on_spline.at(i)));
        // ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(), step_duration, y_footprint_on_spline.at(i)-invert_y*pow(-1,i)*parameters.robot_width/2*cos(yaw_footprint_on_spline.at(i)));
        // ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.end(), step_duration, yaw_footprint_on_spline.at(i));
        // Calculate the line of the last and this point
        // if(leftFootStart == 0){

        // }
        if(i%2 == 1){
            zmp_offset_y = parameters.zmp_offset_y_right;
        }
        else{
            zmp_offset_y = parameters.zmp_offset_y_left;
        }
        double x1 = x_footprint_on_spline.at(i)+ parameters.zmp_offset_x+invert_y*pow(-1,i)*parameters.robot_width/2*sin(yaw_footprint_on_spline.at(i));
        double y1 = y_footprint_on_spline.at(i)+pow(-1,i)*zmp_offset_y-invert_y*pow(-1,i)*parameters.robot_width/2*cos(yaw_footprint_on_spline.at(i));
        double dsp_duration = ceil(step_duration*parameters.dsp_percentage/100);
        double ssp_duration = step_duration - dsp_duration;
        // std::cout << "y1: " << y1 << std::endl;

        // std:cout << "========" << std::endl;
        ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ssp_duration, x1);
        ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ssp_duration, y1);

        double x2,y2;
        
        // if( i != x_footprint_on_spline.size()-2){
            if(i%2 == 1){
                zmp_offset_y = parameters.zmp_offset_y_left;
            }
            else{
                zmp_offset_y = parameters.zmp_offset_y_right;
            }
            x2 = x_footprint_on_spline.at(i+1)+ parameters.zmp_offset_x+invert_y*pow(-1,i+1)*parameters.robot_width/2*sin(yaw_footprint_on_spline.at(i+1));
            y2 = y_footprint_on_spline.at(i+1)+pow(-1,i+1)*zmp_offset_y-invert_y*pow(-1,i+1)*parameters.robot_width/2*cos(yaw_footprint_on_spline.at(i+1));
        // }
        // else{
        //     x2 = x_footprint_on_spline.back();
        //     y2 = y_footprint_on_spline.back();
        //     // x2 = x1;
        //     // y2 = y1;
        // }

        // std::cout << "y2: " << y2 << std::endl;

        double cx = x1;
        double cy = y1;
        double mx = (x2-cx)/parameters.t_dsp;
        double my = (y2-cy)/parameters.t_dsp;
        double increment = parameters.t_dsp/dsp_duration;

        for (int k = 0; k < dsp_duration; k++){
            double x_increment = mx*k*increment + cx;
            double y_increment = my*k*increment + cy;
            ref_x_zmp_trajectory.push_back(x_increment);
            ref_y_zmp_trajectory.push_back(y_increment);
        }
        //         }
        // else{
        //     for(int k = 0;k < dsp_duration; k++){
        //         ref_x_zmp_trajectory.push_back(ref_x_zmp_trajectory.back());
        //         ref_y_zmp_trajectory.push_back(ref_y_zmp_trajectory.back());
        //     }

        // }
        // std::cout << "length: " << ref_x_zmp_trajectory.size() << std::endl;
    }

    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, ref_y_zmp_trajectory.back());
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, ref_x_zmp_trajectory.back());

    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, x_footprint_on_spline.back());
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, y_footprint_on_spline.back());
    
    
    
    
}


void Robot::calc_p_ref_transition_circling(int clockwise, double radius, int step_amount, double step_psi)
{
    //Sx, Sy, So means the Virtual CoM's coordinate position
    double sy[step_amount];
    double sx[step_amount];
    double so[step_amount];

    int  invert_y;
    
    if(clockwise == 0){
        invert_y = -1;
    }
    else{
        invert_y = 1;
    }
    for (int i = 0; i < step_amount; i++){  //bug fixed.
        so[i] =  (i - floor(i / 2))*step_psi;
        sy[i] =  invert_y * radius * sin(so[i]);
        sx[i] =  (-1) * radius * (cos(so[i]) -1);
        //printf("%f %f %f\n",so[i],sx[i],sy[i]);
    }       

    
    double step_duration = parameters.step_duration/parameters.Ts;

    //First step
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ceil(step_duration*parameters.dsp_percentage/100), 0);    
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ceil(step_duration*parameters.dsp_percentage/100), 0);
    ref_psi_body.insert(ref_psi_body.end(), ceil(step_duration*parameters.dsp_percentage/100),0);
    
    double clcwise_offset = 0.001;

    for (int i = 0; i < step_amount - 1; i++){
        if(i == 0){
            ref_x_zmp_trajectory.insert(
                ref_x_zmp_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), 
                    sx[i] +     (-1)    * pow(-1,i) * sin(so[i]) * ( parameters.robot_width/2 + (parameters.zmp_offset_y + pow(-1,i) * clcwise_offset) * sin(so[i]) ));
            ref_y_zmp_trajectory.insert(
                ref_y_zmp_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)),
                    sy[i] + invert_y    * pow(-1,i) * cos(so[i]) * (-parameters.robot_width/2 - (parameters.zmp_offset_y + pow(-1,i) * clcwise_offset) * cos(so[i]) ));
            ref_psi_body.insert(ref_psi_body.end(),floor(step_duration*(1-parameters.dsp_percentage/100)),0);
        }
        else{
            // Calculate the line of the last and this point
            double x1 = ref_x_zmp_trajectory.back();
            double y1 = ref_y_zmp_trajectory.back();
            double psi1 = ref_psi_body.back();

            double x2 = sx[i] + (-1) * pow(-1,i) * sin(so[i]) * ( parameters.robot_width/2 + (parameters.zmp_offset_y + pow(-1,i) * clcwise_offset)* sin(so[i]) );
            double y2 = sy[i] + invert_y * pow(-1,i) * cos(so[i]) * (-parameters.robot_width/2 - (parameters.zmp_offset_y + pow(-1,i) * clcwise_offset) * cos(so[i]) );
            
            double psi2 = psi1 + -invert_y*step_psi;
            if(i % 2 == 0)
            {
                psi2 = psi1;
            }
            double cx = x1;
            double cy = y1;

            //double cpsi = psi1;

            double mx = (x2-cx)/parameters.t_dsp;   //slope 
            double my = (y2-cy)/parameters.t_dsp;
            //double mpsi = (psi2 - cpsi) / parameters.t_dsp;

            double dsp_duration = ceil(step_duration*parameters.dsp_percentage/100);
            double increment = parameters.t_dsp/dsp_duration;

            std::vector<double> t_psi(6), psi(6);
            double pk0,pk1,pk2,pk3,pk4,pk5;
            double delta_psi = psi2 - psi1;
            pk0 = 0;
            pk1 = 0.0;
            pk2 = 0.05;
            pk3 = 0.6;
            pk4 = 1.0;
            pk5 = 1.0;
            psi[0]      =   psi1    +   delta_psi * pk0;
            psi[1]      =   psi1    +   delta_psi * pk1;
            psi[2]      =   psi1    +   delta_psi * pk2;
            psi[3]      =   psi1    +   delta_psi * pk3;
            psi[4]      =   psi1    +   delta_psi * pk4;
            psi[5]      =   psi1    +   delta_psi * pk5;
            
            t_psi[0]    =   0.0 * dsp_duration;
            t_psi[1]    =   0.01* dsp_duration;
            t_psi[2]    =   0.2 * dsp_duration;
            t_psi[3]    =   0.6 * dsp_duration;
            t_psi[4]    =   0.99* dsp_duration;
            t_psi[5]    =   1.0 * dsp_duration;

            tk::spline psi_ref;
            psi_ref.set_points(t_psi,psi);

            for (int k = 0; k < dsp_duration; k++){
                //double t_increment = increment * k;
                double x_increment = mx*k*increment + cx;
                double y_increment = my*k*increment + cy;
                //double psi_increment = mpsi * k * increment + cpsi;

                ref_x_zmp_trajectory.push_back(x_increment);
                ref_y_zmp_trajectory.push_back(y_increment);

                ref_psi_body.push_back(psi_ref(k));
                //ref_psi_body.push_back(psi_increment);
            }
            double ssp_duration = step_duration - dsp_duration;
            
            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ssp_duration, x2);
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ssp_duration, y2);


            
            ref_psi_body.insert(ref_psi_body.end(),ssp_duration,psi2);
        }
    }
     //Last step
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, sx[step_amount-1]);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, sy[step_amount-1]);   

    ref_psi_body.insert(ref_psi_body.end(),step_duration,ref_psi_body.back());

}
void Robot::calc_p_ref_transition_straight(int forward, int leftFootStart, int step_amount, double step_length){
    step_amount  -= 1;
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < step_amount; i++){
        s[i] = i*parameters.step_length;
    }

    double step_duration = parameters.step_duration/parameters.Ts;

    int invert_x, invert_y;
    if(forward == 0){
        invert_x = -1;
    }
    else{
        invert_x = 1;
    }

    if(leftFootStart == 0){
        invert_y = 1;
    }
    else{
        invert_y = -1;
    }
    double zmp_offset_y;
    // if(left)
    // zmp_offset_y

    //First step
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ceil(step_duration*parameters.dsp_percentage/100), (s[0]+parameters.zmp_offset_x)*invert_x);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ceil(step_duration*parameters.dsp_percentage/100), 0);

    for (int i = 0; i < step_amount; i++){

        if(i == 0){
            ref_x_zmp_trajectory.insert(
                ref_x_zmp_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), (s[i]+parameters.zmp_offset_x)*invert_x);
            ref_y_zmp_trajectory.insert(
                ref_y_zmp_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), (pow(-1,i+1)*(parameters.robot_width/2 + parameters.zmp_offset_y))*invert_y);
        }
        else{
            // Calculate the line of the last and this point
            double x1 = ref_x_zmp_trajectory.back();
            double y1 = ref_y_zmp_trajectory.back();
            double x2 = invert_x*(s[i] + parameters.zmp_offset_x);
            double y2 = invert_y*(pow(-1, i+1)*(parameters.robot_width/2 + parameters.zmp_offset_y));
            double cx = x1;
            double cy = y1;
            double mx = (x2-cx)/parameters.t_dsp;
            double my = (y2-cy)/parameters.t_dsp;
            double dsp_duration = ceil(step_duration*parameters.dsp_percentage/100);
            double increment = parameters.t_dsp/dsp_duration;
            for (int k = 0; k < dsp_duration; k++){
                double x_increment = mx*k*increment + cx;
                double y_increment = my*k*increment + cy;
                ref_x_zmp_trajectory.push_back(x_increment);
                ref_y_zmp_trajectory.push_back(y_increment);
            }
            double ssp_duration = step_duration - dsp_duration;

            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ssp_duration, x2);
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ssp_duration, y2);
        }
    }

    //Last step
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, invert_x*(s[step_amount-1]+parameters.zmp_offset_x));
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, 0);
}

void Robot::calc_p_ref_transition(){

    int step_amount = parameters.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < step_amount; i++){
        s[i] = i*parameters.step_length;
    }

    double step_duration = parameters.step_duration/parameters.Ts;

    //First step

    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ceil(step_duration*parameters.dsp_percentage/100), s[0]+parameters.zmp_offset_x);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ceil(step_duration*parameters.dsp_percentage/100), 0);

    for (int i = 0; i < step_amount; i++){

        if(i == 0){
            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), s[i]+parameters.zmp_offset_x);
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), pow(-1,i+1)*(parameters.robot_width/2 + parameters.zmp_offset_y));
        }
        else{
            // Calculate the line of the last and this point
            double x1 = ref_x_zmp_trajectory.back();
            double y1 = ref_y_zmp_trajectory.back();
            double x2 = s[i] + parameters.zmp_offset_x;
            double y2 = pow(-1, i+1)*(parameters.robot_width/2 + parameters.zmp_offset_y);
            double cx = x1;
            double cy = y1;
            double mx = (x2-cx)/parameters.t_dsp;
            double my = (y2-cy)/parameters.t_dsp;
            double dsp_duration = ceil(step_duration*parameters.dsp_percentage/100);
            double increment = parameters.t_dsp/dsp_duration;
            for (int k = 0; k < dsp_duration; k++){
                double x_increment = mx*k*increment + cx;
                double y_increment = my*k*increment + cy;
                ref_x_zmp_trajectory.push_back(x_increment);
                ref_y_zmp_trajectory.push_back(y_increment);
            }
            double ssp_duration = step_duration - dsp_duration;

            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ssp_duration, x2);
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ssp_duration, y2);
        }
    }

    //Last step
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, s[step_amount-1]+parameters.zmp_offset_x);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, 0);
}
void Robot::calc_p_ref_transition_only_ssp(){

    int step_amount = parameters.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < step_amount; i++){
        s[i] = i*(parameters.step_length/2);
    }

    double step_duration = ceil(parameters.N/parameters.steps_real);

    //First step
    //p_ref_x->insert(p_ref_x->end(), ceil(step_duration*parameters.dsp_percentage/100), s[0]+parameters.zmp_offset_x);
    //p_ref_y->insert(p_ref_y->end(), ceil(step_duration*parameters.dsp_percentage/100), 0);
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, s[0]+parameters.zmp_offset_x);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, 0);



    for (int i = 0; i < step_amount; i++){

        if(i == 0){
            //             p_ref_x->insert(p_ref_x->end(), step_duration, s[i]+parameters.zmp_offset_x);
            // //            p_ref_y->insert(p_ref_y->end(), step_duration, pow(-1,i)*(parameters.robot_width/2 + parameters.zmp_offset_y));
            //             p_ref_y->insert(p_ref_y->end(), step_duration, 0);
        }
        else if(i == 1){
            // Calculate the line of the last and this point
            double x1 = ref_x_zmp_trajectory.back();
            double y1 = ref_y_zmp_trajectory.back();
            double x2 = s[i] + parameters.zmp_offset_x;
            double y2 = pow(-1, i+1)*(parameters.robot_width/2 + parameters.zmp_offset_y);
            double cx = x1;
            double cy = y1;
            double mx = (x2-cx)/(parameters.t_dsp);
            double my = (y2-cy)/(parameters.t_dsp);
            double dsp_duration = ceil(step_duration*parameters.dsp_percentage/100);
            double increment = parameters.t_dsp/dsp_duration;
            for (int k = 0; k < dsp_duration; k++){
                double x_increment = mx*k*increment + cx;
                double y_increment = my*k*increment + cy;
                ref_x_zmp_trajectory.push_back(x_increment);
                //p_ref_y->push_back(p_ref_y->back());
                ref_y_zmp_trajectory.push_back(y_increment);

            }

            // p_ref_x->insert(p_ref_x->end(), parameters.step_duration, x2);
            // p_ref_y->insert(p_ref_y->end(), parameters.step_duration, y2);
        }
        else{
            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, ref_x_zmp_trajectory.back());
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, ref_y_zmp_trajectory.back());
        }
    }

    // std::cout << "x length zmp: " << ref_x_zmp_trajectory.size() << std::endl;
    // std::cout << "x length support: " << ref_x_support_foot_trajectory.size() << std::endl;
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), 
        (ref_x_support_foot_trajectory.size()-ref_x_zmp_trajectory.size()), ref_x_zmp_trajectory.back());
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(),
        (ref_y_support_foot_trajectory.size()-ref_y_zmp_trajectory.size()), ref_y_zmp_trajectory.back());
    // std::cout << "after appending: " << ref_x_zmp_trajectory.size() << std::endl;

    //Last step
    //p_ref_x->insert(p_ref_x->end(), floor(step_duration*(1-parameters.dsp_percentage/100)), s[step_amount-1]+parameters.zmp_offset_x);
    //p_ref_y->insert(p_ref_y->end(), floor(step_duration*(1-parameters.dsp_percentage/100)), 0);
    //p_ref_x->insert(p_ref_x->end(), step_duration, s[step_amount-1]+parameters.zmp_offset_x);
    //p_ref_y->insert(p_ref_y->end(), step_duration, 0);
}

void Robot::calc_p_ref_transition_only_ssp_sine(){
    int step_amount = parameters.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < step_amount; i++){
        s[i] = i*(parameters.step_length/2);
    }
    double step_duration = ceil(parameters.N/parameters.steps_real);

    //First step
    //p_ref_x->insert(p_ref_x->end(), ceil(step_duration*parameters.dsp_percentage/100), s[0]+parameters.zmp_offset_x);
    //p_ref_y->insert(p_ref_y->end(), ceil(step_duration*parameters.dsp_percentage/100), 0);
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, s[0]+parameters.zmp_offset_x);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, 0);
    // std::cout << "x length zmp: " << ref_x_zmp_trajectory.size() << std::endl;


    for (int i = 0; i < step_amount; i++){
        if(i == 0){
            // Calculate the line of the last and this point
            double x1 = ref_x_zmp_trajectory.back();
            double y1 = ref_y_zmp_trajectory.back();
            double x2 = s[i] + parameters.zmp_offset_x;
            double y2 = pow(-1, i)*(parameters.robot_width/2 + parameters.zmp_offset_y);
            double cx = x1;
            double cy = y1;
            double mx = (x2-cx)/(parameters.t_dsp);
            double my = (y2-cy)/(parameters.t_dsp);
            double dsp_duration = ceil(step_duration*parameters.dsp_percentage/100);
            double increment = parameters.t_dsp/dsp_duration;
            for (int k = 0; k < dsp_duration; k++){
                double x_increment = mx*k*increment + cx;
                double y_increment = my*k*increment + cy;
                ref_x_zmp_trajectory.push_back(x_increment);
                //p_ref_y->push_back(p_ref_y->back());
                ref_y_zmp_trajectory.push_back(y_increment);

            }
            // std::cout << "x length zmp: " << ref_x_zmp_trajectory.size() << std::endl;
            // std::cout << "y length zmp: " << ref_y_zmp_trajectory.size() << std::endl;

            //             p_ref_x->insert(p_ref_x->end(), step_duration, s[i]+parameters.zmp_offset_x);
            // //            p_ref_y->insert(p_ref_y->end(), step_duration, pow(-1,i)*(parameters.robot_width/2 + parameters.zmp_offset_y));
            //             p_ref_y->insert(p_ref_y->end(), step_duration, 0);
        }
        else if(i == 1 || i == 2 || i == 3){
            double ampl = 0.03;
            double per  = 2*pi/(step_duration);
            for (int k = 0; k < step_duration; k++){
                ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), 1, ref_x_zmp_trajectory.back());
                ref_y_zmp_trajectory.push_back(ampl*sin(per*k)+parameters.robot_width/2 + parameters.zmp_offset_y);    
            }
            // std::cout << "x length zmp: " << ref_x_zmp_trajectory.size() << std::endl;
            // std::cout << "y length zmp: " << ref_y_zmp_trajectory.size() << std::endl;
        }
        else{
            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, ref_x_zmp_trajectory.back());
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, ref_y_zmp_trajectory.back());
            // std::cout << "x length zmp: " << ref_x_zmp_trajectory.size() << std::endl;
            // std::cout << "y length zmp: " << ref_y_zmp_trajectory.size() << std::endl;
        }
    }
    // std::cout << "x length zmp: " << ref_x_zmp_trajectory.size() << std::endl;
    // std::cout << "x length support: " << ref_x_support_foot_trajectory.size() << std::endl;
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), 
        (ref_x_support_foot_trajectory.size()-ref_x_zmp_trajectory.size()), ref_x_zmp_trajectory.back());

    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(),
        (ref_y_support_foot_trajectory.size()-ref_y_zmp_trajectory.size()), ref_y_zmp_trajectory.back());
    // std::cout << "after appending: " << ref_x_zmp_trajectory.size() << std::endl;

    //Last step
    //p_ref_x->insert(p_ref_x->end(), floor(step_duration*(1-parameters.dsp_percentage/100)), s[step_amount-1]+parameters.zmp_offset_x);
    //p_ref_y->insert(p_ref_y->end(), floor(step_duration*(1-parameters.dsp_percentage/100)), 0);
    //p_ref_x->insert(p_ref_x->end(), step_duration, s[step_amount-1]+parameters.zmp_offset_x);
    //p_ref_y->insert(p_ref_y->end(), step_duration, 0);
}


void Robot::calc_p_ref_transition_only_dsp_sine(){

    int step_amount = parameters.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < step_amount; i++){
        s[i] = i*parameters.step_length;
    }

    double step_duration = ceil(parameters.N/parameters.steps_real);

    //First step

    // ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ceil(step_duration*parameters.dsp_percentage/100), s[0]+parameters.zmp_offset_x);
    // ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ceil(step_duration*parameters.dsp_percentage/100), 0);

    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ceil(step_duration), s[0]+parameters.zmp_offset_x);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ceil(step_duration), 0);

    for (int i = 0; i < step_amount; i++){

        if(i == 0){
            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), s[i]+parameters.zmp_offset_x);
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), floor(step_duration*(1-parameters.dsp_percentage/100)), pow(-1,i+1)*(0.04));
        }
        else{
            // Calculate the line of the last and this point
            double x1 = ref_x_zmp_trajectory.back();
            double y1 = ref_y_zmp_trajectory.back();
            double x2 = s[i] + parameters.zmp_offset_x;
            double y2 = pow(-1, i+1)*(0.04);
            double cx = x1;
            double cy = y1;
            double mx = (x2-cx)/parameters.t_dsp;
            double my = (y2-cy)/parameters.t_dsp;
            double dsp_duration = ceil(step_duration*parameters.dsp_percentage/100);
            double increment = parameters.t_dsp/dsp_duration;
            for (int k = 0; k < dsp_duration; k++){
                double x_increment = mx*k*increment + cx;
                double y_increment = my*k*increment + cy;
                ref_x_zmp_trajectory.push_back(x_increment);
                ref_y_zmp_trajectory.push_back(y_increment);
            }
            double ssp_duration = step_duration - dsp_duration;

            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), ssp_duration, x2);
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), ssp_duration, y2);
        }
    }

    //Last step
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), step_duration, s[step_amount-1]+parameters.zmp_offset_x);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), step_duration, 0);
}
double Robot::calc_g_j( int j, int N, dlib::matrix<double> f_tilde){
    double g_j = 0;
    for(int i = j; i < N+j; i++){
        g_j += f_tilde(i);
    }
    return g_j;
}

void Robot::set_LQR_gains(){
    arma::mat A_arma, B_arma, C_arma;
    A_arma << 1 << parameters.Ts << pow(parameters.Ts,2)/2 << arma::endr
    << 0 << 1 << parameters.Ts << arma::endr
    << 0 << 0 << 1 << arma::endr;
    B_arma << pow(parameters.Ts,3)/6 << arma::endr
    << pow(parameters.Ts,2)/2 << arma::endr
    << parameters.Ts << arma::endr;
    C_arma << 1 << 0 << -parameters.z_com_ctm/parameters.g << arma::endr;

    arma::mat A_tilde, B_tilde, C_tilde, P, K_tilde, f_tilde_new, A1, A2, A3, A4, A5, A6, CA, tmp, tmp2, CB, Q_e, R;
    Q_e = 1e0;//1e0;
    R = 1e-4;//1e-5;
    CA = C_arma*A_arma;

    tmp << 1;
    tmp2 << 0;
    A1 = arma::join_horiz(tmp, CA);
    A2 = arma::join_horiz(tmp2, A_arma.row(0));
    A3 = arma::join_horiz(tmp2, A_arma.row(1));
    A4 = arma::join_horiz(tmp2, A_arma.row(2));
    A5 = arma::join_vert(A1, A2);
    A6 = arma::join_vert(A3, A4);
    A_tilde = arma::join_vert(A5, A6);

    CB = C_arma*B_arma;
    B_tilde = arma::join_vert(CB, B_arma);

    C_tilde << 1 << 0 << 0 << 0 << arma::endr;

    ////R=1e-6 Q_e = 1e0

    //     K_tilde << 659.837044024723      <<    66693.4003797182  <<        17186.1493971068   <<       155.471554807457<< arma::endr;


    // P <<          101.075562494984     <<     5057.59688559088     <<     1270.24683098069       <<   3.27597450572226<< arma::endr
    //          << 5057.59688559088      <<    259678.882330607       <<    65246.610054497        <<  174.832212197645<<arma::endr
    //         <<  1270.24683098069      <<     65246.610054497       <<   16394.1284572455       <<   44.0147597097014<<arma::endr
    //         <<  3.27597450572226     <<     174.832212197645      <<    44.0147597097014      <<   0.140038508909529<<arma::endr;

    //R=1e-5 Q_e = 1e0
    // P   << 103.0148 << 5.245e3 << 1.3212e3 << 3.7673 << arma::endr
    //     << 5.245e3 << 2.7969e5 << 7.0423e4 << 225.5349 << arma::endr
    //     << 1.3212e3 << 7.0423e4 << 1.7734e4 << 57.3439 << arma::endr
    //     << 3.7673 << 225.5349 << 57.3439 << 0.3280 << arma::endr;
    // K_tilde << 248.6270 << 2.5612e4 << 6.7242e3 << 93.0196 << arma::endr;

    ////R = 1e-1 Q_e = 1e0
    // P   <<  140.235222608089      <<     9762.84121866896     <<      2686.2852328849     <<       67.5183667764935 << arma::endr
    //     <<   9762.84121866896     <<     836718.585620353     <<      242466.264580579    <<       9136.16668510101 << arma::endr
    //     <<    2686.2852328849     <<     242466.264580579     <<      72040.0559419522    <<       3213.61075641625 << arma::endr
    //     <<   67.5183667764935     <<     9136.16668510101     <<      3213.61075641625    <<       311.124643121467 << arma::endr;
    // K_tilde << 3.04318357259503 << 426.761525740497  <<  151.751301535754 << 15.1573858636106 << arma::endr;

    //// R = 1e-2 Q_e = 0
    // P       <<  123.058555821367        <<   7510.17480250913      <<    1960.56734793799       <<   23.7475909162506<< arma::endr
    //         <<  7510.17480250913        <<       535832.882976395   <<       143685.978325481   <<       2680.82496202312<< arma::endr
    //         <<  1960.56734793799        <<       143685.978325481   <<        38892.666923116    <<      823.223658313809<< arma::endr
    //         <<  23.7475909162506        <<       2680.82496202312    <<      823.223658313809    <<      46.8943495595413<< arma::endr;
    // K_tilde << 9.44655556946055      <<      1162.479485864     <<     363.445014185274        <<  22.4265227848847<< arma::endr;

    ////R = 1e-2, Q_e = 1e-1
    // P<< 14.0235222608173      <<    976.284121867649      <<    268.628523288715     <<     6.75183667765908<< arma::endr
    // <<  976.284121867649    <<      83671.8585621035    <<      24246.6264580783   <<       913.616668511012<<arma::endr
    // <<  268.628523288715    <<      24246.6264580783    <<      7204.00559420137   <<         321.3610756419<<arma::endr
    // <<  6.75183667765908    <<      913.616668511012    <<        321.3610756419   <<       31.1124643121598<<arma::endr;
    // K_tilde << 3.04318357259947       <<   426.761525740913         << 151.751301535879        <<  15.1573858636166<< arma::endr;

    //// R = 1e-3 Q_e = 1e0
    // K_tilde <<          28.9484521684284   <<       3259.73906211331        <<  933.955595517462         << 34.7119390300774 << arma::endr;


    // P       <<           112.60495183463    <<      6283.63511292275      <<    1600.83669866678    <<       9.7525094608969<<arma::endr
    //         <<  6283.63511292275            <<     390542.907060209       <<   100662.890116198      <<    901.164144544033<<arma::endr
    //         <<  1600.83669866678            <<     100662.890116198      <<     26015.984207411     <<     251.201936558795<<arma::endr
    //         <<   9.7525094608969            <<     901.164144544033      <<    251.201936558795     <<     7.56476037150906<<arma::endr;

    //// R = 1e-3 Q_e = 1e-1


    // K_tilde << 9.44655556946076 << 1162.47948586408 << 363.445014185298 << 22.4265227848859 << arma::endr;

    // P << 12.3058555821358      <<    751.017480250871      <<    196.056734793791      <<    2.37475909162509 << arma::endr
    //        <<   751.017480250871    <<      53583.2882976383  <<         14368.597832548  <<        268.082496202328 << arma::endr 
    //        <<   196.056734793791    <<       14368.597832548  <<        3889.26669231164  <<        82.3223658313861 << arma::endr
    //         <<  2.37475909162509    <<      268.082496202328   <<       82.3223658313861   <<       4.68943495595441 << arma::endr;
    //// R = 1e-4 Q_e = 1e0
    K_tilde <<          86.6575298492714        <<  9228.54560079943         << 2503.11484728163<<          56.0095591168141 << arma::endr;


    P <<    106.494445628131        <<  5617.28625200757        <<   1417.2420008996      <<    5.24114756015971<< arma::endr 
    <<    5617.28625200757      <<    317570.028288164    <<      80472.6122339247       <<   383.767629924021<< arma::endr
    <<     1417.2420008996      <<    80472.6122339247    <<       20404.591773328      <<    100.570822552496<<arma::endr
    <<    5.24114756015971      <<    383.767629924021    <<      100.570822552496        <<  1.36921969103054<<arma::endr;

    K_s = K_tilde(0,0);
    K_x = K_tilde(0,1), K_tilde(0,2), K_tilde(0,3);

    arma::mat tmp3, tmp4, tmp5, tmp6;

    dlib::matrix<double> f_tilde;
    f_tilde = dlib::ones_matrix<double>(1,2*parameters.N_T);

    for(int j = 1; j < 2*parameters.N_T+1; j++){
        tmp3 = arma::inv(R+B_tilde.t()*P*B_tilde);
        tmp4 = arma::trans(A_tilde-B_tilde*K_tilde);
        tmp5 = arma::real(arma::logmat(tmp4));        
        tmp6 = arma::expmat(tmp5*(j-1));
        f_tilde_new = tmp3*B_tilde.t()*tmp6*C_tilde.t()*Q_e;
        dlib::set_subm(f_tilde, dlib::range(0,0), dlib::range(j-1,j-1)) = f_tilde_new(0,0);
    }

    A = 1, parameters.Ts, pow(parameters.Ts,2)/2,
    0, 1, parameters.Ts,
    0, 0, 1;

    B = pow(parameters.Ts,3)/6, pow(parameters.Ts,2)/2, parameters.Ts;

    C = 1, 0, -parameters.z_com_ctm/parameters.g;


    g_j = dlib::ones_matrix<double>(1, parameters.N_T);
    for(int j = 0; j < parameters.N_T; j++){
        dlib::set_colm(g_j, j) = calc_g_j(j, parameters.N_T, f_tilde);
    }
}

void Robot::calculate_X_ref_integrated(double p_measured, std::vector<double> p_ref, int useEst,
    std::vector<dlib::matrix<double,3,1>> * X_ref, dlib::matrix<double,3,1> X_est, std::vector<double> * u, dlib::matrix<double,3,1> * x_mpc){
    //, KalmanFilter * KF){
    dlib::matrix<double> p_ref_piece;

    p_ref_piece = dlib::ones_matrix<double>(parameters.N_T, 1);
    // std::cout << parameters.N << "N" << std::endl;
    int k = 0;

    if(time_step < parameters.N - parameters.N_T){
        for (int j = time_step+1; j < time_step+parameters.N_T+1; j++){
            dlib::set_rowm(p_ref_piece, k) = p_ref.at(j);
            k++;
        }
    }
    else {
        // std::cout << "in else" << std::endl;
        int counter = 0;
        for(std::vector<double>::iterator it = p_ref.begin()+time_step+1; it != p_ref.end(); ++it){
            dlib::set_rowm(p_ref_piece, k) = *it;
            k++;
            counter++;
        }
        for (int m = 0; m < (parameters.N_T -  counter); m++){
            dlib::set_rowm(p_ref_piece, k) = p_ref.back();
            k++;
        }
    }  

// std::cout << "here"  << std::endl;

    double u_k;
    if(useEst == 0){
        u_k = -K_s*(C*X_ref->back() - p_ref.at(time_step)) - K_x*X_ref->back() + g_j*p_ref_piece;
    }
    else{
        // u_k = -K_s*(p_ref.at(time_step)-p_measured) - K_x*X_est + g_j*p_ref_piece;
        // std::cout << "YES" << std::endl;
        // u_k = -K_s*(p_measured - p_ref.at(time_step)) - K_x*X_ref->back() + g_j*p_ref_piece;
        if(time_step > 400){
            u_k = -K_s*(p_measured - p_ref.at(time_step)) - K_x*X_est + g_j*p_ref_piece;
        }
        else{
            u_k = -K_s*(p_measured - p_ref.at(time_step)) - K_x*X_ref->back() + g_j*p_ref_piece;
        }

    }
// std::cout << "here2"  << std::endl;

    // tmp1 = -K_s*(C*X_ref->back() - p_ref.at(time_step));
    // tmp2 = - K_x*X_ref->back();
    // // double p_ref_control = p_ref.at(time_step);
    // // if((p_ref.at(time_step)) != 0.13){

    // // }
    // tmp7 = -K_s*(p_measured - (p_ref.at(time_step)));
    // tmp3 = -K_s*( y_output_estimation(1) - (p_ref.at(time_step)));

    // tmp4 = - K_x*X_est ;

    // tmp5 = g_j*p_ref_piece;
    // tmp6 = u_k;
    // tmp8 = -K_x*X_est;
// std::cout << "here3"  << std::endl;

    // // Only for MPC
    dlib::matrix<double,3,1> x_mpc_state;
    x_mpc_state = A*x_mpc_state+B*(-K_s*(p_ref.at(time_step)-p_measured) - K_x*x_mpc_state + g_j*p_ref_piece);
    *x_mpc = x_mpc_state;
    /* ========================
       Build safety in here*/
// std::cout << "here4"  << std::endl;

    /*=========================*/
    X_ref->push_back(A*X_ref->back()+B*u_k);
    u->push_back(u_k);
// std::cout << "here5"  << std::endl;
}

// Calculate piece of a swing foot trajectory

void Robot::calculate_swing_foot_piece_trajectory_stay_at_height(int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double y,
    std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete){

    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory

    std::vector<double> t_x(7), t_z(5), x(7), z(5);
    // Calculate x
    double step_length = x_swing_foot_end-x_swing_foot_start;

    t_x[0]=0.0*parameters.step_duration;
    t_x[1]=0.001*parameters.step_duration;
    t_x[2]=0.3*parameters.step_duration;
    t_x[3]=0.6*parameters.step_duration;
    t_x[4]=0.8*parameters.step_duration;
    t_x[5]=0.999*parameters.step_duration;
    t_x[6]=1.0*parameters.step_duration;

    //if(x_swing_foot_start != x_swing_foot_end){
    x[0]=x_swing_foot_start;
    x[1]=x_swing_foot_start;
    x[2]=x_swing_foot_start+0.15*(x_swing_foot_end - x_swing_foot_start);
    x[3]=x_swing_foot_start+0.70*(x_swing_foot_end - x_swing_foot_start);
    x[4]=x_swing_foot_start+0.95*(x_swing_foot_end - x_swing_foot_start);
    x[5]=x_swing_foot_end;
    x[6]=x_swing_foot_end;

    //printf("t: %.3f, %.3f, %.3f, %.3f, %.3f\n", t_x[0],t_x[1], t_x[2], t_x[3], t_x[4]);
    //printf("x: %.3f, %.3f, %.3f, %.3f, %.3f\n", x[0], x[1], x[2], x[3], x[4]);
    //}
    //else{
    //x[0] = x_swing_foot_start;
    //x[1] = x_swing_foot_start;
    //x[2] = x_swing_foot_start;
    //x[3] = x_swing_foot_start;
    //x[4] = x_swing_foot_start;
    //}

    // Calculate z
    t_z[0]=0.0*parameters.step_duration;
    t_z[1]=0.001*parameters.step_duration;
    // t_z[2]=0.1*parameters.step_duration;
    // t_z[3]=0.3*parameters.step_duration;
    t_z[2]=0.9*parameters.step_duration; // Good parameter was 0.5
    t_z[3]=0.999*parameters.step_duration;
    t_z[4]=1.0*parameters.step_duration;

    z[0]=parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    z[1]=parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // z[2]=parameters.robot_ankle_to_foot+0.1*swing_foot_z_peak;
    // z[3]=parameters.robot_ankle_to_foot+0.4*swing_foot_z_peak;
    z[2]=parameters.robot_ankle_to_foot+1.0*swing_foot_z_peak;
    z[3]=parameters.robot_ankle_to_foot+1.0*swing_foot_z_peak;
    z[4]=parameters.robot_ankle_to_foot+1.0*swing_foot_z_peak;

    tk::spline x_ref;
    x_ref.set_points(t_x,x);
    tk::spline z_ref;
    z_ref.set_points(t_z,z);

    for(int i=1; i<parameters.step_duration/parameters.Ts+1; i++){
        double t_increment=parameters.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        y_ref_swing_foot_complete->push_back(y);
        z_ref_swing_foot_complete->push_back(z_ref(t_increment));
    }
    //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameter_init.t_ssp - parameter_init.Ts) << std::endl;
    //std::cout << "x_ref at t_ssp is: " << x_ref(parameter_init.t_ssp) << std::endl;
    //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;
}
void Robot::calculate_swing_foot_piece_trajectory_heel( int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y,
    std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, 
    std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete){
    double cycle_duration;
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

    //=====================FLC parameter
    // std::vector<double> t_posz(6), t_posx(6), t_theta(7), theta(7), x(6), z(6);
    // // Calculate x
    // double step_length = x_swing_foot_end-x_swing_foot_start;

    // t_posz[0] = 0.0*cycle_duration;
    // t_posz[1] = 0.001*cycle_duration;
    // t_posz[2] = 0.2*cycle_duration;
    // t_posz[3] = 0.6*cycle_duration;
    // // t_posz[3] = cycle_duration - parameters.t_dsp;
    // // t_posz[5] = cycle_duration - parameters.t_dsp/2;
    // t_posz[4] = 0.999*cycle_duration;
    // t_posz[5] = 1*cycle_duration;

    // z[0] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // z[1] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // z[2] = parameters.robot_ankle_to_foot+1.0*swing_foot_z_peak;
    // // z[3] = parameters.robot_ankle_to_foot+0.2*swing_foot_z_peak;
    // z[3] = parameters.robot_ankle_to_foot*cos(q_f)+parameters.foot_length_back*sin(q_f);
    // z[4] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // // z[6] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // z[5] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;

    // t_posx[0] = 0.0*cycle_duration;
    // t_posx[1] = 0.001*cycle_duration;
    // t_posx[2] = 0.4*cycle_duration;
    // // t_posx[3] = cycle_duration - parameters.t_dsp;
    // t_posx[3] = cycle_duration - parameters.t_dsp/2;
    // t_posx[4] = 0.999*cycle_duration;
    // t_posx[5] = 1*cycle_duration;

    // //if(x_swing_foot_start ! = x_swing_foot_end){

    //z[4] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;

    // x[0]                      = x_swing_foot_start;
    // x[1]                      = x_swing_foot_start;
    // // x[2]                      = x_swing_foot_start + swing_foot_x_peak;
    // x[2]                      = x_swing_foot_end - parameters.robot_ankle_to_foot*sin(q_f)-parameters.foot_length_back*(1-cos(q_f));
    // x[3]                      = x_swing_foot_end;
    // x[4]                      = x_swing_foot_end;
    // x[5]                      = x_swing_foot_end;
    // ===============================================
    // std::cout << "x peadk: " << x[2] << ", x angle: " << x[3] << ", x end: " << x[4] << std::endl;
    t_theta[0] = 0.0*cycle_duration;
    t_theta[1] = 0.001*cycle_duration;
    t_theta[2] = cycle_duration - parameters.t_dsp;
    t_theta[3] = cycle_duration - parameters.t_dsp/2;
    t_theta[4] = cycle_duration - parameters.t_dsp/2+0.001;
    t_theta[5] = 0.999*cycle_duration;
    t_theta[6] = 1*cycle_duration;

    theta[0] = 0;
    theta[1] = 0;
    theta[2] = -q_f;
    theta[3] = 0;
    theta[4] = 0;
    theta[5] = 0;
    theta[6] = 0;


    tk::spline x_ref;
    x_ref.set_points(t_posx,x);
    tk::spline z_ref;
    z_ref.set_points(t_posz,z);
    tk::spline theta_ref;
    theta_ref.set_points(t_theta,theta);

    for(int i=1; i<parameters.step_duration/parameters.Ts+1; i++){
        double t_increment=parameters.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        y_ref_swing_foot_complete->push_back(y);
        if (z_ref(t_increment) >= z[0]){
            z_ref_swing_foot_complete->push_back(z_ref(t_increment));
        }
        else{
            z_ref_swing_foot_complete->push_back(z[0]);
        }
        theta_ref_swing_foot_complete->push_back(theta_ref(t_increment));
    }

    //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameters.t_ssp - parameters.Ts) << std::endl;
    //std::cout << "x_ref at t_ssp is: " << x_ref(parameters.t_ssp) << std::endl;
    //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;
}

// Calculate complete swing foot trajectory
void Robot::calculate_swing_foot_complete_trajectory_heel_side(std::vector<double> p_ref_x, std::vector<double> p_ref_y,  
    std::vector<double> * x_ref_swing_foot_complete, 
    std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete,
    int leftside, int leftFootStart, int step_amount, double step_width){
    double y_swing_foot_start, y_swing_foot_end, x_follow;
    double amount_step_samples  = parameters.step_duration/parameters.Ts;
    double swing_foot_z_peak    = 0.06;
    double swing_foot_y_peak    = step_width;

    double amount_first_step_samples = ceil(amount_step_samples*parameters.dsp_percentage/100);
    double amount_last_step_samples = floor(amount_step_samples-amount_step_samples*parameters.dsp_percentage/100);
    for (int i = 0; i < parameters.steps_real+1; i++){

        if ((i > 1) && (i < parameters.steps_real - 1)){
            y_swing_foot_start = p_ref_y.at((i-1)*amount_step_samples-1 + amount_first_step_samples);
            y_swing_foot_end   = p_ref_y.at((i)*amount_step_samples + amount_first_step_samples );
            x_follow           = p_ref_x.at((i-1)*amount_step_samples);
            calculate_swing_foot_piece_trajectory_heel_side(0, y_swing_foot_start, y_swing_foot_end, swing_foot_z_peak, swing_foot_y_peak, x_follow, 
                x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete,leftside);
        }
        else if (i == 1){
            // y_swing_foot_start  = p_ref_y.at(0) + parameters.robot_width ;
            y_swing_foot_start  = -p_ref_y.at(0);
            y_swing_foot_end    = p_ref_y.at(1*amount_step_samples + amount_first_step_samples);
            x_follow            = -p_ref_x.at(0);
            calculate_swing_foot_piece_trajectory_heel_side(0, y_swing_foot_start, y_swing_foot_end, swing_foot_z_peak, swing_foot_y_peak/2, x_follow, 
                x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete,leftside);
        }
        else if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_first_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_first_step_samples, -p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_first_step_samples, parameters.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_first_step_samples, 0);

        }
        else if (i == parameters.steps_real-1){
            y_swing_foot_start = p_ref_y.at((i-1)*amount_step_samples - 1 + amount_first_step_samples);
            y_swing_foot_end   = p_ref_y.at(i*amount_step_samples +1 + amount_first_step_samples);
            x_follow           = -p_ref_x.at((i-1)*amount_step_samples + amount_first_step_samples);
            calculate_swing_foot_piece_trajectory_heel_side(0, y_swing_foot_start, y_swing_foot_end, swing_foot_z_peak, swing_foot_y_peak/2, x_follow, 
                x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete,leftside);

        }
        else if (i == parameters.steps_real){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_last_step_samples, x_ref_swing_foot_complete->back());
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_last_step_samples, p_ref_y.at((i - 1)*amount_step_samples - 1 + amount_first_step_samples));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_last_step_samples, parameters.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_last_step_samples, 0);

        }
    }    
}


void Robot::calculate_swing_foot_complete_trajectory_heel(std::vector<double> p_ref_x, std::vector<double> p_ref_y,  
    std::vector<double> * x_ref_swing_foot_complete, 
    std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete){
    double x_swing_foot_start, x_swing_foot_end, y_follow;
    double amount_step_samples  = parameters.step_duration/parameters.Ts;
    double swing_foot_z_peak    = 0.04;
    double swing_foot_x_peak    = parameters.step_length;

    double amount_first_step_samples = ceil(amount_step_samples*parameters.dsp_percentage/100);
    double amount_last_step_samples = floor(amount_step_samples-amount_step_samples*parameters.dsp_percentage/100);
    for (int i = 0; i < parameters.steps_real+1; i++){

        if ((i > 1) && (i < parameters.steps_real - 1)){


            x_swing_foot_start = p_ref_x.at((i-1)*amount_step_samples-1 + amount_first_step_samples);
            x_swing_foot_end   = p_ref_x.at((i)*amount_step_samples + amount_first_step_samples );
            y_follow           = p_ref_y.at((i-1)*amount_step_samples);
            calculate_swing_foot_piece_trajectory_heel(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_follow, 
                x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete);
        }
        else if (i == 1){
            x_swing_foot_start  = p_ref_x.at(0);
            x_swing_foot_end    = p_ref_x.at(1*amount_step_samples + amount_first_step_samples);
            y_follow            = -p_ref_y.at(0);
            calculate_swing_foot_piece_trajectory_heel(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak/2, y_follow, 
                x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete);

        }
        else if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_first_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_first_step_samples, -p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_first_step_samples, parameters.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_first_step_samples, 0);

        }
        else if (i == parameters.steps_real-1){
            x_swing_foot_start = p_ref_x.at((i-1)*amount_step_samples-1 + amount_first_step_samples);
            x_swing_foot_end   = p_ref_x.at(i*amount_step_samples-1 + amount_first_step_samples);
            y_follow           = -p_ref_y.at((i-1)*amount_step_samples + amount_first_step_samples);
            calculate_swing_foot_piece_trajectory_heel(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak/2, y_follow, 
                x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete);

        }
        else if (i == parameters.steps_real){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_last_step_samples, x_ref_swing_foot_complete->back());
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_last_step_samples, -y_ref_swing_foot_complete->back());
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_last_step_samples, parameters.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_last_step_samples, 0);

        }

    }
}


void Robot::calculate_swing_foot_piece_trajectory_heel_circling(int flag, double swing_foot_z_peak, double x_swing_foot_start, double x_swing_foot_end,  double swing_foot_x_peak,
    double y_swing_foot_start, double y_swing_foot_end, double swing_foot_y_peak,
    double psi_swing_foot_start, double psi_swing_foot_end, double swing_foot_psi_peak,
    std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete,
    std::vector<double> * theta_ref_swing_foot_complete, std::vector<double> * psi_ref_swing_foot_complete)
{


    double cycle_duration;
    if(flag == 1){
        cycle_duration = parameters.t_ssp;
    }
    else{
        cycle_duration = parameters.step_duration;
    }
    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory
    double q_f = 0*pi/180;

    std::vector<double> t_posz(8), t_posy(6),t_posx(6), t_theta(7), theta(7), x(6), y(6), z(8);
    std::vector<double> t_psi(7), psi(7);
    // Calculate x

    double t0,t1,t2,t3,t4,t5,t6,t7,zk0,zk1,zk2,zk3,zk4,zk5,zk6, zk7;
    double yk0,yk1,yk2,yk3,yk4,yk5,yk6;
    double xk0,xk1,xk2,xk3,xk4,xk5,xk6;
    double pk0,pk1,pk2,pk3,pk4,pk5,pk6;

    t0 = 0;
    t1 = 0.01;
    t2 = 0.2;
    t3 = 0.3;
    t4 = 0.5;
    t5 = 0.7;
    t6 = 0.99;
    t7 = 1;

    zk0 = 0;
    zk1 = 0;
    zk2 = 0.3;
    zk3 = 0.6;
    zk4 = 0.1;
    zk5 = 0.01;
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





    //double swinging_distance = y_swing_foot_end - y_swing_foot_start;
    t_posx[0] = t0*cycle_duration;
    t_posx[1] = (t3-0.1)*cycle_duration;
    t_posx[2] = t3*cycle_duration;
    t_posx[3] = t4*cycle_duration;
    t_posx[4] = t5*cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posx[5] = t6*cycle_duration;//cycle_duration - parameters.t_dsp/2;

    xk0 = 0;
    xk1 = 0.4;
    xk2 = 0.9;
    xk3 = 1;
    xk4 = 1;
    xk5 = 1;

    x[0]                      = x_swing_foot_start + swing_foot_x_peak * xk0;
    x[1]                      = x_swing_foot_start + swing_foot_x_peak * xk1;
    x[2]                      = x_swing_foot_start + swing_foot_x_peak * xk2;
    x[3]                      = x_swing_foot_start + swing_foot_x_peak * xk3;
    x[4]                      = x_swing_foot_start + swing_foot_x_peak * xk4;
    x[5]                      = x_swing_foot_start + swing_foot_x_peak * xk5;

    //double swinging_distance = y_swing_foot_end - y_swing_foot_start;
    t_posy[0] = t0*cycle_duration;
    t_posy[1] = (t3-0.1)*cycle_duration;
    t_posy[2] = t3*cycle_duration;
    t_posy[3] = t4*cycle_duration;
    t_posy[4] = t5*cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posy[5] = t6*cycle_duration;//cycle_duration - parameters.t_dsp/2;

    yk0 = 0;
    yk1 = 0.4;
    yk2 = 0.9;
    yk3 = 1;
    yk4 = 1;
    yk5 = 1;

    y[0]                      = y_swing_foot_start + swing_foot_y_peak * yk0;
    y[1]                      = y_swing_foot_start + swing_foot_y_peak * yk1;
    y[2]                      = y_swing_foot_start + swing_foot_y_peak * yk2;
    y[3]                      = y_swing_foot_start + swing_foot_y_peak * yk3;
    y[4]                      = y_swing_foot_start + swing_foot_y_peak * yk4;
    y[5]                      = y_swing_foot_start + swing_foot_y_peak * yk5;

    t_theta[0] = 0.0*cycle_duration;
    t_theta[1] = 0.001*cycle_duration;
    t_theta[2] = cycle_duration - parameters.t_dsp;
    t_theta[3] = cycle_duration - parameters.t_dsp/2;
    t_theta[4] = cycle_duration - parameters.t_dsp/2+0.001;
    t_theta[5] = 0.999*cycle_duration;
    t_theta[6] = 1*cycle_duration;

    theta[0] = 0;
    theta[1] = 0;
    theta[2] = -q_f;
    theta[3] = 0;
    theta[4] = 0;
    theta[5] = 0;
    theta[6] = 0;

    t_psi[0] = 0.0  * cycle_duration;
    t_psi[1] = 0.01 * cycle_duration;
    t_psi[2] = 0.5  * cycle_duration;
    t_psi[3] = 0.8  * cycle_duration;
    t_psi[4] = 0.9 * cycle_duration;
    t_psi[5] = 0.99  * cycle_duration;
    t_psi[6] = 1.0  * cycle_duration;

    pk0 = 0;
    pk1 = 0.0;
    pk2 = 0.5;
    pk3 = 0.6;
    pk4 = 0.8;;
    pk5 = 1;
    pk6 = 1;
    
    psi[0] = psi_swing_foot_start + swing_foot_psi_peak * pk0;
    psi[1] = psi_swing_foot_start + swing_foot_psi_peak * pk1;
    psi[2] = psi_swing_foot_start + swing_foot_psi_peak * pk2;
    psi[3] = psi_swing_foot_start + swing_foot_psi_peak * pk3;
    psi[4] = psi_swing_foot_start + swing_foot_psi_peak * pk4;
    psi[5] = psi_swing_foot_start + swing_foot_psi_peak * pk5;
    psi[6] = psi_swing_foot_start + swing_foot_psi_peak * pk6;
    
    tk::spline x_ref;
    x_ref.set_points(t_posx,x);
    tk::spline y_ref;
    y_ref.set_points(t_posy,y);
    tk::spline z_ref;
    z_ref.set_points(t_posz,z);
    tk::spline theta_ref;
    theta_ref.set_points(t_theta,theta);

    tk::spline psi_ref;
    psi_ref.set_points(t_psi,psi);

    for(int i=1; i<parameters.step_duration/parameters.Ts+1; i++){
        double t_increment=parameters.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        y_ref_swing_foot_complete->push_back(y_ref(t_increment));
        x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        if (z_ref(t_increment) >= z[0]){
            z_ref_swing_foot_complete->push_back(z_ref(t_increment));
        }
        else{
            z_ref_swing_foot_complete->push_back(z[0]);
        }
        theta_ref_swing_foot_complete->push_back(theta_ref(t_increment));
        psi_ref_swing_foot_complete->push_back(psi_ref(t_increment));
    }
    
}
void Robot::calculate_swing_foot_piece_trajectory_heel_straight( int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y,
    std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, 
    std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete,
    int forward
    ){

    double cycle_duration;
    if(flag == 1){
        cycle_duration = parameters.t_ssp;
    }
    else{
        cycle_duration = parameters.step_duration;
    }
    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory
    double q_f = 0*pi/180;

    std::vector<double> t_posz(9), t_posx(14), t_theta(7), theta(7), x(14), z(9);
    // Calculate x
    double step_length = x_swing_foot_end-x_swing_foot_start;
    double t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,zk0,zk1,zk2,zk3,zk4,zk5,zk6, zk7,zk8;
    double xk0,xk1,xk2,xk3,xk4,xk5,xk6;


//====================
    t0 = 0;
    t1 = 0.01;
    t2 = 0.1;
    t3 = 0.35;
    t4 = 0.45;
    t5 = 0.6;
    t6 = 0.7;
    t7 = 0.99;
    t8 = 1;
    t9 = 1;

    zk0 = 0;
    zk1 = 0;
    zk2 = 0.3;
    zk3 = 1;
    zk4 = 0.6;
    zk5 = 0.2;
    zk6 = 0;
    zk7 = 0;
    zk8 = 0;


    t_posz[0] = t0*cycle_duration;
    t_posz[1] = t1*cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posz[2] = t2*cycle_duration;
    t_posz[3] = t3*cycle_duration;
    t_posz[4] = t4*cycle_duration;
    t_posz[5] = t5*cycle_duration;
    t_posz[6] = t6*cycle_duration;
    t_posz[7] = t7*cycle_duration;
    t_posz[8] = t8*cycle_duration;

    z[0] = parameters.robot_ankle_to_foot+zk0 * swing_foot_z_peak;
    z[1] = parameters.robot_ankle_to_foot+zk1 * swing_foot_z_peak;
    z[2] = parameters.robot_ankle_to_foot+zk2 * swing_foot_z_peak;
    z[3] = parameters.robot_ankle_to_foot+zk3 * swing_foot_z_peak;
    z[4] = parameters.robot_ankle_to_foot+zk4 * swing_foot_z_peak;
    z[5] = parameters.robot_ankle_to_foot+zk5 * swing_foot_z_peak;
    z[6] = parameters.robot_ankle_to_foot+zk6 * swing_foot_z_peak;
    z[7] = parameters.robot_ankle_to_foot+zk7 * swing_foot_z_peak;
    z[8] = parameters.robot_ankle_to_foot+zk8 * swing_foot_z_peak;


    t_posx[0] = t0*cycle_duration;
    t_posx[1] = 0.01*cycle_duration;
    t_posx[2] = 0.02*cycle_duration;
    t_posx[3] = 0.03*cycle_duration;
    t_posx[4] = 0.04*cycle_duration;   
    t_posx[5] = (t3-0.1)*cycle_duration;
    t_posx[6] = t3*cycle_duration;
    t_posx[7] = t4*cycle_duration;
    t_posx[8] = 0.55*cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posx[9] = 0.65*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    t_posx[10] = 0.75*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    t_posx[11] = 0.85*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    t_posx[12] = 0.99*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    t_posx[13] = 1*cycle_duration;//cycle_duration - parameters.t_dsp/2;


    double swinging_distance = x_swing_foot_end - x_swing_foot_start;

    xk0 = 0;
    xk1 = 0.3;
    xk2 = 0.8;
    xk3 = 1;
    xk4 = 1;
    xk5 = 1;

    x[0]                      = x_swing_foot_start + swinging_distance * xk0;
    x[1]                      = x_swing_foot_start + swinging_distance * xk0;
    x[2]                      = x_swing_foot_start + swinging_distance * xk0;
    x[3]                      = x_swing_foot_start + swinging_distance * xk0;
    x[4]                      = x_swing_foot_start + swinging_distance * xk0;
    x[5]                      = x_swing_foot_start + swinging_distance * xk1;
    x[6]                      = x_swing_foot_start + swinging_distance * xk2;
    x[7]                      = x_swing_foot_start + swinging_distance * xk3;
    x[8]                      = x_swing_foot_start + swinging_distance * xk4;
    x[9]                      = x_swing_foot_start + swinging_distance * xk5;
    x[10]                      = x_swing_foot_start + swinging_distance * xk5;
    x[11]                      = x_swing_foot_start + swinging_distance * xk5;
    x[12]                      = x_swing_foot_start + swinging_distance * xk5;
    x[13]                      = x_swing_foot_start + swinging_distance * xk5;


    //===============

    // t0 = 0;
    // t1 = 0.01;
    // t2 = 0.3;
    // t3 = 0.45;
    // t4 = 0.6;
    // t5 = 0.8;
    // t6 = 0.99;
    // t7 = 1;

    // zk0 = 0;
    // zk1 = 0;
    // zk2 = 1;
    // zk3 = 0.2;
    // zk4 = 0;
    // zk5 = 0;
    // zk6 = 0;
    // zk7 = 0;


    // t_posz[0] = t0*cycle_duration;
    // t_posz[1] = t1*cycle_duration;//cycle_duration - parameters.t_dsp;
    // t_posz[2] = t2*cycle_duration;
    // t_posz[3] = t3*cycle_duration;
    // t_posz[4] = t4*cycle_duration;
    // t_posz[5] = t5*cycle_duration;
    // t_posz[6] = t6*cycle_duration;
    // t_posz[7] = t7*cycle_duration;

    // z[0] = parameters.robot_ankle_to_foot+zk0 * swing_foot_z_peak;
    // z[1] = parameters.robot_ankle_to_foot+zk1 * swing_foot_z_peak;
    // z[2] = parameters.robot_ankle_to_foot+zk2 * swing_foot_z_peak;
    // z[3] = parameters.robot_ankle_to_foot+zk3 * swing_foot_z_peak;
    // z[4] = parameters.robot_ankle_to_foot+zk4 * swing_foot_z_peak;
    // z[5] = parameters.robot_ankle_to_foot+zk5 * swing_foot_z_peak;
    // z[6] = parameters.robot_ankle_to_foot+zk6 * swing_foot_z_peak;
    // z[7] = parameters.robot_ankle_to_foot+zk7 * swing_foot_z_peak;

    // // t_posx[0] = 0.0*cycle_duration;
    // // t_posx[1] = 0.4*cycle_duration;
    // // t_posx[2] = 0.6*cycle_duration;
    // // t_posx[3] = 0.3*cycle_duration;//cycle_duration - parameters.t_dsp;
    // // t_posx[4] = 0.7*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    // // t_posx[5] = 0.99*cycle_duration;
    // // t_posx[6] = 1*cycle_duration;
    // t_posx[0] = t0*cycle_duration;
    // // t_posx[1] = (t3-0.2)*cycle_duration;
    // t_posx[1] = (t3-0.1)*cycle_duration;
    // t_posx[2] = t3*cycle_duration;
    // t_posx[3] = t4*cycle_duration;
    // t_posx[4] = t5*cycle_duration;//cycle_duration - parameters.t_dsp;
    // t_posx[5] = t6*cycle_duration;//cycle_duration - parameters.t_dsp/2;


    // double swinging_distance = x_swing_foot_end - x_swing_foot_start;

    // //if(x_swing_foot_start ! = x_swing_foot_end){
    // // x[0]                      = x_swing_foot_start;
    // // x[1]                      = x_swing_foot_start;
    // // x[2]                      = x_swing_foot_start + swing_foot_x_peak;
    // // x[3]                      = x_swing_foot_start + swinging_distance * 0.5;//x_swing_foot_end * 0.8 - parameters.robot_ankle_to_foot*sin(q_f)-parameters.foot_length_back*(1-cos(q_f));
    // // x[4]                      = x_swing_foot_start + swinging_distance * 0.8;
    // // x[5]                      = x_swing_foot_end;
    // // x[6]                      = x_swing_foot_end;

    // xk0 = 0;
    // xk1 = 1;
    // xk2 = 1;
    // xk3 = 1;
    // xk4 = 1;
    // xk5 = 1;

    // x[0]                      = x_swing_foot_start + swinging_distance * xk0;
    // x[1]                      = x_swing_foot_start + swinging_distance * xk1;
    // x[2]                      = x_swing_foot_start + swinging_distance * xk2;
    // x[3]                      = x_swing_foot_start + swinging_distance * xk3;
    // x[4]                      = x_swing_foot_start + swinging_distance * xk4;
    // x[5]                      = x_swing_foot_start + swinging_distance * xk5;

    //=====================FLC parameter
    // std::vector<double> t_posz(6), t_posx(6), t_theta(7), theta(7), x(6), z(6);
    // // Calculate x
    // double step_length = x_swing_foot_end-x_swing_foot_start;

    // t_posz[0] = 0.0*cycle_duration;
    // t_posz[1] = 0.001*cycle_duration;
    // t_posz[2] = 0.2*cycle_duration;
    // t_posz[3] = 0.6*cycle_duration;
    // // t_posz[3] = cycle_duration - parameters.t_dsp;
    // // t_posz[5] = cycle_duration - parameters.t_dsp/2;
    // t_posz[4] = 0.999*cycle_duration;
    // t_posz[5] = 1*cycle_duration;

    // z[0] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // z[1] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // z[2] = parameters.robot_ankle_to_foot+1.0*swing_foot_z_peak;
    // // z[3] = parameters.robot_ankle_to_foot+0.2*swing_foot_z_peak;
    // z[3] = parameters.robot_ankle_to_foot*cos(q_f)+parameters.foot_length_back*sin(q_f);
    // z[4] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // // z[6] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // z[5] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;

    // t_posx[0] = 0.0*cycle_duration;
    // t_posx[1] = 0.001*cycle_duration;
    // t_posx[2] = 0.4*cycle_duration;
    // // t_posx[3] = cycle_duration - parameters.t_dsp;
    // t_posx[3] = cycle_duration - parameters.t_dsp/2;
    // t_posx[4] = 0.999*cycle_duration;
    // t_posx[5] = 1*cycle_duration;

    // //if(x_swing_foot_start ! = x_swing_foot_end){

    //z[4] = parameters.robot_ankle_to_foot+0.0*swing_foot_z_peak;

    // x[0]                      = x_swing_foot_start;
    // x[1]                      = x_swing_foot_start;
    // // x[2]                      = x_swing_foot_start + swing_foot_x_peak;
    // x[2]                      = x_swing_foot_end - parameters.robot_ankle_to_foot*sin(q_f)-parameters.foot_length_back*(1-cos(q_f));
    // x[3]                      = x_swing_foot_end;
    // x[4]                      = x_swing_foot_end;
    // x[5]                      = x_swing_foot_end;
    // ===============================================
    // std::cout << "x peadk: " << x[2] << ", x angle: " << x[3] << ", x end: " << x[4] << std::endl;
    t_theta[0] = 0.0*cycle_duration;
    t_theta[1] = 0.001*cycle_duration;
    t_theta[2] = cycle_duration - parameters.t_dsp;
    t_theta[3] = cycle_duration - parameters.t_dsp/2;
    t_theta[4] = cycle_duration - parameters.t_dsp/2+0.001;
    t_theta[5] = 0.999*cycle_duration;
    t_theta[6] = 1*cycle_duration;

    theta[0] = 0;
    theta[1] = 0;
    theta[2] = -q_f;
    theta[3] = 0;
    theta[4] = 0;
    theta[5] = 0;
    theta[6] = 0;


    tk::spline x_ref;
    x_ref.set_points(t_posx,x);
    tk::spline z_ref;
    z_ref.set_points(t_posz,z);
    tk::spline theta_ref;
    theta_ref.set_points(t_theta,theta);

    for(int i=1; i<parameters.step_duration/parameters.Ts+1; i++){
        double t_increment=parameters.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        y_ref_swing_foot_complete->push_back(y);
        if (z_ref(t_increment) >= z[0]){
            z_ref_swing_foot_complete->push_back(z_ref(t_increment));
        }
        else{
            z_ref_swing_foot_complete->push_back(z[0]);
        }
        theta_ref_swing_foot_complete->push_back(theta_ref(t_increment));
    }

    //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameters.t_ssp - parameters.Ts) << std::endl;
    //std::cout << "x_ref at t_ssp is: " << x_ref(parameters.t_ssp) << std::endl;
    //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;
}

void Robot::calculate_swing_foot_piece_trajectory_heel_side( int flag, double y_swing_foot_start, double y_swing_foot_end, double swing_foot_z_peak, double swing_foot_y_peak, double x_follow,
    std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, 
    std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete,
    int leftside
    ){

    double cycle_duration;
    if(flag == 1){
        cycle_duration = parameters.t_ssp;
    }
    else{
        cycle_duration = parameters.step_duration;
    }
    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory
    double q_f = 0*pi/180;

    std::vector<double> t_posz(9), t_posy(14), t_theta(7), theta(7), y(14), z(9);
    // Calculate x
    double step_width = y_swing_foot_end-y_swing_foot_start;
    double t0,t1,t2,t3,t4,t5,t6,t7,t8,zk0,zk1,zk2,zk3,zk4,zk5,zk6, zk7,zk8;
    double yk0,yk1,yk2,yk3,yk4,yk5,yk6;

    t0 = 0;
    t1 = 0.01;
    t2 = 0.1;
    t3 = 0.35;
    t4 = 0.45;
    t5 = 0.6;
    t6 = 0.8;
    t7 = 0.99;
    t8 = 1;

    zk0 = 0;
    zk1 = 0;
    zk2 = 0.3;
    zk3 = 1;
    zk4 = 0.6;
    zk5 = 0.2;
    zk6 = 0;
    zk7 = 0;
    zk8 = 0;
    ///
    // t0 = 0;
    // t1 = 0.01;
    // t2 = 0.3;
    // t3 = 0.45;
    // t4 = 0.6;
    // t5 = 0.8;
    // t6 = 0.99;
    // t7 = 1;

    // zk0 = 0;
    // zk1 = 0;
    // zk2 = 1;
    // zk3 = 0.2;
    // zk4 = 0;
    // zk5 = 0;
    // zk6 = 0;
    // zk7 = 0;


    t_posz[0] = t0*cycle_duration;
    t_posz[1] = t1*cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posz[2] = t2*cycle_duration;
    t_posz[3] = t3*cycle_duration;
    t_posz[4] = t4*cycle_duration;
    t_posz[5] = t5*cycle_duration;
    t_posz[6] = t6*cycle_duration;
    t_posz[7] = t7*cycle_duration;
    t_posz[8] = t8*cycle_duration;

    z[0] = parameters.robot_ankle_to_foot+zk0 * swing_foot_z_peak;
    z[1] = parameters.robot_ankle_to_foot+zk1 * swing_foot_z_peak;
    z[2] = parameters.robot_ankle_to_foot+zk2 * swing_foot_z_peak;
    z[3] = parameters.robot_ankle_to_foot+zk3 * swing_foot_z_peak;
    z[4] = parameters.robot_ankle_to_foot+zk4 * swing_foot_z_peak;
    z[5] = parameters.robot_ankle_to_foot+zk5 * swing_foot_z_peak;
    z[6] = parameters.robot_ankle_to_foot+zk6 * swing_foot_z_peak;
    z[7] = parameters.robot_ankle_to_foot+zk7 * swing_foot_z_peak;
    z[8] = parameters.robot_ankle_to_foot+zk8 * swing_foot_z_peak;


    // t_posy[0] = t0*cycle_duration;
    // t_posy[1] = (t3-0.1)*cycle_duration;
    // t_posy[2] = t3*cycle_duration;
    // t_posy[3] = t4*cycle_duration;
    // t_posy[4] = t5*cycle_duration;//cycle_duration - parameters.t_dsp;
    // t_posy[5] = t6*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    t_posy[0] = t0*cycle_duration;
    t_posy[1] = 0.01*cycle_duration;
    t_posy[2] = 0.02*cycle_duration;
    t_posy[3] = 0.03*cycle_duration;
    t_posy[4] = 0.04*cycle_duration;   
    t_posy[5] = (t3-0.1)*cycle_duration;
    t_posy[6] = t3*cycle_duration;
    t_posy[7] = t4*cycle_duration;
    t_posy[8] = 0.55*cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posy[9] = 0.65*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    t_posy[10] = 0.75*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    t_posy[11] = 0.85*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    t_posy[12] = 0.99*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    t_posy[13] = 1*cycle_duration;//cycle_duration - parameters.t_dsp/2;



    yk0 = 0;
    yk1 = 0.3;
    yk2 = 0.8;
    yk3 = 1;
    yk4 = 1;
    yk5 = 1;
    double swinging_distance = y_swing_foot_end - y_swing_foot_start;

    y[0]                      = y_swing_foot_start + swinging_distance * yk0;
    y[1]                      = y_swing_foot_start + swinging_distance * yk0;
    y[2]                      = y_swing_foot_start + swinging_distance * yk0;
    y[3]                      = y_swing_foot_start + swinging_distance * yk0;
    y[4]                      = y_swing_foot_start + swinging_distance * yk0;
    y[5]                      = y_swing_foot_start + swinging_distance * yk1;
    y[6]                      = y_swing_foot_start + swinging_distance * yk2;
    y[7]                      = y_swing_foot_start + swinging_distance * yk3;
    y[8]                      = y_swing_foot_start + swinging_distance * yk4;
    y[9]                      = y_swing_foot_start + swinging_distance * yk5;
    y[10]                      = y_swing_foot_start + swinging_distance * yk5;
    y[11]                      = y_swing_foot_start + swinging_distance * yk5;
    y[12]                      = y_swing_foot_start + swinging_distance * yk5;
    y[13]                      = y_swing_foot_start + swinging_distance * yk5;

    // t_posy[0] = t0*cycle_duration;
    // t_posy[1] = (t3-0.1)*cycle_duration;
    // t_posy[2] = t3*cycle_duration;
    // t_posy[3] = t5*cycle_duration;
    // t_posy[4] = t6*cycle_duration;//cycle_duration - parameters.t_dsp;
    // t_posy[5] = t7*cycle_duration;//cycle_duration - parameters.t_dsp/2;





    // yk0 = 0;
    // yk1 = 1;
    // yk2 = 1;
    // yk3 = 1;
    // yk4 = 1;
    // yk5 = 1;

    // y[0]                      = y_swing_foot_start + swinging_distance * yk0;
    // y[1]                      = y_swing_foot_start + swinging_distance * yk1;
    // y[2]                      = y_swing_foot_start + swinging_distance * yk2;
    // y[3]                      = y_swing_foot_start + swinging_distance * yk3;
    // y[4]                      = y_swing_foot_start + swinging_distance * yk4;
    // y[5]                      = y_swing_foot_start + swinging_distance * yk5;

    t_theta[0] = 0.0*cycle_duration;
    t_theta[1] = 0.001*cycle_duration;
    t_theta[2] = cycle_duration - parameters.t_dsp;
    t_theta[3] = cycle_duration - parameters.t_dsp/2;
    t_theta[4] = cycle_duration - parameters.t_dsp/2+0.001;
    t_theta[5] = 0.999*cycle_duration;
    t_theta[6] = 1*cycle_duration;

    theta[0] = 0;
    theta[1] = 0;
    theta[2] = -q_f;
    theta[3] = 0;
    theta[4] = 0;
    theta[5] = 0;
    theta[6] = 0;


    tk::spline y_ref;
    y_ref.set_points(t_posy,y);
    tk::spline z_ref;
    z_ref.set_points(t_posz,z);
    tk::spline theta_ref;
    theta_ref.set_points(t_theta,theta);

    for(int i=1; i<parameters.step_duration/parameters.Ts+1; i++){
        double t_increment=parameters.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        y_ref_swing_foot_complete->push_back(y_ref(t_increment));
        x_ref_swing_foot_complete->push_back(x_follow);
        if (z_ref(t_increment) >= z[0]){
            z_ref_swing_foot_complete->push_back(z_ref(t_increment));
        }
        else{
            z_ref_swing_foot_complete->push_back(z[0]);
        }
        theta_ref_swing_foot_complete->push_back(theta_ref(t_increment));
    }

}
void Robot::calculate_swing_foot_piece_trajectory_heel_curve( int flag, 
    double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y_swing_foot_start, double y_swing_foot_end,
    double psi_start, double psi_stop,
    std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, 
    std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete, std::vector<double> * psi_ref_swing_foot_complete,
    int forward
    ){

    double cycle_duration;
    if(flag == 1){
        cycle_duration = parameters.t_ssp;
    }
    else{
        cycle_duration = parameters.step_duration;
    }
    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory
    double q_f = 0*pi/180;

    std::vector<double> t_posz(8), t_posx(5),t_posy(6), t_theta(7), theta(7), x(5), z(8), y(6);
    std::vector<double> t_psi(8),psi(8);
    // Calculate x
    double step_length = x_swing_foot_end-x_swing_foot_start;
    double t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,zk0,zk1,zk2,zk3,zk4,zk5,zk6, zk7,zk8;
    double xk0,xk1,xk2,xk3,xk4,xk5,xk6,yk0,yk1,yk2,yk3,yk4,yk5,yk6;
    t0 = 0;
    t1 = 0.01;
    t2 = 0.1;
    t3 = 0.25;
    t4 = 0.35;
    t5 = 0.5;
    t6 = 0.6;
    t7 = 0.99;
    t8 = 1;
    t9 = 1;

    zk0 = 0;
    zk1 = 0;
    zk2 = 0.3;
    zk3 = 1;
    zk4 = 0.6;
    zk5 = 0.15;
    zk6 = 0;
    zk7 = 0;
    zk8 = 0;


    t_posz[0] = t0*cycle_duration;
    t_posz[1] = t1*cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posz[2] = t2*cycle_duration;
    t_posz[3] = t3*cycle_duration;
    t_posz[4] = t4*cycle_duration;
    t_posz[5] = t5*cycle_duration;
    t_posz[6] = t6*cycle_duration;
    t_posz[7] = t7*cycle_duration;
    // t_posz[8] = t8*cycle_duration;

    z[0] = parameters.robot_ankle_to_foot+zk0 * swing_foot_z_peak;
    z[1] = parameters.robot_ankle_to_foot+zk1 * swing_foot_z_peak;
    z[2] = parameters.robot_ankle_to_foot+zk2 * swing_foot_z_peak;
    z[3] = parameters.robot_ankle_to_foot+zk3 * swing_foot_z_peak;
    z[4] = parameters.robot_ankle_to_foot+zk4 * swing_foot_z_peak;
    z[5] = parameters.robot_ankle_to_foot+zk5 * swing_foot_z_peak;
    z[6] = parameters.robot_ankle_to_foot+zk6 * swing_foot_z_peak;
    z[7] = parameters.robot_ankle_to_foot+zk7 * swing_foot_z_peak;

    // t_posz[0] = t0*cycle_duration;
    // t_posz[1] = t1*cycle_duration;//cycle_duration - parameters.t_dsp;
    // t_posz[2] = t2*cycle_duration;
    // t_posz[3] = t3*cycle_duration;
    // t_posz[4] = t4*cycle_duration;
    // t_posz[5] = t5*cycle_duration;
    // t_posz[6] = t6*cycle_duration;
    // t_posz[7] = t7*cycle_duration;
    // t_posz[8] = t8*cycle_duration;

    // z[0] = parameters.robot_ankle_to_foot+zk0 * swing_foot_z_peak;
    // z[1] = parameters.robot_ankle_to_foot+zk1 * swing_foot_z_peak;
    // z[2] = parameters.robot_ankle_to_foot+zk2 * swing_foot_z_peak;
    // z[3] = parameters.robot_ankle_to_foot+zk3 * swing_foot_z_peak;
    // z[4] = parameters.robot_ankle_to_foot+zk4 * swing_foot_z_peak;
    // z[5] = parameters.robot_ankle_to_foot+zk5 * swing_foot_z_peak;
    // z[6] = parameters.robot_ankle_to_foot+zk6 * swing_foot_z_peak;
    // z[7] = parameters.robot_ankle_to_foot+zk7 * swing_foot_z_peak;
    // z[8] = parameters.robot_ankle_to_foot+zk8 * swing_foot_z_peak;


    // t_posx[0] = t0*cycle_duration;
    // t_posx[1] = 0.01*cycle_duration;
    // t_posx[2] = 0.02*cycle_duration;
    // t_posx[3] = 0.03*cycle_duration;
    // t_posx[4] = 0.04*cycle_duration;   
    // t_posx[5] = (t3-0.1)*cycle_duration;
    // t_posx[6] = t3*cycle_duration;
    // t_posx[7] = t4*cycle_duration;
    // t_posx[8] = 0.55*cycle_duration;//cycle_duration - parameters.t_dsp;
    // t_posx[9] = 0.65*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    // t_posx[10] = 0.75*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    // t_posx[11] = 0.85*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    // t_posx[12] = 0.99*cycle_duration;//cycle_duration - parameters.t_dsp/2;
    // t_posx[13] = 1*cycle_duration;//cycle_duration - parameters.t_dsp/2;


    double swinging_distance = x_swing_foot_end - x_swing_foot_start;

    t_posx[0] = 0*cycle_duration;
    t_posx[1] = 0.01*cycle_duration;
    t_posx[2] = 0.48*cycle_duration;
    t_posx[3] = 0.49*cycle_duration;
    t_posx[4] = 0.5*cycle_duration;   


    x[0]                      = x_swing_foot_start + swinging_distance * 0;
    x[1]                      = x_swing_foot_start + swinging_distance * 0;
    x[2]                      = x_swing_foot_start + swinging_distance * 1;
    x[3]                      = x_swing_foot_start + swinging_distance * 1;
    x[4]                      = x_swing_foot_start + swinging_distance * 1;
    // xk0 = 0;
    // xk1 = 0.3;
    // xk2 = 0.8;
    // xk3 = 1;
    // xk4 = 1;
    // xk5 = 1;

    // x[0]                      = x_swing_foot_start + swinging_distance * xk0;
    // x[1]                      = x_swing_foot_start + swinging_distance * xk0;
    // x[2]                      = x_swing_foot_start + swinging_distance * xk0;
    // x[3]                      = x_swing_foot_start + swinging_distance * xk0;
    // x[4]                      = x_swing_foot_start + swinging_distance * xk0;
    // x[5]                      = x_swing_foot_start + swinging_distance * xk1;
    // x[6]                      = x_swing_foot_start + swinging_distance * xk2;
    // x[7]                      = x_swing_foot_start + swinging_distance * xk3;
    // x[8]                      = x_swing_foot_start + swinging_distance * xk4;
    // x[9]                      = x_swing_foot_start + swinging_distance * xk5;
    // x[10]                      = x_swing_foot_start + swinging_distance * xk5;
    // x[11]                      = x_swing_foot_start + swinging_distance * xk5;
    // x[12]                      = x_swing_foot_start + swinging_distance * xk5;
    // x[13]                      = x_swing_foot_start + swinging_distance * xk5;


    t_posy[0] = t0*cycle_duration;
    t_posy[1] = (t3-0.1)*cycle_duration;
    t_posy[2] = t3*cycle_duration;
    t_posy[3] = t4*cycle_duration;
    t_posy[4] = t5*cycle_duration;//cycle_duration - parameters.t_dsp;
    t_posy[5] = t6*cycle_duration;//cycle_duration - parameters.t_dsp/2;


    swinging_distance = y_swing_foot_end - y_swing_foot_start;

    yk0 = 0;
    yk1 = 0.3;
    yk2 = 0.8;
    yk3 = 1;
    yk4 = 1;
    yk5 = 1;

    y[0]                      = y_swing_foot_start + swinging_distance * yk0;
    y[1]                      = y_swing_foot_start + swinging_distance * yk1;
    y[2]                      = y_swing_foot_start + swinging_distance * yk2;
    y[3]                      = y_swing_foot_start + swinging_distance * yk3;
    y[4]                      = y_swing_foot_start + swinging_distance * yk4;
    y[5]                      = y_swing_foot_start + swinging_distance * yk5;

    t_theta[0] = 0.0*cycle_duration;
    t_theta[1] = 0.001*cycle_duration;
    t_theta[2] = cycle_duration - parameters.t_dsp;
    t_theta[3] = cycle_duration - parameters.t_dsp/2;
    t_theta[4] = cycle_duration - parameters.t_dsp/2+0.001;
    t_theta[5] = 0.999*cycle_duration;
    t_theta[6] = 1*cycle_duration;

    theta[0] = 0;
    theta[1] = 0;
    theta[2] = -q_f;
    theta[3] = 0;
    theta[4] = 0;
    theta[5] = 0;
    theta[6] = 0;
    double delta_psi = psi_stop - psi_start;
    
    t_psi[0] = 0.0  * cycle_duration;
    t_psi[1] = 0.01 * cycle_duration;
    t_psi[2] = 0.1  * cycle_duration;
    t_psi[3] = 0.4 * cycle_duration;
    t_psi[4] = 0.65 * cycle_duration;
    t_psi[5] = 0.9 * cycle_duration;
    t_psi[6] = 0.99 * cycle_duration;
    t_psi[7] = 1.00 * cycle_duration;

        psi[0] = psi_start + 0.00 * delta_psi;
        psi[1] = psi_start + 0.00 * delta_psi;;
        psi[2] = psi_start + 0.01 * delta_psi;
        psi[3] = psi_start + 0.6 * delta_psi;;
        psi[4] = psi_start + 0.8 * delta_psi;;
        psi[5] = psi_start + 0.99 * delta_psi;
        psi[6] = psi_start + 1.00 * delta_psi;
        psi[7] = psi_start + 1.00 * delta_psi;       


    //double psi_increment = (psi_stop - psi_start)/(parameters.step_duration/parameters.Ts);
    

    tk::spline x_ref;
    x_ref.set_points(t_posx,x);    
    tk::spline y_ref;
    y_ref.set_points(t_posy,y);
    tk::spline z_ref;
    z_ref.set_points(t_posz,z);
    tk::spline theta_ref;
    theta_ref.set_points(t_theta,theta);

    tk::spline psi_ref;
    psi_ref.set_points(t_psi,psi);
    double t_increment;
    // std::cout << "x(0): " << x_ref(0) << ", x(end): " << x_ref(cycle_duration) << std::endl;
    // std::cout << "x(1): " << x_ref(1) << ", x(end): " << x_ref(parameters.step_duration) << std::endl;
    for(int i=0; i<parameters.step_duration/parameters.Ts; i++){

         t_increment=parameters.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        // x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        y_ref_swing_foot_complete->push_back(y_ref(t_increment));
        // if (z_ref(t_increment) >= z[0]){
        //     z_ref_swing_foot_complete->push_back(z_ref(t_increment));
        // }
        // else{
        //     z_ref_swing_foot_complete->push_back(z[0]);
        // }
        theta_ref_swing_foot_complete->push_back(theta_ref(t_increment));
        //psi_ref_swing_foot_complete->push_back(psi_start+psi_increment*(i-1));

        psi_ref_swing_foot_complete->push_back(psi_ref(t_increment));


    }

    for(int i=0; i<parameters.step_duration/parameters.Ts*0.6; i++){

        t_increment=parameters.Ts*i;
     
        
        if (z_ref(t_increment) >= z[0]){
            z_ref_swing_foot_complete->push_back(z_ref(t_increment));
        }
        else{
            z_ref_swing_foot_complete->push_back(z[0]);
        }
        

    }

    for(int i=0; i<parameters.step_duration/parameters.Ts*0.4; i++){
        z_ref_swing_foot_complete->push_back(z[0]);
        
    }

    for(int i=0; i<parameters.step_duration/parameters.Ts*0.5; i++){

        t_increment=parameters.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;
        if (x_ref(t_increment) <= x[4]){
            x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        }
        else{
            x_ref_swing_foot_complete->push_back(x[4]);   
        }
      
    }
    for(int i=0; i<parameters.step_duration/parameters.Ts*0.5; i++){
        x_ref_swing_foot_complete->push_back(x[4]);         
    }
    //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameters.t_ssp - parameters.Ts) << std::endl;
    //std::cout << "x_ref at t_ssp is: " << x_ref(parameters.t_ssp) << std::endl;
    //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;
}
// void Robot::calculate_swing_foot_piece_trajectory_heel_curve( int flag, 
//     double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y_swing_foot_start, double y_swing_foot_end,
//     double psi_start, double psi_stop,
//     std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, 
//     std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete, std::vector<double> * psi_ref_swing_foot_complete,
//     int forward
//     ){

//     double cycle_duration;
//     if(flag == 1){
//         cycle_duration = parameters.t_ssp;
//     }
//     else{
//         cycle_duration = parameters.step_duration;
//     }
//     // This function appends the swing_foot_piece to the complete trajectory
//     // First the SSP trajectory
//     double q_f = 0*pi/180;

//     std::vector<double> t_posz(9), t_posx(14),t_posy(6), t_theta(7), theta(7), x(14), z(9), y(6);
//     std::vector<double> t_psi(8),psi(8);
//     // Calculate x
//     double step_length = x_swing_foot_end-x_swing_foot_start;
//     double t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,zk0,zk1,zk2,zk3,zk4,zk5,zk6, zk7,zk8;
//     double xk0,xk1,xk2,xk3,xk4,xk5,xk6,yk0,yk1,yk2,yk3,yk4,yk5,yk6;
//     t0 = 0;
//     t1 = 0.01;
//     t2 = 0.1;
//     t3 = 0.25;
//     t4 = 0.35;
//     t5 = 0.5;
//     t6 = 0.6;
//     t7 = 0.99;
//     t8 = 1;
//     t9 = 1;

//     zk0 = 0;
//     zk1 = 0;
//     zk2 = 0.3;
//     zk3 = 1;
//     zk4 = 0.6;
//     zk5 = 0.2;
//     zk6 = 0;
//     zk7 = 0;
//     zk8 = 0;


//     t_posz[0] = t0*cycle_duration;
//     t_posz[1] = t1*cycle_duration;//cycle_duration - parameters.t_dsp;
//     t_posz[2] = t2*cycle_duration;
//     t_posz[3] = t3*cycle_duration;
//     t_posz[4] = t4*cycle_duration;
//     t_posz[5] = t5*cycle_duration;
//     t_posz[6] = t6*cycle_duration;
//     t_posz[7] = t7*cycle_duration;
//     t_posz[8] = t8*cycle_duration;

//     z[0] = parameters.robot_ankle_to_foot+zk0 * swing_foot_z_peak;
//     z[1] = parameters.robot_ankle_to_foot+zk1 * swing_foot_z_peak;
//     z[2] = parameters.robot_ankle_to_foot+zk2 * swing_foot_z_peak;
//     z[3] = parameters.robot_ankle_to_foot+zk3 * swing_foot_z_peak;
//     z[4] = parameters.robot_ankle_to_foot+zk4 * swing_foot_z_peak;
//     z[5] = parameters.robot_ankle_to_foot+zk5 * swing_foot_z_peak;
//     z[6] = parameters.robot_ankle_to_foot+zk6 * swing_foot_z_peak;
//     z[7] = parameters.robot_ankle_to_foot+zk7 * swing_foot_z_peak;
//     z[8] = parameters.robot_ankle_to_foot+zk8 * swing_foot_z_peak;


//     t_posx[0] = t0*cycle_duration;
//     t_posx[1] = 0.01*cycle_duration;
//     t_posx[2] = 0.02*cycle_duration;
//     t_posx[3] = 0.03*cycle_duration;
//     t_posx[4] = 0.04*cycle_duration;   
//     t_posx[5] = (t3-0.1)*cycle_duration;
//     t_posx[6] = t3*cycle_duration;
//     t_posx[7] = t4*cycle_duration;
//     t_posx[8] = 0.55*cycle_duration;//cycle_duration - parameters.t_dsp;
//     t_posx[9] = 0.65*cycle_duration;//cycle_duration - parameters.t_dsp/2;
//     t_posx[10] = 0.75*cycle_duration;//cycle_duration - parameters.t_dsp/2;
//     t_posx[11] = 0.85*cycle_duration;//cycle_duration - parameters.t_dsp/2;
//     t_posx[12] = 0.99*cycle_duration;//cycle_duration - parameters.t_dsp/2;
//     t_posx[13] = 1*cycle_duration;//cycle_duration - parameters.t_dsp/2;


//     double swinging_distance = x_swing_foot_end - x_swing_foot_start;

//     xk0 = 0;
//     xk1 = 0.3;
//     xk2 = 0.8;
//     xk3 = 1;
//     xk4 = 1;
//     xk5 = 1;

//     x[0]                      = x_swing_foot_start + swinging_distance * xk0;
//     x[1]                      = x_swing_foot_start + swinging_distance * xk0;
//     x[2]                      = x_swing_foot_start + swinging_distance * xk0;
//     x[3]                      = x_swing_foot_start + swinging_distance * xk0;
//     x[4]                      = x_swing_foot_start + swinging_distance * xk0;
//     x[5]                      = x_swing_foot_start + swinging_distance * xk1;
//     x[6]                      = x_swing_foot_start + swinging_distance * xk2;
//     x[7]                      = x_swing_foot_start + swinging_distance * xk3;
//     x[8]                      = x_swing_foot_start + swinging_distance * xk4;
//     x[9]                      = x_swing_foot_start + swinging_distance * xk5;
//     x[10]                      = x_swing_foot_start + swinging_distance * xk5;
//     x[11]                      = x_swing_foot_start + swinging_distance * xk5;
//     x[12]                      = x_swing_foot_start + swinging_distance * xk5;
//     x[13]                      = x_swing_foot_start + swinging_distance * xk5;


//     t_posy[0] = t0*cycle_duration;
//     t_posy[1] = (t3-0.1)*cycle_duration;
//     t_posy[2] = t3*cycle_duration;
//     t_posy[3] = t4*cycle_duration;
//     t_posy[4] = t5*cycle_duration;//cycle_duration - parameters.t_dsp;
//     t_posy[5] = t6*cycle_duration;//cycle_duration - parameters.t_dsp/2;


//     swinging_distance = y_swing_foot_end - y_swing_foot_start;

//     yk0 = 0;
//     yk1 = 0.3;
//     yk2 = 0.8;
//     yk3 = 1;
//     yk4 = 1;
//     yk5 = 1;

//     y[0]                      = y_swing_foot_start + swinging_distance * yk0;
//     y[1]                      = y_swing_foot_start + swinging_distance * yk1;
//     y[2]                      = y_swing_foot_start + swinging_distance * yk2;
//     y[3]                      = y_swing_foot_start + swinging_distance * yk3;
//     y[4]                      = y_swing_foot_start + swinging_distance * yk4;
//     y[5]                      = y_swing_foot_start + swinging_distance * yk5;

//     t_theta[0] = 0.0*cycle_duration;
//     t_theta[1] = 0.001*cycle_duration;
//     t_theta[2] = cycle_duration - parameters.t_dsp;
//     t_theta[3] = cycle_duration - parameters.t_dsp/2;
//     t_theta[4] = cycle_duration - parameters.t_dsp/2+0.001;
//     t_theta[5] = 0.999*cycle_duration;
//     t_theta[6] = 1*cycle_duration;

//     theta[0] = 0;
//     theta[1] = 0;
//     theta[2] = -q_f;
//     theta[3] = 0;
//     theta[4] = 0;
//     theta[5] = 0;
//     theta[6] = 0;
//     double delta_psi = psi_stop - psi_start;
    
//     t_psi[0] = 0.0  * cycle_duration;
//     t_psi[1] = 0.01 * cycle_duration;
//     t_psi[2] = 0.1  * cycle_duration;
//     t_psi[3] = 0.4 * cycle_duration;
//     t_psi[4] = 0.65 * cycle_duration;
//     t_psi[5] = 0.9 * cycle_duration;
//     t_psi[6] = 0.99 * cycle_duration;
//     t_psi[7] = 1.00 * cycle_duration;

//         psi[0] = psi_start + 0.00 * delta_psi;
//         psi[1] = psi_start + 0.00 * delta_psi;;
//         psi[2] = psi_start + 0.01 * delta_psi;
//         psi[3] = psi_start + 0.6 * delta_psi;;
//         psi[4] = psi_start + 0.8 * delta_psi;;
//         psi[5] = psi_start + 0.99 * delta_psi;
//         psi[6] = psi_start + 1.00 * delta_psi;
//         psi[7] = psi_start + 1.00 * delta_psi;       


//     //double psi_increment = (psi_stop - psi_start)/(parameters.step_duration/parameters.Ts);
    

//     tk::spline x_ref;
//     x_ref.set_points(t_posx,x);    
//     tk::spline y_ref;
//     y_ref.set_points(t_posy,y);
//     tk::spline z_ref;
//     z_ref.set_points(t_posz,z);
//     tk::spline theta_ref;
//     theta_ref.set_points(t_theta,theta);

//     tk::spline psi_ref;
//     psi_ref.set_points(t_psi,psi);

//     // std::cout << "x(0): " << x_ref(0) << ", x(end): " << x_ref(cycle_duration) << std::endl;
//     // std::cout << "x(1): " << x_ref(1) << ", x(end): " << x_ref(parameters.step_duration) << std::endl;
//     for(int i=0; i<parameters.step_duration/parameters.Ts; i++){

//         double t_increment=parameters.Ts*i;
//         //std::cout << "t_increment: " << t_increment << std::endl;

//         x_ref_swing_foot_complete->push_back(x_ref(t_increment));
//         y_ref_swing_foot_complete->push_back(y_ref(t_increment));
//         if (z_ref(t_increment) >= z[0]){
//             z_ref_swing_foot_complete->push_back(z_ref(t_increment));
//         }
//         else{
//             z_ref_swing_foot_complete->push_back(z[0]);
//         }
//         theta_ref_swing_foot_complete->push_back(theta_ref(t_increment));
//         //psi_ref_swing_foot_complete->push_back(psi_start+psi_increment*(i-1));

//         psi_ref_swing_foot_complete->push_back(psi_ref(t_increment));


//     }


//     //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameters.t_ssp - parameters.Ts) << std::endl;
//     //std::cout << "x_ref at t_ssp is: " << x_ref(parameters.t_ssp) << std::endl;
//     //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;
// }

// Calculate complete swing foot trajectory
void Robot::calculate_swing_foot_complete_trajectory_heel_curve(std::vector<double> x_footprint_on_spline, std::vector<double> y_footprint_on_spline, std::vector<double> yaw_footprint_on_spline,
    std::vector<double> * x_ref_swing_foot_complete, 
    std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete, std::vector<double> * psi_ref_swing_foot_complete, 
    int forward, int leftFootStart){
    double invert_y;
    if(leftFootStart){
        invert_y = -1;
    }
    else{
        invert_y = 1;
    }   
    double step_duration = parameters.step_duration/parameters.Ts;
    double x_swing_foot_start, x_swing_foot_end, y_swing_foot_start, y_swing_foot_end, y_follow, psi_start, psi_stop;
    double amount_step_samples  = parameters.step_duration/parameters.Ts;
    double swing_foot_z_peak    = 0.05;
    double swing_foot_x_peak    = parameters.step_length;

    double amount_first_step_samples = ceil(amount_step_samples*parameters.dsp_percentage/100);
    double amount_last_step_samples = floor(amount_step_samples-amount_step_samples*parameters.dsp_percentage/100);

    ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(), step_duration, 0);
    if(leftFootStart == 0){
        ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(), step_duration, parameters.robot_width/2);
    }
    else{
        ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(), step_duration, -parameters.robot_width/2);
    }
    ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(), step_duration, parameters.robot_ankle_to_foot);

    ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(), step_duration, 0);
    ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(), step_duration, 0);

    x_swing_foot_start = x_footprint_on_spline.at(0)-invert_y*pow(-1,0)*parameters.robot_width/2*sin(yaw_footprint_on_spline.at(0));
    x_swing_foot_end   = x_footprint_on_spline.at(1)-invert_y*pow(-1,0)*parameters.robot_width/2*sin(yaw_footprint_on_spline.at(1));
    y_swing_foot_start = y_footprint_on_spline.at(0)+invert_y*pow(-1,0)*parameters.robot_width/2*cos(yaw_footprint_on_spline.at(0));
    y_swing_foot_end   = y_footprint_on_spline.at(1)+invert_y*pow(-1,0)*parameters.robot_width/2*cos(yaw_footprint_on_spline.at(1));
    psi_start          = yaw_footprint_on_spline.at(0);
    psi_stop          = yaw_footprint_on_spline.at(1);
    calculate_swing_foot_piece_trajectory_heel_curve(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_swing_foot_start, y_swing_foot_end, psi_start, psi_stop,
        x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete, psi_ref_swing_foot_complete, forward);

    for (int i = 1; i < x_footprint_on_spline.size()-1; i++){
        x_swing_foot_start = x_footprint_on_spline.at(i-1)-invert_y*pow(-1,i)*parameters.robot_width/2*sin(yaw_footprint_on_spline.at(i-1));
        x_swing_foot_end   = x_footprint_on_spline.at(i+1)-invert_y*pow(-1,i)*parameters.robot_width/2*sin(yaw_footprint_on_spline.at(i+1));
        y_swing_foot_start = y_footprint_on_spline.at(i-1)+invert_y*pow(-1,i)*parameters.robot_width/2*cos(yaw_footprint_on_spline.at(i-1));
        y_swing_foot_end   = y_footprint_on_spline.at(i+1)+invert_y*pow(-1,i)*parameters.robot_width/2*cos(yaw_footprint_on_spline.at(i+1));
        psi_start          = yaw_footprint_on_spline.at(i-1);
        psi_stop           = yaw_footprint_on_spline.at(i+1);
        calculate_swing_foot_piece_trajectory_heel_curve(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_swing_foot_start, y_swing_foot_end, psi_start, psi_stop,
            x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete, psi_ref_swing_foot_complete, forward);

    }
    int pen = x_footprint_on_spline.size()-1;

    //Penultimate Step
    x_swing_foot_start = x_footprint_on_spline.at(pen-1)+invert_y*pow(-1,pen-1)*parameters.robot_width/2*sin(yaw_footprint_on_spline.at(pen-1));
    x_swing_foot_end   = x_footprint_on_spline.at(pen)-invert_y*pow(-1,pen)*parameters.robot_width/2*sin(yaw_footprint_on_spline.at(pen));
    y_swing_foot_start = y_footprint_on_spline.at(pen-1)-invert_y*pow(-1,pen-1)*parameters.robot_width/2*cos(yaw_footprint_on_spline.at(pen-1));
    y_swing_foot_end   = y_footprint_on_spline.at(pen)+invert_y*pow(-1,pen)*parameters.robot_width/2*cos(yaw_footprint_on_spline.at(pen));
    psi_start          = psi_ref_swing_foot_complete->back();
    // std::cout << "x: " << x_swing_foot_end << ", y: " << y_swing_foot_end << std::endl;

    psi_stop           = yaw_footprint_on_spline.at(x_footprint_on_spline.size()-1);
    calculate_swing_foot_piece_trajectory_heel_curve(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_swing_foot_start, y_swing_foot_end, psi_start, psi_stop,
        x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete, psi_ref_swing_foot_complete, forward);

    // std::cout << "size. " << x_ref_swing_foot_complete->size() << ", " << parameters.step_duration/parameters.Ts << std::endl;
    x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), 1*parameters.step_duration/parameters.Ts ,x_ref_swing_foot_complete->back());
    y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), 1*parameters.step_duration/parameters.Ts ,y_ref_swing_foot_complete->back());
    z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), 1*parameters.step_duration/parameters.Ts ,z_ref_swing_foot_complete->back());
    theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), 1*parameters.step_duration/parameters.Ts ,theta_ref_swing_foot_complete->back());
    psi_ref_swing_foot_complete->insert(psi_ref_swing_foot_complete->end(), 1*parameters.step_duration/parameters.Ts , psi_ref_swing_foot_complete->back());
}


// Calculate complete swing foot trajectory
void Robot::calculate_swing_foot_complete_trajectory_heel_circling(
            std::vector<double> p_ref_x,    std::vector<double> p_ref_y,    std::vector<double> * x_ref_swing_foot_complete,    
            std::vector<double> * y_ref_swing_foot_complete,                std::vector<double> * z_ref_swing_foot_complete, 
            std::vector<double> * theta_ref_swing_foot_complete,            std::vector<double> * psi_ref_swing_foot_complete, 
            int clockwise, int step_amount, double step_psi)
{
    double y_swing_foot_start, y_swing_foot_end, x_swing_foot_start, x_swing_foot_end;
    double amount_step_samples  = parameters.step_duration/parameters.Ts;
    
    double swing_foot_z_peak    = 0.06;
    double swing_foot_y_peak    = 0;
    double swing_foot_x_peak    = 0;
    double swing_foot_psi_peak  = 0;
    double psi_swing_foot_start, psi_swing_foot_end;


    double amount_first_step_samples = ceil(amount_step_samples*parameters.dsp_percentage/100);
    double amount_last_step_samples = floor(amount_step_samples-amount_step_samples*parameters.dsp_percentage/100);

    int  invert_psi;
    
    if(clockwise == 0){
        invert_psi = 1;
    }
    else{
        invert_psi = -1;
    }
    double so[step_amount];

    for (int i = 0; i < step_amount; i++){  //bug fixed.
        so[i] = (i - floor(i / 2))*step_psi;
    }

    for (int i = 0; i < step_amount; i++){

        


        if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_first_step_samples, -p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_first_step_samples, -p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_first_step_samples, parameters.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_first_step_samples, 0);
            psi_ref_swing_foot_complete->insert(psi_ref_swing_foot_complete->end(),amount_first_step_samples,0);
        }
        else if ((i > 0) && (i < step_amount)){

            psi_swing_foot_end      = invert_psi*so[i];
            psi_swing_foot_start    = (so[i] - step_psi) * invert_psi;
            swing_foot_psi_peak     =  psi_swing_foot_end - psi_swing_foot_start;
            
            printf("Psi Start @ %f, end @ %f\n",psi_swing_foot_start,psi_swing_foot_end);
            if(i == 1)
            {
                y_swing_foot_start  = -p_ref_y.at(0);
            }
            else
            {
                y_swing_foot_start  = p_ref_y.at((i-1)*amount_step_samples -1 + amount_first_step_samples);
            
            }
            y_swing_foot_end    = p_ref_y.at(( i )*amount_step_samples    + amount_first_step_samples );
            swing_foot_y_peak   = y_swing_foot_end - y_swing_foot_start;

            x_swing_foot_start  = p_ref_x.at((i-1)*amount_step_samples -1 + amount_first_step_samples);
            x_swing_foot_end    = p_ref_x.at(( i )*amount_step_samples    + amount_first_step_samples);
            swing_foot_x_peak   = x_swing_foot_end - x_swing_foot_start;

            calculate_swing_foot_piece_trajectory_heel_circling(0,swing_foot_z_peak, x_swing_foot_start, x_swing_foot_end, swing_foot_x_peak,
                y_swing_foot_start, y_swing_foot_end, swing_foot_y_peak,  
                psi_swing_foot_start, psi_swing_foot_end, swing_foot_psi_peak,
                x_ref_swing_foot_complete,      y_ref_swing_foot_complete,      z_ref_swing_foot_complete, 
                theta_ref_swing_foot_complete,  psi_ref_swing_foot_complete);
        }
        
        if (i == step_amount - 1){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_last_step_samples, p_ref_x.at((i)*amount_step_samples - 1 + amount_first_step_samples));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_last_step_samples, p_ref_y.at((i)*amount_step_samples - 1 + amount_first_step_samples));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_last_step_samples, parameters.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_last_step_samples, 0);
            psi_ref_swing_foot_complete->insert(psi_ref_swing_foot_complete->end(),amount_last_step_samples,invert_psi * so[step_amount-1]);
        }
    }      
}

void Robot::calculate_swing_foot_complete_trajectory_heel_straight(std::vector<double> p_ref_x, std::vector<double> p_ref_y,  
    std::vector<double> * x_ref_swing_foot_complete, 
    std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete,
    int forward, int leftFootStart, int step_amount, double step_length){
    double x_swing_foot_start, x_swing_foot_end, y_follow;
    double amount_step_samples  = parameters.step_duration/parameters.Ts;
    double swing_foot_z_peak    = 0.04;
    double swing_foot_x_peak    = parameters.step_length;

    double amount_first_step_samples = ceil(amount_step_samples*parameters.dsp_percentage/100);
    double amount_last_step_samples = floor(amount_step_samples-amount_step_samples*parameters.dsp_percentage/100);
    for (int i = 0; i < parameters.steps_real+1; i++){

        if ((i > 1) && (i < parameters.steps_real - 1)){


            x_swing_foot_start = p_ref_x.at((i-1)*amount_step_samples-1 + amount_first_step_samples);
            x_swing_foot_end   = p_ref_x.at((i)*amount_step_samples + amount_first_step_samples );
            y_follow           = p_ref_y.at((i-1)*amount_step_samples);
            calculate_swing_foot_piece_trajectory_heel_straight(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_follow, 
                x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete,forward);
        }
        else if (i == 1){
            x_swing_foot_start  = p_ref_x.at(0);
            x_swing_foot_end    = p_ref_x.at(1*amount_step_samples + amount_first_step_samples);
            y_follow            = -p_ref_y.at(0);
            calculate_swing_foot_piece_trajectory_heel_straight(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak/2, y_follow, 
                x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete,forward);

        }
        else if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_first_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_first_step_samples, -p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_first_step_samples, parameters.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_first_step_samples, 0);

        }
        else if (i == parameters.steps_real-1){
            x_swing_foot_start = p_ref_x.at((i-1)*amount_step_samples-1 + amount_first_step_samples);
            x_swing_foot_end   = p_ref_x.at(i*amount_step_samples-1 + amount_first_step_samples);
            y_follow           = -p_ref_y.at((i-1)*amount_step_samples + amount_first_step_samples);
            calculate_swing_foot_piece_trajectory_heel_straight(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak/2, y_follow, 
                x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete,forward);

        }
        else if (i == parameters.steps_real){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_last_step_samples, x_ref_swing_foot_complete->back());
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_last_step_samples, -y_ref_swing_foot_complete->back());
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_last_step_samples, parameters.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_last_step_samples, 0);

        }

    }
}


void Robot::calculate_swing_foot_complete_trajectory_only_dsp(std::vector<double> p_ref_x, std::vector<double> p_ref_y,
    std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete){
    double x_swing_foot_start, x_swing_foot_end, y_follow;
    double amount_step_samples  = parameters.step_duration/parameters.Ts;
    double swing_foot_z_peak    = 0.06;

    double amount_first_step_samples = ceil(amount_step_samples*(parameters.dsp_percentage/100));
    double amount_last_step_samples = floor(amount_step_samples);
    for (int i = 0; i < parameters.steps_real; i++){
        //std::cout << "============" << std::endl;
        //std::cout << "real i: " << i << std::endl;

        if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_step_samples, -p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_step_samples, parameters.robot_ankle_to_foot);
        }
        else {
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_step_samples, p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_step_samples, parameters.robot_ankle_to_foot);
        }

        //std::cout << "i: " << i << " y_follow: " << y_follow << std::endl;

        //std::cout << i << std::endl;
        // std::cout << x_ref_swing_foot_complete->size() << std::endl;
        // std::cout << y_ref_swing_foot_complete->size() << std::endl;
        // std::cout << z_ref_swing_foot_complete->size() << std::endl;
    }

    x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), 
        ref_x_support_foot_trajectory.size()-x_ref_swing_foot_complete->size(), x_ref_swing_foot_complete->back());
    y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), 
        ref_y_support_foot_trajectory.size()-y_ref_swing_foot_complete->size(), y_ref_swing_foot_complete->back());
    z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), 
        ref_z_support_foot_trajectory.size()-z_ref_swing_foot_complete->size(), z_ref_swing_foot_complete->back());
    // show_vector(p_ref_x);
    //show_vector(p_ref_y);
    //Gnuplot gp;
    //gp << "set term x11 0" << std::endl;
    //gp << "plot" << gp.file1d(*x_ref_swing_foot_complete) << std::endl;
    //gp << "set term x11 1" << std::endl;
    //gp << "plot" << gp.file1d(*y_ref_swing_foot_complete) << std::endl;
    //gp << "set term x11 2" << std::endl;
    //gp << "plot" << gp.file1d(*z_ref_swing_foot_complete) << std::endl;
    //printf("\n");
}

void Robot::calculate_swing_foot_complete_trajectory_only_ssp(std::vector<double> p_ref_x, std::vector<double> p_ref_y,
    std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete){
    double x_swing_foot_start, x_swing_foot_end, y_follow;
    double amount_step_samples  = parameters.step_duration/parameters.Ts;
    double swing_foot_z_peak    = 0.1;

    double amount_first_step_samples = ceil(amount_step_samples*(parameters.dsp_percentage/100));
    double amount_last_step_samples = floor(amount_step_samples);
    for (int i = 0; i < parameters.steps_real; i++){
        //std::cout << "============" << std::endl;
        //std::cout << "real i: " << i << std::endl;

        if (i == 1){
            x_swing_foot_start  = p_ref_x.at(0);
            x_swing_foot_end    = p_ref_x.at(1*amount_step_samples + amount_first_step_samples)/2;
            y_follow            = p_ref_y.at(0);
            calculate_swing_foot_piece_trajectory_stay_at_height(1, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, y_follow, 
                x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete);
        }
        else if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_step_samples, -p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_step_samples, parameters.robot_ankle_to_foot);
        }
        else {
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_step_samples, x_ref_swing_foot_complete->back());
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_step_samples, y_ref_swing_foot_complete->back());
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_step_samples, z_ref_swing_foot_complete->back());           
        }


        //std::cout << "i: " << i << " y_follow: " << y_follow << std::endl;

        //std::cout << i << std::endl;
        // std::cout << x_ref_swing_foot_complete->size() << std::endl;
        // std::cout << y_ref_swing_foot_complete->size() << std::endl;
        // std::cout << z_ref_swing_foot_complete->size() << std::endl;
    }

    x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), 
        ref_x_support_foot_trajectory.size()-x_ref_swing_foot_complete->size(), x_ref_swing_foot_complete->back());
    y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), 
        ref_y_support_foot_trajectory.size()-y_ref_swing_foot_complete->size(), y_ref_swing_foot_complete->back());
    z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), 
        ref_z_support_foot_trajectory.size()-z_ref_swing_foot_complete->size(), z_ref_swing_foot_complete->back());
    // show_vector(p_ref_x);
    //show_vector(p_ref_y);
    //Gnuplot gp;
    //gp << "set term x11 0" << std::endl;
    //gp << "plot" << gp.file1d(*x_ref_swing_foot_complete) << std::endl;
    //gp << "set term x11 1" << std::endl;
    //gp << "plot" << gp.file1d(*y_ref_swing_foot_complete) << std::endl;
    //gp << "set term x11 2" << std::endl;
    //gp << "plot" << gp.file1d(*z_ref_swing_foot_complete) << std::endl;
    //printf("\n");
}

// IK
void Robot::convert_foot2ankle(joint_state foot, joint_state * ankle){
    dlib::matrix<double, 3, 1> foot_to_ankle;
    //foot_to_ankle = foot_length_front, 0, -robot_ankle_to_foot;
    foot_to_ankle = 0, 0, -parameters.robot_ankle_to_foot;
    ankle->p = foot.p - ankle->R*foot_to_ankle;
    ankle->R = foot.R;
}

dlib::matrix<double, 3, 3> Robot::Rroll(double phi){
    dlib::matrix<double, 3, 3> R_x;
    R_x     =   1, 0        , 0,
    0, cos(phi) , -sin(phi),
    0, sin(phi) , cos(phi);
    return R_x;
}

dlib::matrix<double, 3, 3> Robot::Rpitch(double theta){
    dlib::matrix<double, 3, 3> R_y;
    R_y     =   cos(theta),  0, sin(theta),
    0,           1, 0,
    -sin(theta), 0, cos(theta);
    return R_y;
}

dlib::matrix<double, 3, 3> Robot::Ryaw(double psi){
    dlib::matrix<double, 3, 3> R_z;
    R_z     =   cos(psi),   -sin(psi),  0,
    sin(psi),   cos(psi),   0,
    0,          0,          1;
    return R_z;
}

dlib::matrix<double, 6, 1> Robot::InverseKinematics_analytical(int left, joint_state Foot, joint_state Body){

    // Foot.p(1) = Foot.p(1) - Body.p(1);
    // std::cout << Foot.p << std::endl;
    // std::cout << Body.p << std::endl;
    // Body.p(1) = 0;
    // std::cout << Body.p - Foot.p << std::endl;
    double D = parameters.robot_width/2;
    // double D = Foot.p(1); 
    // std::cout << D << std::endl;
    if(left == 0){
        D = -parameters.robot_width/2;
    }
    
    double A = parameters.robot_knee_length;
    double B = parameters.robot_shin_length;
    double q2, q3, q4, q5, q6, q6a, q7;

    dlib::matrix<double, 3, 1> tmp_vector;
    tmp_vector = 0, D, -parameters.pz_com;
    //std::cout << tmp_vector << std::endl;
    dlib::matrix<double, 3, 1> r = trans(Foot.R)*(Body.p + Body.R * tmp_vector - Foot.p);
    double C = sqrt(pow(r(0),2)+pow(r(1),2)+pow(r(2),2));
    double c5 = (pow(C,2)-pow(A,2)-pow(B,2))/(2.0*A*B);

    if (c5 >= 1) q5 = 0;
    else if (c5 <= -1) q5 = pi;
    else q5 = acos(c5);

    q6a = asin((A/C)*sin(pi-q5));
    q7  = atan2(r(1),r(2));

    if (q7 > pi/2) q7 = q7-pi;
    else if (q7 < -pi/2) q7 = q7 + pi;

    q6 = -atan2(r(0),sign(r(2))*sqrt(pow(r(1),2)+pow(r(2),2))) -q6a;  // ankle pitch

    dlib::matrix<double, 3, 3> R = trans(Body.R) * Foot.R * Rroll(-q7) * Rpitch(-q6-q5);    // hipZ*hipX*hipY


    q2 = atan2(-R(0,1),R(1,1));                            // hip yaw

    q3 = atan2(R(2,1),-R(0,1)*sin(q2) + R(1,1)*cos(q2));    // hip roll
    q4 = atan2( -R(2,0), R(2,2));                           // hip pitch

    dlib::matrix<double, 6, 1> q;
    q = q2, q3, q4, q5, q6, q7;
    // std::cout << q << std::endl;

    //std::cout << "===============" << std::endl;
    //std::cout << "Foot p: " << Foot.p << std::endl;
    //std::cout << "Foot R: " << Foot.R << std::endl;
    //std::cout << "Body p: " << Body.p << std::endl;
    //std::cout << "Body R: " << Body.R << std::endl;
    //std::cout << "D: " << D << std::endl;
    //std::cout << "A: " << A << std::endl;
    //std::cout << "B: " << B << std::endl;
    //std::cout << "tmp_vector" << tmp_vector << std::endl;
    //std::cout << "r: " << r << std::endl;
    //std::cout << "C: " << C << std::endl;
    //std::cout << "c5: " << c5 << std::endl;
    //std::cout << "q5: " << q5 << std::endl;
    //std::cout << "q6a: " << q6a << std::endl;
    //std::cout << "q7: " << q7 << std::endl;
    //std::cout << "q7 after is else: " << q7 << std::endl;
    //std::cout << "q6: " << q6 << std::endl;
    //std::cout << "R: " << R << std::endl;
    //std::cout << "q2: " << q2 << std::endl;
    //std::cout << "q3: " << q3 << std::endl;
    //std::cout << "q4: " << q4 << std::endl;
    //std::cout << "q: " << std::endl;
    //std::cout << q << std::endl;
    //std::cout << "===============" << std::endl;
    return q;
}

void Robot::calculate_walking_pattern_ankle(){

    // std::cout << "Left: " << dlib::trans(left_foot.p) << std::endl;
    // std::cout << "Rightt: " << dlib::trans(right_foot.p) << std::endl;
    // std::cout << "Body: " << dlib::trans(body.p) << std::endl;
    dlib::set_colm(joint_angles, dlib::range(0, 5))   = InverseKinematics_analytical(1, left_foot, body);

    dlib::set_colm(joint_angles, dlib::range(6, 11))  = InverseKinematics_analytical(0, right_foot, body);
}

void Robot::calculate_ZMP(){

    double d = 0.006;



    ZMP_xl_local = (-m_gati_data[0].Ty - m_gati_data[0].Fx*d) / m_gati_data[0].Fz;
    ZMP_xl = ZMP_xl_local + position_left_foot_x;
    ZMP_yl = ( m_gati_data[0].Tx - m_gati_data[0].Fy*d) / m_gati_data[0].Fz + parameters.robot_width/2;
    // std::cout << "ZMP_xl " << ZMP_xl << std::endl;
    // std::cout << "ZMP_yl " << ZMP_yl << std::endl;

    // ZMP right foot
    ZMP_xr_local = (-m_gati_data[1].Ty - m_gati_data[1].Fx*d) / m_gati_data[1].Fz;
    ZMP_xr = ZMP_xr_local + position_right_foot_x;
    ZMP_yr = ( m_gati_data[1].Tx - m_gati_data[1].Fy*d) / m_gati_data[1].Fz - parameters.robot_width/2;
    // std::cout << "ZMP_xr " << ZMP_xr << std::endl;
    // std::cout << "ZMP_yr " << ZMP_yr << std::endl;

    // Check if ZMP is nan?
    if( std::isnan(ZMP_xl) || std::isinf(ZMP_xl)){
        gait_flawed = 1;
        ZMP_xl = 0;
    }

    if( std::isnan(ZMP_yl) || std::isinf(ZMP_yl) ){
        gait_flawed = 1;
        ZMP_yl = 0;   
    }

    if( std::isnan(ZMP_xr) || std::isinf(ZMP_xr) ){
        gait_flawed = 1;
        ZMP_xr = 0;
    }

    if( std::isnan(ZMP_yr) || std::isinf(ZMP_yr) ){
        gait_flawed = 1;
        ZMP_yr = 0;
    }



    // Total ZMP
    // unused?
    // ZMP_x_previous = ZMP_x;
    // ZMP_y_previous = ZMP_y;

    ZMP_x = (ZMP_xr*m_gati_data[1].Fz + ZMP_xl*m_gati_data[0].Fz)/(m_gati_data[0].Fz + m_gati_data[1].Fz);
    // std::cout << "ZMP_x: " << ZMP_x << std::endl;
    // std::cout << "Fzl " << m_gati_data[0].Fz << ",Fzr " << m_gati_data[1].Fz << std::endl;
    if( (ZMP_x > parameters.ZMP_x_max) && (ZMP_x < parameters.ZMP_x_min) ){
        gait_flawed = 1;
        ZMP_x  = 0;
        std::cout << "The x-ZMP is out of boundaries: " << ZMP_x << "/" << (ZMP_x < parameters.ZMP_x_max) <<","<< (ZMP_x > parameters.ZMP_x_min)  << std::endl;
    } 
    // else{
    //     ZMP_x = (ZMP_xr*m_gati_data[1].Fz + ZMP_xl*m_gati_data[0].Fz)/(m_gati_data[0].Fz + m_gati_data[1].Fz);
    // }

    ZMP_y = (ZMP_yr*m_gati_data[1].Fz + ZMP_yl*m_gati_data[0].Fz)/(m_gati_data[0].Fz + m_gati_data[1].Fz);

    if ( (ZMP_y > parameters.ZMP_y_max)  && (ZMP_y < parameters.ZMP_y_min) ){
        gait_flawed = 1;
        ZMP_y = 0;
        std::cout << "The y-ZMP is out of boundaries: "<< ZMP_y << "/"  <<(ZMP_y < parameters.ZMP_y_max) <<","<< (ZMP_y > parameters.ZMP_y_min)  << std::endl;

    }
    // else{
    // ZMP_y = (ZMP_yr*m_gati_data[1].Fz + ZMP_yl*m_gati_data[0].Fz)/(m_gati_data[0].Fz + m_gati_data[1].Fz);
    // }

    // std::cout << "ZMP x, y: " << ZMP_x << "," << ZMP_y << std::endl;
    // std::cout << "l ZMP x, y: " << ZMP_xl << "," << ZMP_yl << std::endl;
    // std::cout << "r ZMP x, y: " << ZMP_xr << "," << ZMP_yr << std::endl;
}

void Robot::calculateComToAnkle(  dlib::matrix<double, 3,1> COM, dlib::matrix<double, 3,1> ankle, dlib::matrix<double, 3,1> * direction){
    dlib::matrix<double, 3,1> tmp;

    tmp = ankle - COM;
    // std::cout << "tmp: " << tmp << std::endl;
    double height_distance_left = parameters.robot_ankle_to_foot - ankle(2);
    double ratio         = height_distance_left/tmp(2);
    // std::cout << "height left:  " << height_distance_left << std::endl;
    // std::cout << "ratio: " << ratio << std::endl;
    *direction = tmp*ratio;
}

void Robot::left_foot_is_support_leg(){

    dlib::matrix<double,3,1> v1, v2, vpsi, dv, check;
    v1 = ref_x_support_foot_trajectory.at(time_step), ref_y_support_foot_trajectory.at(time_step), 0; 
    v2 = ref_x_swing_foot_trajectory.at(time_step), ref_y_swing_foot_trajectory.at(time_step), 0; 
    

    vpsi = cos(ref_psi_body.at(time_step) + pi/2), sin(ref_psi_body.at(time_step) + pi/2), 0;
    dv = v2-v1;

    if( vpsi(0) * dv(0) + vpsi(1) * dv(1) > 0 )
    {
        leftIsSupportLeg = 0;
    }
    else
    {
        leftIsSupportLeg = 1;
    }
    // std::cout << "v1: " << v1 << std::endl;
    // std::cout << "v2: " << v2 << std::endl;
    // std::cout << "dv: " << dv << std::endl;

    
        // double angle = atan(fabs(dv(1)/dv(0)));
        // if(sign(dv(1)*dv(0))==-1){
        //     angle = angle;
        // }
        // else{
        //     angle = -angle;
        // }   
        // // double angle = atan2(dv(0), dv(1)); 
        // // std::cout << "angle: " << angle*180/pi << std::endl;
        // // std::cout << "R: " << Ryaw(angle) << std::endl;

        // check = Ryaw(angle)*dv;
        // if(sign(check(0))==1){
        //     leftIsSupportLeg = 1;
        // }
        // else{
        //     leftIsSupportLeg = 0;
        // }
        // // leftIsSupportLeg = sign(check(0));
        // std::cout << check << std::endl;
        // std::cout << "CHECK IS: " << check(1) << std::endl;
        // std::cout << "============" << std::endl;
}


void Robot::force_control_leg_length( double Fx, double Fz){

    // Calculate the increment to follow a given force Fz_ref
    double x_increment = 0;
    double z_increment = 0;

    Fz = -Fz;
    // Calculate angle between ground and line (CoM to ankle). Necessary for coodinate transformation
    double height = fabs(COM(2) - ankle(2));
    double length = fabs(COM(0) - ankle(0));
    double alpha = 90-atan(height/length)*180/pi;
    dlib::matrix<double, 2,2> R;
    dlib::matrix<double, 2,2> C;

    alpha = alpha*pi/180;
    R = cos(alpha), -sin(alpha), sin(alpha), cos(alpha);
    // The smaller C the stiffer. The bigger C the more compliant
    C = 0, 0, 0, 2e-6; // 3e-6 for good parameter (flc)
    //C = 0, 0, 0, 5e-5; soft

    // Remove mod! and make experiment
    Fx = Fx - fmod(Fx, 1);
    Fz = Fz - fmod(Fz, 1);

    // std::cout << "Fx: " << Fx << " Fz:" << Fz << std::endl;

    if(fabs(Fx) < 10){
        Fx = 0;
    }
    if(fabs(Fz) < 10){
        Fz = 0;
    }

    dlib::matrix<double, 2,1> F;
    F = Fx-parameters.Fx_ref, Fz-parameters.Fz_ref;
    dlib::matrix<double, 2, 1> delta;
    // std::cout << "alpha: " << alpha*180/pi << std::endl;
    delta = R*C*F;
    // std::cout << "Fx: " << Fx << " Fz:" << Fz << std::endl;
    // std::cout << "dx adding: " << delta(0) << ",dz adding: " << delta(1) << std::endl;


    if(abs(Fz) < 250){
        if(fabs(delta(0)) < 0.01){
            x_increment = delta(0);
        }
        else{
            x_increment = 0;
        } if(fabs(delta(1)) < 0.01){
            z_increment = delta(1);
        }
        else{
            z_increment = 0;
        }
    }
    else{
        std::cout << "The force in z-direction exceeds 250N: " << Fz << "N" << std::endl;
    }

    if(fabs(x_increment) > 1e-3){
        std::cout << "x_increment too much: " << x_increment << std::endl;
        x_increment = 0;
    }
    if(fabs(z_increment) > 1e-3){
        std::cout << "z_increment too much: " << z_increment << std::endl;
        z_increment = 0;
    }
    // std::cout << "dx after: " << x_increment << ", dz after: " << z_increment << std::endl;

    ref_x_swing_foot_trajectory.at(time_step) = ref_x_swing_foot_trajectory.at(time_step-1) + x_increment;
    ref_z_swing_foot_trajectory.at(time_step) = ref_z_swing_foot_trajectory.at(time_step-1) + z_increment;
}

int Robot::modify_swing_foot_trajectory(double Fx, double Fz){
    if(in_foot_landing_control_phase == 1){
        // if(time_step > 1700){
        if( (ref_z_swing_foot_trajectory.at(time_step-1) - parameters.robot_ankle_to_foot > -0.02) ){
            force_control_leg_length(Fx, Fz);
            // std::cout << "Should be force controlling (commented out)" << std::endl;
        }
        else{
            std::cout << "extended more than 2cm. Stopping to extend" << std::endl;
            ref_x_swing_foot_trajectory.at(time_step) = ref_x_swing_foot_trajectory.at(time_step-1);
            ref_z_swing_foot_trajectory.at(time_step) = ref_z_swing_foot_trajectory.at(time_step-1);
        }
        return 1;
    }
    else{
        return 0;
    }
}

void Robot::softening_ankle(int left, int32_t * t_x, int32_t * t_y){
    double T0, T1, tx_left, ty_left, tx_right, ty_right, tx_start, ty_start, tau_convert_x, tau_convert_y;

    if (left == 1){
        T1 = -m_gati_data[0].Tx;
        T0 = m_gati_data[0].Ty;
    }
    else{
        T1 = -m_gati_data[1].Tx;
        T0 = m_gati_data[1].Ty;
    }

        // std::cout << "Ty:" << T0 << std::endl;
        // 
        // tx_left = 35;
        // ty_left = 20;

        // tx_right = 30;
        // ty_left = 10;
    tx_left = 50;
    ty_left = 40;

    tx_right = 50;
    ty_right = 5;

    if(left == 1){
        tx_start = tx_left;
        ty_start = ty_left;
            tau_convert_x = 4; // Scaling from Nm (ATI) to current (dynamixel)
            tau_convert_y = 1.5; // Scaling from Nm (ATI) to current (dynamixel)

        }
        else{
            tx_start = tx_right;
            ty_start = ty_right;
            tau_convert_x = 4; // Scaling from Nm (ATI) to current (dynamixel)
            tau_convert_y = 1.5; // Scaling from Nm (ATI) to current (dynamixel)
        }

        // tau_convert_x = 4; // Scaling from Nm (ATI) to current (dynamixel)
        // tau_convert_y = 1.5; // Scaling from Nm (ATI) to current (dynamixel)
        if(parameters.debug_msg == 1){
            std::cout << "Setting torque to soft" << std::endl;  
        }
        // std::cout << "T0 " << T0 << std::endl;

        if(sign(T0) == 1){
            *t_x = tx_start + tau_convert_x * (T0-fmod(T0,0.1));
        }
        else{
            //std::cout << "negative" << std::endl;
            *t_x = -tx_start + tau_convert_x * (T0-fmod(T0,0.1));
        }

        if (*t_x > 90){
            std::cout << "Exceed torque limit of 90 (x). Setting to 90: " << *t_x << std::endl;
            *t_x = 90;
        }
        else if (*t_x < -90){
            std::cout << "Exceed 4 torque limit of -90 (x). Setting to -90: " << *t_x  << std::endl;
            *t_x = -90;
        }

        if(sign(T1) == 1){
            *t_y = ty_start + tau_convert_y * (T1-fmod(T1,0.1));
        }
        else{
            *t_y = -ty_start + tau_convert_y * (T1-fmod(T1,0.1));
        }

        if (*t_y > 50){
            *t_y = 50;
            std::cout << "Exceed torque limit of 50 (y). Setting to 50: " << *t_y  << std::endl;
        }
        else if (*t_y < - 50){
            *t_y = -50;
            std::cout << "Exceed torque limit of -50 (y). Setting to -50: " << *t_y  << std::endl;
        }
        // std::cout << "Torque goal x: " << *t_x << " Torque x: " << T0.back() << "Nm" << std::endl;
        // std::cout << "Torque goal y: " << *t_x << std::endl;
        // *t_y = 0;
    }

    // void Robot::ForwardKinematic_COM(int left, dlib::matrix<double, 6,1> q, dlib::matrix<double, 6, 1> * state){
    //     // From CoM to leg
    //     double q1,q2,q3,q4,q5,q6;
    //     q1 = q(0)*pi/180        ;
    //     q2 = q(1)*pi/180+pi/2   ;
    //     q3 = q(2)*pi/180       ;
    //     q4 = q(3)*pi/180       ;
    //     q5 = q(4)*pi/180       ;
    //     q6 = q(5)*pi/180-pi/2   ;

    //     double s1,s2,s3,s4,s5,s6,c1,c2,c3,c4,c5,c6;
    //     s1 = sin(q1);
    //     s2 = sin(q2);
    //     s3 = sin(q3);
    //     s4 = sin(q4);
    //     s5 = sin(q5);
    //     s6 = sin(q6);

    //     c1 = cos(q1);
    //     c2 = cos(q2); 
    //     c3 = cos(q3); 
    //     c4 = cos(q4);
    //     c5 = cos(q5);
    //     c6 = cos(q6);

    //     dlib::matrix<double, 4, 4> A;
    //     A= -c6*(s5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3))+c5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3)))-c1*s2*s6, 
    //     s5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3))-c5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3)), 
    //     c1*c6*s2-s6*(s5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3))+c5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3))), 
    //     0.275*c1*c2*c3-0.275*s4*(c3*s1+c1*c2*s3)-0.275*c4*(s1*s3-c1*c2*c3)-0.275*s1*s3,
    //     c6*(s5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))+c5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)))-s1*s2*s6,
    //     c5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))-s5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)),
    //     s6*(s5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))+c5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)))+c6*s1*s2,
    //     0.275*c1*s3+0.275*s4*(c1*c3-c2*s1*s3)+0.275*c4*(c1*s3+c2*c3*s1)+0.275*c2*c3*s1,
    //     c6*(c5*(s2*s3*s4-c3*c4*s2)+s5*(c3*s2*s4+c4*s2*s3))-c2*s6, 
    //     c5*(c3*s2*s4+c4*s2*s3)-s5*(s2*s3*s4-c3*c4*s2), 
    //     c2*c6+s6*(c5*(s2*s3*s4-c3*c4*s2)+s5*(c3*s2*s4+c4*s2*s3)), 
    //     0.275*s2*s3*s4-0.275*c3*s2-0.275*c3*c4*s2,
    //     0,
    //     0,
    //     0,
    //     1;

    //     dlib::matrix<double, 6, 1> state_tmp;
    //     double phi, theta, psi;
    //     rotm2eul(dlib::subm(A,dlib::range(0,2), dlib::range(0,2)),&phi, &theta, &psi);

    //     if(left){
    //         // state_tmp = -A(1,3), A(0,3), A(2,3), -phi, -psi, -theta;
    //         state_tmp = -A(1,3), A(0,3), A(2,3), theta, psi, phi;
    //     }
    //     else{
    //         // state_tmp = -A(1,3), A(0,3), A(2,3), -theta, -psi, -phi;        
    //         state_tmp = -A(1,3), A(0,3), A(2,3), theta, psi, phi;

    //     }

    //     state_tmp(2) += parameters.z_com_mbm;

    //     state_tmp(0) += ref_x_support_foot_trajectory.at(time_step);
    //     *state = state_tmp;
    // }

    void Robot::ForwardKinematic_COM(int left, dlib::matrix<double, 6,1> q, dlib::matrix<double, 6, 1> * state){
        // From CoM to leg
        double q1,q2,q3,q4,q5,q6;
        q1 = q(0)*pi/180;
        q2 = q(1)*pi/180;
        q3 = q(2)*pi/180;
        q4 = q(3)*pi/180;
        q5 = q(4)*pi/180;
        q6 = q(5)*pi/180;

        dlib::matrix<double, 6, 1> state_tmp;
        dlib::matrix<double, 3, 1> p;
        dlib::matrix<double, 3, 3> R;
        if(left == 1){
            p =   -(11*cos(q1)*sin(q3))/40 - (11*cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/40 - (11*sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))/40 - (11*cos(q3)*sin(q1)*sin(q2))/40,
            (11*cos(q1)*cos(q3)*sin(q2))/40 - (11*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)))/40 - (11*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))/40 - (11*sin(q1)*sin(q3))/40 + 33/400,
            (11*cos(q2)*sin(q3)*sin(q4))/40 - (11*cos(q2)*cos(q3)*cos(q4))/40 - (11*cos(q2)*cos(q3))/40;

            R =    cos(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3))), 
            sin(q6)*(sin(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) + cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))) - cos(q2)*cos(q6)*sin(q1), 
            cos(q6)*(sin(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) + cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))) + cos(q2)*sin(q1)*sin(q6),
            cos(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) - sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3))), 
            sin(q6)*(sin(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) + cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))) + cos(q1)*cos(q2)*cos(q6), 
            cos(q6)*(sin(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) + cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))) - cos(q1)*cos(q2)*sin(q6),
            - cos(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3)) - sin(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)),
            cos(q6)*sin(q2) + sin(q6)*(cos(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)) - sin(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3))),                                                                                                                         
            cos(q6)*(cos(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)) - sin(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3))) - sin(q2)*sin(q6);
            state_tmp(0) = -p(0);
            state_tmp(1) = -p(1);
            state_tmp(2) = p(2);   
        }
        else{

            p =  - (11*cos(q1)*sin(q3))/40 - (11*cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/40 - (11*sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))/40 - (11*cos(q3)*sin(q1)*sin(q2))/40,
            (11*cos(q1)*cos(q3)*sin(q2))/40 - (11*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)))/40 - (11*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))/40 - (11*sin(q1)*sin(q3))/40 - 33/400,
            (11*cos(q2)*sin(q3)*sin(q4))/40 - (11*cos(q2)*cos(q3)*cos(q4))/40 - (11*cos(q2)*cos(q3))/40;

            R = cos(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3))), 
            sin(q6)*(sin(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) + cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))) - cos(q2)*cos(q6)*sin(q1), 
            cos(q6)*(sin(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) + cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))) + cos(q2)*sin(q1)*sin(q6),
            cos(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) - sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3))), 
            sin(q6)*(sin(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) + cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))) + cos(q1)*cos(q2)*cos(q6), 
            cos(q6)*(sin(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) + cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))) - cos(q1)*cos(q2)*sin(q6),
            - cos(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3)) - sin(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)), 
            cos(q6)*sin(q2) + sin(q6)*(cos(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)) - sin(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3))),
            cos(q6)*(cos(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)) - sin(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3))) - sin(q2)*sin(q6);

            state_tmp(0) = -p(0);
            state_tmp(1) = -p(1);
            state_tmp(2) = p(2);
        }


        double phi, theta, psi;
        rotm2eul(R,&phi, &theta, &psi);


        // state_tmp(2) += parameters.z_com_mbm;

        state_tmp(0) += ref_x_support_foot_trajectory.at(time_step);

        state_tmp(3) = psi;
        state_tmp(4) = theta;
        state_tmp(5) = phi;

        *state = state_tmp;
    }

    // void Robot::ForwardKinematic(int left, dlib::matrix<double, 6,1> q, dlib::matrix<double, 6, 1> * state){
    //     // From CoM to leg
    //     double q1,q2,q3,q4,q5,q6;
    //     q1 = q(0)*pi/180        ;
    //     q2 = q(1)*pi/180+pi/2   ;
    //     // q3 = -q(2)*pi/180       ;
    //     // q4 = -q(3)*pi/180       ;
    //     // q5 = -q(4)*pi/180       ;
    //     q3 = q(2)*pi/180       ;
    //     q4 = q(3)*pi/180       ;
    //     q5 = q(4)*pi/180       ;
    //     q6 = q(5)*pi/180-pi/2   ;

    //     double s1,s2,s3,s4,s5,s6,c1,c2,c3,c4,c5,c6;
    //     s1 = sin(q1);
    //     s2 = sin(q2);
    //     s3 = sin(q3);
    //     s4 = sin(q4);
    //     s5 = sin(q5);
    //     s6 = sin(q6);

    //     c1 = cos(q1);
    //     c2 = cos(q2); 
    //     c3 = cos(q3); 
    //     c4 = cos(q4);
    //     c5 = cos(q5);
    //     c6 = cos(q6);

    //     dlib::matrix<double, 4, 4> A;
    //     A= -c6*(s5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3))+c5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3)))-c1*s2*s6, 
    //         s5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3))-c5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3)), 
    //         c1*c6*s2-s6*(s5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3))+c5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3))), 
    //         0.275*c1*c2*c3-0.275*s4*(c3*s1+c1*c2*s3)-0.275*c4*(s1*s3-c1*c2*c3)-0.275*s1*s3,
    //         c6*(s5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))+c5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)))-s1*s2*s6,
    //         c5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))-s5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)),
    //         s6*(s5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))+c5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)))+c6*s1*s2,
    //         0.275*c1*s3+0.275*s4*(c1*c3-c2*s1*s3)+0.275*c4*(c1*s3+c2*c3*s1)+0.275*c2*c3*s1,
    //         c6*(c5*(s2*s3*s4-c3*c4*s2)+s5*(c3*s2*s4+c4*s2*s3))-c2*s6, 
    //         c5*(c3*s2*s4+c4*s2*s3)-s5*(s2*s3*s4-c3*c4*s2), 
    //         c2*c6+s6*(c5*(s2*s3*s4-c3*c4*s2)+s5*(c3*s2*s4+c4*s2*s3)), 
    //         0.275*s2*s3*s4-0.275*c3*s2-0.275*c3*c4*s2,
    //         0,
    //         0,
    //         0,
    //         1;

    //     dlib::matrix<double, 6, 1> state_tmp;
    //     double phi, theta, psi;
    //     rotm2eul(dlib::subm(A,dlib::range(0,2), dlib::range(0,2)),&phi, &theta, &psi);


    //     if(left){
    //         // state_tmp = -A(1,3), -A(0,3), A(2,3), -phi, -psi, -theta;
    //         state_tmp = -A(1,3), -A(0,3), A(2,3), theta, psi, phi;
    //         // state_tmp = -A(1,3), -A(0,3), A(2,3), phi, psi, theta;

    //         state_tmp(1) = state_tmp(1) + parameters.robot_width/2 + y_ref.at(time_step);
    //     }
    //     else{
    //         // state_tmp = -A(1,3), -A(0,3), A(2,3), -theta, -psi, -phi;
    //         state_tmp = -A(1,3), -A(0,3), A(2,3), theta, psi, phi;
    //         // state_tmp = -A(1,3), -A(0,3), A(2,3), phi, psi, theta;

    //         state_tmp(1) = state_tmp(1) - parameters.robot_width/2 + y_ref.at(time_step);
    //     }

    //     state_tmp(0) += COM_state_fk(0);
    //     state_tmp(2) += parameters.z_com_mbm;

    //     *state = state_tmp;
    // }

    void Robot::ForwardKinematic(int left, dlib::matrix<double, 6,1> q, dlib::matrix<double, 6, 1> * state){
        // From CoM to leg
        double q1,q2,q3,q4,q5,q6;
        q1 = q(0)*pi/180;
        q2 = q(1)*pi/180;
        q3 = q(2)*pi/180;
        q4 = q(3)*pi/180;
        q5 = q(4)*pi/180;
        q6 = q(5)*pi/180;

        dlib::matrix<double, 6, 1> state_tmp;
        dlib::matrix<double, 3, 1> p;
        dlib::matrix<double, 3, 3> R;
        if(left == 1){
            p =   -(11*cos(q1)*sin(q3))/40 - (11*cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/40 - (11*sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))/40 - (11*cos(q3)*sin(q1)*sin(q2))/40,
            (11*cos(q1)*cos(q3)*sin(q2))/40 - (11*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)))/40 - (11*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))/40 - (11*sin(q1)*sin(q3))/40 + 33/400,
            (11*cos(q2)*sin(q3)*sin(q4))/40 - (11*cos(q2)*cos(q3)*cos(q4))/40 - (11*cos(q2)*cos(q3))/40;

            R =    cos(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3))), 
            sin(q6)*(sin(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) + cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))) - cos(q2)*cos(q6)*sin(q1), 
            cos(q6)*(sin(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) + cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))) + cos(q2)*sin(q1)*sin(q6),
            cos(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) - sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3))), 
            sin(q6)*(sin(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) + cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))) + cos(q1)*cos(q2)*cos(q6), 
            cos(q6)*(sin(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) + cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))) - cos(q1)*cos(q2)*sin(q6),
            - cos(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3)) - sin(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)),
            cos(q6)*sin(q2) + sin(q6)*(cos(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)) - sin(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3))),                                                                                                                         
            cos(q6)*(cos(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)) - sin(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3))) - sin(q2)*sin(q6);

            state_tmp(0) = p(0);
            if(time_step > parameters.N - 2){
                state_tmp(1) = p(1) + parameters.robot_width/2 + y_ref.at(parameters.N-2);                
            }
            else{
                state_tmp(1) = p(1)+ parameters.robot_width/2 + y_ref.at(time_step);
                
            }

            state_tmp(2) = p(2);   
        }
        else{

            p =  - (11*cos(q1)*sin(q3))/40 - (11*cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/40 - (11*sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))/40 - (11*cos(q3)*sin(q1)*sin(q2))/40,
            (11*cos(q1)*cos(q3)*sin(q2))/40 - (11*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)))/40 - (11*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))/40 - (11*sin(q1)*sin(q3))/40 - 33/400,
            (11*cos(q2)*sin(q3)*sin(q4))/40 - (11*cos(q2)*cos(q3)*cos(q4))/40 - (11*cos(q2)*cos(q3))/40;

            R = cos(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3))), 
            sin(q6)*(sin(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) + cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))) - cos(q2)*cos(q6)*sin(q1), 
            cos(q6)*(sin(q5)*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) + cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))) + cos(q2)*sin(q1)*sin(q6),
            cos(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) - sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3))), 
            sin(q6)*(sin(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) + cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))) + cos(q1)*cos(q2)*cos(q6), 
            cos(q6)*(sin(q5)*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))) + cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))) - cos(q1)*cos(q2)*sin(q6),
            - cos(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3)) - sin(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)), 
            cos(q6)*sin(q2) + sin(q6)*(cos(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)) - sin(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3))),
            cos(q6)*(cos(q5)*(cos(q2)*cos(q3)*cos(q4) - cos(q2)*sin(q3)*sin(q4)) - sin(q5)*(cos(q2)*cos(q3)*sin(q4) + cos(q2)*cos(q4)*sin(q3))) - sin(q2)*sin(q6);

            state_tmp(0) = p(0);
            if(time_step > parameters.N - 2){
                state_tmp(1) = p(1) - parameters.robot_width/2 + y_ref.at(parameters.N-2);                
            }
            else{

                state_tmp(1) = p(1) - parameters.robot_width/2 + y_ref.at(time_step);
            }
            state_tmp(2) = p(2);
        }


        double phi, theta, psi;
        rotm2eul(R,&phi, &theta, &psi);


        state_tmp(0) += COM_state_fk(0);
        state_tmp(2) += parameters.z_com_mbm;

        //    if(control_pitch == 1 && control_roll == 1){

        //    dlib::matrix<double,3,1> state_tmp2;
        //    state_tmp2 = Ryaw(0)*Rpitch(body_theta_fk)*Rroll(body_phi_fk)*dlib::rowm(state_tmp, dlib::range(0,2));

        //    state_tmp(0) = state_tmp2(0);
        //    state_tmp(1) = state_tmp2(1);
        //    state_tmp(2) = state_tmp2(2);
        //     }

        // if(control_pitch == 1 && control_roll == 1){
        // state_tmp(3) = psi  + body_phi_fk;
        // state_tmp(4) = theta + body_theta_fk;
        // state_tmp(5) = phi;   
        // }
        // else{
        state_tmp(3) = psi;
        state_tmp(4) = theta;
        state_tmp(5) = phi;

        // }

        *state = state_tmp;
    }

    //==========================================
    //===============UNUSED=====================
    //==========================================
    void Robot::calculate_absolute_foot_position(){
        //===Unused===
        // if(state == 0){
        if(leftIsSupportLeg == 1){
            // COM_state_fk(0) += left_support_foot_state.p(0);
            // swing_foot_state_fk(0) += left_support_foot_state.p(0);
            if(parameters.control_leg_length == 1){
                support_foot_state_fk(0) = left_support_foot_state.p(0);
                support_foot_state_fk(1) = left_support_foot_state.p(1);
                support_foot_state_fk(2) = left_support_foot_state.p(2);
            }
            else{
                support_foot_state_fk(0) = ref_x_support_foot_trajectory.at(time_step);
                support_foot_state_fk(1) = ref_y_support_foot_trajectory.at(time_step);
                support_foot_state_fk(2) = ref_z_support_foot_trajectory.at(time_step);   
            }
        }
        else{
            // COM_state_fk(0) += right_support_foot_state.p(0);
            // swing_foot_state_fk(0) += right_support_foot_state.p(0);
            if(parameters.control_leg_length == 1){
                support_foot_state_fk(0) = right_support_foot_state.p(0);
                support_foot_state_fk(1) = right_support_foot_state.p(1);
                support_foot_state_fk(2) = right_support_foot_state.p(2);
            }
            else{
                support_foot_state_fk(0) = ref_x_support_foot_trajectory.at(time_step);
                support_foot_state_fk(1) = ref_y_support_foot_trajectory.at(time_step);
                support_foot_state_fk(2) = ref_z_support_foot_trajectory.at(time_step);      
            }
        }

        // }
        // else{
        // if(left_support_foot_state){
        // }
        // else{
        // }   
        // }
    }

    void Robot::rotm2eul(dlib::matrix<double, 3, 3> R, double * phi, double * theta, double * psi){
        *phi = atan2(R(1,0), R(0,0));
        *theta = atan2(-R(2,0),sqrt(pow(R(0,0),2)+pow(R(1,0),2)));
        //*theta = atan2(-R(2,0),sqrt(pow(R(2,1),2)+pow(R(2,2),2)));
        *psi = atan2(R(2,1),R(2,2));
    }

    double Robot::determineCOPfoot(double Fzl, double Fzr){    return(((Fzl/(Fzl+Fzr))-0.5)*(-0.2)); }

    void Robot::start_foot_landing_control(){

        //The current state will be retrieved from FK

        // Start control after initial phase (2 x Dsp)
        if(time_step >= 2*parameters.t_dsp/parameters.Ts){   
            // Flags for the different states in_foot_landing_control_phase: 
            // Ascending = -3
            // Descending = -2
            // Foot landing control active = 1
            // Finished FLC  = 0
            int ascending, descending, flc, finished, touching_ground; 
            double body_adjustment_time = (parameters.t_dsp/parameters.Ts)/2;
            double theta_body_increment, phi_body_increment;

            double distance = 0.03; //This should be calculated from when the foot is supposed to hit with cos and sin. But for now, start with FLC when 2cm above the ground

            // Determine whether ascending or descending using the trajectory of the swing leg
            ascending       = (z_ref_swing_foot_complete_unmodified.at(time_step)-z_ref_swing_foot_complete_unmodified.at(time_step-1) > 0); 
            descending      = (z_ref_swing_foot_complete_unmodified.at(time_step)-z_ref_swing_foot_complete_unmodified.at(time_step-1) <= 0);
            if(ascending == descending){
                std::cout << "ERROR ascending and descending state at the same time" << std::endl;
            }

            // Determine whether the swing foot is touching the ground
            if(leftIsSupportLeg){
                touching_ground = m_gati_data[1].Fz < -20;
                // std::cout << "Touching ground: " << touching_ground << ": " << m_gati_data[1].Fz << std::endl;
            }
            else{
                touching_ground = m_gati_data[0].Fz < -20;
                // std::cout << "Touching ground: " << touching_ground << ": " << m_gati_data[0].Fz << std::endl;
            }
            // Determine whether flc is active
            flc = (((z_ref_swing_foot_complete_unmodified.at(time_step)-parameters.robot_ankle_to_foot) < distance) || (descending && touching_ground));
            // std::cout << "ascending: " << ascending << ", abs distance: " << (z_ref_swing_foot_complete_unmodified.at(time_step)) << "/" << parameters.robot_ankle_to_foot
            //         << ",Descending and touching ground :" << (descending && touching_ground) << std::endl;
            // Determine whether flc finished
            if(leftIsSupportLeg){
                // finished = ( touching_ground && (ZMP_xr < 0.15) && (ZMP_xr > -0.06) && (ZMP_yr > parameters.robot_width/2-0.04) );
                finished = ( touching_ground && (ZMP_xr_local < 0.15) && (ZMP_xr_local > -0.06) );
            }
            else{
                // finished = ( touching_ground && (ZMP_xl < 0.15) && (ZMP_xl > -0.06) && (ZMP_yl < -parameters.robot_width/2+0.04) );
                finished = ( touching_ground && (ZMP_xl_local < 0.15) && (ZMP_xl_local > -0.06) );
            }

            if(ascending){
                // if(leftIsSupportLeg){
                //     orientation_offset_l(1) = 0;
                // }
                // else{
                //     orientation_offset_r(1) = 0;
                // }
                // 
                // body_theta_ref_l_sup = 0;

                // body_theta_ref_r_sup = 0;           

                dlib::matrix<double,3,1> orientation_offset_tmp;

                double adjustment_time = (parameters.t_ssp/parameters.Ts)/5;
                double phi_increment, theta_increment, psi_increment;
                double phi_offset, theta_offset, psi_offset;
                if(leftIsSupportLeg){
                    rotm2eul(right_support_foot_state.R, &phi_offset, &theta_offset, &psi_offset);
                    phi_increment   = phi_offset/adjustment_time;
                    theta_increment = theta_offset/adjustment_time;
                    psi_increment   = psi_offset/adjustment_time;

                    orientation_offset_tmp = orientation_offset_r;

                    if( (fabs(orientation_offset_tmp(0)) > 0.1*pi/180) && (phi_increment < 0.05*pi/180)){
                        orientation_offset_tmp(0) -= psi_increment;
                    }
                    else if(phi_increment > 0.1*pi/180){
                        std::cout << "Increment for phi too large" << std::endl;
                    }
                    else{
                        //std::cout << "Phi smaller than 0.2 degree" << std::endl;
                    }

                    if( (fabs(orientation_offset_tmp(1)) > 0.1*pi/180) && (theta_increment < 0.05*pi/180)){
                        orientation_offset_tmp(1) -= theta_increment;
                    }
                    else if(theta_increment > 0.1*pi/180){
                        std::cout << "Increment for theta too large" << std::endl;
                    }
                    else{
                        //std::cout << "Phi smaller than 0.2 degree" << std::endl;
                    }

                    if( (fabs(orientation_offset_tmp(2)) > 0.1*pi/180) && (psi_increment < 0.05*pi/180)){
                        orientation_offset_tmp(2) -= phi_increment; 
                    }
                    else if(psi_increment > 0.1*pi/180){
                        std::cout << "Increment for psi too large" << std::endl;
                    }
                    else{
                        //std::cout << "Psi smaller than 0.2 degree" << std::endl;
                    }
                    /* std::cout << "Phi offset: " << phi_offset << ", Phi increment: " << phi_increment << std::endl;*/
                    /*std::cout << "Psi offset: " << psi_offset << ", Psi increment: " << psi_increment << std::endl;*/
                    orientation_offset_r = orientation_offset_tmp(0), orientation_offset_tmp(1), orientation_offset_tmp(2);
                }
                else{
                    rotm2eul(left_support_foot_state.R, &phi_offset, &theta_offset, &psi_offset);
                    phi_increment = phi_offset/adjustment_time;
                    theta_increment = theta_offset/adjustment_time;
                    psi_increment = psi_offset/adjustment_time;

                    orientation_offset_tmp = orientation_offset_l;
                    if( (fabs(orientation_offset_tmp(0)) > 0.1*pi/180) && (phi_increment < 0.05*pi/180)){
                        orientation_offset_tmp(0) -= psi_increment;
                    }
                    else if(phi_increment > 0.1*pi/180){
                        std::cout << "Increment for phi too large" << std::endl;
                    }
                    else{
                        //std::cout << "Phi smaller than 0.2 degree" << std::endl;
                    }

                    if( (fabs(orientation_offset_tmp(1)) > 0.1*pi/180) && (theta_increment < 0.05*pi/180)){
                        orientation_offset_tmp(1) -= theta_increment;
                    }
                    else if(theta_increment > 0.1*pi/180){
                        std::cout << "Increment for theta too large" << std::endl;
                    }
                    else{
                        //std::cout << "Phi smaller than 0.2 degree" << std::endl;
                    }

                    if( (fabs(orientation_offset_tmp(2)) > 0.1*pi/180) && (psi_increment < 0.05*pi/180)){
                        orientation_offset_tmp(2) -= phi_increment; 
                    }
                    else if(psi_increment > 0.1*pi/180){
                        std::cout << "Increment for psi too large" << std::endl;
                    }
                    else{
                        //std::cout << "Psi smaller than 0.2 degree" << std::endl;
                    }
                    orientation_offset_l = orientation_offset_tmp;
                }
                // std::cout << "Ascending" << std::endl;

                ascended_before = 1; // For FSM
                in_foot_landing_control_phase = 3;
            }
            else if(descending && !flc && (in_foot_landing_control_phase == 3 || in_foot_landing_control_phase == 2) ) {
                in_foot_landing_control_phase = 2;
                // std::cout << "Descending" << std::endl;
            }
            else if(flc && !finished && ascended_before){

                in_foot_landing_control_phase = 1;
                // std::cout << "Leg soft" << std::endl;
            }

            else if(flc && ascended_before && finished){
                //else if(*in_foot_landing_control_phase == 1 && finished){
                ascended_before = 0;
                in_foot_landing_control_phase = 0;
                // *is_at_ref = 0,0,0;

                // FLC finished, saving offsets and foot position

                if(leftIsSupportLeg){
                    right_support_foot_state = right_swing_foot_state;
                    // right_support_foot_state.R = Ryaw(swing_foot_state_fk(5))*Rpitch(swing_foot_state_fk(4))*Rroll(swing_foot_state_fk(3));

                    xr_offset = ref_x_support_foot_trajectory.at(time_step)+parameters.step_length - right_support_foot_state.p(0);
                    zr_offset = parameters.robot_ankle_to_foot - right_support_foot_state.p(2);

                    std::cout << "Offset: " << xr_offset << "," << zr_offset << std::endl;
                    orientation_offset_r = swing_foot_state_fk(3), swing_foot_state_fk(4), swing_foot_state_fk(5);
                    orientation_offset_r_unmodified = orientation_offset_r;
                    std::cout << "Stopped with offset right: " << dlib::trans(orientation_offset_r)*180/pi<< std::endl;
                }
                else{
                    left_support_foot_state = left_swing_foot_state;
                    // left_support_foot_state.R = Ryaw(swing_foot_state_fk(5))*Rpitch(swing_foot_state_fk(4))*Rroll(swing_foot_state_fk(3));

                    xl_offset = ref_x_support_foot_trajectory.at(time_step)+parameters.step_length - left_support_foot_state.p(0);
                    zl_offset = parameters.robot_ankle_to_foot - left_support_foot_state.p(2);

                    std::cout << "Offset: " << xl_offset << "," << zl_offset << std::endl;
                    orientation_offset_l = swing_foot_state_fk(3), swing_foot_state_fk(4), swing_foot_state_fk(5);
                    orientation_offset_l_unmodified = orientation_offset_l;
                    std::cout << "Stopped with offset left: " << dlib::trans(orientation_offset_l)*180/pi << std::endl;
                }
                // std::cout << "landed successfully" << std::endl;
            }
            else if(in_foot_landing_control_phase == 0){
                // Returning to original state
                double x_increment, z_increment;

                if(leftIsSupportLeg){ // For left foot being support
                    // x_increment = xr_offset/((parameters.t_ssp/parameters.Ts)/3);
                    // z_increment = zr_offset/((parameters.t_ssp/parameters.Ts)/3);
                    x_increment = xr_offset/((parameters.t_dsp/parameters.Ts));
                    z_increment = zr_offset/((parameters.t_dsp/parameters.Ts));
                    // std::cout << "x increment: " << x_increment << std::endl;
                    // std::cout <<  "fabs: " << (fabs(right_support_foot_state.p(0) - (ref_x_support_foot_trajectory.at(time_step)+parameters.step_length))) << std::endl;
                    if( (fabs(right_support_foot_state.p(2) - parameters.robot_ankle_to_foot) > 5e-4) && (z_increment < 5e-4) ){
                        right_support_foot_state.p(2) += z_increment;
                    } 
                    else if((fabs(right_support_foot_state.p(2) - parameters.robot_ankle_to_foot) > 5e-4) && z_increment > 5e-4){
                        std::cout << "The z increment is too much" << std::endl;
                        std::cout << "z: " << z_increment << std::endl;                    
                        right_support_foot_state.p(2) += 5e-4;
                    }
                    else{
                        // std::cout << "in else" << std::endl;
                    }

                    if( (fabs(right_support_foot_state.p(0) - (ref_x_support_foot_trajectory.at(time_step)+parameters.step_length)) > 5e-4) && (x_increment < 5e-4) ){
                        right_support_foot_state.p(0) += x_increment;
                        // std::cout << "Adding x_increment" << x_increment << std::endl;
                    }                 
                    else if((fabs(right_support_foot_state.p(0) - (ref_x_support_foot_trajectory.at(time_step)+parameters.step_length)) > 5e-4) && x_increment > 5e-4){
                        std::cout << "The x increment is too much" << std::endl;
                        std::cout << "x: " << x_increment << std::endl;
                        right_support_foot_state.p(0) += 5e-4;
                    }
                    else{

                    }                
                    //=========================



                    theta_body_increment = orientation_offset_r_unmodified(1)/body_adjustment_time;
                    // std::cout << "Offset: " << orientation_offset_r(1) << ", time: " << body_adjustment_time << std::endl;
                    // std::cout << "increment: " << theta_body_increment << ", body_theta_ref_r_sup: " << body_theta_ref_r_sup << std::endl;

                    if( (fabs(orientation_offset_r(1)) > 0.1*pi/180) && (theta_body_increment < 0.05*pi/180)){
                        orientation_offset_r(1) -= theta_body_increment;
                    }
                    else if(theta_body_increment > 0.05*pi/180){
                        std::cout << "Increment for theta too large" << std::endl;
                    }
                    else{
                        // std::cout << "Theta smaller than 0.1 degree" << std::endl;
                    }

                    phi_body_increment = orientation_offset_r_unmodified(0)/body_adjustment_time;
                    // std::cout << "Offset: " << orientation_offset_r(1) << ", time: " << body_adjustment_time << std::endl;
                    // std::cout << "increment: " << theta_body_increment << ", body_theta_ref_r_sup: " << body_theta_ref_r_sup << std::endl;

                    if( (fabs(orientation_offset_r(0)) > 0.1*pi/180) && (phi_body_increment < 0.05*pi/180)){
                        orientation_offset_r(0) -= phi_body_increment;
                    }
                    else if(phi_body_increment > 0.05*pi/180){
                        std::cout << "Increment for phi too large" << std::endl;
                    }
                    else{
                        // std::cout << "Phi smaller than 0.2 degree" << std::endl;
                    }

                    //========================


                }
                else{                                     // For right foot being support

                    // x_increment = xl_offset/((parameters.t_ssp/parameters.Ts)/3);
                    // z_increment = zl_offset/((parameters.t_ssp/parameters.Ts)/3);
                    x_increment = xl_offset/((parameters.t_dsp/parameters.Ts));
                    z_increment = zl_offset/((parameters.t_dsp/parameters.Ts));
                    // std::cout << "x increment" << x_increment << std::endl;

                    // std::cout << "left support foot state" << left_support_foot_state.p(0) << ", ref x support foot trajectory: " << ref_x_support_foot_trajectory.at(time_step) << std::endl;
                    // std::cout <<  "fabs: " << (fabs(left_support_foot_state.p(0) - (ref_x_support_foot_trajectory.at(time_step)+parameters.step_length))) << std::endl;

                    if( (fabs(left_support_foot_state.p(2) - parameters.robot_ankle_to_foot) > 5e-4) && (z_increment < 5e-4)){
                        left_support_foot_state.p(2) += z_increment;          
                    }   
                    else if((fabs(left_support_foot_state.p(2) - (ref_x_support_foot_trajectory.at(time_step)+parameters.step_length)) > 5e-4) && z_increment > 5e-4){
                        std::cout << "The z increment is too much (left leg)" << std::endl;
                        std::cout << "z: " << z_increment << std::endl;
                        left_support_foot_state.p(2) += 5e-4;
                    }
                    else{
                    }


                    if( (fabs(left_support_foot_state.p(0) - (ref_x_support_foot_trajectory.at(time_step)+parameters.step_length)) > 5e-4) && (x_increment < 5e-4) ){
                        left_support_foot_state.p(0) += x_increment;
                        // std::cout << "Adding x_increment" << x_increment << std::endl;
                    }  
                    else if((fabs(left_support_foot_state.p(0) - (ref_x_support_foot_trajectory.at(time_step)+parameters.step_length)) > 5e-4) && x_increment > 5e-4){
                        std::cout << "The x increment is too much (left leg)" << std::endl;
                        std::cout << "x: " << x_increment << std::endl;
                        left_support_foot_state.p(0) += 5e-4;
                    }
                    else{
                    }

                    //=========================
                    theta_body_increment = orientation_offset_l_unmodified(1)/body_adjustment_time;
                    // std::cout << "Offset: " << orientation_offset_l(1) << ", time: " << body_adjustment_time << std::endl;
                    // std::cout << "increment: " << theta_body_increment << ", body_theta_ref_r_sup: " << body_theta_ref_l_sup << std::endl;
                    if( (fabs(orientation_offset_l(1)) > 0.1*pi/180) && (theta_body_increment < 0.05*pi/180)){
                        orientation_offset_l(1) -= theta_body_increment;
                    }
                    else if(theta_body_increment > 0.05*pi/180){
                        std::cout << "Increment for theta too large" << std::endl;
                    }
                    else{
                        // std::cout << "Phi smaller than 0.2 degree" << std::endl;
                    }

                    phi_body_increment = orientation_offset_l_unmodified(0)/body_adjustment_time;
                    // std::cout << "Offset: " << orientation_offset_l(0) << ", time: " << body_adjustment_time << std::endl;
                    // std::cout << "increment: " << theta_body_increment << ", body_theta_ref_r_sup: " << body_theta_ref_l_sup << std::endl;
                    if( (fabs(orientation_offset_l(0)) > 0.1*pi/180) && (phi_body_increment < 0.05*pi/180)){
                        orientation_offset_l(0) -= phi_body_increment;
                    }
                    else if(theta_body_increment > 0.05*pi/180){
                        std::cout << "Increment for theta too large" << std::endl;
                    }
                    else{
                        // std::cout << "Phi smaller than 0.2 degree" << std::endl;
                    }
                    //=========================


                }
                // std::cout << "Waiting to ascend" << std::endl;

            }
            else{
                std::cout << "shouldnt exist. Probably landed while descending and flc is not 1 yet" << std::endl;
            }
        }
        else{
            if(leftIsSupportLeg){
                    // std::cout << ref_z_support_foot_trajectory.at(time_step) << std::endl;

                left_support_foot_state.p = ref_x_support_foot_trajectory.at(time_step), ref_y_support_foot_trajectory.at(time_step), ref_z_support_foot_trajectory.at(time_step);
                left_support_foot_state.R = Ryaw(0)*Rpitch(0)*Rroll(0);
            }
            else{
                right_support_foot_state.p = ref_x_support_foot_trajectory.at(time_step), ref_y_support_foot_trajectory.at(time_step), ref_z_support_foot_trajectory.at(time_step);
                right_support_foot_state.R = Ryaw(0)*Rpitch(0)*Rroll(0);
            }
                // std::cout << "Right support foot state at " << time_step << ": " << right_support_foot_state.p << std::endl;
        }

    }
    void Robot::generate_foot_print_on_spline(std::vector<double> xSpline, std::vector<double> ySpline, std::vector<double> * x_footprint_on_spline, std::vector<double> * y_footprint_on_spline,std::vector<double> * yaw_footprint_on_spline){
        double xtmp, ytmp, yawtmp;
        int idx;
            int found_nn = 1; // nearest neighbor
            //std::cout << "Length xSpline: " << xSpline.size() << std::endl;
            //std::cout << "Length ySpline: " << ySpline.size() << std::endl;
            int started = 1;
            //for(int i = 0;i < 10;i++){
            while(1){
                if (started == 1){
                    xtmp = xSpline.at(0);
                    ytmp = ySpline.at(0);
                    idx = 1;
                    started = 0;
                    yawtmp = 0;
                }
                else{
                    xtmp = x_footprint_on_spline->back() + parameters.step_length*cos(yaw_footprint_on_spline->back());
                    idx = closest(xSpline,xtmp);
                    if(idx == -1){
                        // std::cout << "idx is -1. No nearest neighbor found. Breaking." << std::endl;
                        break;
                    }
                    ytmp = ySpline.at(idx);
                    yawtmp = atan((ySpline.at(idx)-ySpline.at(idx-1))/(xSpline.at(idx)-xSpline.at(idx-1)));
                }


                //std::cout << "idx: " << idx << std::endl;
                //std::cout << "xtmp: " << xtmp << std::endl;
                //std::cout << "ytmp: " << ytmp << std::endl;
                //std::cout << "yaw: " << yawtmp << std::endl;

                yaw_footprint_on_spline->push_back(yawtmp);
                x_footprint_on_spline->push_back(xtmp);
                y_footprint_on_spline->push_back(ytmp);

                //std::cout << "yaw size: " << yaw_footprint_on_spline->size() << std::endl;
                //std::cout << "x size: " << x_footprint_on_spline->size() << std::endl;
                //std::cout << "y size: " << y_footprint_on_spline->size() << std::endl;
            }

        }

        int Robot::closest(std::vector<double> vec, double value) {
            //std::cout << "value: " << value << std::endl;
            std::vector<double>::iterator it = std::lower_bound(vec.begin(), vec.end(), value);
            //std::cout << "found at: " << std::distance(vec.begin(), it);
            if (it == vec.end()) { return -1; }
            //std::cout << "Value: " << vec.at(it) << std::endl;

            return std::distance(vec.begin(), it);
        }

     
        void Robot::generate_orientated_spline(double x, double y, double psi, int samples, std::vector<double> * xSpline, std::vector<double> * ySpline){
            int invert_psi = 0;
            if (psi < 0){
                invert_psi = 1;
                // psi = -psi;
            }
            double xshort, yshort, x1, y1, x2, y2;
            std::vector<double> xsample(5), ysample(5), xsample2(4), ysample2(4);
            std::vector<double> xspline, yspline1, yspline2;
            double dist1, dist2;

            xshort = 0.0001;
            yshort = tan(psi)*xshort;

            x2 = x-(sign(x))*xshort;
            y2 = y-yshort;

            x1 = x2*psi*180/pi/100;

            if( fabs(x1) > fabs(x)){
                x1 = x2/2;
            }

            if(sign(x) != sign(x1)){
                x1 = -1*x1;
            }
            y1 = 0;

            xsample[0] = 0;
            xsample[1] = sign(x)*0.01;
            xsample[2] = x1;
            xsample[3] = x2;
            xsample[4] = x;

            ysample[0] = 0;
            ysample[1] = 0;
            ysample[2] = y1;
            ysample[3] = y2;
            ysample[4] = y;

            xsample2[0] = 0;
            xsample2[1] = sign(x)*0.01;
            xsample2[2] = x2;
            xsample2[3] = x;

            ysample2[0] = 0;
            ysample2[1] = 0;
            ysample2[2] = y2;
            ysample2[3] = y;

            std::cout << "xsample: " << xsample[0]<< "," <<xsample[1]<< "," <<xsample[2]<< "," <<xsample[3] << "," <<xsample[4]<< std::endl;
            std::cout << "ysample: " << ysample[0]<<"," << ysample[1]<< "," <<ysample[2]<< "," <<ysample[3] << "," <<ysample[4]<< std::endl;

            std::cout << "xsample: " << xsample2[0]<< "," <<xsample2[1]<< "," <<xsample2[2]<< "," <<xsample2[3] << std::endl;
            std::cout << "ysample: " << ysample2[0]<<"," << ysample2[1]<< "," <<ysample2[2]<< "," <<ysample2[3] << std::endl;

            tk::spline spline2;
            spline2.set_points(xsample2,ysample2);
            std::cout << "1" << std::endl;
            tk::spline spline1;
            spline1.set_points(xsample,ysample);
            //std::cout << "1" << std::endl;
            double x_increment = 0;
            for(int i=0; i<samples; i++){
                x_increment += x/samples;
                xspline.push_back(x_increment);
                yspline1.push_back(spline1(x_increment));
                yspline2.push_back(spline2(x_increment));

                if(i != 0){
                    dist1 += sqrt(pow((x/samples),2)+pow((yspline1.back()-yspline1.at(yspline1.size()-1)),2));
                    dist2 += sqrt(pow((x/samples),2)+pow((yspline2.back()-yspline2.at(yspline2.size()-1)),2));
                }
            }
            //std::cout << "Dist 1: " << dist1 << ", Dist 2: " << dist2 << std::endl;
            *xSpline = xspline;
            if (dist1 < dist2){
                *ySpline = yspline1;
            }
            else{
                *ySpline = yspline2;
            }

        }

        void Robot::determine_states_from_FK(dxl_Actuator *dxl_actuator){

            if(leftIsSupportLeg){
                q_support = dxl_actuator[0].get_present_theta(),
                dxl_actuator[1].get_present_theta()/parameters.Tipping_Scale_left,
                dxl_actuator[2].get_present_theta(),
                dxl_actuator[3].get_present_theta(),
                dxl_actuator[4].get_present_theta(),
                dxl_actuator[5].get_present_theta();

                q_swing =   dxl_actuator[6].get_present_theta(),
                dxl_actuator[7].get_present_theta()/parameters.Tipping_Scale_right,
                dxl_actuator[8].get_present_theta(),
                dxl_actuator[9].get_present_theta(),
                dxl_actuator[10].get_present_theta(),
                dxl_actuator[11].get_present_theta();

                ForwardKinematic_COM(1, q_support, &COM_state_fk); 
                ForwardKinematic(1, q_support, &support_foot_state_fk); 
                ForwardKinematic(0, q_swing, &swing_foot_state_fk); 
            }

            else{ 

                q_swing   = dxl_actuator[0].get_present_theta(),
                dxl_actuator[1].get_present_theta()/parameters.Tipping_Scale_left,
                dxl_actuator[2].get_present_theta(),
                dxl_actuator[3].get_present_theta(),
                dxl_actuator[4].get_present_theta(),
                dxl_actuator[5].get_present_theta();

                q_support =   dxl_actuator[6].get_present_theta(),
                dxl_actuator[7].get_present_theta()/parameters.Tipping_Scale_right,
                dxl_actuator[8].get_present_theta(),
                dxl_actuator[9].get_present_theta(),
                dxl_actuator[10].get_present_theta(),
                dxl_actuator[11].get_present_theta();
                // std::cout << "q support" << q_support << std::endl;

                ForwardKinematic_COM(0, q_support, &COM_state_fk); 
                ForwardKinematic(0, q_support, &support_foot_state_fk); 
                ForwardKinematic(1, q_swing, &swing_foot_state_fk); 

            }

            // calculate_absolute_foot_position();
            // std::cout << "COM position (FK)" << COM_state_fk << std::endl;
        }       

        void Robot::printJointAnglesToFile(){
            FILE *fp;

            if( ( fp = fopen( "./joint_angles.txt", "w" ) ) == NULL )
            {
                printf( "error opening data file ./data.txt\n" );
            }
            for(int i = 0; i < q1.size(); i++){
                fprintf(fp,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
                    q1.at(i),q2.at(i),q3.at(i),q4.at(i),q5.at(i),q6.at(i),q7.at(i),q8.at(i),q9.at(i),q10.at(i),q11.at(i),q12.at(i));
                fprintf(fp,"\n");
            }

        }

        void Robot::set_MPC_initial_position(double x0, double y0){
            dlib::matrix<double, 3, 1> x_init, y_init;
            x_init = x0, 0, 0;
            y_init = y0, 0, 0;

            X_ref.push_back(x_init); 
            Y_ref.push_back(y_init); 
            
        }

        void Robot::connect_steps(){
            double x_swing_foot_start, x_swing_foot_end, y_swing_foot_start, y_swing_foot_end, psi_start, psi_stop;
            double step_duration = parameters.step_duration/parameters.Ts;
            
            double zmp_x_now = 0;
            double zmp_y_now = 0;

            // double swing_goal_x swing_goal_y, swing_goal_psi, support_goal_x, support_goal_y, support_goal_psi, zmp_goal_x, zmp_goal_y;


            // swing_goal_x = ref_x_swing_foot_trajectory.front();
            // swing_goal_y = ref_y_swing_foot_trajectory.front();
            // swing_goal_psi = ref_psi_swing_foot_trajectory.front();

            // support_goal_x = ref_x_support_foot_trajectory.front();
            // support_goal_y = ref_y_support_foot_trajectory.front();
            // support_goal_psi = ref_psi_support_foot_trajectory.front();

            // zmp_goal_x = ref_x_zmp_trajectory.front();
            // zmp_goal_y = ref_y_zmp_trajectory.front();

            // // Move second step

            // left_foot_is_support_leg();

            // ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.begin(), step_duration, swing_goal_x);
            // ref_z_support_foot_trajectory.insert(ref_z_support_foot_trajectory.begin(), step_duration, parameters.robot_ankle_to_foot);          
            // ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.begin(), step_duration, swing_goal_psi);


            // if(leftIsSupportLeg){
            //     ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.begin(), step_duration, swing_goal_y);
            //     y_swing_foot_start = right_foot.p(1);
            // }
            // else{
            //     ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.begin(), step_duration, right_foot.p(1));   
            //     y_swing_foot_start = left_foot.p(1);


            // }
            // y_swing_foot_end   = ref_y_swing_foot_trajectory.front();

            // ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.begin(), step_duration, zmp_x_now);
            // ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.begin(), step_duration, zmp_y_now);

            // x_swing_foot_start = 0;
            // x_swing_foot_end   = 0;
            // psi_start          = 0;
            // psi_stop           = 0;
            // double swing_foot_z_peak = 0.00;
            // double swing_foot_x_peak = x_swing_foot_end - x_swing_foot_start;
            // int forward  = 1;
            // std::vector<double> x_swing_tmp, y_swing_tmp, z_swing_tmp, theta_swing_tmp, psi_swing_tmp;

            // calculate_swing_foot_piece_trajectory_heel_curve(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_swing_foot_start, y_swing_foot_end, psi_start, psi_stop,
            //     &x_swing_tmp, &y_swing_tmp, &z_swing_tmp, &theta_swing_tmp, &psi_swing_tmp, forward);


            // ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.begin(), x_swing_tmp.begin(), x_swing_tmp.end());
            // ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.begin(), y_swing_tmp.begin(), y_swing_tmp.end());
            // ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.begin(), z_swing_tmp.begin(), z_swing_tmp.end());
            // ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.begin(), theta_swing_tmp.begin(), theta_swing_tmp.end());
            // ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.begin(), psi_swing_tmp.begin(), psi_swing_tmp.end());

            // Move first step

            left_foot_is_support_leg();
            ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.begin(), step_duration, 0);
            ref_z_support_foot_trajectory.insert(ref_z_support_foot_trajectory.begin(), step_duration, parameters.robot_ankle_to_foot);          
            ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.begin(), step_duration, ref_psi_support_foot_trajectory.front());


            if(leftIsSupportLeg){
                ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.begin(), step_duration, left_foot.p(1));
                y_swing_foot_start = right_foot.p(1);

            }
            else{
                ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.begin(), step_duration, right_foot.p(1));   
                y_swing_foot_start = left_foot.p(1);


            }
            y_swing_foot_end   = ref_y_swing_foot_trajectory.front();

            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.begin(), step_duration/2, zmp_x_now);
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.begin(), step_duration/2, ref_y_zmp_trajectory.front());


            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.begin(), step_duration/2, zmp_x_now);
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.begin(), step_duration/2, zmp_y_now);

            x_swing_foot_start = 0;
            x_swing_foot_end   = 0;
            psi_start          = 0;
            psi_stop           = 0;
            double swing_foot_z_peak = 0.00;
            double swing_foot_x_peak = x_swing_foot_end - x_swing_foot_start;
            int forward  = 1;
            std::vector<double> x_swing_tmp, y_swing_tmp, z_swing_tmp, theta_swing_tmp, psi_swing_tmp;

            calculate_swing_foot_piece_trajectory_heel_curve(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_swing_foot_start, y_swing_foot_end, psi_start, psi_stop,
                &x_swing_tmp, &y_swing_tmp, &z_swing_tmp, &theta_swing_tmp, &psi_swing_tmp, forward);


            ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.begin(), x_swing_tmp.begin(), x_swing_tmp.end());
            ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.begin(), y_swing_tmp.begin(), y_swing_tmp.end());
            ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.begin(), z_swing_tmp.begin(), z_swing_tmp.end());
            ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.begin(), theta_swing_tmp.begin(), theta_swing_tmp.end());
            ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.begin(), psi_swing_tmp.begin(), psi_swing_tmp.end());

            // // Shift ZMP from momentary to now
            // ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.begin(), step_duration, zmp_x_now);
            // ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.begin(), step_duration, zmp_y_now);




        }

        void Robot::connect_trajectories(int leftFirst, double support_x_last, double support_y_last, double support_z_last, double support_psi_last,
            double  swing_x_last, double  swing_y_last, double  swing_z_last, double  swing_theta_last, double  swing_psi_last,
            double  x_zmp_last, double y_zmp_last){

            double step_duration = parameters.step_duration/parameters.Ts;
            // Support Foot
            double support_x_first = ref_x_support_foot_trajectory.front();
            double support_y_first = ref_y_support_foot_trajectory.front();
            double support_z_first = ref_z_support_foot_trajectory.front();
            double support_psi_first = ref_psi_support_foot_trajectory.front();

            ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.begin(),step_duration, support_x_first); // first step
            ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.begin(),step_duration, support_x_last); // second step

            ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.begin(),step_duration, support_y_first); // first step
            ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.begin(),step_duration, support_y_last); // second step

            ref_z_support_foot_trajectory.insert(ref_z_support_foot_trajectory.begin(),step_duration, support_z_first); // first step
            ref_z_support_foot_trajectory.insert(ref_z_support_foot_trajectory.begin(),step_duration, support_z_last); // second step

            ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.begin(),step_duration, support_psi_first); // first step
            ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.begin(),step_duration, support_psi_last); // second step

            // Swing Foot
            double swing_foot_z_peak = 0.05;

            double swing_x_first = ref_x_swing_foot_trajectory.front();
            double swing_y_first = ref_y_swing_foot_trajectory.front();
            double swing_z_first = ref_z_swing_foot_trajectory.front();
            double swing_theta_first = ref_theta_swing_foot_trajectory.front();
            double swing_psi_first = ref_psi_swing_foot_trajectory.front();
            double swing_foot_x_peak = swing_x_first-swing_x_last;
            // First step
            double x_swing_foot_start = support_x_last;
            double x_swing_foot_end   = support_x_first;
            double y_swing_foot_start = support_y_last;
            double y_swing_foot_end   = support_y_first;
            double psi_start          = support_psi_last;
            double psi_stop           = support_psi_first;
            int forward = 1;
            std::vector<double> x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete, psi_ref_swing_foot_complete;
            calculate_swing_foot_piece_trajectory_heel_curve(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, 
                y_swing_foot_start, y_swing_foot_end, psi_start, psi_stop,
                &x_ref_swing_foot_complete, &y_ref_swing_foot_complete, &z_ref_swing_foot_complete, &theta_ref_swing_foot_complete, &psi_ref_swing_foot_complete, forward);

            // Second step
            x_swing_foot_start = swing_x_last;
            x_swing_foot_end   = swing_x_first;
            y_swing_foot_start = swing_y_last;
            y_swing_foot_end   = swing_y_first;
            psi_start          = swing_psi_last;
            psi_stop           = swing_psi_first;
            calculate_swing_foot_piece_trajectory_heel_curve(0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, 
                y_swing_foot_start, y_swing_foot_end, psi_start, psi_stop,
                & x_ref_swing_foot_complete, &y_ref_swing_foot_complete, &z_ref_swing_foot_complete, &theta_ref_swing_foot_complete, &psi_ref_swing_foot_complete, forward);

            ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.begin(), x_ref_swing_foot_complete.begin(), x_ref_swing_foot_complete.end());
            ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.begin(), y_ref_swing_foot_complete.begin(), y_ref_swing_foot_complete.end());
            ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.begin(), z_ref_swing_foot_complete.begin(), z_ref_swing_foot_complete.end());
            ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.begin(), theta_ref_swing_foot_complete.begin(), theta_ref_swing_foot_complete.end());
            ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.begin(), psi_ref_swing_foot_complete.begin(), psi_ref_swing_foot_complete.end());

            // ZMP Reference
            // double x_zmp_first = ref_x_zmp_trajectory.front();
            // double y_zmp_first = ref_y_zmp_trajectory.front();
            // std::cout << ""
            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.begin(), step_duration, support_x_first);
            ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.begin(), step_duration, support_x_last);

            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.begin(), step_duration, support_y_first);
            ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.begin(), step_duration, support_y_last);


        }

        void Robot::drawTrajectory(){
            FILE *fp, *fp2;
            if( ( fp = fopen( "./data.txt", "w" ) ) == NULL )
            {
                printf( "error opening data file ./data.txt\n" );
            }

            if( ( fp2 = fopen( "./data2.txt", "w" ) ) == NULL )
            {
                printf( "error opening data file ./data.txt\n" );
            }

            // double x,y,psi;
            // int samples;
            // x = 5;
            // y = 2;
            // psi = 20.0001*pi/180;
            // samples = parameters.N;
            // std::vector<double> xSpline, ySpline;
     // generate_orientated_spline(x,y,psi,samples, &xSpline, &ySpline);
            // std::vector<double> x_footprint_on_spline, y_footprint_on_spline, yaw_footprint_on_spline;
            // generate_foot_print_on_spline(xSpline, ySpline, &x_footprint_on_spline, &y_footprint_on_spline, &yaw_footprint_on_spline);
            // int clockwise = 1;
            // double radius = 1;
            // double degree = pi/2;
            // double x_start = 0;
            // double y_start = 0;
            // double x_stop  = 1;
            // double y_stop  = 1;
            // double x_center = 0;
            // double y_center = 1;
            // int samples;
            // double straight_dist = sqrt(pow(x_stop-x_start,2)+pow(y_stop-y_start,2));
            // double estimatedSteps = straight_dist/parameters.step_length+10;

            // samples = estimatedSteps*parameters.step_duration/parameters.Ts;
            // std::vector<double> xCircle, yCircle;
            // generate_circle(clockwise, samples, radius, &xCircle, &yCircle, x_start, y_start, x_stop, y_stop, x_center, y_center);
            // for(int i; i < xCircle.size(); i++){
            //     fprintf(fp, "%f, %f\n", xCircle.at(i), yCircle.at(i));
            // // printf("%f, %f\n", xCircle.at(i), yCircle.at(i));

            // }

            // generate_foot_print_on_circle(xCircle, yCircle, &x_footprint_on_circle, &y_footprint_on_circle, &yaw_footprint_on_circle);
            // for(int i; i < x_footprint_on_circle.size(); i++){
            //     fprintf(fp2, "%f, %f, %f \n", x_footprint_on_circle.at(i), y_footprint_on_circle.at(i), yaw_footprint_on_circle.at(i));
            // }




          // std::cout << xCircle.size() << "," << yCircle.size() << "," << x_footprint_on_circle.size() << "," << y_footprint_on_circle.size() << "," << yaw_footprint_on_circle.size() << std::endl;
          // Gnuplot gp;
          // gp << "set grid xtics mxtics ytics mytics" << std::endl;
          // gp << "set term x11 0" << std::endl;
          // gp << "plot" 
          // << gp.file1d(xCircle) << ","
          //       //<< gp.file1d(xSpline) << ","
          // << gp.file1d(yCircle) << ","
          // << gp.file1d(x_footprint_on_circle) << ","
          // << gp.file1d(y_footprint_on_circle) << ","
          // << gp.file1d(yaw_footprint_on_circle) 

          //       //<< gp.file1d(ref_x_support_foot_trajectory) << "with dots tit 'Support X', "
          //       //<< gp.file1d(ref_x_swing_foot_trajectory) << "with dots tit 'Swing X', "
          //       //<< gp.file1d(ref_x_zmp_trajectory) << "with dots tit 'ZMP reference', "
          //       //<< gp.file1d(x_ref) << "with dots tit 'CoM', "
          // << std::endl;


            //gp << "set term x11 1" << std::endl;
            //gp << "plot" 
            //<< gp.file1d(ref_y_support_foot_trajectory) << "with dots tit 'Support Y', "
            //<< gp.file1d(ref_y_swing_foot_trajectory) << "with dots tit 'Swing Y', "
            //<< gp.file1d(ref_y_zmp_trajectory) << "with dots tit 'ZMP reference', "
            //<< gp.file1d(y_ref) << "with dots tit 'CoM', "
            //<< std::endl;

            //gp << "set term x11 2" << std::endl;
            //gp << "plot" 
            //<< gp.file1d(ref_z_support_foot_trajectory) << "with dots tit 'Support Z', "
            //<< gp.file1d(ref_z_swing_foot_trajectory) << "with dots tit 'Swing Z', "
            //<< gp.file1d(z_ref) << "with dots tit 'CoM', "    << std::endl;

            //gp << "set term x11 3" << std::endl;
            //gp << "plot" 
            //<< gp.file1d(theta_body) << "with lines tit 'theta r fk', "

            //<< std::endl;
        }

        void Robot::push_into_gait_packet(float gait_packet[]){
            gait_packet[0] = X_est(0);
            gait_packet[1] = X_est(1);
            gait_packet[2] = X_est(2);

            gait_packet[3] = Y_est(0);
            gait_packet[4] = Y_est(1);
            gait_packet[5] = Y_est(2);

            gait_packet[6] = ZMP_x;
            gait_packet[7] = ZMP_y;

            gait_packet[8] = COM_state_fk(0);
            gait_packet[9] = COM_state_fk(1);

            gait_packet[10] = support_foot_state_fk(0);
            gait_packet[11] = support_foot_state_fk(1);
            gait_packet[64] = support_foot_state_fk(2);

            gait_packet[12] = swing_foot_state_fk(0);
            gait_packet[13] = swing_foot_state_fk(1);
            gait_packet[65] = swing_foot_state_fk(2);

            gait_packet[27] = COM_state_fk(3);
            gait_packet[28] = COM_state_fk(4);
            gait_packet[29] = COM_state_fk(5);

            gait_packet[30] = support_foot_state_fk(3);
            gait_packet[31] = support_foot_state_fk(4);
            gait_packet[32] = support_foot_state_fk(5);

            gait_packet[33] = swing_foot_state_fk(3);
            gait_packet[34] = swing_foot_state_fk(4);
            gait_packet[35] = swing_foot_state_fk(5);

            gait_packet[14] = x_ref.back();
            gait_packet[15] = y_ref.back();

            gait_packet[16] = X_est(3);
            gait_packet[17] = Y_est(3);

            gait_packet[18] = x_output_estimation(0);
            gait_packet[19] = x_output_estimation(1);
            gait_packet[20] = y_output_estimation(0);
            gait_packet[21] = y_output_estimation(1);

            gait_packet[22] = Y_ref.back()(1);
            gait_packet[23] = Y_ref.back()(2);

            gait_packet[24] = y_mpc(0);
            gait_packet[25] = y_mpc(1);
            gait_packet[26] = y_mpc(2);

            // gait_packet[27] = tmp1;
            // gait_packet[28] = tmp2;
            // gait_packet[29] = tmp3;
            // gait_packet[30] = tmp4;
            // gait_packet[31] = tmp5;
            // gait_packet[32] = tmp6;
            // gait_packet[33] = tmp7;
            // gait_packet[34] = tmp8;
            // gait_packet[35] = tmp9;

            gait_packet[36] = body.p(0);
            gait_packet[37] = body.p(1);
            gait_packet[38] = body.p(2);


            gait_packet[39] = left_foot.p(0);
            gait_packet[40] = left_foot.p(1);
            gait_packet[41] = left_foot.p(2);

            gait_packet[42] = right_foot.p(0);
            gait_packet[43] = right_foot.p(1);
            gait_packet[44] = right_foot.p(2);

            gait_packet[70] = ref_x_support_foot_trajectory.at(time_step);
            gait_packet[71] = ref_y_support_foot_trajectory.at(time_step);
            gait_packet[72] = ref_z_support_foot_trajectory.at(time_step);

            gait_packet[73] = ref_x_swing_foot_trajectory.at(time_step);
            gait_packet[74] = ref_y_swing_foot_trajectory.at(time_step);
            gait_packet[75] = ref_z_swing_foot_trajectory.at(time_step);

            gait_packet[58] = phi1;
            gait_packet[59] = theta1;
            gait_packet[60] = psi1;


            gait_packet[61] = phi2;
            gait_packet[62] = theta2;
            gait_packet[63] = psi2;
            gait_packet[54] = ref_x_zmp_trajectory.at(time_step);

            gait_packet[45] = ref_y_zmp_trajectory.at(time_step);
            gait_packet[46] = in_foot_landing_control_phase;

            gait_packet[47] = orientation_offset_l(0);
            gait_packet[48] = orientation_offset_l(1);
            gait_packet[49] = orientation_offset_l(2);

            gait_packet[50] = orientation_offset_r(0);
            gait_packet[51] = orientation_offset_r(1);
            gait_packet[52] = orientation_offset_r(2);

            gait_packet[53] = parameters.Tipping_Scale_left;
            gait_packet[55] = parameters.Tipping_Scale_right;

            gait_packet[56] = body_theta_fk;
            gait_packet[57] = body_phi_fk;

            gait_packet[66] = ref_psi_support_foot_trajectory.at(time_step);
            gait_packet[67] = ref_psi_swing_foot_trajectory.at(time_step);

            gait_packet[68] = leftIsSupportLeg;
            gait_packet[69] = ref_psi_body.at(time_step);

            gait_packet[76] = walking_state;
        }


        // This function performs the actual movements
        int Robot::walk(dxl_Actuator *dxl_actuator, float gait_packet[]){


            std::cout << "i: " << time_step << std::endl;

            // Walk for the amount steps and then stop
            // if( time_step < ( (parameters.steps_real)*parameters.step_duration/parameters.Ts -1) ){
            if( time_step < (parameters.N-1)){
                if( walking_mode == 0 || walking_mode == 4 ){
                    // only walking mode 0 is walking forward and x changes. For this the position of the left and right foot needs to be determined
                    if(leftIsSupportLeg == 1){
                        position_left_foot_x = ref_x_support_foot_trajectory.at(time_step);
                        position_right_foot_x = ref_x_swing_foot_trajectory.at(time_step);
                    }
                    else{
                        position_left_foot_x = ref_x_swing_foot_trajectory.at(time_step);
                        position_right_foot_x = ref_x_support_foot_trajectory.at(time_step);
                    }

                }
                // Determine whether leftIsSupportLeg = 1 or not
                this->left_foot_is_support_leg();
                // From ATI information, calculate the ZMP 
                this->calculate_ZMP();
                // Calculate the COM trajectory via MPC
// std::cout << "here"  << std::endl;
                this->calc_next_COM_position();
// std::cout << "here"  << std::endl;
                // Determine foot and COM states via FK
                this->determine_states_from_FK(dxl_actuator);
                // Estimate the State based on the position via FK and ZMP measurement
                this->estimateState();
                // Calculate the pitch and roll angles such that the robot upper body will be upright.
                if(parameters.control_pitch == 1){
                    this->controlPitch();
                }
                if(parameters.control_pitch == 1 && parameters.control_roll == 1){
                    this->controlRoll();
                }

                // If there is foot landing control, start foot landing control
                if( (walking_mode == 0 || walking_mode == 1) && (parameters.control_leg_length == 1)) {  
                    COM       = x_ref.at(time_step), ref_y_swing_foot_trajectory.at(time_step), z_ref.at(time_step);
                    ankle     = ref_x_swing_foot_trajectory.at(time_step), ref_y_swing_foot_trajectory.at(time_step), ref_z_swing_foot_trajectory.at(time_step);
                    // ankle       = ref_x_support_foot_trajectory.at(time_step)+parameters.step_length, ref_y_swing_foot_trajectory.at(time_step), parameters.robot_ankle_to_foot;

                    start_foot_landing_control();          
                }
// std::cout << "1" << std::endl;
                // Based on which leg is support and swing leg, determine the position and orientation for the feet and COM
                if(leftIsSupportLeg){
                    // std::cout << "left" << std::endl;

                    if( parameters.control_leg_length == 1 ){
                        modify_swing_foot_trajectory(m_gati_data[1].Fx, m_gati_data[1].Fz);
                        right_swing_foot_state.p = ref_x_swing_foot_trajectory.at(time_step), ref_y_swing_foot_trajectory.at(time_step), ref_z_swing_foot_trajectory.at(time_step);
                        right_swing_foot_state.R = Ryaw(0)*Rpitch(0)*Rroll(0);
                        left_foot = left_support_foot_state;
                        if(parameters.control_pitch == 1){
                            left_foot.R = Ryaw(orientation_offset_l(2))*Rpitch(orientation_offset_l(1)+body_theta_fk)*Rroll(orientation_offset_l(0));
                        }
                        else{
                            left_foot.R = Ryaw(orientation_offset_l(2))*Rpitch(orientation_offset_l(1))*Rroll(orientation_offset_l(0));   
                        }

                    }     
                    else{
                        left_foot.p     = ref_x_support_foot_trajectory.at(time_step), ref_y_support_foot_trajectory.at(time_step), ref_z_support_foot_trajectory.at(time_step);
                        // left_foot.R     = Ryaw(0)*Rpitch(0)*Rroll(0);
                                    // std::cout << "2 " << std::endl;

                        left_foot.R     = Ryaw(ref_psi_support_foot_trajectory.at(time_step))*Rpitch(0)*Rroll(0);

            // std::cout << "3 " << std::endl;

                    }

                    if((in_foot_landing_control_phase == 0) && (time_step > 2*parameters.t_dsp/parameters.Ts) && parameters.control_leg_length == 1){
                        right_foot = right_support_foot_state;
                        // std::cout << "right foot" << right_foot.p(0) << std::endl;
                        right_foot.p(1) = ref_y_swing_foot_trajectory.at(time_step);
                        if(parameters.control_pitch == 1){
                            right_foot.R = Ryaw(orientation_offset_r(2))*Rpitch(orientation_offset_r(1)+body_theta_fk)*Rroll(orientation_offset_r(0));
                        }
                        else{
                            right_foot.R = Ryaw(orientation_offset_r(2))*Rpitch(orientation_offset_r(1))*Rroll(orientation_offset_r(0));   
                        }
                    }
                    else{
                        right_foot.p    =   ref_x_swing_foot_trajectory.at(time_step), ref_y_swing_foot_trajectory.at(time_step), ref_z_swing_foot_trajectory.at(time_step);
                        if ( (walking_mode == 0) ){
                            if( (orientation_offset_r(0) < 10*pi/180) && (orientation_offset_r(2) < 10*pi/180) && (parameters.control_leg_length == 1) ){
                                // if (parameters.control_roll == 1 && parameters.control_pitch == 1){
                                if ( parameters.control_pitch == 1 ){
                                    // right_foot.R    =   Ryaw(orientation_offset_r(2))*Rpitch(ref_theta_swing_foot_trajectory.at(time_step)+orientation_offset_r(1)-body_theta_fk)*Rroll(orientation_offset_r(0)+body_phi_fk);
                                    right_foot.R    =   Ryaw(orientation_offset_r(2))
                                    *Rpitch(ref_theta_swing_foot_trajectory.at(time_step)+orientation_offset_r(1)+body_theta_fk)
                                    *Rroll(orientation_offset_r(0));
                                }
                                else{
                                    right_foot.R    =   Ryaw(orientation_offset_r(2))*Rpitch(ref_theta_swing_foot_trajectory.at(time_step)+orientation_offset_r(1))*Rroll(orientation_offset_r(0));
                                }    

                            }
                            else{
                                right_foot.R    =   Ryaw(0)*Rpitch(ref_theta_swing_foot_trajectory.at(time_step))*Rroll(0*pi/180);
                                // std::cout << "Orientation offset to large: " << dlib::trans(orientation_offset_r) << std::endl;
                            }
                        }
                        right_foot.R    =   Ryaw(ref_psi_swing_foot_trajectory.at(time_step))*Rpitch(ref_theta_swing_foot_trajectory.at(time_step))*Rroll(0*pi/180);
                    }
                    body.p          = x_ref.at(time_step), y_ref.at(time_step), z_ref.at(time_step);
                    if(parameters.control_pitch == 1){
                        if(parameters.control_pitch == 1 && parameters.control_roll == 0){
                            body.R          = Ryaw(0)*Rpitch(body_theta_fk)*Rroll(0); 
                        }   
                        else if (parameters.control_roll == 1 && parameters.control_pitch == 1){
                            body.R          = Ryaw(0)*Rpitch(body_theta_fk)*Rroll(body_phi_fk);
                        }            
                    }
                    rotm2eul(right_foot.R,&phi2, &theta2, &psi2);

                    body.R          = Ryaw(ref_psi_body.at(time_step))*Rpitch(0)*Rroll(0);
                }
                else{
                    // std::cout << "right" << std::endl;
                    if( parameters.control_leg_length == 1 ){
                        modify_swing_foot_trajectory(m_gati_data[0].Fx, m_gati_data[0].Fz);
                        left_swing_foot_state.p = ref_x_swing_foot_trajectory.at(time_step), ref_y_swing_foot_trajectory.at(time_step), ref_z_swing_foot_trajectory.at(time_step);
                        left_swing_foot_state.R = Ryaw(0)*Rpitch(0)*Rroll(0);
                        right_foot = right_support_foot_state;
                        if(parameters.control_pitch == 1 ){
                            right_foot.R = Ryaw(orientation_offset_r(2))*Rpitch(orientation_offset_r(1)+body_theta_fk)*Rroll(orientation_offset_r(0));                    
                        }
                        else{
                            right_foot.R = Ryaw(orientation_offset_r(2))*Rpitch(orientation_offset_r(1))*Rroll(orientation_offset_r(0));
                        }
                    }
                    else{
                        // std::cout << "1"  << std::endl;
                        // std::cout << "length: " << ref_psi_support_foot_trajectory.size() << std::endl;
                        right_foot.p    = ref_x_support_foot_trajectory.at(time_step), ref_y_support_foot_trajectory.at(time_step), ref_z_support_foot_trajectory.at(time_step);
                        right_foot.R     = Ryaw(ref_psi_support_foot_trajectory.at(time_step))*Rpitch(0)*Rroll(0);
                                                // std::cout << "2"  << std::endl;

                    }

                    if((in_foot_landing_control_phase == 0) && (time_step > 2*parameters.t_dsp/parameters.Ts) && parameters.control_leg_length == 1){
                        left_foot = left_support_foot_state;
                            // std::cout << "left foot" << left_foot.p(0) << std::endl;
                        left_foot.p(1) = ref_y_swing_foot_trajectory.at(time_step);
                        if(parameters.control_pitch == 1){
                            left_foot.R = Ryaw(orientation_offset_l(2))*Rpitch(orientation_offset_l(1)+body_theta_fk)*Rroll(orientation_offset_l(0));
                        }
                        else{
                            left_foot.R = Ryaw(orientation_offset_l(2))*Rpitch(orientation_offset_l(1))*Rroll(orientation_offset_l(0));   
                        }
                    }
                    else{
                                                // std::cout << "3"  << std::endl;

                        left_foot.p     =   ref_x_swing_foot_trajectory.at(time_step), ref_y_swing_foot_trajectory.at(time_step), ref_z_swing_foot_trajectory.at(time_step);
                                                // std::cout << "4"  << std::endl;

                        if( walking_mode == 0 ){
                            if( (orientation_offset_l(0) < 10*pi/180) && (orientation_offset_l(2) < 10*pi/180) && (parameters.control_leg_length == 1) ){
                                    // if (parameters.control_roll == 1 && parameters.control_pitch == 1){
                                if (parameters.control_pitch == 1){

                                        // left_foot.R    =   Ryaw(orientation_offset_l(2))*Rpitch(ref_theta_swing_foot_trajectory.at(time_step)+orientation_offset_l(1)-body_theta_fk)*Rroll(orientation_offset_l(0)+body_phi_fk);
                                    left_foot.R    =   Ryaw(orientation_offset_l(2))
                                    *Rpitch(ref_theta_swing_foot_trajectory.at(time_step)+orientation_offset_l(1)+body_theta_fk)
                                    *Rroll(orientation_offset_l(0));
                                }
                                else{
                                    left_foot.R    =   Ryaw(orientation_offset_l(2))*Rpitch(ref_theta_swing_foot_trajectory.at(time_step)+orientation_offset_l(1))*Rroll(orientation_offset_l(0));    
                                }

                            }
                            else{
// std::cout << "length: " << ref_psi_support_foot_trajectory.size() << std::endl;
                                left_foot.R    = Ryaw(ref_psi_swing_foot_trajectory.at(time_step))*Rpitch(ref_theta_swing_foot_trajectory.at(time_step))*Rroll(0*pi/180);
            // std::cout << "1 " << std::endl;

                                    // std::cout << "Orientation offset to large: " << dlib::trans(orientation_offset_l) << std::endl;
                            }
                        }
                                            // std::cout << "5 " << std::endl;
                        left_foot.R    = Ryaw(ref_psi_swing_foot_trajectory.at(time_step))*Rpitch(ref_theta_swing_foot_trajectory.at(time_step))*Rroll(0*pi/180);
                                            // std::cout << "5 " << std::endl;

                    }


                    body.p          = x_ref.at(time_step), y_ref.at(time_step), z_ref.at(time_step);
                    rotm2eul(left_foot.R,&phi1, &theta1, &psi1);




                    if(parameters.control_pitch == 1){
                        if(parameters.control_pitch == 1 && parameters.control_roll == 0){
                            body.R          = Ryaw(0)*Rpitch(body_theta_fk)*Rroll(0);    
                        }
                        else if (parameters.control_roll == 1 && parameters.control_pitch == 1){
                            body.R          = Ryaw(0)*Rpitch(body_theta_fk)*Rroll(body_phi_fk);
                        }
                    }


                    body.R          = Ryaw(ref_psi_body.at(time_step))*Rpitch(0)*Rroll(0);
                            // body.R          = Ryaw(0)*Rpitch(body_theta_ref_l_sup)*Rroll(0);
                            // std::cout << "i:" << time_step << ", Rightphase: " << in_foot_landing_control_phase << ", Theta pitch: " << body_theta_ref_l_sup << std::endl;
                }

                        // Prevent the robot from swaying to the side too much (if MPC is instable, the robot may overshoot over 25cm)
                // if((body.p(1)) > 0.25){
                //     body.p(1) = 0.25;
                //     std::cout << "i: " << time_step << "," << body.p(1) << ": exceeds 0.25" << std::endl;
                // }
                // else if ((body.p(1) < -0.25)){
                //     body.p(1) = -0.25;
                //     std::cout << "i: " << time_step << "," << body.p(1) << ": exceeds -0.25" << std::endl;
                // }

                        // Use IK to calculate the joint angles
                calculate_walking_pattern_ankle();
                dlib::matrix<double, 6, 1> state_tmp1;

                ForwardKinematic(1, dlib::rowm(joint_angles*180/pi, dlib::range(0,5)), &state_tmp1);
                phi1 = state_tmp1(3);
                theta1 = state_tmp1(4);
                psi1 = state_tmp1(5);

                ForwardKinematic(0, dlib::rowm(joint_angles*180/pi, dlib::range(6,11)), &state_tmp1);
                phi2 = state_tmp1(3);
                theta2 = state_tmp1(4);
                psi2 = state_tmp1(5);
                // std::cout << "===========" << std::endl;
                // return 1;
            }
            else{
                std::cout << "Finished walk" << std::endl;
                std::cout << "=====" << std::endl;
                // return 0;

                        // if(time_step == ((parameters.steps_real)*parameters.step_duration/parameters.Ts) ){
                        //     // this->printJointAnglesToFile();
                        //     // this->drawTrajectory();
                        // }
            }

                    // for saving it in gait_package
            q1.push_back(joint_angles(0));
            q2.push_back(joint_angles(1));
            q3.push_back(joint_angles(2));
            q4.push_back(joint_angles(3));
            q5.push_back(joint_angles(4));
            q6.push_back(joint_angles(5));
            q7.push_back(joint_angles(6));
            q8.push_back(joint_angles(7));
            q9.push_back(joint_angles(8));
            q10.push_back(joint_angles(9));
            q11.push_back(joint_angles(10));
            q12.push_back(joint_angles(11));

                    // Send the joint angles to the actuator
            dxl_actuator[0].set_goal_theta(joint_angles(0)*180/pi);           
            dxl_actuator[1].set_goal_theta(joint_angles(1)*180/pi*parameters.Tipping_Scale_left);           
            dxl_actuator[2].set_goal_theta(joint_angles(2)*180/pi);  
            dxl_actuator[3].set_goal_theta(joint_angles(3)*180/pi); 
            dxl_actuator[4].set_goal_theta(joint_angles(4)*180/pi);   
            dxl_actuator[5].set_goal_theta(joint_angles(5)*180/pi);  

            dxl_actuator[6].set_goal_theta(joint_angles(6)*180/pi);          
            dxl_actuator[7].set_goal_theta(joint_angles(7)*180/pi*parameters.Tipping_Scale_right);           
            dxl_actuator[8].set_goal_theta(joint_angles(8)*180/pi);  
            dxl_actuator[9].set_goal_theta(joint_angles(9)*180/pi); 
            dxl_actuator[10].set_goal_theta(joint_angles(10)*180/pi);    
            dxl_actuator[11].set_goal_theta(joint_angles(11)*180/pi);  
                    // Update time step
            time_step++;

            // if( time_step < ( (parameters.steps_real)*parameters.step_duration/parameters.Ts -1) ){
            if( time_step < parameters.N-1){
                        // push into gait packet and send via udp in P1MC.cpp
                push_into_gait_packet(gait_packet);
            }
            // if( time_step < ( (parameters.steps_real)*parameters.step_duration/parameters.Ts -1) ){

            if (time_step < parameters.N-1){
                return 1;
            
            }
            else{
                return 0;
            }

        };
/*******************************Kicking Part START*******************************/


void Robot::kick(bool isLeftLeg, int kickType, double yaw_angle) {
    walking_state = 8;
    double target_x = 0.2;
    double target_y = 0.09;
    double target_z = parameters.robot_ankle_to_foot + LIFT_HEIGHT;
    double kick_pitch = 0; //-pi/10;
    double landing_pitch = -pi/12;
    
    int size=0;
    kick_calc_swing(isLeftLeg, kickType, KICK_DISTANCE , KICK_DISTANCE , target_z, yaw_angle, kick_pitch, landing_pitch);
    std::cout << "before support" << std::endl;
    kick_calc_support(isLeftLeg,kickType);
    std::cout << "zmp" << std::endl;
    kick_calc_zmp(isLeftLeg, kickType);

    //return the size of trajectory
    size=ref_x_support_foot_trajectory.size();

    // push_into_gait_packet(gait_packet);
    std::cout<<"length of ref_x_swing_foot_trajectory"<<ref_x_swing_foot_trajectory.size()<<endl;
    std::cout<<"length of ref_x_support_foot_trajectory"<<ref_x_support_foot_trajectory.size()<<endl;
    std::cout<<"length of ref_x_zmp_trajectory"<<ref_x_zmp_trajectory.size()<<endl;
    parameters.N = size;
    // return size;
    ref_psi_body.insert(ref_psi_body.end(), parameters.N, 0);
    set_MPC_initial_position(ref_x_zmp_trajectory.at(0), ref_y_zmp_trajectory.at(0));

}

//Kicking Swing Leg Calculation, created by rpf
void Robot::kick_calc_swing(bool isLeftLeg, int kickType, double target_x,double target_y, double target_z, double yaw_angle, double kick_pitch,double landing_pitch) {

//    const int pointNum = 7;
    double kick_swing_z = 0.05;
    //Initial
    if(isLeftLeg){
        ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(),KICK_INIT_TIME/parameters.Ts, 0);
        ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(),KICK_INIT_TIME/parameters.Ts,parameters.robot_width/2);
        ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(),KICK_INIT_TIME/parameters.Ts, parameters.robot_ankle_to_foot);
        ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(),KICK_INIT_TIME/parameters.Ts,0);
        ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(),KICK_INIT_TIME/parameters.Ts,0);
    }
    else{
        ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(),KICK_INIT_TIME/parameters.Ts, 0);
        ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(),KICK_INIT_TIME/parameters.Ts,-parameters.robot_width/2);
        ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(),KICK_INIT_TIME/parameters.Ts, parameters.robot_ankle_to_foot);
        ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(),KICK_INIT_TIME/parameters.Ts,0);
        ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(),KICK_INIT_TIME/parameters.Ts,0);
    }

    //Move COM
    if(isLeftLeg){
        ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts, 0);
        ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts,parameters.robot_width/2);
        ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts, parameters.robot_ankle_to_foot);
        ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts,0);
        ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts,0);
    }
    else{
        ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts, 0);
        ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts,-parameters.robot_width/2);
        ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts, parameters.robot_ankle_to_foot);
        ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts,0);
        ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts,0);
    }
    
    double kick_time;
    //double kick2lift_time;
    double kick2dsp_time;
    switch(kickType)
    {
        case KICK_SOFT:
        {
            kick_time = KICK_SWING_TIME_SOFT;
            kick2dsp_time= KICK_KICK2DSP_TIME_SOFT;
            break;
        }
        case KICK_MEDIUM:
        {
            kick_time = KICK_SWING_TIME_MEDIUM;
            kick2dsp_time= KICK_KICK2DSP_TIME_MEDIUM;
            break;
        }
        case KICK_STRONG:
        {
            kick_time = KICK_SWING_TIME_STRONG;
            kick2dsp_time= KICK_KICK2DSP_TIME_STRONG;
            break;
        }
        default:
        {
            kick_time = KICK_SWING_TIME_STRONG;
            kick2dsp_time= KICK_KICK2DSP_TIME_STRONG;
            break;
        }
    }
    std::cout<<"kick_pitch = "<<kick_pitch<<", landing_pitch = "<<landing_pitch<<std::endl;
    if(isLeftLeg){
        //init to lift
        Robot::kick_swing_trajectory_generator(0, parameters.robot_width/2, parameters.robot_ankle_to_foot, 0, 0, LIFT_X_BACK , parameters.robot_width/2, parameters.robot_ankle_to_foot + LIFT_HEIGHT, 0, 0, KICK_LIFT_TIME,0);
        //lift to kick
        Robot::kick_swing_trajectory_generator(LIFT_X_BACK , parameters.robot_width/2, parameters.robot_ankle_to_foot + LIFT_HEIGHT, 0, 0, target_x, parameters.robot_width/2, target_z + kick_swing_z, kick_pitch, 0, kick_time,1);
        //kick to dsp
        // Robot::kick_swing_trajectory_generator(target_x, parameters.robot_width/2, target_z, kick_pitch, 0, DSP_KICKLEG_X, parameters.robot_width/2, parameters.robot_ankle_to_foot, landing_pitch, 0, kick2dsp_time, 2); 
    }
    else {
        //init to lift
        Robot::kick_swing_trajectory_generator(0, -parameters.robot_width/2, parameters.robot_ankle_to_foot, 0, 0, 0, -parameters.robot_width/2, parameters.robot_ankle_to_foot + LIFT_HEIGHT, 0, 0, KICK_LIFT_TIME,0);
        //lift to kick
        Robot::kick_swing_trajectory_generator(0, -parameters.robot_width/2, parameters.robot_ankle_to_foot + LIFT_HEIGHT, 0, 0, target_x, -parameters.robot_width/2, target_z + kick_swing_z, kick_pitch, 0, KICK_SWING_TIME_STRONG,1);
        //kick to dsp
        // Robot::kick_swing_trajectory_generator(target_x, -parameters.robot_width/2, target_z, kick_pitch, 0, DSP_KICKLEG_X, -parameters.robot_width/2, parameters.robot_ankle_to_foot, landing_pitch, 0, kick2dsp_time, 2); 
    }

    //dsp
    // if(isLeftLeg){
    //     ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(),KICK_DSP_TIME/parameters.Ts, DSP_KICKLEG_X);
    //     ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(),KICK_DSP_TIME/parameters.Ts,parameters.robot_width/2);
    //     ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(),KICK_DSP_TIME/parameters.Ts, parameters.robot_ankle_to_foot);
    //     ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(),KICK_DSP_TIME/parameters.Ts,0);
    //     ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(),KICK_DSP_TIME/parameters.Ts,0);
    // }
    // else{
    //     ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(),KICK_DSP_TIME/parameters.Ts, DSP_KICKLEG_X);
    //     ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(),KICK_DSP_TIME/parameters.Ts,-parameters.robot_width/2);
    //     ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(),KICK_DSP_TIME/parameters.Ts, parameters.robot_ankle_to_foot);
    //     ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(),KICK_DSP_TIME/parameters.Ts,0);
    //     ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(),KICK_DSP_TIME/parameters.Ts,0);
    // }

    //move a step
    // if(isLeftLeg){
    //     Robot::kick_swing_trajectory_generator(0, -parameters.robot_width/2, parameters.robot_ankle_to_foot, 0, 0, DSP_KICKLEG_X , -parameters.robot_width/2, parameters.robot_ankle_to_foot, 0, 0, KICK_LAST_STEP_TIME,3);
    // }
    // else {
    //     Robot::kick_swing_trajectory_generator(0, parameters.robot_width/2, parameters.robot_ankle_to_foot, 0, 0, DSP_KICKLEG_X, parameters.robot_width/2, parameters.robot_ankle_to_foot, 0, 0, KICK_LAST_STEP_TIME,3);
    // }

    //move CoM back to middle
    // if(isLeftLeg){
    //     ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts, DSP_KICKLEG_X);
    //     ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts,-parameters.robot_width/2);
    //     ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts, parameters.robot_ankle_to_foot);
    //     ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts,0);
    //     ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts,0);
    // }
    // else{
    //     ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts, DSP_KICKLEG_X);
    //     ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts, parameters.robot_width/2);
    //     ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts, parameters.robot_ankle_to_foot);
    //     ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts,0);
    //     ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(),KICK_COMTRANS_TIME/parameters.Ts,0);
    // }
}

<<<<<<< HEAD
//Kick Swing Trajectory Generator, created by liuyuezhang 2017-7-16
void Robot::kick_swing_trajectory_generator(double startX, double startY, double startZ, double startTheta, double startPsi, 
                                            double endX, double endY, double endZ, double endTheta, double endPsi, double time,int motionMode) {
    const int pointNum = 7;
    double t[pointNum];
    const int motionModeNum=4;
    double last_step_height =0.05;

    for(int i = 0 ;i < pointNum; i++){
        t[i] = time * i / (pointNum - 1.0);
    }

    double offsetX = endX - startX;
    double offsetY = endY - startY;
    double offsetZ = endZ - startZ;
    double offsetTheta = endTheta - startTheta;//theta is pitch
    double offsetPsi = endPsi - startPsi;//psi is yaw

    double x[pointNum], y[pointNum], z[pointNum], theta[pointNum], psi[pointNum];
    double propPara[motionModeNum][pointNum] = {
                                                    {0, 0.1, 0.4, 0.7, 0.9, 0.97, 1.0}, //lift
                                                    {0, 0.05, 0.23, 0.5, 0.81, 0.97, 1.0}, //kick
                                                    {0, 0.1 , 0.4, 0.7, 0.9, 0.97, 1.0}, //kick_2_dsp
                                                    {0, 0.1, 0.4, 0.7, 0.9, 0.97, 1.0}  //move a step
                                                };
    for(int i = 0; i < pointNum; i++) {
        x[i] = startX + offsetX * propPara[motionMode][i];
        y[i] = startY + offsetY * propPara[motionMode][i];
        z[i] = startZ + offsetZ * propPara[motionMode][i];
        theta[i] = startTheta + offsetTheta * propPara[motionMode][i];
        psi[i] = startPsi + offsetPsi * propPara[motionMode][i];
    }
    //move another foot, motionMode == KICK_MODE_MOVE_A_STEP
    if(motionMode == 3) {
        z[0]= startZ;
        z[1]= startZ + last_step_height*0.5;
        z[2]= startZ + last_step_height*1.0;
        z[3]= startZ + last_step_height*1.0;
        z[4]= endZ   + last_step_height*1.0;
        z[5]= endZ   + last_step_height*0.5;
        z[6]= endZ;
    }
    
    const vector<double> resT(t, t + pointNum);
    const vector<double> resX(x, x + pointNum);
    const vector<double> resY(y, y + pointNum);
    const vector<double> resZ(z, z + pointNum);
    const vector<double> resTheta(theta, theta + pointNum);
    const vector<double> resPsi(psi, psi + pointNum);

    tk::spline x_ref, y_ref, z_ref, theta_ref, psi_ref;
    x_ref.set_points(resT, resX);
    y_ref.set_points(resT, resY);
    z_ref.set_points(resT, resZ);
    theta_ref.set_points(resT, resTheta);
    psi_ref.set_points(resT, resPsi);

    //std::cout<<"swing begin: " << ref_x_swing_foot_trajectory.size() << std::endl;
    //lyz: using double may cause bugs here - float percise error
    for(int i = 0; i < time/parameters.Ts; i++) {
        ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(), 1, x_ref(i * parameters.Ts));
        ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(), 1, y_ref(i * parameters.Ts));
        ref_z_swing_foot_trajectory.insert(ref_z_swing_foot_trajectory.end(), 1, z_ref(i * parameters.Ts));
        ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(), 1, theta_ref(i * parameters.Ts));
        std::cout<<"theta_ref["<<i<<"]= "<<theta_ref(i * parameters.Ts)<<std::endl;
        ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(), 1, psi_ref(i * parameters.Ts));
    }
    //std::cout<<"swing end: " << ref_x_swing_foot_trajectory.size() << std::endl;
=======
//The Spline Function, created by liuyuezhang 2017-7-18
void Robot::kick_swing_trajectory_generator_one_dimension(int mode, double startX, double offsetX, int pointNumX, double* parametersX, double time, int pointNumT, double* parametersT) {
    //Data Spline
    double ResX[pointNumX];
    for(int i = 0; i < pointNumX; i++) {
        ResX[i] = startX + offsetX * parametersX[i];
    }
    const vector<double> resX(ResX, ResX + pointNumX);
    //T Spline
    double ResT[pointNumT];
    for(int i = 0; i < pointNumT; i++) {
        ResT[i] = time * parametersT[i];
    }
    const vector<double> resT(ResT, ResT + pointNumT);

    tk::spline ref;
    ref.set_points(resT, resX);

    switch (mode) {
        case 1: {
            //lyz: using double may cause bugs here - float percise error
            for(int i = 0; i < time/parameters.Ts; i++) {
                ref_x_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(), 1, ref(i * parameters.Ts));
            }
            break;
        }
        case 2: {
            for(int i = 0; i < time/parameters.Ts; i++) {
                ref_y_swing_foot_trajectory.insert(ref_y_swing_foot_trajectory.end(), 1, ref(i * parameters.Ts));
            }
            break;
        }
        case 3: {
            for(int i = 0; i < time/parameters.Ts; i++) {
                ref_z_swing_foot_trajectory.insert(ref_x_swing_foot_trajectory.end(), 1, ref(i * parameters.Ts));
            }
            break;
        }
        case 4: {
            for(int i = 0; i < time/parameters.Ts; i++) {
                ref_theta_swing_foot_trajectory.insert(ref_theta_swing_foot_trajectory.end(), 1, theta_ref(i * parameters.Ts));
            }
            break;
        }
        case 5: {
            for(int i = 0; i < time/parameters.Ts; i++) {
                ref_psi_swing_foot_trajectory.insert(ref_psi_swing_foot_trajectory.end(), 1, psi_ref(i * parameters.Ts));
            }
            break;
        }
        default: {
            break;
        }
    }
}

//Kick Swing Trajectory Generator, created by liuyuezhang 2017-7-16, modified by liuyuezhang 2017-7-18
//start: startPoint, end: endPoint, parameters: spice parameters, time: kick time
void Robot::kick_swing_trajectory_generator(kickPoint* start, kickPoint* end, kickParameters* parameters, double time) {

    kickPoint offset;
    offset.x = end.x - start.x;
    offset.y = end.y - start.y;
    offset.z = end.z - start.z;
    offset.pitch = end.pitch - start.pitch;
    offset.yaw = end.yaw - start.yaw;

    kick_swing_trajectory_generator_one_dimension(1, start.x, offset.x, parameters->x->num, parameters->x->data, time,
                                                  parameters->t_x->num, parametes->t_x->data);
    kick_swing_trajectory_generator_one_dimension(2, start.y, offset.y, parameters->y->num, parameters->y->data, time,
                                                  parameters->t_y->num, parametes->t_y->data);
    kick_swing_trajectory_generator_one_dimension(3, start.z, offset.z, parameters->z->num, parameters->z->data, time,
                                                  parameters->t_z->num, parametes->t_z->data);
    kick_swing_trajectory_generator_one_dimension(4, start.pitch, offset.pitch, parameters->pitch->num, parameters->pitch->data, time,
                                                  parameters->t_pitch->num, parametes->t_pitch->data);
    kick_swing_trajectory_generator_one_dimension(5, start.yaw, offset.yaw, parameters->yaw->num, parameters->yaw->data, time,
                                                  parameters->t_yaw->num, parametes->t_yaw->data);
>>>>>>> ec9eee8b8782be83cecaf34638371c46800dc52f
}

//Kicking Support Leg Calculation, created by lhy
void Robot::kick_calc_support(bool isLeftLeg, int kickType) {
    double kick_time;
    double kick2dsp_time;
    switch(kickType)
    {
        case KICK_SOFT:
        {
            kick_time = KICK_SWING_TIME_SOFT;
            kick2dsp_time= KICK_KICK2DSP_TIME_SOFT;
            break;
        }
        case KICK_MEDIUM:
        {
            kick_time = KICK_SWING_TIME_MEDIUM;
            kick2dsp_time= KICK_KICK2DSP_TIME_MEDIUM;
            break;
        }
        case KICK_STRONG:
        {
            kick_time = KICK_SWING_TIME_STRONG;
            kick2dsp_time= KICK_KICK2DSP_TIME_STRONG;
            break;
        }
        default:
        {
            kick_time = KICK_SWING_TIME_STRONG;
            kick2dsp_time= KICK_KICK2DSP_TIME_STRONG;
            break;
        }
    }
    std::cout << "x legntzh: " << ref_x_support_foot_trajectory.size() << std::endl;
    //init to dsp
    std::cout << (KICK_INIT_TIME + KICK_COMTRANS_TIME + KICK_LIFT_TIME+ kick_time + kick2dsp_time +  KICK_DSP_TIME) << std::endl;
    if(isLeftLeg){
        ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(),ceil((KICK_INIT_TIME + KICK_COMTRANS_TIME + KICK_LIFT_TIME+ kick_time )/parameters.Ts), 0); // + kick2dsp_time +  KICK_DSP_TIME)/parameters.Ts), 0);
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(),ceil((KICK_INIT_TIME + KICK_COMTRANS_TIME + KICK_LIFT_TIME+ kick_time )/parameters.Ts),-(parameters.robot_width/2)); // + kick2dsp_time +  KICK_DSP_TIME)/parameters.Ts),-(parameters.robot_width/2));
    }
    else{
        ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(),ceil((KICK_INIT_TIME + KICK_COMTRANS_TIME + KICK_LIFT_TIME+ kick_time )/parameters.Ts), 0); // + kick2dsp_time +  KICK_DSP_TIME)/parameters.Ts), 0);
        ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(),ceil((KICK_INIT_TIME + KICK_COMTRANS_TIME + KICK_LIFT_TIME+ kick_time )/parameters.Ts),parameters.robot_width/2); //kick2dsp_time +  KICK_DSP_TIME)/parameters.Ts),parameters.robot_width/2);
    }
    std::cout << "x legntzh: " << ref_x_support_foot_trajectory.size() << std::endl;

    ref_z_support_foot_trajectory.insert(ref_z_support_foot_trajectory.end(),(KICK_INIT_TIME + KICK_COMTRANS_TIME + KICK_LIFT_TIME+ kick_time)/parameters.Ts,parameters.robot_ankle_to_foot);// + kick2dsp_time +  KICK_DSP_TIME)/parameters.Ts,parameters.robot_ankle_to_foot);
    ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.end(),(KICK_INIT_TIME + KICK_COMTRANS_TIME + KICK_LIFT_TIME+ kick_time)/parameters.Ts,0);// + kick2dsp_time +  KICK_DSP_TIME)/parameters.Ts,0);

    //last step + move com to middle
    // if(isLeftLeg){
    //     ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(),(KICK_LAST_STEP_TIME+ KICK_COMTRANS_TIME)/parameters.Ts, DSP_KICKLEG_X);
    //     ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(),(KICK_LAST_STEP_TIME+ KICK_COMTRANS_TIME)/parameters.Ts, parameters.robot_width/2); //switch to left leg is support leg
    // }
    // else{
    //     ref_x_support_foot_trajectory.insert(ref_x_support_foot_trajectory.end(),(KICK_LAST_STEP_TIME+ KICK_COMTRANS_TIME)/parameters.Ts, DSP_KICKLEG_X);
    //     ref_y_support_foot_trajectory.insert(ref_y_support_foot_trajectory.end(),(KICK_LAST_STEP_TIME+ KICK_COMTRANS_TIME)/parameters.Ts, -(parameters.robot_width/2)); //switch to right leg is support leg
    // }
    // ref_z_support_foot_trajectory.insert(ref_z_support_foot_trajectory.end(),(KICK_LAST_STEP_TIME+ KICK_COMTRANS_TIME)/parameters.Ts,parameters.robot_ankle_to_foot);
    // ref_psi_support_foot_trajectory.insert(ref_psi_support_foot_trajectory.end(),(KICK_LAST_STEP_TIME+ KICK_COMTRANS_TIME)/parameters.Ts,0);
    std::cout << "x legntzh: " << ref_x_support_foot_trajectory.size() << std::endl;

}

//Kicking ZMP Calculation, created by csj
void Robot::kick_calc_zmp(bool isLeftLeg, int kickType) {
    int invert_y;
    if(isLeftLeg == true)//kick with left legin
        invert_y = -1;
    else
        invert_y = 1;

    double kick_time;

    //double kick2lift_time;
    switch(kickType)
    {
        case KICK_SOFT:
        {
            kick_time = KICK_SWING_TIME_SOFT; //+ KICK_KICK2DSP_TIME_SOFT;
            
            break;
        }
        case KICK_MEDIUM:
        {
            kick_time = KICK_SWING_TIME_MEDIUM; //+ KICK_KICK2DSP_TIME_MEDIUM;
            
            break;
        }
        case KICK_STRONG:
        {
            kick_time = KICK_SWING_TIME_STRONG; //+ KICK_KICK2DSP_TIME_STRONG;
            
            break;
        }
        default:
        {
            kick_time = KICK_SWING_TIME_STRONG; //+ KICK_KICK2DSP_TIME_STRONG;
            
            break;
        }
    }
    kick_time += KICK_LIFT_TIME;

    //init
    
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), KICK_INIT_TIME/parameters.Ts, 0);
    
    // ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), KICK_INIT_TIME/parameters.Ts/2, 0+parameters.zmp_offset_x);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), KICK_INIT_TIME/parameters.Ts, 0);

    //move to one side
    double com_trans_duration = ceil((KICK_COMTRANS_TIME / parameters.Ts) * KICK_COMTRANS_PERCENTAGE / 100.0);
    double stay_duration = ceil((KICK_COMTRANS_TIME / parameters.Ts) * (1.0 - KICK_COMTRANS_PERCENTAGE / 100.0));

    double x1 = ref_x_zmp_trajectory.back();
    double y1 = ref_y_zmp_trajectory.back();
    double x2 = parameters.zmp_offset_x;
    double y2 = invert_y*(parameters.robot_width/2 + KICK_INCREMENT);
    double cx = x1;
    double cy = y1;
    double mx = (x2-cx)/com_trans_duration;   //slope
    double my = (y2-cy)/com_trans_duration;
    // std::cout << "com_trans_duration is: " << com_trans_duration << endl;
    // std::cout << "stay_duration is: " << stay_duration << endl;
    // std::cout << "mx: " << mx << endl;
    // std::cout << "my: " << my << endl;Stop (Stop: 4) 

    for (int k = 0; k < com_trans_duration; k++){
        double x_increment = mx*k + cx;
        double y_increment = my*k + cy;
        ref_x_zmp_trajectory.push_back(x_increment);
        ref_y_zmp_trajectory.push_back(y_increment);
    }

    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), stay_duration, x2);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), stay_duration, y2);

//    //start kicking
    ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), kick_time/parameters.Ts, parameters.zmp_offset_x);
    ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), kick_time/parameters.Ts,invert_y*(parameters.robot_width/2 + KICK_INCREMENT));

//   dsp
    // com_trans_duration = ceil((KICK_DSP_TIME / parameters.Ts) * KICK_COMTRANS_PERCENTAGE / 100.0);
    // stay_duration = ceil((KICK_DSP_TIME / parameters.Ts) * (1.0 - KICK_COMTRANS_PERCENTAGE / 100.0));

    // x1 = ref_x_zmp_trajectory.back();
    // y1 = ref_y_zmp_trajectory.back();
    // x2 = DSP_KICKLEG_X+parameters.zmp_offset_x;
    // y2 = parameters.robot_width/2 + KICK_INCREMENT_DSP;
    // cx = x1;
    // cy = y1;
    // mx = (x2-cx)/com_trans_duration;   //slope
    // my = (y2-cy)/com_trans_duration;
    // for (int k = 0; k < com_trans_duration; k++) {
    //     double x_increment = mx*k + cx;
    //     double y_increment = my*k + cy;
    //     // std::cout << "my*k: " << my*k << endl;
    //     // std::cout << "y_inc: " << y_increment << endl;
    //     ref_x_zmp_trajectory.push_back(x_increment);
    //     ref_y_zmp_trajectory.push_back(y_increment);
    // }

    // ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), stay_duration, x2);
    // ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), stay_duration, y2);

    // //last step
    // ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), KICK_LAST_STEP_TIME/parameters.Ts, x2);
    // ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), KICK_LAST_STEP_TIME/parameters.Ts, y2);

    // //move back to the middle
    // com_trans_duration = ceil((KICK_COMTRANS_TIME / parameters.Ts) * KICK_COMTRANS_PERCENTAGE / 100.0);
    // stay_duration = ceil((KICK_COMTRANS_TIME / parameters.Ts) * (1.0 - KICK_COMTRANS_PERCENTAGE / 100.0));
    
    // x1 = ref_x_zmp_trajectory.back();
    // y1 = ref_y_zmp_trajectory.back();
    // x2 = DSP_KICKLEG_X;
    // y2 = 0;
    // cx = x1;
    // cy = y1;
    // mx = (x2-cx)/com_trans_duration;   //slope
    // my = (y2-cy)/com_trans_duration;
    // for (int k = 0; k < com_trans_duration; k++) {
    //     double x_increment = mx*k + cx;
    //     double y_increment = my*k + cy;
    //     ref_x_zmp_trajectory.push_back(x_increment);
    //     ref_y_zmp_trajectory.push_back(y_increment);
    // }

    // ref_x_zmp_trajectory.insert(ref_x_zmp_trajectory.end(), stay_duration, x2);
    // ref_y_zmp_trajectory.insert(ref_y_zmp_trajectory.end(), stay_duration, y2);
}

/*******************************Kicking Part END*******************************/
