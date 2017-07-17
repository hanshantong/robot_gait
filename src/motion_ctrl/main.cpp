#include "zmp_ctrl.h" // Generate Walking Pattern

std::vector<double> x_ref_fk, y_ref_fk;
std::vector<double> clipping1, clipping2, clipping3, clipping4, clipping5, clipping6, clipping7;
std::vector<double> theta_body;
std::vector<double> total_time;
std::vector<double> landing_phase;
// The calculated and measured ZMP is stored in this vector
std::vector<double> ZMP_x_measure_array, ZMP_y_measure_array; 
// Raw means that the center of the foot is the origin, otherwise the middle of the robot is the origin
std::vector<double> ZMP_xl, ZMP_xr, ZMP_yl, ZMP_yr, ZMP_xl_raw, ZMP_xr_raw, ZMP_yl_raw, ZMP_yr_raw; 
// These vectors are needed for RVIZ
std::vector<double> stamp, time_msg, t_vector;
// Calculated state for the left and right foot
std::vector<double> xl_fk, yl_fk, zl_fk, phil_fk, thetal_fk, psil_fk;
std::vector<double> xr_fk, yr_fk, zr_fk, phir_fk, thetar_fk, psir_fk;
// Measured Force and Torque
std::vector<double> F0_l, F1_l, F2_l, T0_l, T1_l, T2_l, F0_r, F1_r, F2_r, T0_r, T1_r, T2_r;
// Distribution of the vertical force on each leg: alpha = FzL/(FzL+FzR)
std::vector<double> alpha_vector;
// Orientation of each foot
std::vector<double> phi_l, theta_l, psi_l, phi_r, theta_r, psi_r;
// Offset between landed foot and reference foot trajectory
double xl_offset, xr_offset, zl_offset, zr_offset;
dlib::matrix<double,3,1> orientation_offset_l, orientation_offset_r;

double ZMP_x_measure, ZMP_y_measure, position_left_foot_x, position_right_foot_x, Fzl;
dlib::matrix<double,3,1> is_at_ref;
//cv::KalmanFilter KFx, KFy;
int ascended_before;
double acc_xh, acc_yh, acc_zh, acc_xf, acc_yf, acc_zf;
double gyro_xh, gyro_yh, gyro_zh, gyro_xf, gyro_yf, gyro_zf;

double t_initial, time_passed;

// Plotting the torque on the knees
std::vector<double> q4_vector, q10_vector;


std::vector<double> acc_xh_vector, acc_yh_vector, acc_zh_vector, acc_xf_vector, acc_yf_vector, acc_zf_vector;
std::vector<double> gyro_xh_vector, gyro_yh_vector, gyro_zh_vector, gyro_xf_vector, gyro_yf_vector, gyro_zf_vector;
dlib::matrix<double, 3,1> direction;

joint_state left_support_foot_state;
joint_state right_support_foot_state;
joint_state left_swing_foot_state;
joint_state right_swing_foot_state;

    int writing_servo            = 0;
    int reading_sensor           = 1;
    int control_leg_length       = 0;

    float ZMP[3]                           = {0.0};
    float ZMP2[3]                          = {0.0};
    float FORCE_DATA_l[3]                  = {0.0};
    float TORQUE_DATA_l[3]                 = {0.0};
    float FORCE_DATA_r[3]                  = {0.0};
    float TORQUE_DATA_r[3]                 = {0.0};
    float ATI_FT_BIAS_MATRIX[BIAS_NUM][6]  = {{0.0}};
    float ATI_FT_BIAS_MATRIX2[BIAS_NUM][6] = {{0.0}};
    float ATI_FT_BIAS[6]                   = {0.0};
    float ATI_FT_BIAS2[6]                  = {0.0};

    /*==========================================================
      ============================================================
      Force sensor
      ============================================================
      ============================================================*/

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = STOP_STREAM;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT,&sigIntHandler,NULL);
    int fd, fd2;

    /*==========================================================
      ============================================================
      Initializing others
      ============================================================
      ============================================================*/
    struct timeval tv;
    struct timezone tz;
    double Time_Initial;
    double Time_Now, t_control;
    double t_control_init = 0;


    // This is for rviz
    ros::init(argc, argv, "mpc_walking_pattern_generator");
    ros::NodeHandle n;
    ros::Publisher walking_generator = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher publisher_walking_data = n.advertise<biped_gait::walking_data_all>("walking_data", 1);
    ros::ServiceServer status_update = n.advertiseService("status_update",status_call);
    ros::Subscriber subGyroHip = n.subscribe("gyro_angle_hip", 1000, getGyroDataHip);
    ros::Subscriber subGyroFoot = n.subscribe("gyro_angle_lfoot", 1000, getGyroDataFoot);
    ros::Subscriber subZMP = n.subscribe("chatter", 1000, chatterCallback);
    ros::Rate loop_rate(200); // Frequency of the loop
    sensor_msgs::JointState msg_joint_state;
    msg_joint_state.name.resize(12);
    msg_joint_state.position.resize(12);
    msg_joint_state.name[0]  = "HipYawL";
    msg_joint_state.name[1]  = "HipRollL";
    msg_joint_state.name[2]  = "HipPitchL";
    msg_joint_state.name[3]  = "KneeL";
    msg_joint_state.name[4]  = "AnkleL";
    msg_joint_state.name[5]  = "Ankle2L";
    msg_joint_state.name[6]  = "HipYawR";
    msg_joint_state.name[7]  = "HipRollR";
    msg_joint_state.name[8]  = "HipPitchR";
    msg_joint_state.name[9]  = "KneeR";
    msg_joint_state.name[10] = "AnkleR";
    msg_joint_state.name[11] = "Ankle2R";

    biped_gait::gait_angle msg;
    biped_gait::walking_data_all walking_data_msg;
    // CoM trajectory
    std::vector<dlib::matrix<double,3,1>> X_ref, Y_ref, Z_ref, X_ref_all, X_ref_est, Y_ref_est, X_ref_pred, Y_ref_pred;
    std::vector<double> u_kx_array, u_ky_array;
    u_kx_array.push_back(0);
    u_ky_array.push_back(0);
    dlib::matrix<double, 3, 1> x_init, y_init;
    x_init = 0, 0, 0;
    y_init = 0, 0, 0;


    X_ref.push_back(x_init); 
    X_ref_est.push_back(x_init);
    X_ref_all.push_back(x_init); 
    Y_ref.push_back(y_init); 
    Y_ref_est.push_back(y_init);
    // Position of CoM
    std::vector<double> x_ref, y_ref, z_ref, x_ref_est, y_ref_est, x_ref_pred, y_ref_pred;
    // Support Foot Trajectory
    std::vector<double> p_ref_x, p_ref_y,p_ref_x_heel, p_ref_y_heel, p_ref_z, p_ref_x_follow, p_ref_y_follow;
    // Measured ZMP
    std::vector<float> ZMP_x, ZMP_x2, ZMP_y, ZMP_y2;
    // ZMP Reference
    std::vector<double> p_ref_x_transition, p_ref_y_transition, p_ref_z_transition, p_real;
    // Swing Foot Trajectory
    std::vector<double> x_support_complete, y_support_complete, z_support_complete;
    std::vector<double> x_swing_complete, y_swing_complete, z_swing_complete;
    std::vector<double> x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete;
    std::vector<double> z_ref_swing_foot_complete_unmodified;
    // Joint Angle position, velocity and acceleration
    std::vector<double> tau0, tau1, tau2, tau3, tau4, tau5, tau6, tau7, tau8, tau9, tau10, tau11;
    std::vector<double> q0, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11;
    std::vector<double> q0_ref, q1_ref, q2_ref, q3_ref, q4_ref, q5_ref, q6_ref, q7_ref, q8_ref, q9_ref, q10_ref, q11_ref;
    std::vector<double> q0d, q1d, q2d, q3d, q4d, q5d, q6d, q7d, q8d, q9d, q10d, q11d;
    std::vector<double> q0dd, q1dd, q2dd, q3dd, q4dd, q5dd, q6dd, q7dd, q8dd, q9dd, q10dd, q11dd;


    /*==========================================================
      ============================================================
      Calculate the three trajectories: swing foot, support foot and CoM
      ============================================================
      ============================================================*/
    // Calculate the support foot trajectory
    //for ssp

    calc_p_ref(parameter_init, &p_ref_x_heel, &p_ref_y_heel);
    calc_p_ref_heel(parameter_init, &p_ref_x, &p_ref_y);
    p_ref_z.assign(p_ref_x.size(), parameter_init.robot_ankle_to_foot);

    // Calculate the reference zmp trajectory

    calc_p_ref_transition(parameter_init, &p_ref_x_transition, &p_ref_y_transition);
    p_ref_z_transition.assign(p_ref_x_transition.size(), 0); //unused atm

    // Calculate swing foot piece

    calculate_swing_foot_complete_trajectory_heel(p_ref_x_heel, p_ref_y_heel, parameter_init,
            &x_ref_swing_foot_complete, &y_ref_swing_foot_complete, &z_ref_swing_foot_complete, &theta_ref_swing_foot_complete);
    z_ref_swing_foot_complete_unmodified = z_ref_swing_foot_complete;




    // LQR gains

    set_LQR_gains(parameter_init);
    /*==========================================================
      ============================================================
      Initializing Motor
      ============================================================
      ============================================================*/
    Initial(motor_angle);
    //Get Multiport DEVICENAME 
    // Initialize PortHandler instance
    PortHandler *portHandler_gait     = PortHandler::GetPortHandler(DEVICE_GAIT);

    PacketHandler *packetHandler = PacketHandler::GetPacketHandler(PROTOCOL_VERSION);
    if (writing_servo == 1)
    {

        if( portHandler_gait->OpenPort() ){
            printf( "Succeeded to open the port!\n" );
        }
        else{
            printf( "Failed to open the port!\n" );
            return 0;
        }

        // Set port baudrate
        if( portHandler_gait->SetBaudRate(BAUDRATE) ){
            printf( "Succeeded to change the baudrate!\n" );
        }
        else{
            printf( "Failed to change the baudrate!\n" );
            return 0;
        }

        for(int i = 0;i < 12; i++)
        {
            GroupA_Motor[i] = Motor(portHandler_gait, packetHandler, GroupA_Id[i], 200);
            GroupA_Motor[i].EnableTorque();
            GroupA_Motor[i].ReadPosition();
            GroupA_Motor[i].ReadVelocity();
            GroupA_Motor[i].DisableTorque();
            GroupA_Motor[i].JointMode();
            GroupA_Motor[i].EnableTorque();

            cout << "Motor ID: " << GroupA_Motor[i].id << ", Angle = " << GroupA_Motor[i].present_position_deg << endl;
            GroupA_Motor[i].initial_position_deg = motor_angle[i];
            GroupA_Motor[i].SetGoalVelocity(3000);
        }

        /* ============================================
        //// TEST FK
        ============================================ */
        //dlib::matrix<double, 6,1> q_swing2;
        //dlib::matrix<double, 6,1> current_state;
        ////// LEFT
        //q_swing2 =  -GroupA_Motor[0].present_position_deg,
        //-GroupA_Motor[1].present_position_deg,
        //+GroupA_Motor[2].present_position_deg,
        //-GroupA_Motor[3].present_position_deg,
        //-GroupA_Motor[4].present_position_deg,
        //+GroupA_Motor[5].present_position_deg;
        //ForwardKinematic(0, parameter_init, 0, q_swing2, &current_state); 
        //std::cout << dlib::trans(q_swing2) << std::endl;
        ////std::cout << "Current state: " << dlib::trans(current_state) << std::endl;
        ////std::cout << "Euler: " << current_state(3)*180/pi << "," << current_state(4)*180/pi<< "," << current_state(5)*180/pi<< std::endl;


        //// RIGHT

        //q_swing2 =  -GroupA_Motor[6].present_position_deg,
        //-GroupA_Motor[7].present_position_deg,
        //-GroupA_Motor[8].present_position_deg,
        //+GroupA_Motor[9].present_position_deg,
        //+GroupA_Motor[10].present_position_deg,
        //+GroupA_Motor[11].present_position_deg;

        //std::cout << dlib::trans(q_swing2) << std::endl;
        //ForwardKinematic(1, parameter_init, 0, q_swing2, &current_state); 
        ////std::cout << "Current state: " << dlib::trans(current_state) << std::endl;
        ////std::cout << "Euler: " << current_state(3)*180/pi<< "," << current_state(4)*180/pi<< "," << current_state(5)*180/pi<< std::endl;
        //return 0;

        //Add Slow Initial
        std::vector<dlib::matrix<double, 12, 1>> q_now_tmp;

        left_foot.p     = p_ref_x.at(0), p_ref_y.at(0), p_ref_z.at(0);
        right_foot.p    = x_ref_swing_foot_complete.at(0), y_ref_swing_foot_complete.at(0), z_ref_swing_foot_complete.at(0);
        body.p          = 0, 0, parameter_init.z_com_mbm;

        calculate_walking_pattern_ankle(left_foot, right_foot, body, parameter_init, &q_now_tmp);
        dlib::matrix<double, 12, 1> q_now = q_now_tmp.back()*180/pi;

        // Getting the current q12
        simu_position[0]  = q_now(0);
        simu_position[1]  = q_now(1);
        simu_position[2]  = -q_now(2);
        simu_position[3]  = -q_now(3);
        simu_position[4]  = -q_now(4);
        simu_position[5]  = q_now(5);

        simu_position[6]  = q_now(6);
        simu_position[7]  = -q_now(7);
        simu_position[8]  = -q_now(8);
        simu_position[9]  = -q_now(9);
        simu_position[10] = -q_now(10); 
        simu_position[11] = q_now(11);

        for(int i = 0; i < 12; i++){
            std::cout << simu_position[i] << std::endl;
        }

        GroupA_Degree_Goal[0]   = GroupA_Motor[0].initial_position_deg  + simu_position[0];
        GroupA_Degree_Goal[1]   = GroupA_Motor[1].initial_position_deg  + simu_position[1];
        GroupA_Degree_Goal[2]   = GroupA_Motor[2].initial_position_deg  - simu_position[2];
        GroupA_Degree_Goal[3]   = GroupA_Motor[3].initial_position_deg  + simu_position[3];
        GroupA_Degree_Goal[4]   = GroupA_Motor[4].initial_position_deg  + simu_position[4];
        GroupA_Degree_Goal[5]   = GroupA_Motor[5].initial_position_deg  - simu_position[5];
        GroupA_Degree_Goal[6]   = GroupA_Motor[6].initial_position_deg  + simu_position[6];
        GroupA_Degree_Goal[7]   = GroupA_Motor[7].initial_position_deg  + simu_position[7];
        GroupA_Degree_Goal[8]   = GroupA_Motor[8].initial_position_deg  + simu_position[8];
        GroupA_Degree_Goal[9]   = GroupA_Motor[9].initial_position_deg  - simu_position[9];
        GroupA_Degree_Goal[10]  = GroupA_Motor[10].initial_position_deg - simu_position[10];
        GroupA_Degree_Goal[11]  = GroupA_Motor[11].initial_position_deg - simu_position[11];

        //GroupA_Degree_Goal[0]   = GroupA_Motor[0].initial_position_deg  + 0;
        //GroupA_Degree_Goal[1]   = GroupA_Motor[1].initial_position_deg  + 0;
        //GroupA_Degree_Goal[2]   = GroupA_Motor[2].initial_position_deg  - 0;
        //GroupA_Degree_Goal[3]   = GroupA_Motor[3].initial_position_deg  + 0;
        //GroupA_Degree_Goal[4]   = GroupA_Motor[4].initial_position_deg  + 0;
        //GroupA_Degree_Goal[5]   = GroupA_Motor[5].initial_position_deg  - 0;
        //GroupA_Degree_Goal[6]   = GroupA_Motor[6].initial_position_deg  + 0;
        //GroupA_Degree_Goal[7]   = GroupA_Motor[7].initial_position_deg  + 0;
        //GroupA_Degree_Goal[8]   = GroupA_Motor[8].initial_position_deg  + 0;
        //GroupA_Degree_Goal[9]   = GroupA_Motor[9].initial_position_deg  - 0;
        //GroupA_Degree_Goal[10]  = GroupA_Motor[10].initial_position_deg - 0;
        //GroupA_Degree_Goal[11]  = GroupA_Motor[11].initial_position_deg - 0;


        //Pos Control
        if( GroupSyncWritePos(portHandler_gait, packetHandler, GroupA_Id,GroupA_Degree_Goal,12))
        {
            cout << "Initial Pos Control Success" << endl;
        }
        else
        {
            return 0;
            cout <<"Error in Pos Initial Control" << endl;
        }


        while(1)
        {
            printf("Initial End? (y/n)\n");
            char key = '0';
            scanf("%c",&key);
            if(key == 'y')
                break;
        }

        for(int i = 0;i < 12; i++){


            GroupA_Motor[i].ReadPosition();
            GroupA_Motor[i].ReadVelocity();
            GroupA_Motor[i].DisableTorque();
        }

        //Disable Torque to change mode to TORQUE_DISABLE and Set the IndirectAddress_Map_Velocity_Pos
        if (IndirectAddress_Map_Velocity_Pos(portHandler_gait, packetHandler, GroupA_Id, GroupA_Motor, 12))
        {
            cout << "GAIT Indirect Address Initial Succeed" << endl;
        }
        else 
            cout << "LEFT-A Side Indirect Address Initial Error" << endl;

        printf("MOTO_GROUP_A:\n");
        for(int i = 0;i < 12; i++)
        {
            //GroupA_Motor[i].JointMode();
            //Torque
            GroupA_Motor[i].TorqueMode();
            GroupA_Motor[i].EnableTorque();
            Error_Addup[i] = 0;
            printf("MOTO[%d].Initial = %f\n",GroupA_Id[i], GroupA_Motor[i].initial_position_deg);
        }

        // 100 W = 310
        // 200 W = 620

        Torque_Limit[0] = 310;  Torque_Limit[ 1] = 620;  Torque_Limit[ 2] = 620;  
        Torque_Limit[3] = 620;  Torque_Limit[ 4] = 620;  Torque_Limit[ 5] = 310;  

        Torque_Limit[6] = 310;  Torque_Limit[ 7] = 620;  Torque_Limit[ 8] = 620;  
        Torque_Limit[9] = 620;  Torque_Limit[10] = 620;  Torque_Limit[11] = 310;  

        //Torque_Limit[0] = 150;  Torque_Limit[ 1] = 150;  Torque_Limit[ 2] = 150;  
        //Torque_Limit[3] = 150;  Torque_Limit[ 4] = 150;  Torque_Limit[ 5] = 150;  

        //Torque_Limit[6] = 150;  Torque_Limit[ 7] = 150;  Torque_Limit[ 8] = 150;  
        //Torque_Limit[9] = 150;  Torque_Limit[10] = 150;  Torque_Limit[11] = 150;  

        //Torque_Limit[0] = 300;  Torque_Limit[ 1] = 300;  Torque_Limit[ 2] = 300;  
        //Torque_Limit[3] = 300;  Torque_Limit[ 4] = 300;  Torque_Limit[ 5] = 300;  

        //Torque_Limit[6] = 300;  Torque_Limit[ 7] = 300;  Torque_Limit[ 8] = 300;  
        //Torque_Limit[9] = 300;  Torque_Limit[10] = 300;  Torque_Limit[11] = 300;  

        //[0]-Hip_Yaw_L,    [1]-Hip_Roll_L,     [2]-Hip_Pitch_L,    [3]-Knee_L,      [ 4]-Ankle_Pitch_L,     [5]-Ankle_Roll_L,
        // Pitch need more torque, so the limit_torque and the Kp of the motor should be larger.
        //[6]-Hip_Yaw_R,    [7]-Hip_Roll_R,     [8]-Hip_Pitch_R,    [9]-Knee_R,      [10]-Ankle_Pitch_R,     [11]-Ankle_Roll_R

        Pos_Kp[0] = 215;    Pos_Kp[1] = 215;    Pos_Kp[2]  = 215;    Pos_Kp[3]  = 215;    Pos_Kp[ 4]  = 215;   Pos_Kp[ 5] = 215;
        Pos_Kd[0] = 0.08;   Pos_Kd[1] = 0.08;    Pos_Kd[2] = 0.08;    Pos_Kd[3] = 0.08;    Pos_Kd[ 4] = 0.08;  Pos_Kd[ 5] = 0.08;

        Pos_Kp[6] = 215;    Pos_Kp[7] = 215;     Pos_Kp[8] = 215;    Pos_Kp[9]  = 215;    Pos_Kp[10]  = 215;   Pos_Kp[11] = 215;
        Pos_Kd[6] = 0.08;   Pos_Kd[7] = 0.08;    Pos_Kd[8] = 0.08;    Pos_Kd[9] = 0.08;    Pos_Kd[10] = 0.08;  Pos_Kd[11] = 0.08;

        //Pos_Kp[0] = 215;   Pos_Kp[1] = 215;    Pos_Kp[2] = 215;    Pos_Kp[3] = 650;    Pos_Kp[ 4] = 215;   Pos_Kp[ 5] = 215;   
        //Pos_Kd[0] = 0.08;   Pos_Kd[1] = 0.08;    Pos_Kd[2] = 0.08;    Pos_Kd[3] = 0.08;    Pos_Kd[ 4] = 0.08;  Pos_Kd[ 5] = 0.08;   

        //Pos_Kp[6] = 215;    Pos_Kp[7] = 215;    Pos_Kp[8] = 215;    Pos_Kp[9] = 650;    Pos_Kp[10] = 215;   Pos_Kp[11] = 215;    
        //Pos_Kd[6] = 0.08;   Pos_Kd[7] = 0.08;    Pos_Kd[8] = 0.08;    Pos_Kd[9] = 0.08;    Pos_Kd[10] = 0.08;  Pos_Kd[11] = 0.08;   

    }
    //==============================0 LCJ PID
    int32_t outMin = -300;
    int32_t outMax = 300;

    // float pi = 3.1415926;

    int32_t Velocity_Max = 16613;   //33.1*501.9 = 16612.89
    int32_t Velocity_Min = -16613;

    float error_theta_max_torque_degree = 1.0;
    float Kp = outMax / DEGREE_TO_REG_VALUE_SF / error_theta_max_torque_degree;

    float time_to_max_torque_s = 8.0;  //time to take from 1 degree error to full scale torque through integrator 
    float Ki = outMax * CTRL_PERIOD_S / DEGREE_TO_REG_VALUE_SF / time_to_max_torque_s;

    float negtive_torque_max_velocity = 600;
    float Kd1 = negtive_torque_max_velocity / Velocity_Max;


    float Instructtion_velocity_max_torque = 60;    //instruction velocity to generate 300 torque
    float Kd2 = outMax / Instructtion_velocity_max_torque / DEGREE_TO_REG_VALUE_SF ;

    Pid_Controller *pid_controller = new Pid_Controller[12];

    int32_t minTorque = -620;
    int32_t maxTorque = 620;

    for(uint16_t i=0;i<12;i++)
    {
        (pid_controller+i)->PID_setgains(Kp, Ki, Kd1, Kd2);
        (pid_controller+i)->PID_setoutMin(minTorque);
        (pid_controller+i)->PID_setoutMax(maxTorque);
    }

    //=========================LCJ PID end
    /*==========================================================
      ============================================================
      Using IK to calculate the walking pattern
      ============================================================
      ============================================================*/
    //while(ros::ok()){
    gettimeofday (&tv , &tz);

    Time_Initial = tv.tv_usec * 0.001 + tv.tv_sec * 1000;
    for(int i = 0; i < parameter_init.t_sim/parameter_init.Ts; i++){
        std::cout << "============" << std::endl;

        // Look for Ctrl + C
        if(kbhit() || !(ros::ok())){
            break;
        }
        //if((i > 2*parameter_init.t_dsp/parameter_init.Ts) && (in_foot_landing_control_phase == 0)){
        //break;}

        //Determine alpha
        gettimeofday (&tv , &tz);
        Time_Now = (tv.tv_usec * 0.001 + tv.tv_sec * 1000) - Time_Initial;
        Time_Now = Time_Now / 1000.0;
        t_vector.push_back(Time_Now);

        if(sign(p_ref_y.at(i)) == -1){
            position_left_foot_x = p_ref_x.at(i);
            position_right_foot_x = x_ref_swing_foot_complete.at(i);
        }
        else if(sign(p_ref_y.at(i)) == 1){
            position_left_foot_x = x_ref_swing_foot_complete.at(i);
            position_right_foot_x = p_ref_x.at(i);
        }
        else {
            std::cout << "determining foot positioning error" << std::endl;
        }

        ros::spinOnce();

        //std::cout << "Fzl is zero. restart the walk" << std::endl;
        if(Fzl == 0 && reading_sensor == 1){
            break;
        }
        //if(gyro_xh > 10 || gyro_yh > 10){
        //std::cout << "hip euler is greater than 10" << std::endl;
        //break;}
        if(reading_sensor == 1){
            alpha_vector.push_back(determineCOPfoot(F2_l.back(), F2_r.back()));
        }


        // Calculate the CoM trajectory for x
        calculate_X_ref_integrated(parameter_init, i, ZMP_x_measure, p_ref_x_transition, 0, &X_ref, X_ref_est.back(), &u_kx_array);
        dlib::matrix<double,3,1> state_vector = X_ref.back();
        x_ref.push_back(state_vector(0));

        // Calculate the CoM trajectory for y
        calculate_X_ref_integrated(parameter_init, i, ZMP_y_measure, p_ref_y_transition, 0, &Y_ref, Y_ref_est.back(), &u_ky_array);
        dlib::matrix<double,3,1> ystate_vector = Y_ref.back();
        y_ref.push_back(ystate_vector(0));

        // Calculate the CoM trajectory for z
        dlib::matrix<double, 3, 1> Z_ref_vector;
        Z_ref_vector = parameter_init.z_com_mbm, 0, 0;
        Z_ref.assign(1, Z_ref_vector);
        extract_pos_of_vector(Z_ref, &z_ref);
        
        // The swing leg trajectory is only calculated till a few cm above the ground. Then the trajectory can be designed manually:

        t_now = i*parameter_init.Ts; // simulated time

        dlib::matrix<double, 3,1> COM;
        dlib::matrix<double, 3,1> ankle;

        COM     = x_ref.back(), y_ref_swing_foot_complete.at(i), z_ref.back();
        ankle   = x_ref_swing_foot_complete.at(i), y_ref_swing_foot_complete.at(i), z_ref_swing_foot_complete.at(i);

        // Calculate Walking Pattern

        dlib::matrix<double, 6,1> q_swing;
        dlib::matrix<double, 6,1> q_support;
        dlib::matrix<double,6,1> state_for_body;
        if(reading_sensor == 1){
            if(left_foot_is_support_leg(i, p_ref_y)){
                q_swing =   -GroupA_Motor[6].present_position_deg,
                        -GroupA_Motor[7].present_position_deg/tipping_scale,
                        -GroupA_Motor[8].present_position_deg,
                        GroupA_Motor[9].present_position_deg,
                        GroupA_Motor[10].present_position_deg,
                        GroupA_Motor[11].present_position_deg;
                q_support =   -GroupA_Motor[0].present_position_deg,
                          -GroupA_Motor[1].present_position_deg/tipping_scale,
                          GroupA_Motor[2].present_position_deg,
                          -GroupA_Motor[3].present_position_deg,
                          -GroupA_Motor[4].present_position_deg,
                          GroupA_Motor[5].present_position_deg;
                ForwardKinematic(1, parameter_init, y_ref[i], q_support, &state_for_body); 
            }
            else{
                q_swing =   -GroupA_Motor[0].present_position_deg,
                        -GroupA_Motor[1].present_position_deg/tipping_scale,
                        GroupA_Motor[2].present_position_deg,
                        -GroupA_Motor[3].present_position_deg,
                        -GroupA_Motor[4].present_position_deg,
                        GroupA_Motor[5].present_position_deg;
                q_support =   -GroupA_Motor[6].present_position_deg,
                          -GroupA_Motor[7].present_position_deg/tipping_scale,
                          -GroupA_Motor[8].present_position_deg,
                          GroupA_Motor[9].present_position_deg,
                          GroupA_Motor[10].present_position_deg,
                          GroupA_Motor[11].present_position_deg;
                ForwardKinematic(0, parameter_init, y_ref[i], q_support, &state_for_body); 
            }


            theta_body.push_back(state_for_body(4)*180/pi);
            //std::cout << "Theta: " << state_for_body(4) << std::endl;

            dlib::matrix<double,3,1> hip_euler;
            hip_euler= gyro_xh, gyro_yh, gyro_zh;
            hip_euler = hip_euler*pi/180;
            start_foot_landing_control(i, &in_foot_landing_control_phase, parameter_init, z_ref_swing_foot_complete_unmodified,
                    p_ref_x, p_ref_y, p_ref_z,
                    F2_l, F2_r, ZMP_xl_raw.back(), ZMP_xr_raw.back(), y_ref[i],
                    q_swing, &left_support_foot_state, &right_support_foot_state,
                    left_swing_foot_state, right_swing_foot_state,
                    &xl_offset, &zl_offset, &xr_offset, &zr_offset,
                    &orientation_offset_l, &orientation_offset_r,
                    &ascended_before, &is_at_ref, hip_euler, tipping_scale
                    );
            std::cout << i << ": In landing phase " << in_foot_landing_control_phase << std::endl;
            landing_phase.push_back(in_foot_landing_control_phase);

        }
        //clipping3.push_back(time_passed);

        if(left_foot_is_support_leg(i, p_ref_y)){
            if(reading_sensor == 1 && control_leg_length == 1){
                modify_swing_foot_trajectory(i, COM, ankle, parameter_init, F0_r, F2_r, ZMP_xr_raw.back(), in_foot_landing_control_phase,
                        &x_ref_swing_foot_complete, &y_ref_swing_foot_complete, &z_ref_swing_foot_complete);
                right_swing_foot_state.p = x_ref_swing_foot_complete.at(i), y_ref_swing_foot_complete.at(i), z_ref_swing_foot_complete.at(i);
                right_swing_foot_state.R = Ryaw(0)*Rpitch(0)*Rroll(0);
            }
            //clipping4.push_back(time_passed);

            if(control_leg_length == 1){
                left_foot = left_support_foot_state;
            }
            else{
                left_foot.p     = p_ref_x.at(i), p_ref_y.at(i), p_ref_z.at(i);
                left_foot.R     = Ryaw(0)*Rpitch(0)*Rroll(0);
            }

            if((in_foot_landing_control_phase == 0) && (i > 2*parameter_init.t_dsp/parameter_init.Ts) && control_leg_length == 1){
                right_foot = right_support_foot_state;
                right_foot.p(1) = y_ref_swing_foot_complete.at(i);
            }
            else{
                //right_foot.p    =   x_ref_swing_foot_complete.at(i) - xr_offset,
                //y_ref_swing_foot_complete.at(i), 
                //z_ref_swing_foot_complete.at(i) - zr_offset;
                right_foot.p    =   x_ref_swing_foot_complete.at(i),
                    y_ref_swing_foot_complete.at(i), 
                    z_ref_swing_foot_complete.at(i);
                if( (orientation_offset_r(0) < 10*pi/180) && (orientation_offset_r(2) < 10*pi/180)){
                    //right_foot.R    =   Ryaw(orientation_offset_r(2))*Rpitch(theta_ref_swing_foot_complete.at(i))*Rroll(orientation_offset_r(0));
                    right_foot.R    =   Ryaw(orientation_offset_r(2))*Rpitch(theta_ref_swing_foot_complete.at(i)+orientation_offset_r(1))*Rroll(orientation_offset_r(0));
                }
                else{

                    right_foot.R    =   Ryaw(0)*Rpitch(theta_ref_swing_foot_complete.at(i))*Rroll(0);
                    std::cout << "Orientation offset to large: " << dlib::trans(orientation_offset_r) << std::endl;
                }
            }

            //std::cout << "BODY: " << dlib::trans(body.p) << std::endl; 
            //body.p          = x_ref.at(i)-xl_offset, y_ref.at(i), z_ref.at(i)-zl_offset;
            body.p          = x_ref.at(i), y_ref.at(i), z_ref.at(i);
            //body.R          = Ryaw(0)*Rpitch(orientation_offset_l(1))*Rroll(0);
            //body.R          = Ryaw(0)*Rpitch(state_for_body(4))*Rroll(0);

            x_support_complete.push_back(left_foot.p(0));
            y_support_complete.push_back(left_foot.p(1));
            z_support_complete.push_back(left_foot.p(2));

            x_swing_complete.push_back(right_foot.p(0));
            y_swing_complete.push_back(right_foot.p(1));
            z_swing_complete.push_back(right_foot.p(2));
            //std::cout << "Left foot (support): " << dlib::trans(left_foot.p) << std::endl;
            //std::cout << "Right foot (swing): " << dlib::trans(right_foot.p) << std::endl;
            //clipTime(&time_passed, &t_initial);
            //clipping4.push_back(time_passed);

        }
        else if(right_foot_is_support_leg(i, p_ref_y)){
            if(reading_sensor == 1 && control_leg_length == 1){
                modify_swing_foot_trajectory(i, COM, ankle, parameter_init, F0_l, F2_l, ZMP_xl_raw.back(), in_foot_landing_control_phase,
                        &x_ref_swing_foot_complete, &y_ref_swing_foot_complete, &z_ref_swing_foot_complete);
                left_swing_foot_state.p = x_ref_swing_foot_complete.at(i), y_ref_swing_foot_complete.at(i), z_ref_swing_foot_complete.at(i);
                //left_swing_foot_state.R = Rroll(0)*Rpitch(theta_ref_swing_foot_complete.at(i))*Rpitch(0);
                left_swing_foot_state.R = Ryaw(0)*Rpitch(0)*Rroll(0);
            }
            //clipTime(&time_passed, &t_initial);
            //clipping3.push_back(time_passed);
            if((in_foot_landing_control_phase == 0) && (i > 2*parameter_init.t_dsp/parameter_init.Ts) && control_leg_length == 1){
                left_foot = left_support_foot_state;
                left_foot.p(1) = y_ref_swing_foot_complete.at(i);
            }
            else{
                //left_foot.p     =   x_ref_swing_foot_complete.at(i) - xl_offset,    
                //y_ref_swing_foot_complete.at(i), 
                //z_ref_swing_foot_complete.at(i) - zl_offset;
                left_foot.p     =   x_ref_swing_foot_complete.at(i),    
                    y_ref_swing_foot_complete.at(i), 
                    z_ref_swing_foot_complete.at(i);
                if( (orientation_offset_l(0) < 10*pi/180) && (orientation_offset_l(2) < 10*pi/180)){
                    //left_foot.R    =   Ryaw(orientation_offset_l(2))*Rpitch(theta_ref_swing_foot_complete.at(i))*Rroll(orientation_offset_l(0));
                    left_foot.R    =   Ryaw(orientation_offset_l(2))*Rpitch(theta_ref_swing_foot_complete.at(i)+orientation_offset_l(1))*Rroll(orientation_offset_l(0));

                }
                else{
                    left_foot.R    = Ryaw(0)*Rpitch(theta_ref_swing_foot_complete.at(i))*Rroll(0);
                    std::cout << "Orientation offset to large: " << dlib::trans(orientation_offset_l) << std::endl;
                }
            }

            if(control_leg_length == 1){
                right_foot = right_support_foot_state;
            }
            else{
                right_foot.p    = p_ref_x.at(i), p_ref_y.at(i), p_ref_z.at(i);
                right_foot.R     = Ryaw(0)*Rpitch(0)*Rroll(0);
            }

            //body.p          = x_ref.at(i)-xr_offset, y_ref.at(i), z_ref.at(i)-zr_offset;
            body.p          = x_ref.at(i), y_ref.at(i), z_ref.at(i);
            //body.R          = Ryaw(0)*Rpitch(state_for_body(4))*Rroll(0);


            x_support_complete.push_back(left_foot.p(0));
            y_support_complete.push_back(left_foot.p(1));
            z_support_complete.push_back(left_foot.p(2));

            x_swing_complete.push_back(right_foot.p(0));
            y_swing_complete.push_back(right_foot.p(1));
            z_swing_complete.push_back(right_foot.p(2));

            //x_support_complete.push_back(right_foot.p(0));
            //y_support_complete.push_back(right_foot.p(1));
            //z_support_complete.push_back(right_foot.p(2));

            //x_swing_complete.push_back(left_foot.p(0));
            //y_swing_complete.push_back(left_foot.p(1));
            //z_swing_complete.push_back(left_foot.p(2));
            //std::cout << "Right foot (support): " << dlib::trans(right_foot.p) << std::endl;
            //std::cout << "Left foot (swing): " << dlib::trans(left_foot.p) << std::endl;
            //clipTime(&time_passed, &t_initial);
            //clipping4.push_back(time_passed);
        }
        else {
            std::cout << "ERROR. P_ref_y is 0 at some point" << std::endl;
        }




        calculate_walking_pattern_ankle(left_foot, right_foot, body, parameter_init, &walking_pattern);

        if (reading_sensor == 1){
            double phitmpl, thetatmpl, psitmpl;
            double phitmpr, thetatmpr, psitmpr;
            rotm2eul(left_foot.R, &phitmpl, &thetatmpl, &psitmpl);
            rotm2eul(right_foot.R, &phitmpr, &thetatmpr, &psitmpr);
            phi_l.push_back(phitmpl);
            theta_l.push_back(thetatmpl);
            psi_l.push_back(psitmpl);

            phi_r.push_back(phitmpr);
            theta_r.push_back(thetatmpr);
            psi_r.push_back(psitmpr);




        }

        // Getting the current q12
        dlib::matrix<double, 12, 1> q_now = walking_pattern.back()*180/pi;
        q0_ref.push_back(q_now(0));
        q1_ref.push_back(q_now(1));
        q2_ref.push_back(q_now(2));
        q3_ref.push_back(q_now(3));
        q4_ref.push_back(q_now(4));
        q5_ref.push_back(q_now(5));
        //q3_ref.push_back(-q_now(3));
        //q4_ref.push_back(-q_now(4));
        //q5_ref.push_back(-q_now(5));

        q6_ref.push_back(q_now(6));
        q7_ref.push_back(q_now(7));
        q8_ref.push_back(q_now(8));
        q9_ref.push_back(q_now(9));
        q10_ref.push_back(q_now(10));
        q11_ref.push_back(q_now(11));

        q0.push_back(GroupA_Motor[0].present_position_deg);
        q1.push_back(GroupA_Motor[1].present_position_deg);
        q2.push_back(GroupA_Motor[2].present_position_deg);
        q3.push_back(GroupA_Motor[3].present_position_deg);
        q4.push_back(GroupA_Motor[4].present_position_deg);
        q5.push_back(GroupA_Motor[5].present_position_deg);

        q6.push_back(GroupA_Motor[6].present_position_deg);
        q7.push_back(GroupA_Motor[7].present_position_deg);
        q8.push_back(GroupA_Motor[8].present_position_deg);
        q9.push_back(GroupA_Motor[9].present_position_deg);
        q10.push_back(GroupA_Motor[10].present_position_deg);
        q11.push_back(GroupA_Motor[11].present_position_deg);
        // For rviz
        msg_joint_state.position[0] = q_now(0)*pi/180;
        msg_joint_state.position[1] = q_now(1)*pi/180;
        msg_joint_state.position[2] = -q_now(2)*pi/180;
        msg_joint_state.position[3] = -q_now(3)*pi/180;
        msg_joint_state.position[4] = -q_now(4)*pi/180;
        msg_joint_state.position[5] = q_now(5)*pi/180;

        msg_joint_state.position[6] = q_now(6)*pi/180;
        msg_joint_state.position[7] = q_now(7)*pi/180;
        msg_joint_state.position[8] = -q_now(8)*pi/180;
        msg_joint_state.position[9] = -q_now(9)*pi/180;
        msg_joint_state.position[10] = -q_now(10)*pi/180;
        msg_joint_state.position[11] = q_now(11)*pi/180;

        msg_joint_state.header.stamp = ros::Time::now();
        walking_generator.publish(msg_joint_state);

        if (writing_servo == 1){
            //clipTime(&time_passed, &t_initial);
            // For the actuator

            dlib::matrix<double, 12, 1> q_act;

            q_act(0) = q_now(0);
            //if(left_foot_is_support_leg(i, p_ref_y)){
            q_act(1) = q_now(1)*tipping_scale;
            //}
            //else{
            //q_act(1) = q_now(1)*(tipping_scale_support);
            //}
            q_act(2) = -q_now(2);
            q_act(3) = -q_now(3);
            q_act(4) = -q_now(4);
            if(!left_foot_is_support_leg(i, p_ref_y)){
                q_act(5) = q_now(5)*tipping_scale_support;
            }
            else{
                q_act(5) = q_now(5);
            }

            q_act(6) = q_now(6);
            //if(!left_foot_is_support_leg(i, p_ref_y)){
            q_act(7) = q_now(7)*tipping_scale;
            //}
            //else{
            //q_act(7) = q_now(7)*(tipping_scale_support);
            //}
            q_act(8) = -q_now(8);
            q_act(9) = -q_now(9);
            q_act(10) = -q_now(10);
            if(left_foot_is_support_leg(i, p_ref_y)){
                q_act(11) = q_now(11)*tipping_scale_support;
            }
            else{
                q_act(11) = q_now(11);
            }

            GroupA_Degree_Goal[0]   = GroupA_Motor[0].initial_position_deg  + q_act(0);
            GroupA_Degree_Goal[1]   = GroupA_Motor[1].initial_position_deg  + q_act(1);
            GroupA_Degree_Goal[2]   = GroupA_Motor[2].initial_position_deg  - q_act(2);
            GroupA_Degree_Goal[3]   = GroupA_Motor[3].initial_position_deg  + q_act(3);
            GroupA_Degree_Goal[4]   = GroupA_Motor[4].initial_position_deg  + q_act(4);
            GroupA_Degree_Goal[5]   = GroupA_Motor[5].initial_position_deg  - q_act(5);

            GroupA_Degree_Goal[6]   = GroupA_Motor[6].initial_position_deg  + q_act(6);
            GroupA_Degree_Goal[7]   = GroupA_Motor[7].initial_position_deg  + q_act(7);
            GroupA_Degree_Goal[8]   = GroupA_Motor[8].initial_position_deg  + q_act(8);
            GroupA_Degree_Goal[9]   = GroupA_Motor[9].initial_position_deg  - q_act(9);
            GroupA_Degree_Goal[10]  = GroupA_Motor[10].initial_position_deg - q_act(10);
            GroupA_Degree_Goal[11]  = GroupA_Motor[11].initial_position_deg - q_act(11);

            // GroupA_Degree_Goal[0]   = GroupA_Motor[0].initial_position_deg  + q_act(0)*0;
            // GroupA_Degree_Goal[1]   = GroupA_Motor[1].initial_position_deg  + q_act(1)*0;
            // GroupA_Degree_Goal[2]   = GroupA_Motor[2].initial_position_deg  - q_act(2)*0;
            // GroupA_Degree_Goal[3]   = GroupA_Motor[3].initial_position_deg  + q_act(3)*0;
            // GroupA_Degree_Goal[4]   = GroupA_Motor[4].initial_position_deg  + q_act(4)*0;
            // GroupA_Degree_Goal[5]   = GroupA_Motor[5].initial_position_deg  - q_act(5)*0;
            // GroupA_Degree_Goal[6]   = GroupA_Motor[6].initial_position_deg  + q_act(6)*0;
            // GroupA_Degree_Goal[7]   = GroupA_Motor[7].initial_position_deg  + q_act(7)*0;
            // GroupA_Degree_Goal[8]   = GroupA_Motor[8].initial_position_deg  + q_act(8)*0;
            // GroupA_Degree_Goal[9]   = GroupA_Motor[9].initial_position_deg  - q_act(9)*0;
            // GroupA_Degree_Goal[10]  = GroupA_Motor[10].initial_position_deg - q_act(10)*0;
            // GroupA_Degree_Goal[11]  = GroupA_Motor[11].initial_position_deg - q_act(11)*0;

            /*===========NEW TORQUE CONTROL ==================================*/

            // Read velocity and position
            if(IndirectAddress_Read_Velocity_Pos(portHandler_gait, packetHandler, GroupA_Id, GroupA_Motor, 12))
            {
                read_error_conti = 0;
                //cout<<"Succeed"<<endl;
            }
            else
            {
                read_error_addup ++;
                read_error_conti ++;
                if(read_error_conti > 5)
                    break;
                cout << "Error" << endl;
            }
            dlib::matrix<double,6,1> state_for_body2;
            dlib::matrix<double,6,1> q_support2;

            if(left_foot_is_support_leg(i, p_ref_y)){
                q_support2 =   -GroupA_Motor[0].present_position_deg,
                           -GroupA_Motor[1].present_position_deg/tipping_scale,
                           GroupA_Motor[2].present_position_deg,
                           -GroupA_Motor[3].present_position_deg,
                           -GroupA_Motor[4].present_position_deg,
                           GroupA_Motor[5].present_position_deg;
                ForwardKinematic_COM(1, parameter_init, q_support2, &state_for_body2); 

                x_ref_fk.push_back(state_for_body2(0)+p_ref_x.at(i));
                y_ref_fk.push_back(state_for_body2(1));
            }
            else{
                q_support2 =   -GroupA_Motor[6].present_position_deg,
                           -GroupA_Motor[7].present_position_deg/tipping_scale,
                           -GroupA_Motor[8].present_position_deg,
                           GroupA_Motor[9].present_position_deg,
                           GroupA_Motor[10].present_position_deg,
                           GroupA_Motor[11].present_position_deg;
                ForwardKinematic_COM(0, parameter_init, q_support2, &state_for_body2); 
                x_ref_fk.push_back(state_for_body2(0)+p_ref_x.at(i));
                y_ref_fk.push_back(state_for_body2(1));
            }

            // calculate torque

            for(int i=0;i<12;i++)
            {
                (pid_controller+i)->PID_counter_update();
                (pid_controller+i)->PID_setrefValue(GroupA_Degree_Goal[i]);
                (pid_controller+i)->PID_setfbackValue(GroupA_Motor[i].present_position_deg);
                //std::cout << "Pos " << i << ": " << GroupA_Motor[i].present_position_deg;
                (pid_controller+i)->PID_setfbackValue_d(GroupA_Motor[i].present_velocity);
                (pid_controller+i)->PID_run();

                GroupA_Torque_Goal[i] = (int32_t)((pid_controller+i)->PID_getOut());
                if(GroupA_Torque_Goal[i] > Torque_Limit[i])
                {
                    GroupA_Torque_Goal[i] = Torque_Limit[i];
                }
                else if (GroupA_Torque_Goal[i] < -Torque_Limit[i] )
                {
                    GroupA_Torque_Goal[i] = -Torque_Limit[i];
                }
            }

            /*========================*/


            ////Torque control 
            //double Kp,Kd,KI;
            //int Max_Torque = 0;

            //for(int i = 0;i < 12; i++)
            //{
            //double Error_Now = (GroupA_Degree_Goal[i] - GroupA_Motor[i].present_position_deg);
            //Error_Addup[i] = Error_Addup[i] + Error_Now;

            //Kp = Pos_Kp[i];
            //Kd = Pos_Kd[i];
            //Max_Torque = Torque_Limit[i];

            //GroupA_Torque_Goal[i] = (int)(Kp * Error_Now - Kd * GroupA_Motor[i].present_velocity);// + KI * Error_Addup[i];

            //if(GroupA_Torque_Goal[i] > Max_Torque)
            //{
            //GroupA_Torque_Goal[i] = Max_Torque;
            //}
            //else if (GroupA_Torque_Goal[i] < -Max_Torque )
            //{
            //GroupA_Torque_Goal[i] = -Max_Torque;
            //}
            //}


            /* =================
               MODIFY TORQUE FOR FOOT LANDING CONTROL
               =================
               */

            if(reading_sensor == 1 && control_leg_length == 1){
                if((in_foot_landing_control_phase == 1) && (F2_l.back() < -15)  && (F2_r.back() < -15)){
                    int tx, ty;
                    if(left_foot_is_support_leg(i, p_ref_y)){
                        softening_ankle(T0_r, T1_r, &tx, &ty);
                        //GroupA_Torque_Goal[10] = 0;
                        //GroupA_Torque_Goal[11] = 0;
                        GroupA_Torque_Goal[10] = -tx;
                        GroupA_Torque_Goal[11] = ty;
                    }
                    else{
                        softening_ankle(T0_l, T1_l, &tx, &ty);
                        GroupA_Torque_Goal[4] = tx;
                        GroupA_Torque_Goal[5] = ty;
                        //GroupA_Torque_Goal[4] = 0;
                        //GroupA_Torque_Goal[5] = 0;
                    }
                }
            }


            //std::cout << "==============" << std::endl;

            q4_vector.push_back(GroupA_Torque_Goal[3]);
            q10_vector.push_back(GroupA_Torque_Goal[9]);

            /*==========end of foot landing control =================*/
            // std::cout << GroupA_Torque_Goal[4] << std::endl; 
            //DATA FIFO with MOTO STM32
            tau0.push_back(GroupA_Torque_Goal[0]);
            tau1.push_back(GroupA_Torque_Goal[1]);
            tau2.push_back(GroupA_Torque_Goal[2]);
            tau3.push_back(GroupA_Torque_Goal[3]);
            tau4.push_back(GroupA_Torque_Goal[4]);
            tau5.push_back(GroupA_Torque_Goal[5]);
            tau6.push_back(GroupA_Torque_Goal[6]);
            tau7.push_back(GroupA_Torque_Goal[7]);
            tau8.push_back(GroupA_Torque_Goal[8]);
            tau9.push_back(GroupA_Torque_Goal[9]);
            tau10.push_back(GroupA_Torque_Goal[10]);
            tau11.push_back(GroupA_Torque_Goal[11]);
            //std::cout << "Torques: " 
                //<< GroupA_Torque_Goal[0] << "," << GroupA_Torque_Goal[1] << ","<< GroupA_Torque_Goal[2] << ","<< GroupA_Torque_Goal[3] << ","<< GroupA_Torque_Goal[4] << ","<< GroupA_Torque_Goal[5]
                //<< ","<< GroupA_Torque_Goal[6] << ","<< GroupA_Torque_Goal[7] << ","<< GroupA_Torque_Goal[8] << ","<< GroupA_Torque_Goal[9] << ","<< GroupA_Torque_Goal[10] << ","<< GroupA_Torque_Goal[11] 
                //<< std::endl;

            //Write Torque
            if (GroupSyncWriteTorque(portHandler_gait, packetHandler, GroupA_Id, GroupA_Torque_Goal, 12))
            {
                ;//success_time ++;
                //cout << "Succeed" << endl;
            }
            else 
            {
                cout << "Error" << endl;
                break;
            }
            //Torque Control End


        }

        //total_time.push_back(time_passed);
        //printf("Supposed time: %f, real time: %f, Time_delay: %f\n", t_now, Time_Now, t_now-Time_Now);
        loop_rate.sleep();   
    }
    double sum = std::accumulate(total_time.begin(), total_time.end(), 0.0);
    double mean = sum / total_time.size();
    std::cout << "Mean time: " << mean << "ms" << std::endl;

    printf("Finished walking\n");
    //}


    if(writing_servo == 1) {
        for(int i = 0; i < 12; i++)
        {
            GroupA_Motor[i].WriteTorque(0);
        }     
        std::cout << "FINISH by Ctrl-C" << std::endl;

        // Quit to Stop at the broken place.
        for(int i = 0;i < 12; i++)
        {
            GroupA_Motor[i].ReadPosition();
            GroupA_Motor[i].ReadVelocity();
            cout << "Motor ID: " << GroupA_Motor[i].id << ", Angle = " << GroupA_Motor[i].present_position_deg << endl;
            GroupA_Motor[i].DisableTorque();
            GroupA_Motor[i].JointMode();
            GroupA_Motor[i].EnableTorque();
            GroupA_Degree_Goal[i] = GroupA_Motor[i].present_position_deg;
        }

        // //Pos Control

        if( GroupSyncWritePos(portHandler_gait, packetHandler, GroupA_Id,GroupA_Degree_Goal,12))
        {
            ;
        }
        else
        {
            cout <<"Error in Pos Control" << endl;
            for(int i = 0;i < 12; i++)
            {
                GroupA_Motor[i].DisableTorque();
                GroupA_Motor[i].TorqueMode();
                GroupA_Motor[i].EnableTorque();
                GroupA_Motor[i].WriteTorque(0);
            }
        }
        //Pos Control End

    }

    // Analyzing the walking pattern (position, velocity and acceleration)

    for(int i = 0; i < q0.size()-1; i++){
        if(i != 0){
            q0d.push_back((q0.at(i) - q0.at(i-1))/parameter_init.Ts);
            q1d.push_back((q1.at(i) - q1.at(i-1))/parameter_init.Ts);
            q2d.push_back((q2.at(i) - q2.at(i-1))/parameter_init.Ts);
            q3d.push_back((q3.at(i) - q3.at(i-1))/parameter_init.Ts);
            q4d.push_back((q4.at(i) - q4.at(i-1))/parameter_init.Ts);
            q5d.push_back((q5.at(i) - q5.at(i-1))/parameter_init.Ts);
            q6d.push_back((q6.at(i) - q6.at(i-1))/parameter_init.Ts);
            q7d.push_back((q7.at(i) - q7.at(i-1))/parameter_init.Ts);
            q8d.push_back((q8.at(i) - q8.at(i-1))/parameter_init.Ts);
            q9d.push_back((q9.at(i) - q9.at(i-1))/parameter_init.Ts);
            q10d.push_back((q10.at(i) - q10.at(i-1))/parameter_init.Ts);
            q11d.push_back((q11.at(i) - q11.at(i-1))/parameter_init.Ts);
        }

    }

    for(int i = 0; i < q0d.size()-1; i++){
        if(i != 0){
            q0dd.push_back((q0d.at(i) - q0d.at(i-1))/parameter_init.Ts);
            q1dd.push_back((q1d.at(i) - q1d.at(i-1))/parameter_init.Ts);
            q2dd.push_back((q2d.at(i) - q2d.at(i-1))/parameter_init.Ts);
            q3dd.push_back((q3d.at(i) - q3d.at(i-1))/parameter_init.Ts);
            q4dd.push_back((q4d.at(i) - q4d.at(i-1))/parameter_init.Ts);
            q5dd.push_back((q5d.at(i) - q5d.at(i-1))/parameter_init.Ts);
            q6dd.push_back((q6d.at(i) - q6d.at(i-1))/parameter_init.Ts);
            q7dd.push_back((q7d.at(i) - q7d.at(i-1))/parameter_init.Ts);
            q8dd.push_back((q8d.at(i) - q8d.at(i-1))/parameter_init.Ts);
            q9dd.push_back((q9d.at(i) - q9d.at(i-1))/parameter_init.Ts);
            q10dd.push_back((q10d.at(i) - q10d.at(i-1))/parameter_init.Ts);
            q11dd.push_back((q11d.at(i) - q11d.at(i-1))/parameter_init.Ts);
        }

    }

    for(int i = 0; i < q0.size(); i++){
        dlib::matrix<double, 6, 1> q_tmp;
        dlib::matrix<double, 6, 1> state;
        //q_tmp = q0[i],q1[i],-q2[i],q3[i],q4[i],-q5[i];
        q_tmp = q0_ref[i],q1_ref[i],q2_ref[i],q3_ref[i],q4_ref[i],q5_ref[i];
        ForwardKinematic(1, parameter_init, y_ref[i],  q_tmp, &state);

        xl_fk.push_back(state(0));
        //yl_fk.push_back(state(1)-parameter_init.robot_width/2 + y_ref[i]);
        yl_fk.push_back(state(1));
        zl_fk.push_back(state(2));
        phil_fk.push_back(state(3));
        thetal_fk.push_back(state(4));
        psil_fk.push_back(state(5));
    }

    for(int i = 0; i < q0.size(); i++){
        dlib::matrix<double, 6, 1> q_tmp;
        dlib::matrix<double, 6, 1> state;
        //q_tmp = q6[i],q7[i],q8[i],-q9[i],-q10[i],-q11[i];
        q_tmp = q6_ref[i],q7_ref[i],q8_ref[i], q9_ref[i], q10_ref[i],q11_ref[i];
        ForwardKinematic(0, parameter_init, y_ref[i],  q_tmp, &state);

        xr_fk.push_back(state(0));
        yr_fk.push_back(state(1));
        zr_fk.push_back(state(2));
        phir_fk.push_back(state(3));
        thetar_fk.push_back(state(4));
        psir_fk.push_back(state(5));
    }

    // Calculate all foot trajectories
    //gp << "set grid xtics mxtics ytics mytics" << std::endl;
    //gp << "set term x11 reset" << std::endl;
    //gp << "set term x11 0" << std::endl;
    //gp << "plot" 
    //<< gp.file1d(q4_vector) <<"," << gp.file1d(q10_vector) << std::endl;

    gp << "set grid xtics mxtics ytics mytics" << std::endl;
    gp << "set term x11 0" << std::endl;
    gp << "plot" 
    << gp.file1d(x_ref_fk) << "with dots tit 'left x fk', "
    //<< gp.file1d(xr_fk) << "with dots tit 'right x fk', " 
    //<< gp.file1d(x_support_complete) << "with dots tit 'Support foot x', " 
    //<< gp.file1d(x_swing_complete) << "with dots tit 'Swing foot x', " 
    //<< gp.file1d(x_ref) << "with dots tit 'x ref'"  
    //<< gp.file1d(x_ref_swing_foot_complete) << "with lines tit 'Swing foot x'," 
    //<< gp.file1d(p_ref_x) << "with dots tit 'x ref'"  
    << std::endl;

    gp << "set term x11 1" << std::endl;
    gp << "plot" 
    //<< gp.file1d(y_ref) << "with dots tit 'CoM y', "
    << gp.file1d(y_ref_fk) << "with dots tit 'left y fk', "
    //<< gp.file1d(yr_fk) << "with dots tit 'right y fk', "
    //<< gp.file1d(p_ref_y) << "with lines tit 'Support foot y', "  
    //<< gp.file1d(y_support_complete) << "with dots tit 'Support foot y', "  
    //<< gp.file1d(ZMP_y_plot) << "with dots tit 'ZMP y', "
    //<< gp.file1d(alpha_vector) << "with lines tit 'Alpha', "  
    //<< gp.file1d(y_swing_complete) << "with dots tit 'Swing foot x'" 
    //<< gp.file1d(y_ref_swing_foot_complete) << "with lines tit 'Swing foot y'" 
    << std::endl;

    //gp << "set term x11 2" << std::endl;
    //gp << "plot" 
    //<< gp.file1d(zl_fk) << "with dots tit 'left z fk', "
    //<< gp.file1d(zr_fk) << "with dots tit 'right z fk', "
    //<< gp.file1d(z_ref) << "with dots tit 'z ref', "  
    //<< gp.file1d(z_support_complete) << "with dots tit 'Support foot z', "  
    //<< gp.file1d(z_ref_swing_foot_complete_unmodified) << "with dots tit 'Swing foot z'," 
    //<< std::endl;

    //gp << "set term x11 3" << std::endl;
    //gp << "plot" 
    //<< gp.file1d(theta_body) << "with lines tit 'theta r fk', "
    //<< gp.file1d(thetal_fk) << "with lines tit 'theta l fk', "
    //<< gp.file1d(thetar_fk) << "with lines tit 'theta r fk', "
    //<< gp.file1d(theta_ref_swing_foot_complete) << "with lines tit 'theta ref', " 
    //<< gp.file1d(phil_fk) << "with lines tit 'phi l fk', "
    //<< gp.file1d(phir_fk) << "with lines tit 'phi r fk', "
    //<< gp.file1d(psil_fk) << "with lines tit 'psi l fk', "
    //<< gp.file1d(psir_fk) << "with lines tit 'psi r fk'"
    //<< std::endl;

    //gp << "set grid xtics mxtics ytics mytics" << std::endl;
    //gp << "set term x11 0" << std::endl;
    //gp
    //<< "plot" << gp.file1d(p_ref_x_transition) 
    //<< "with lines tit 'p_{ref} x', " << gp.file1d(x_ref) << "with lines tit 'CoM x', " 
    //<< gp.file1d(p_ref_x) << "with dots tit 'Support foot x', " << gp.file1d(x_ref_swing_foot_complete) << "with dots tit 'Swing foot x', " 
    //<< std::endl;


    //gp  << "set term x11 1" << std::endl;
    //gp  << "plot" << gp.file1d(p_ref_y_transition) << "with lines tit 'p_{ref} y', " << gp.file1d(y_ref) << "with lines tit 'CoM y', "
    //<< gp.file1d(p_ref_y) << "with dots tit 'Support foot y', "  << gp.file1d(y_ref_swing_foot_complete) << "with dots tit 'Swing foot y', " 
    //<< std::endl;


    //gp << "set term x11 2" << std::endl;
    //gp << "plot" << gp.file1d(z_ref_swing_foot_complete) << "with lines tit 'Swing foot z'" << std::endl;

    //gp << "set term x11 3" << std::endl;
    //gp << "plot" << gp.file1d(theta_ref_swing_foot_complete) << "with lines tit 'Theta'" << std::endl;


    //gp << "set grid xtics mxtics ytics mytics" << std::endl;
    //gp << "set term x11 0" << std::endl;
    //gp
    //<< "plot " << gp.file1d(x_ref_swing_foot_complete) << "with dots tit 'Swing foot x', " 
    //<< std::endl;


    //gp << "set term x11 1" << std::endl;
    //gp << "plot" << gp.file1d(z_ref_swing_foot_complete) << "with lines tit 'Swing foot z'" << std::endl;




    //   gp << "set term x11 2" << std::endl;
    // gp  << "plot"
    //      << gp.file1d(ZMP_x_measure_array) << "with lines tit 'p measured x'" << std::endl;

    //  gp << "set term x11 3" << std::endl;
    // gp  << "plot"
    //       << gp.file1d(ZMP_y_measure_array) << "with dots tit 'p measured y'" << std::endl;

    // if(reading_sensor == 1){
    //     gp << "set term x11 0" << std::endl;
    //     gp << "plot" << gp.file1d(ZMP_x) << "with lines tit 'ZMP_x left', " << gp.file1d(ZMP_x2) << "with lines tit 'ZMP_x right', " << std::endl;

    //     gp << "set term x11 1" << std::endl;
    //     gp << "plot" << gp.file1d(ZMP_y) << "with lines tit 'ZMP_y left', " << gp.file1d(ZMP_y2) << "with lines tit 'ZMP_y right', " << std::endl;
    // }
    // std::cout << q0.size() << std::endl;

    //gp << "set term x11 4" << std::endl;
    //gp << "set multiplot layout 6, 1 title \"left leg joint angle\"" << std::endl;
    //gp << "plot" << gp.file1d(q0) << "with dots tit 'q1', "
    //<< gp.file1d(q0_ref) << "with dots tit 'q1 ref' "
    //<< std::endl;
    //gp << "plot" << gp.file1d(q1) << "with dots tit 'q2', "
    //<< gp.file1d(q1_ref) << "with dots tit 'q2 ref' "
    //<< std::endl;
    //gp << "plot" << gp.file1d(q2) << "with dots tit 'q3', "
    //<< gp.file1d(q2_ref) << "with dots tit 'q3 ref' "
    //<< std::endl;
    //gp << "plot" << gp.file1d(q3) << "with dots tit 'q4', "
    //<< gp.file1d(q3_ref) << "with dots tit 'q4 ref' "
    //<< std::endl;
    //gp << "plot" << gp.file1d(q4) << "with dots tit 'q5', "
    //<< gp.file1d(q4_ref) << "with dots tit 'q5 ref' "
    //<< std::endl;
    //gp << "plot" << gp.file1d(q5) << "with dots tit 'q6', " 
    //<< gp.file1d(q5_ref) << "with dots tit 'q6 ref' "
    //<< std::endl;
    //gp << "unset multiplot" << std::endl;

    walking_data_msg.Fx_l                          = F0_l;
    walking_data_msg.Fy_l                          = F1_l;
    walking_data_msg.Fz_l                          = F2_l;
    walking_data_msg.Tx_l                          = T0_l;
    walking_data_msg.Ty_l                          = T1_l;
    walking_data_msg.Tz_l                          = T2_l;
    walking_data_msg.Fx_r                          = F0_r;
    walking_data_msg.Fy_r                          = F1_r;
    walking_data_msg.Fz_r                          = F2_r;
    walking_data_msg.Tx_r                          = T0_r;
    walking_data_msg.Ty_r                          = T1_r;
    walking_data_msg.Tz_r                          = T2_r;
    walking_data_msg.Fx_l_offset                   = offset_l[0];
    walking_data_msg.Fy_l_offset                   = offset_l[1];
    walking_data_msg.Fz_l_offset                   = offset_l[2];
    walking_data_msg.Tx_l_offset                   = offset_l[3];
    walking_data_msg.Ty_l_offset                   = offset_l[4];
    walking_data_msg.Tz_l_offset                   = offset_l[5];
    walking_data_msg.Fx_r_offset                   = offset_r[0];
    walking_data_msg.Fy_r_offset                   = offset_r[1];
    walking_data_msg.Fz_r_offset                   = offset_r[2];
    walking_data_msg.Tx_r_offset                   = offset_r[3];
    walking_data_msg.Ty_r_offset                   = offset_r[4];
    walking_data_msg.Tz_r_offset                   = offset_r[5];
    walking_data_msg.ZMP_xl                        = ZMP_xl;
    walking_data_msg.ZMP_xr                        = ZMP_xr;
    walking_data_msg.ZMP_yl                        = ZMP_yl;
    walking_data_msg.ZMP_yr                        = ZMP_yr;
    walking_data_msg.ZMP_x                         = ZMP_x_measure_array;
    walking_data_msg.ZMP_y                         = ZMP_y_measure_array;
    walking_data_msg.gyro_xh                       = gyro_xh_vector;
    walking_data_msg.gyro_yh                       = gyro_yh_vector;
    walking_data_msg.gyro_zh                       = gyro_zh_vector;
    walking_data_msg.acc_xh                        = acc_xh_vector;
    walking_data_msg.acc_yh                        = acc_yh_vector;
    walking_data_msg.acc_zh                        = acc_zh_vector;
    walking_data_msg.gyro_xf                       = gyro_xf_vector;
    walking_data_msg.gyro_yf                       = gyro_yf_vector;
    walking_data_msg.gyro_zf                       = gyro_zf_vector;
    walking_data_msg.acc_xf                        = acc_xf_vector;
    walking_data_msg.acc_yf                        = acc_yf_vector;
    walking_data_msg.acc_zf                        = acc_zf_vector;
    walking_data_msg.q1_ref                        = q0_ref;
    walking_data_msg.q2_ref                        = q1_ref;
    walking_data_msg.q3_ref                        = q2_ref;
    walking_data_msg.q4_ref                        = q3_ref;
    walking_data_msg.q5_ref                        = q4_ref;
    walking_data_msg.q6_ref                        = q5_ref;
    walking_data_msg.q7_ref                        = q6_ref;
    walking_data_msg.q8_ref                        = q7_ref;
    walking_data_msg.q9_ref                        = q8_ref;
    walking_data_msg.q10_ref                       = q9_ref;
    walking_data_msg.q11_ref                       = q10_ref;
    walking_data_msg.q12_ref                       = q11_ref;
    walking_data_msg.q1                            = q0;
    walking_data_msg.q2                            = q1;
    walking_data_msg.q3                            = q2;
    walking_data_msg.q4                            = q3;
    walking_data_msg.q5                            = q4;
    walking_data_msg.q6                            = q5;
    walking_data_msg.q7                            = q6;
    walking_data_msg.q8                            = q7;
    walking_data_msg.q9                            = q8;
    walking_data_msg.q10                           = q9;
    walking_data_msg.q11                           = q10;
    walking_data_msg.q12                           = q11;
    walking_data_msg.tau1                          = tau0;
    walking_data_msg.tau2                          = tau1;
    walking_data_msg.tau3                          = tau2;
    walking_data_msg.tau4                          = tau3;
    walking_data_msg.tau5                          = tau4;
    walking_data_msg.tau6                          = tau5;
    walking_data_msg.tau7                          = tau6;
    walking_data_msg.tau8                          = tau7;
    walking_data_msg.tau9                          = tau8;
    walking_data_msg.tau10                         = tau9;
    walking_data_msg.tau11                         = tau10;
    walking_data_msg.tau12                         = tau11;
    //walking_data_msg.x_ref_swing_foot_complete   = x_ref_swing_foot_complete;
    //walking_data_msg.y_ref_swing_foot_complete   = y_ref_swing_foot_complete;
    //walking_data_msg.z_ref_swing_foot_complete   = z_ref_swing_foot_complete;
    walking_data_msg.x_ref_swing_foot_complete     = x_swing_complete;
    walking_data_msg.y_ref_swing_foot_complete     = y_swing_complete;
    walking_data_msg.z_ref_swing_foot_complete     = z_swing_complete;
    walking_data_msg.theta_ref_swing_foot_complete = theta_ref_swing_foot_complete;
    walking_data_msg.p_ref_x_transition            = p_ref_x_transition;
    walking_data_msg.p_ref_y_transition            = p_ref_y_transition;
    walking_data_msg.p_ref_z_transition            = p_ref_z_transition;
    walking_data_msg.p_ref_x                       = x_support_complete;
    walking_data_msg.p_ref_y                       = y_support_complete;
    walking_data_msg.p_ref_z                       = z_support_complete;
    walking_data_msg.x_ref                         = x_ref_fk;
    walking_data_msg.y_ref                         = y_ref_fk;
    walking_data_msg.z_ref                         = z_ref;
    walking_data_msg.tipping_scale                 = tipping_scale;
    walking_data_msg.t_vector                      = t_vector;
    walking_data_msg.phi_left                      = phi_l;
    walking_data_msg.theta_left                    = theta_l;
    walking_data_msg.psi_left                      = psi_l;
    walking_data_msg.phi_right                     = phi_r;
    walking_data_msg.theta_right                   = theta_r;
    walking_data_msg.psi_right                     = psi_r;
    walking_data_msg.phase                         = landing_phase;
    walking_data_msg.clipping1                     = clipping1;
    walking_data_msg.clipping2                     = clipping2;
    walking_data_msg.clipping3                     = clipping3;
    walking_data_msg.clipping4                     = clipping4;
    walking_data_msg.clipping5                     = clipping5;
    walking_data_msg.clipping6                     = clipping6;
    walking_data_msg.clipping7                     = clipping7;

    std::cout << "Publishing the walking data in a while ros::ok-loop. Press ctrl+c to stop" << std::endl;
    while(ros::ok()){
        publisher_walking_data.publish(walking_data_msg);
    }

    return 1;

}

