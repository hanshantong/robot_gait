#ifndef ZMP_CTRL_H
#define ZMP_CTRL_H
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <stdlib.h>     
#include <stdio.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <dlib/matrix.h>
#include <armadillo>
#include <random>
#include "P1MC.h"
#include "motion_ctrl/kalman.h"
#include <numeric>
#include "spline.h"
#include "gnuplot-iostream/gnuplot-iostream.h"
// #define pi 	          3.14159265358979323846 

// Unused ROS
// #include <ros/ros.h>
// #include <sensor_msgs/JointState.h>
// #include "biped_gait/gait_angle.h"
// #include "pid_control.h"
// #include "biped_gait/zmp_data.h"
// #include "biped_gait/walking_data_all.h"
// #include "gyro_info/com.h"
// #include "gyro_info/gyro_euler.h"

	/***********************Kick Part START*************************/
#define KICK_INCREMENT 0.053
#define KICK_INCREMENT_DSP 0.05
#define LIFT_HEIGHT 0.10
#define LIFT_X_BACK -0.05
#define KICK_DISTANCE 0.2
#define DSP_KICKLEG_X 0.15
#define KICK_HEIGHT 0.05

#define KICK_COMTRANS_PERCENTAGE 50.0

//Kick Time Parameters
#define KICK_INIT_TIME 1.0
#define KICK_COMTRANS_TIME 2.0
#define KICK_LIFT_TIME 2.0
#define KICK_SWING_TIME_SOFT 1.2	//0.5
#define KICK_SWING_TIME_MEDIUM 0.7
#define KICK_SWING_TIME_STRONG 0.4

// #define KICK_LIFT2INIT_TIME 2.0

#define KICK_KICK2DSP_TIME_SOFT 1.2
#define KICK_KICK2DSP_TIME_MEDIUM 1.0
#define KICK_KICK2DSP_TIME_STRONG 1.0

#define KICK_DSP_TIME 3.0

#define KICK_LAST_STEP_TIME 3.0 

//kickType Parameters
#define KICK_SOFT 0
#define KICK_MEDIUM 1
#define KICK_STRONG 2
/***********************Kick Part END*************************/


struct Parameters{
	//DoF
	double step_duration;
	int steps_real;
	double Ts; // Sample time
	double z_com_ctm;  // The CoM height used for CTM and calculating the CoM trajectory of the MBM
	double z_com_mbm;  // The CoM height used for the MBM and the actual height of the robot CoM (how to follow)
	double zmp_offset_x;
	double zmp_offset_y_left;
	double zmp_offset_y_right;
	double zmp_offset_y;

	double step_length;
	double step_width;
	double dsp_percentage;

	// double pz_upper_body;
	double pz_com;
	double z_peak;

	double Fx_ref;
	double Fz_ref;

	// Fixed
	double steps;
	double t_sim;
	double robot_width;
	double robot_knee_length;
	double robot_shin_length;
	double robot_ankle_to_foot;
	double foot_length_front;
	double foot_length_back;
	double foot_width;
	int N;
	int N_T;
	double t_dsp;
	double t_ssp;
	double g;
	double Tipping_Scale_left, Tipping_Scale_right;
	double Tipping_Scale_left_swing, Tipping_Scale_left_support;
	double Tipping_Scale_right_swing, Tipping_Scale_right_support;
	double Q_x;
	double R_pos_x;
	double R_cop_x;
	double Q_y;
	double R_pos_y;
	double R_cop_y;
	double ZMP_x_min;
	double ZMP_x_max;
	double ZMP_y_max;
	double ZMP_y_min;

	int control_leg_length;
	int control_pitch;
	int control_roll;
	int debug_msg;
	int use_feedback_x;
	int use_feedback_y;
};

struct joint_state {
	dlib::matrix<double, 3, 1> p;
	dlib::matrix<double, 3, 3> R;

	joint_state(){
		R = 1, 0, 0, 
		0, 1, 0,
		0, 0, 1;
	}
};

// struct trajectory3d {
//     std::vector<double> x;
//     std::vector<double> y;
//     std::vector<double> z;
// };

class Robot
{
public:

	Robot(int mode);
	Robot(Parameters robot_parameters, int mode);
	~Robot(){};
	Parameters parameters;
	void	update_ati(gAti_Data *_ati);
	void	update_mti(gMti_Data _mti);
	bool	goto_zero(dxl_Actuator *dxl_actuator);
		void	stand(dxl_Actuator *dxl_actuator, int nNum);     //added by Howard Dong 2017.03.20
		void 	stop(dxl_Actuator *dxl_actuator, float gait_packet[]);

		void 	set_parameters();
		int 	walk(dxl_Actuator *dxl_actuator, float gait_packet[]);
		void 	calc_trajectories_walk();
		void    calc_trajectories_walk_straight(int forward, int leftFootStart, int step_amount, double step_length);   
		void 	calc_trajectories_stand_ssp();
		void 	calc_trajectories_stand_ssp_sine();
		void 	calc_trajectories_stand_dsp_sine();
		void  	calc_trajectories_walk_curve(int forward, int leftFootStart, double x_goal, double y_goal, double psi_goal);  
		void 	calc_trajectories_walk_side(int leftside, int leftFootStart, int step_amount, double step_length); 
		void 	calc_trajectories_ssp_squat();
		void calc_trajectories_walk_circling(int clockwise, double radius, double step_width, double psi_all);

		void stopGait();


		int in_foot_landing_control_phase;
		void printJointAnglesToFile();
		void set_MPC_initial_position(double x0, double y0);

		void print_ref_length();

		void drawTrajectory();
		void initKalman();
		void estimateState();
		int  ati_not_working();
		int gait_flawed;
		void softening_ankle(int left, int32_t * t_x, int32_t * t_y);
		void erase_trajectories();
		void save_last_step();	
		void connect_trajectories(int leftFirst, double support_x_last, double support_y_last, double support_z_last, double support_psi_last,
			double  swing_x_last, double  swing_y_last, double  swing_z_last, double  swing_theta_last, double  swing_psi_last,
			double  x_zmp_last, double y_zmp_last);
		int get_walking_state();

		float stand_zero[12]={0,0,0,0,0,0,
			0,0,0,0,0,0};
			int walking_mode; 
			int leftIsSupportLeg;
			int is_touching_ground();
			int get_time_step();

			void save_last_step(double * support_x_last, double * support_y_last, double * support_z_last, double * support_psi_last,
				double * swing_x_last, double * swing_y_last, double * swing_z_last, double * swing_theta_last, double * swing_psi_last,
				double * x_zmp_last, double * y_zmp_last);
			void kick(bool isLeftLeg, int kickType, double yaw_angle=0.0);

		private:
			std::vector<double> leftIsSupportLeg_vector;

			int walking_state;
			double x_starting_left, y_starting_left, x_starting_right, y_starting_right, psi_starting_left, psi_starting_right, body_starting;
			std::vector<double> x_footprint_on_spline, y_footprint_on_spline, yaw_footprint_on_spline;
			std::vector<double> x_footprint_on_circle, y_footprint_on_circle, yaw_footprint_on_circle;

			std::vector<double> ref_psi_body;
			void connect_steps();

			double Tipping_Scale_left_ref, Tipping_Scale_right_ref;
			double phi1,phi2,theta1,theta2,psi1,psi2;
			double body_theta_ref_l_sup, body_theta_ref_r_sup, body_theta, body_theta_fk;
			double body_phi_fk, body_phi;
		// For softening ankle

			int left_ati_fault, right_ati_fault;
			double tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7, tmp8, tmp9;
			dlib::matrix<double,3,1> x_mpc, y_mpc;
			std::vector<double> u_x, u_y;
			KalmanFilter kfx;

			KalmanFilter kfy;
	// For printing
			std::vector<double> q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12;

			int control_leg_length;

	// ZMP from measurements
			double ZMP_xl, ZMP_yl, ZMP_xr, ZMP_yr, ZMP_x, ZMP_y, ZMP_xl_local, ZMP_xr_local;
			double ZMP_x_previous, ZMP_y_previous;
	// Left and right foot position (for ZMP)
			double position_left_foot_x, position_right_foot_x;

	// Support foot trajectory
			std::vector<double> ref_x_support_foot_trajectory;
			std::vector<double> ref_y_support_foot_trajectory;
			std::vector<double> ref_z_support_foot_trajectory;
			std::vector<double> ref_psi_support_foot_trajectory;

	// Swing foot trajectory
			std::vector<double> ref_x_swing_foot_trajectory;
			std::vector<double> ref_y_swing_foot_trajectory;
			std::vector<double> ref_z_swing_foot_trajectory;
			std::vector<double> ref_theta_swing_foot_trajectory;
			std::vector<double> ref_psi_swing_foot_trajectory;

	//ZMP trajectory
			std::vector<double> ref_x_zmp_trajectory;
			std::vector<double> ref_y_zmp_trajectory;
			std::vector<double> ref_z_zmp_trajectory;

	//COM trajectory
			std::vector<double> x_ref;
			std::vector<double> y_ref;
			std::vector<double> z_ref; 

	// For force control
    // std::vector<double> xSpline, ySpline;
    // std::vector<double> x_footprint_on_spline, y_footprint_on_spline, yaw_footprint_on_spline;
// 

	//Utility
			dlib::matrix<double, 12, 1> joint_angles;


			dlib::matrix<double, 6,1> q_swing;
			dlib::matrix<double, 6,1> q_support;

			dlib::matrix<double,6,1> COM_state_fk;
			dlib::matrix<double,6,1> swing_foot_state_fk;
			dlib::matrix<double,6,1> support_foot_state_fk;

			dlib::matrix<double, 3,1> COM;
			dlib::matrix<double, 3,1> ankle;

			std::vector<double> z_ref_swing_foot_complete_unmodified;
			std::vector<dlib::matrix<double,3,1>> X_ref, Y_ref, Z_ref;

			dlib::matrix<double,4,1> X_est;
			dlib::matrix<double,2,1> x_output_estimation;
			dlib::matrix<double,4,1> Y_est;
			dlib::matrix<double,2,1> y_output_estimation;

			joint_state left_foot, right_foot, body;
			joint_state left_support_foot_state;
			joint_state right_support_foot_state;
			joint_state left_swing_foot_state;
			joint_state right_swing_foot_state;

			double xl_offset, xr_offset, zl_offset, zr_offset;
			dlib::matrix<double,3,1> orientation_offset_l, orientation_offset_r;
			dlib::matrix<double,3,1> orientation_offset_l_unmodified, orientation_offset_r_unmodified;

			int ascended_before;
		// For plotting
		gAti_Data m_gati_data[2];//ATI Force Sensor
		gMti_Data m_gmti_data;//MTI IMU

		dlib::matrix<double, 3, 3> A;
		dlib::matrix<double, 3, 1> B;
		dlib::matrix<double, 1, 3> C;
		dlib::matrix<double> g_j;
		double K_s;
		dlib::matrix<double, 1, 3> K_x;
		int  time_step = 0;
		void change_Tipping_Scale();
		void calculateComToAnkle(  dlib::matrix<double, 3,1> COM, dlib::matrix<double, 3,1> ankle, dlib::matrix<double, 3,1> * direction);
		void left_foot_is_support_leg();
		void force_control_leg_length(double Fx, double Fz);
		void start_foot_landing_control();
		int modify_swing_foot_trajectory(double Fx, double Fz);
		void calc_next_COM_position();
		void calculate_yaw_for_body(std::vector<double> yaw_footprint_on_spline, std::vector<double> * ref_psi_body);
		void calculate_yaw_for_body_orthogonal(std::vector<double> * ref_psi_body);
		void calculate_yaw_for_body_stop(double yaw_start, double yaw_end, std::vector<double> * ref_psi_body);

	// Calculate reference ZMP trajectory (will be the support foot position of the robot)
		void calc_p_ref(std::vector<double> *p_ref_x, std::vector<double> *p_ref_y);
		void calc_p_ref_straight(std::vector<double> *p_ref_x, std::vector<double> *p_ref_y, int forward, int leftFootStart, int step_amount, double step_length);
		void calc_p_ref_heel();
		void calc_p_ref_heel_straight(int forward, int leftFootStart, int step_amount, double step_length);
		void calc_p_ref_heel_curve(int forward, int leftFootStart, std::vector<double> x_footprint_on_spline, std::vector<double> y_footprint_on_spline, std::vector<double> yaw_footprint_on_spline);
		void calc_p_ref_heel_side(int leftside, int leftFootStart, int step_amount, double step_length);
		void calc_p_ref_heel_circling(int clockwise, double radius, int step_amount, double step_psi);

		void calc_p_ref_circling(std::vector<double> *p_ref_x, std::vector<double> *p_ref_y, int clockwise, double radius, int step_amount, double step_psi); 

		void calc_p_ref_only_ssp();
		void calc_p_ref_side(std::vector<double> *p_ref_x, std::vector<double> *p_ref_y, int forward, int leftFootStart, int step_amount, double step_length); 
	// Calculate reference ZMP trajectory with DSP transition (will be used to generate the CoM trajectory)
		void calc_p_ref_transition();
		void calc_p_ref_transition_straight(int forward, int leftFootStart, int step_amount, double step_length);
		void calc_p_ref_transition_curve(int forward, int leftFootStart, std::vector<double> x_footprint_on_spline, std::vector<double> y_footprint_on_spline, std::vector<double> yaw_footprint_on_spline);
		void calc_p_ref_transition_side(int leftside, int leftFootStart, int step_amount, double step_length);
		void calc_p_ref_transition_circling(int clockwise, double radius, int step_amount, double step_psi);

		void calc_p_ref_transition_only_ssp();
		void calc_p_ref_transition_only_ssp_sine();
		void calc_p_ref_transition_only_dsp_sine();
		void generate_orientated_spline(double x, double y, double psi, int samples, std::vector<double> * xSpline, std::vector<double> * ySpline);

	// Calculate the COM trajectory

		double calc_g_j( int j, int N, dlib::matrix<double> f_tilde);


		void calculate_X_ref(  std::vector<double> p_ref, 
			std::vector<dlib::matrix<double,3,1>> * X_ref, std::vector<double> * p_real, std::vector<double> * u_k_array_all);

		void set_LQR_gains();

		void calculate_X_ref_integrated( double p_measured, std::vector<double> p_ref, int useEst,
	        std::vector<dlib::matrix<double,3,1>> * X_ref, dlib::matrix<double,3,1> X_est, std::vector<double> * u, dlib::matrix<double,3,1> * x_mpc); //, KalmanFilter * KF);

	// Calculate piece swing foot trajectory
		void calculate_swing_foot_piece_trajectory_heel( int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y, 
			std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, 
			std::vector<double> * theta_ref_swing_foot_complete);
		void calculate_swing_foot_piece_trajectory_heel_straight( int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y, 
			std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, 
			std::vector<double> * theta_ref_swing_foot_complete, int forward);
		
		void calculate_swing_foot_piece_trajectory_stay_at_height(int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double y,
			std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete);
		void calculate_swing_foot_piece_trajectory_heel_curve( int flag, 
			double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y_start, double y_end,
			double psi_start, double psi_stop,
			std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, 
			std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete, std::vector<double> * psi_ref_swing_foot_complete,
			int forward
			);
		void calculate_swing_foot_piece_trajectory_heel_side( int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y, 
			std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, 
			std::vector<double> * theta_ref_swing_foot_complete, int leftside);
		void calculate_swing_foot_piece_trajectory_heel_circling(int flag, double swing_foot_z_peak,
			double x_swing_foot_start, double x_swing_foot_end,  	double swing_foot_x_peak,
			double y_swing_foot_start, double y_swing_foot_end, 	double swing_foot_y_peak,
			double psi_swing_foot_start, double psi_swing_foot_end, double swing_foot_psi_peak,
			std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete,
			std::vector<double> * theta_ref_swing_foot_complete, std::vector<double> * psi_ref_swing_foot_complete);


	    // Calculate complete swing foot trajectory
		void calculate_swing_foot_complete_trajectory_only_ssp(std::vector<double> p_ref_x, std::vector<double> p_ref_y,
			std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete);

		void calculate_swing_foot_complete_trajectory_only_dsp(std::vector<double> p_ref_x, std::vector<double> p_ref_y,
			std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete);

		void calculate_swing_foot_complete_trajectory_heel(std::vector<double> p_ref_x, std::vector<double> p_ref_y,  
			std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, 
			std::vector<double> * theta_ref_swing_foot_complete);
		void calculate_swing_foot_complete_trajectory_heel_straight(std::vector<double> p_ref_x, std::vector<double> p_ref_y,  
			std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, 
			std::vector<double> * theta_ref_swing_foot_complete,
			int forward, int leftFootStart, int step_amount, double step_length);

		void calculate_swing_foot_complete_trajectory_heel_curve(std::vector<double> x_footprint_on_spline, std::vector<double> y_footprint_on_spline, std::vector<double> yaw_footprint_on_spline,
			std::vector<double> * x_ref_swing_foot_complete, 
			std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete, std::vector<double> * psi_ref_swing_foot_complete, 
			int forward, int leftFootStart);
		void calculate_swing_foot_complete_trajectory_heel_side(std::vector<double> p_ref_x, std::vector<double> p_ref_y,  
			std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, 
			std::vector<double> * theta_ref_swing_foot_complete,
			int leftside, int leftFootStart, int step_amount, double step_length);
			// Kinematics
		void calculate_swing_foot_complete_trajectory_heel_circling(
			std::vector<double> p_ref_x, std::vector<double> p_ref_y, 
			std::vector<double> * x_ref_swing_foot_complete, 
			std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, 
			std::vector<double> * theta_ref_swing_foot_complete, std::vector<double> * psi_ref_swing_foot_complete, 
			int clockwise, int step_amount, double step_psi);
		void convert_foot2ankle(joint_state foot, joint_state * ankle);

		dlib::matrix<double, 6, 1> InverseKinematics_analytical(int left, joint_state Foot, joint_state Body);

		void ForwardKinematic_COM(int left,   dlib::matrix<double, 6,1> q, dlib::matrix<double, 6, 1> * state);
		void ForwardKinematic(int left,  dlib::matrix<double, 6,1> q, dlib::matrix<double, 6, 1> * state);
		
		dlib::matrix<double, 3, 3> Rroll(double phi);
		dlib::matrix<double, 3, 3> Rpitch(double theta);
		dlib::matrix<double, 3, 3> Ryaw(double psi);
		void rotm2eul(dlib::matrix<double, 3, 3> R, double * phi, double * theta, double * psi);

	// Generate the walking pattern

		void calculate_walking_pattern_ankle();

	// Calculate the ZMP
		void 	calculate_ZMP();
		void controlPitch();
		void controlRoll();
		double determineCOPfoot(double Fzl, double Fzr);
		void determine_states_from_FK(dxl_Actuator *dxl_actuator);
	// Utility
		void push_into_gait_packet(float gait_packet[]);
		void calculate_absolute_foot_position();
		void generate_foot_print_on_spline(std::vector<double> xSpline, std::vector<double> ySpline, std::vector<double> * x_footprint_on_spline, std::vector<double> * y_footprint_on_spline,std::vector<double> * yaw_footprint_on_spline);
		int closest(std::vector<double>  vec, double value);
//KICK
		 //kick
		void kick_calc_swing(bool isLeftLeg, int kickType, double target_x,double target_y, double target_z, double yaw_angle,double kick_pitch, double landing_pitch);
		void kick_swing_trajectory_generator(double startX, double startY, double startZ, double startTheta, double startPsi, double endX, double endY, double endZ, double endTheta, double endPsi, double time,int motionMode);
		void kick_calc_support(bool isLeftLeg, int kickType);
		void kick_calc_zmp(bool isLeftLeg, int kickType);


	};



// double t_now;
// std::vector<dlib::matrix<double, 12, 1>> walking_pattern;
// joint_state left_foot, right_foot, body;

// void chatterCallback(biped_gait::zmp_data msg);
// void getGyroDataHip(gyro_info::gyro_euler msg);
// void getGyroDataFoot(gyro_info::gyro_euler msg);

// Measured ZMP
// double offset_l[6];
// double offset_r[6];
//std::vector<float> ZMP_x, ZMP_x2, ZMP_y, ZMP_y2, stamp, time_msg, time_now;


// Utility Functions
// void show_vector(std::vector<double> vector_array);
// void show_vector(std::vector<dlib::matrix<double, 12, 1>> vector_array);
// int kbhit();
	int sign(double x);
	void clipTime(double * time_passed, double * t_initial);
	void extract_pos_of_vector(std::vector<dlib::matrix<double,3,1>> vector_array, std::vector<double> * pos);


#endif
