#ifndef KALMAN_H
#define KALMAN_H
#include <iostream>
#include <vector>
#include <sys/time.h>
// #include "walking_generator.h"
#include "dlib/matrix.h"
// #include "calculate_trajectories.h"

using namespace std;

class KalmanFilter{
    int states = 4;
    double omega;
    double Ts = 5e-3;
    double g = 9.81;
    double z_com_ctm = 0.6;
    dlib::matrix<double> x_apriori;
    dlib::matrix<double> x_aposteriori;
    dlib::matrix<double> y_aposteriori;

    dlib::matrix<double> A;
    dlib::matrix<double> B;
    dlib::matrix<double> C;

    dlib::matrix<double> Q;
    dlib::matrix<double> R;
    dlib::matrix<double> P_aposteriori;
    dlib::matrix<double> P_apriori;

    dlib::matrix<double> K_k;

    void predict_state(double input);
    void correct_state(dlib::matrix<double> measurement);

    public:
    KalmanFilter();
    KalmanFilter(dlib::matrix<double> A, dlib::matrix<double> B, dlib::matrix<double> C, 
        dlib::matrix<double> Q, dlib::matrix<double> R, 
        dlib::matrix<double> x0, dlib::matrix<double> P0);
    void estimateState(double input, dlib::matrix<double> measurement, dlib::matrix<double,4,1> * x_est, dlib::matrix<double,2,1> * y_est);
    void initKalmanFilter();
    void setQ(double Q);
    void setR(dlib::matrix<double> R);
    void set_ic_x(dlib::matrix<double> x0);
    void set_ic_P(dlib::matrix<double> P);
};
#endif
