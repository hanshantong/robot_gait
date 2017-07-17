#include "motion_ctrl/kalman.h"
KalmanFilter::KalmanFilter(){};

KalmanFilter::KalmanFilter(dlib::matrix<double> A, dlib::matrix<double> B, dlib::matrix<double> C,
        dlib::matrix<double> Q, dlib::matrix<double> R, 
        dlib::matrix<double> x0, dlib::matrix<double> P0){
    A = A;
    B = B;
    C = C;
    Q = Q;
    R = R;
    P_aposteriori = P0;
    x_aposteriori = x0;
};

void KalmanFilter::setQ(double Q_proc){
    Q.set_size(3,3);

    Q = Q_proc, Q_proc, Q_proc,
        Q_proc, Q_proc, Q_proc, 
        Q_proc, Q_proc, Q_proc;

}
void KalmanFilter::setR(dlib::matrix<double> R_meas){
    R = R_meas;
}
void KalmanFilter::set_ic_x(dlib::matrix<double> x0){x_aposteriori = x0;}
void KalmanFilter::set_ic_P(dlib::matrix<double> P0){P_aposteriori = P0;}

void KalmanFilter::initKalmanFilter(){
    omega = sqrt(g/z_com_ctm);
    A.set_size(3,3);
    // A = 1, Ts, 0, 
    //     pow(omega,2)*Ts, 1, -pow(omega,2)*Ts,
    //     0,0,1;
    // B.set_size(3,1);
    // B = 0,0,Ts;

    // C.set_size(2,3);
    // C = 1,0,0,
    //     0,0,1;
    A = 1, Ts, pow(Ts,2)/2,
        0, 1, Ts,
        0, 0, 1;
    B.set_size(3,1);
    B = pow(Ts,3)/6,pow(Ts,2)/2,Ts;

    C.set_size(2,3);
    C = 1,0,0,
        1,0,-z_com_ctm/g;

    P_aposteriori.set_size(3,3);
    P_aposteriori = dlib::identity_matrix<double>(3);

    x_aposteriori.set_size(3,1);
    x_aposteriori = 0,0,0;

    y_aposteriori.set_size(2,1);
}

void KalmanFilter::predict_state(double input){
    x_apriori = A*x_aposteriori + B*input;
    // std::cout << "input: " << input << std::endl;
    P_apriori = A*P_aposteriori*dlib::trans(A) + Q;
}
    
void KalmanFilter::correct_state(dlib::matrix<double> measurement){
    // std::cout << "Measurement: Pos: " << measurement(0) << std::endl;
    // std::cout << "Measurement: COP: " << measurement(1) << std::endl;

    K_k = P_apriori*dlib::trans(C)*dlib::inv( C*P_apriori*dlib::trans(C)+R );
    //std::cout << "measurement: " << measurement << std::endl;
    //std::cout << "error: " <<  K_k*(measurement - C*x_apriori) << std::endl;
    x_aposteriori = x_apriori + K_k*(measurement - C*x_apriori);
    P_aposteriori = (dlib::identity_matrix<double>(states)-K_k*C)*P_apriori;
    //std::cout << "Q: " << Q << std::endl;
    //std::cout << "R: " << R << std::endl;
    y_aposteriori = C*x_aposteriori;
}

void KalmanFilter::estimateState(double input, dlib::matrix<double> measurement, dlib::matrix<double,4,1> * x_est, dlib::matrix<double,2,1> * y_est){
    this->predict_state(input);
    //std::cout << "Measurement: " << measurement << std::endl;
    this->correct_state(measurement);
    //std::cout << "new aposteriori: " <<  x_aposteriori << std::endl;

    *x_est = x_aposteriori;
    *y_est = y_aposteriori;
    
}
