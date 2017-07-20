#include "calculate_trajectories.h"

dlib::matrix<double, 3, 3> A;
dlib::matrix<double, 3, 1> B;
dlib::matrix<double, 1, 3> C;
dlib::matrix<double> g_j;
double K_s;
dlib::matrix<double, 1, 3> K_x;


int sign(double x){return (x > 0) - (x < 0);}

int kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

void show_vector(std::vector<double> vector_array){
    int counter = 0;
    for(std::vector<double>::iterator it = vector_array.begin(); it != vector_array.end(); ++it){
        std::cout << counter++ << ": " << *it << std::endl;
    }
}

void show_vector(std::vector<dlib::matrix<double, 12, 1>> vector_array){
    int counter = 0;
    for(std::vector<dlib::matrix<double, 12, 1>>::iterator it = vector_array.begin(); it != vector_array.end(); ++it){
        std::cout << counter++ << ": " << *it*180/pi << std::endl;
    }
}

void extract_pos_of_vector(std::vector<dlib::matrix<double,3,1>> vector_array, std::vector<double> * pos){
    for(std::vector<dlib::matrix<double,3,1>>::iterator it = vector_array.begin(); it != vector_array.end(); ++it){
        dlib::matrix<double,3,1> state_vector = *it;
        pos->push_back(state_vector(0));
    }

}

void calc_p_ref_heel(parameters parameter_init,
        std::vector<double> *p_ref_x, std::vector<double> *p_ref_y){

    int step_amount = parameter_init.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < parameter_init.steps_real; i++){
        s[i] = i*parameter_init.step_length;
    }

    double step_duration = ceil(parameter_init.N/parameter_init.steps_real);
    // Every step in the lateral direction is made of this piece
    // Calculating the time that is needed for one step

    //First step
    p_ref_x->insert(p_ref_x->end(), ceil(step_duration+step_duration*parameter_init.dsp_percentage/100), s[0]);
    p_ref_y->insert(p_ref_y->end(), ceil(step_duration+step_duration*parameter_init.dsp_percentage/100), -parameter_init.robot_width/2);

    //p_ref_x->insert(p_ref_x->end(), step_duration, s[0]);
    //p_ref_y->insert(p_ref_y->end(), step_duration, -parameter_init.robot_width/2);


    // Middle steps
    for (int i = 1; i < parameter_init.steps_real-1; i++){


        p_ref_x->insert(p_ref_x->end(), step_duration, s[i]);

        p_ref_y->insert(p_ref_y->end(), step_duration, pow(-1,i+1)*(parameter_init.robot_width/2));

    }

    //Last step
    p_ref_x->insert(p_ref_x->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), s[step_amount-1]);
    p_ref_y->insert(p_ref_y->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), pow(-1, parameter_init.steps_real)*parameter_init.robot_width/2);

    //std::cout << p_ref_x->size() << std::endl;
    //std::cout << p_ref_y->size() << std::endl;


}

void calc_p_ref(parameters parameter_init,
        std::vector<double> *p_ref_x, std::vector<double> *p_ref_y){

    int step_amount = parameter_init.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < parameter_init.steps_real; i++){
        s[i] = i*parameter_init.step_length;
    }

    double step_duration = ceil(parameter_init.N/parameter_init.steps_real);
    // Every step in the lateral direction is made of this piece
    // Calculating the time that is needed for one step

    //First step
    //std::cout << ceil(step_duration/2) << std::endl;
    //std::cout << p_ref_x->size() << std::endl;
    //p_ref_x->insert(p_ref_x->end(), ceil(step_duration/2-step_duration*parameter_init.dsp_percentage/100), s[0]);
    //p_ref_y->insert(p_ref_y->end(), ceil(step_duration/2-step_duration*parameter_init.dsp_percentage/100), -parameter_init.robot_width/2);
    //p_ref_x->insert(p_ref_x->end(), ceil(step_duration+step_duration*parameter_init.dsp_percentage/100), s[0]);
    //p_ref_y->insert(p_ref_y->end(), ceil(step_duration+step_duration*parameter_init.dsp_percentage/100), -parameter_init.robot_width/2);

    p_ref_x->insert(p_ref_x->end(), ceil(step_duration*parameter_init.dsp_percentage/100), s[0]);
    p_ref_y->insert(p_ref_y->end(), ceil(step_duration*parameter_init.dsp_percentage/100), -parameter_init.robot_width/2);
    //p_ref_x->insert(p_ref_x->end(), step_duration, s[0]);
    //p_ref_y->insert(p_ref_y->end(), step_duration, -parameter_init.robot_width/2);



    // Middle steps
    for (int i = 0; i < parameter_init.steps_real; i++){
        if (i == parameter_init.steps_real-1){
            p_ref_x->insert(p_ref_x->end(), step_duration, s[i-1]);
        }
        else{
            p_ref_x->insert(p_ref_x->end(), step_duration, s[i]);
        }
        p_ref_y->insert(p_ref_y->end(), step_duration, pow(-1,i+1)*(parameter_init.robot_width/2));

    }

    //Last step
    //p_ref_x->insert(p_ref_x->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), s[step_amount-1]);
    //p_ref_y->insert(p_ref_y->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), pow(-1, step_amount)*parameter_init.robot_width/2);

    //std::cout << p_ref_x->size() << std::endl;
    //std::cout << p_ref_y->size() << std::endl;


}
void calc_p_ref_only_ssp(parameters parameter_init,
        std::vector<double> *p_ref_x, std::vector<double> *p_ref_y){

    int step_amount = parameter_init.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < parameter_init.steps_real; i++){
        s[i] = i*parameter_init.step_length;
    }

    double step_duration = ceil(parameter_init.N/parameter_init.steps_real);
    // Every step in the lateral direction is made of this piece
    // Calculating the time that is needed for one step

    //First step
    //std::cout << ceil(step_duration/2) << std::endl;
    //std::cout << p_ref_x->size() << std::endl;
    //p_ref_x->insert(p_ref_x->end(), ceil(step_duration*parameter_init.dsp_percentage/100), s[0]);
    //p_ref_y->insert(p_ref_y->end(), ceil(step_duration*parameter_init.dsp_percentage/100), -parameter_init.robot_width/2);

    //p_ref_x->insert(p_ref_x->end(), step_duration, s[0]);
    //p_ref_y->insert(p_ref_y->end(), step_duration, -parameter_init.robot_width/2);



    // Middle steps
    for (int i = 0; i < parameter_init.steps_real; i++){
        if (i == 0){
            p_ref_x->insert(p_ref_x->end(), step_duration, 0);
            p_ref_y->insert(p_ref_y->end(), step_duration, pow(-1,i)*(parameter_init.robot_width/2));
        }
        if (i == 1){
            p_ref_x->insert(p_ref_x->end(), step_duration, 0);
            p_ref_y->insert(p_ref_y->end(), step_duration, pow(-1,i+1)*(parameter_init.robot_width/2));
        }
        else{
            p_ref_x->insert(p_ref_x->end(), step_duration, 0);
            p_ref_y->insert(p_ref_y->end(), step_duration, p_ref_y->back());
        }


    }

    //Last step
    //p_ref_x->insert(p_ref_x->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), s[step_amount-1]);
    //p_ref_y->insert(p_ref_y->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), pow(-1, step_amount)*parameter_init.robot_width/2);

    //std::cout << p_ref_x->size() << std::endl;
    //std::cout << p_ref_y->size() << std::endl;


}

void calc_p_ref_transition(parameters parameter_init,
        std::vector<double> * p_ref_x, std::vector<double> * p_ref_y){

    int step_amount = parameter_init.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < step_amount; i++){
        s[i] = i*parameter_init.step_length;
    }

    double step_duration = ceil(parameter_init.N/parameter_init.steps_real);

    //First step
    //p_ref_x->insert(p_ref_x->end(), step_duration, s[0]+parameter_init.zmp_offset_x);
    //p_ref_y->insert(p_ref_y->end(), step_duration, 0);

    p_ref_x->insert(p_ref_x->end(), ceil(step_duration*parameter_init.dsp_percentage/100), s[0]+parameter_init.zmp_offset_x);
    p_ref_y->insert(p_ref_y->end(), ceil(step_duration*parameter_init.dsp_percentage/100), 0);




    for (int i = 0; i < step_amount; i++){

        if(i == 0){
            p_ref_x->insert(p_ref_x->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), s[i]+parameter_init.zmp_offset_x);
            p_ref_y->insert(p_ref_y->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), pow(-1,i+1)*(parameter_init.robot_width/2 + parameter_init.zmp_offset_y));
        }
        else{
            // Calculate the line of the last and this point
            double x1 = p_ref_x->back();
            double y1 = p_ref_y->back();
            double x2 = s[i] + parameter_init.zmp_offset_x;
            double y2 = pow(-1, i+1)*(parameter_init.robot_width/2 + parameter_init.zmp_offset_y);
            double cx = x1;
            double cy = y1;
            double mx = (x2-cx)/parameter_init.t_dsp;
            double my = (y2-cy)/parameter_init.t_dsp;
            double dsp_duration = ceil(step_duration*parameter_init.dsp_percentage/100);
            double increment = parameter_init.t_dsp/dsp_duration;
            for (int k = 0; k < dsp_duration; k++){
                double x_increment = mx*k*increment + cx;
                double y_increment = my*k*increment + cy;
                p_ref_x->push_back(x_increment);
                p_ref_y->push_back(y_increment);
            }
            double ssp_duration = step_duration - dsp_duration;

            p_ref_x->insert(p_ref_x->end(), ssp_duration, x2);
            p_ref_y->insert(p_ref_y->end(), ssp_duration, y2);
        }
    }

    //Last step
    //p_ref_x->insert(p_ref_x->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), s[step_amount-1]+parameter_init.zmp_offset_x);
    //p_ref_y->insert(p_ref_y->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), 0);
    p_ref_x->insert(p_ref_x->end(), step_duration, s[step_amount-1]+parameter_init.zmp_offset_x);
    p_ref_y->insert(p_ref_y->end(), step_duration, 0);
}

void calc_p_ref_transition_only_ssp(parameters parameter_init,
        std::vector<double> * p_ref_x, std::vector<double> * p_ref_y){

    int step_amount = parameter_init.steps_real - 1; 
    double s[step_amount]; // one less because the first and last step are created manually
    for (int i = 0; i < step_amount; i++){
        s[i] = i*(parameter_init.step_length/2);
    }

    double step_duration = ceil(parameter_init.N/parameter_init.steps_real);

    //First step
    //p_ref_x->insert(p_ref_x->end(), ceil(step_duration*parameter_init.dsp_percentage/100), s[0]+parameter_init.zmp_offset_x);
    //p_ref_y->insert(p_ref_y->end(), ceil(step_duration*parameter_init.dsp_percentage/100), 0);
    p_ref_x->insert(p_ref_x->end(), step_duration, s[0]+parameter_init.zmp_offset_x);
    p_ref_y->insert(p_ref_y->end(), step_duration, 0);



    for (int i = 0; i < step_amount; i++){

        if(i == 0){
            //             p_ref_x->insert(p_ref_x->end(), step_duration, s[i]+parameter_init.zmp_offset_x);
            // //            p_ref_y->insert(p_ref_y->end(), step_duration, pow(-1,i)*(parameter_init.robot_width/2 + parameter_init.zmp_offset_y));
            //             p_ref_y->insert(p_ref_y->end(), step_duration, 0);
        }
        else if(i == 1){
            // Calculate the line of the last and this point
            double x1 = p_ref_x->back();
            double y1 = p_ref_y->back();
            double x2 = s[i] + parameter_init.zmp_offset_x;
            double y2 = pow(-1, i+1)*(parameter_init.robot_width/2 + parameter_init.zmp_offset_y);
            double cx = x1;
            double cy = y1;
            double mx = (x2-cx)/(parameter_init.t_dsp);
            double my = (y2-cy)/(parameter_init.t_dsp);
            double dsp_duration = ceil(step_duration*parameter_init.dsp_percentage/100);
            double increment = parameter_init.t_dsp/dsp_duration;
            for (int k = 0; k < dsp_duration; k++){
                double x_increment = mx*k*increment + cx;
                double y_increment = my*k*increment + cy;
                p_ref_x->push_back(x_increment);
                //p_ref_y->push_back(p_ref_y->back());
                p_ref_y->push_back(y_increment);

            }

            // p_ref_x->insert(p_ref_x->end(), parameter_init.step_duration, x2);
            // p_ref_y->insert(p_ref_y->end(), parameter_init.step_duration, y2);
        }
        else{
            p_ref_x->insert(p_ref_x->end(), step_duration, p_ref_x->back());
            p_ref_y->insert(p_ref_y->end(), step_duration, p_ref_y->back());
        }
    }

    //Last step
    //p_ref_x->insert(p_ref_x->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), s[step_amount-1]+parameter_init.zmp_offset_x);
    //p_ref_y->insert(p_ref_y->end(), floor(step_duration*(1-parameter_init.dsp_percentage/100)), 0);
    //p_ref_x->insert(p_ref_x->end(), step_duration, s[step_amount-1]+parameter_init.zmp_offset_x);
    //p_ref_y->insert(p_ref_y->end(), step_duration, 0);

}



void calcLQR_control_gains(double N, dlib::matrix<double, 3, 3> A, dlib::matrix<double, 3, 1> B, dlib::matrix<double, 1, 3> C, double Q_e, double R, double P, 
        std::vector<dlib::matrix<double,1,3>> * K_k_vector, std::vector<dlib::matrix<double,1,3>> * K_v_vector){

    dlib::matrix<double> S_N = trans(C)*P*C;

    for (int i = 0; i < N; i++){


        dlib::matrix<double> K_k = inv(trans(B)*S_N*B+R)*trans(B)*S_N*A;
        K_k_vector->insert(K_k_vector->begin(), K_k);
        dlib::matrix<double> K_v = inv(trans(B)*S_N*B+R)*trans(B);
        K_v_vector->insert(K_v_vector->begin(), K_v);

        S_N = trans(A)*S_N*(A-B*K_k)+trans(C)*Q_e*C;
    }
}

void calcLQR_feedforward(double N, std::vector<double> p_ref, double Q_e, double P, 
        std::vector<dlib::matrix<double,1,3>> K_k_vector, dlib::matrix<double, 3, 3> A, dlib::matrix<double, 3, 1> B, dlib::matrix<double, 1, 3> C, 
        std::vector<dlib::matrix<double,3,1>> * v_k_vector){
    dlib::matrix<double> v_N = trans(C)*P*(p_ref.back());
    for (int i = 0; i < N; i++){
        double idx = N - i - 1;
        v_k_vector->insert(v_k_vector->begin(), v_N);
        v_N = trans(A-B*K_k_vector.at(idx))*v_N+trans(C)*Q_e*p_ref.at(idx);
    }
}


void calculate_X_ref(parameters parameter_init, std::vector<double> p_ref, 
        std::vector<dlib::matrix<double,3,1>> * X_ref, std::vector<double> * p_real, std::vector<double> * u_k_array_all){
    A = 1, parameter_init.Ts, pow(parameter_init.Ts,2)/2,
    0, 1, parameter_init.Ts,
    0, 0, 1;

    B = pow(parameter_init.Ts,3)/6, pow(parameter_init.Ts,2)/2, parameter_init.Ts;

    C = 1, 0, -parameter_init.z_com_ctm/parameter_init.g;

    // Weightening matrices
    double Q_e = 1;
    double R_x = 1e-7;
    double R_y = 1e-7;
    double P   = 100;

    std::vector<dlib::matrix<double,1,3>> K_x_k_array;
    std::vector<dlib::matrix<double,1,3>> K_x_v_array;
    std::vector<dlib::matrix<double,3,1>> v_k_array;
    //vector<double>                   u_k_array;
    calcLQR_control_gains(parameter_init.N, A, B, C, Q_e, R_x, P, &K_x_k_array, &K_x_v_array);
    calcLQR_feedforward(parameter_init.N, p_ref, Q_e, P, K_x_k_array, A, B, C, &v_k_array);
    dlib::matrix<double,3,1> x_k;
    x_k = 0, 0, 0;
    X_ref->push_back(x_k);
    for(int i = 0; i < parameter_init.N; i++){
        dlib::matrix<double,1,3> K_k = K_x_k_array.at(i);    
        dlib::matrix<double,1,3> K_v = K_x_v_array.at(i);
        dlib::matrix<double,3,1> v_k = v_k_array.at(i);
        double u_k = -K_k*x_k+ K_v*v_k;
        x_k = A*x_k + B*u_k;
        u_k_array_all->push_back(u_k);
        X_ref->push_back(x_k);
        p_real->push_back(C*x_k);
    }


}

double calc_g_j(int j, int N, dlib::matrix<double> f_tilde){
    double g_j = 0;
    for(int i = j; i < N+j; i++){
        g_j += f_tilde(i);
    }
    return g_j;
}

void set_LQR_gains(parameters parameter_init){
    arma::mat A_arma, B_arma, C_arma;
    A_arma << 1 << parameter_init.Ts << pow(parameter_init.Ts,2)/2 << arma::endr
        << 0 << 1 << parameter_init.Ts << arma::endr
        << 0 << 0 << 1 << arma::endr;
    B_arma << pow(parameter_init.Ts,3)/6 << arma::endr
        << pow(parameter_init.Ts,2)/2 << arma::endr
        << parameter_init.Ts << arma::endr;
    C_arma << 1 << 0 << -parameter_init.z_com_ctm/parameter_init.g << arma::endr;

    arma::mat A_tilde, B_tilde, C_tilde, P, K_tilde, f_tilde_new, A1, A2, A3, A4, A5, A6, CA, tmp, tmp2, CB, Q_e, R;
    Q_e = 1e0;
    R = 1e-5;
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

    P   << 103.0148 << 5.245e3 << 1.3212e3 << 3.7673 << arma::endr
        << 5.245e3 << 2.7969e5 << 7.0423e4 << 225.5349 << arma::endr
        << 1.3212e3 << 7.0423e4 << 1.7734e4 << 57.3439 << arma::endr
        << 3.7673 << 225.5349 << 57.3439 << 0.3280 << arma::endr;

    K_tilde << 248.6270 << 2.5612e4 << 6.7242e3 << 93.0196 << arma::endr;
    K_s = K_tilde(0,0);
    K_x = K_tilde(0,1), K_tilde(0,2), K_tilde(0,3);

    arma::mat tmp3, tmp4, tmp5, tmp6;

    dlib::matrix<double> f_tilde;
    f_tilde = dlib::ones_matrix<double>(1,2*parameter_init.N_T);

    for(int j = 1; j < 2*parameter_init.N_T+1; j++){
        tmp3 = arma::inv(R+B_tilde.t()*P*B_tilde);
        tmp4 = arma::trans(A_tilde-B_tilde*K_tilde);
        tmp5 = arma::real(arma::logmat(tmp4));        
        tmp6 = arma::expmat(tmp5*(j-1));
        f_tilde_new = tmp3*B_tilde.t()*tmp6*C_tilde.t()*Q_e;
        dlib::set_subm(f_tilde, dlib::range(0,0), dlib::range(j-1,j-1)) = f_tilde_new(0,0);
    }




    A = 1, parameter_init.Ts, pow(parameter_init.Ts,2)/2,
      0, 1, parameter_init.Ts,
      0, 0, 1;

    B = pow(parameter_init.Ts,3)/6, pow(parameter_init.Ts,2)/2, parameter_init.Ts;

    C = 1, 0, -parameter_init.z_com_ctm/parameter_init.g;



    g_j = dlib::ones_matrix<double>(1, parameter_init.N_T);
    for(int j = 0; j < parameter_init.N_T; j++){
        dlib::set_colm(g_j, j) = calc_g_j(j, parameter_init.N_T, f_tilde);
    }
}

void calculate_X_ref_integrated(parameters parameter_init, int i, double p_measured, std::vector<double> p_ref, int useEst,
        std::vector<dlib::matrix<double,3,1>> * X_ref, dlib::matrix<double,3,1> X_est, std::vector<double> * u_k_array){
    //, KalmanFilter * KF){

    dlib::matrix<double> p_ref_piece;

    p_ref_piece = dlib::ones_matrix<double>(parameter_init.N_T, 1);


    int k = 0;

    if(i < parameter_init.N - parameter_init.N_T){
        for (int j = i+1; j < i+parameter_init.N_T+1; j++){
            dlib::set_rowm(p_ref_piece, k) = p_ref.at(j);
            k++;
        }
    }

    else {
        int counter = 0;
        for(std::vector<double>::iterator it = p_ref.begin()+i+1; it != p_ref.end(); ++it){
            dlib::set_rowm(p_ref_piece, k) = *it;
            k++;
            counter++;
        }
        for (int m = 0; m < (parameter_init.N_T -  counter); m++){
            dlib::set_rowm(p_ref_piece, k) = p_ref.back();
            k++;
        }
    }  
    // double u_k;
    // if(isItX == 1){
    //     u_k = -K_s*((p_ref.at(i)%parameter_init.step_length)-(p_measured)) - K_x*X_ref->back() + g_j*p_ref_piece;}
    // else{
    //     u_k = -K_s*(p_ref.at(i)-(p_measured)) - K_x*X_ref->back() + g_j*p_ref_piece;
    //}
    //std::cout << "p_measure " << p_measured << " error: " << (p_ref.at(i)-(p_measured)) << "\n" << std::endl;
    //std::cout << "K_s " << (-K_s*(p_ref.at(i)-(p_measured))) << " K_x " << (-K_x*X_ref->back()) << " Feed forward: " << (g_j*p_ref_piece) << "\n" << std::endl;

    //double u_k = -K_s*(p_ref.at(i)-(p_measured)) - K_x*X_est + g_j*p_ref_piece;
    //double u_k = -K_s*(p_ref.at(i)-(p_measured)) - K_x*X_ref->back() + g_j*p_ref_piece;


    double u_k;
    if(useEst == 0){
        u_k = -K_s*(p_ref.at(i)-C*X_ref->back()) - K_x*X_ref->back() + g_j*p_ref_piece;
    }
    else{
        u_k = -K_s*(p_ref.at(i)-p_measured) - K_x*X_est + g_j*p_ref_piece;
    }
    X_ref->push_back(A*X_ref->back()+B*u_k);

    //std::cout << X_est << std::endl;
    //std::cout << X_ref->back() << std::endl;
    //std::cout << X_est << std::endl;
    //u_k = -K_s*(p_ref.at(i)-p_measured) - K_x*X_est + g_j*p_ref_piece;
    //X_ref->push_back(A*X_est+B*u_k);



    u_k_array->push_back(u_k);

    //estimateState(u_k, p_ref.at(i), x_est, KF);
}

// Calculate piece of a swing foot trajectory
void calculate_swing_foot_piece_trajectory(parameters parameter_init, 
        int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double y,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete){
    if (flag != 1){
        // DSP trajectory (just the foot position for the dsp duration
        for(int i=0; i<parameter_init.t_dsp/parameter_init.Ts; i++){
            // x_ref_swing_foot_complete->push_back(x_ref_swing_foot_complete->back());
            // y_ref_swing_foot_complete->push_back(y);
            // z_ref_swing_foot_complete->push_back(z_ref_swing_foot_complete->back());        
            x_ref_swing_foot_complete->push_back(x_swing_foot_start);
            y_ref_swing_foot_complete->push_back(y);
            z_ref_swing_foot_complete->push_back(0);
        }
    }

    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory

    std::vector<double> t_x(5), t_z(5), x(5), z(5);
    // Calculate x
    double step_length = x_swing_foot_end-x_swing_foot_start;

    t_x[0]=0.0*parameter_init.t_ssp;
    t_x[1]=0.3*parameter_init.t_ssp;
    t_x[2]=0.6*parameter_init.t_ssp;
    t_x[3]=0.8*parameter_init.t_ssp;
    t_x[4]=1.0*parameter_init.t_ssp;

    //if(x_swing_foot_start != x_swing_foot_end){
    x[0]=x_swing_foot_start;
    x[1]=x_swing_foot_start+0.15*(x_swing_foot_end - x_swing_foot_start);
    x[2]=x_swing_foot_start+0.70*(x_swing_foot_end - x_swing_foot_start);
    x[3]=x_swing_foot_start+0.95*(x_swing_foot_end - x_swing_foot_start);
    x[4]=x_swing_foot_end;

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
    t_z[0]=0.0*parameter_init.t_ssp;
    t_z[1]=0.1*parameter_init.t_ssp;
    t_z[2]=0.3*parameter_init.t_ssp;
    t_z[3]=0.7*parameter_init.t_ssp;
    t_z[4]=1.0*parameter_init.t_ssp;

    z[0]=0.0*swing_foot_z_peak;
    z[1]=0.1*swing_foot_z_peak;
    z[2]=1.0*swing_foot_z_peak;
    z[3]=0.4*swing_foot_z_peak;
    z[4]=0.0*swing_foot_z_peak;

    tk::spline x_ref;
    x_ref.set_points(t_x,x);
    tk::spline z_ref;
    z_ref.set_points(t_z,z);

    for(int i=1; i<parameter_init.t_ssp/parameter_init.Ts+1; i++){
        double t_increment=parameter_init.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        y_ref_swing_foot_complete->push_back(y);
        z_ref_swing_foot_complete->push_back(z_ref(t_increment));
    }
    //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameter_init.t_ssp - parameter_init.Ts) << std::endl;
    //std::cout << "x_ref at t_ssp is: " << x_ref(parameter_init.t_ssp) << std::endl;
    //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;

}

void calculate_swing_foot_piece_trajectory_stay_at_height(parameters parameter_init, 
        int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double y,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete){

    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory

    std::vector<double> t_x(5), t_z(5), x(5), z(5);
    // Calculate x
    double step_length = x_swing_foot_end-x_swing_foot_start;

    t_x[0]=0.0*parameter_init.step_duration;
    t_x[1]=0.3*parameter_init.step_duration;
    t_x[2]=0.6*parameter_init.step_duration;
    t_x[3]=0.8*parameter_init.step_duration;
    t_x[4]=1.0*parameter_init.step_duration;

    //if(x_swing_foot_start != x_swing_foot_end){
    x[0]=x_swing_foot_start;
    x[1]=x_swing_foot_start+0.15*(x_swing_foot_end - x_swing_foot_start);
    x[2]=x_swing_foot_start+0.70*(x_swing_foot_end - x_swing_foot_start);
    x[3]=x_swing_foot_start+0.95*(x_swing_foot_end - x_swing_foot_start);
    x[4]=x_swing_foot_end;

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
    t_z[0]=0.0*parameter_init.step_duration;
    t_z[1]=0.1*parameter_init.step_duration;
    t_z[2]=0.3*parameter_init.step_duration;
    t_z[3]=0.7*parameter_init.step_duration;
    t_z[4]=1.0*parameter_init.step_duration;

    z[0]=0.0*swing_foot_z_peak;
    z[1]=0.1*swing_foot_z_peak;
    z[2]=0.4*swing_foot_z_peak;
    z[3]=0.7*swing_foot_z_peak;
    z[4]=1.0*swing_foot_z_peak;

    tk::spline x_ref;
    x_ref.set_points(t_x,x);
    tk::spline z_ref;
    z_ref.set_points(t_z,z);

    for(int i=1; i<parameter_init.step_duration/parameter_init.Ts+1; i++){
        double t_increment=parameter_init.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        y_ref_swing_foot_complete->push_back(y);
        z_ref_swing_foot_complete->push_back(z_ref(t_increment));
    }
    //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameter_init.t_ssp - parameter_init.Ts) << std::endl;
    //std::cout << "x_ref at t_ssp is: " << x_ref(parameter_init.t_ssp) << std::endl;
    //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;

}

void Robot::calculate_swing_foot_piece_trajectory_heel(parameters parameter_init, 
        int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, 
        std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete){
    double cycle_duration;
    if(flag == 1){
        cycle_duration = parameter_init.t_ssp;
    }
    else{
        cycle_duration = parameter_init.step_duration;
    }
    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory
    double q_f = 0*pi/180;

    std::vector<double> t_posz(8), t_posx(7), t_theta(7), theta(7), x(7), z(8);
    // Calculate x
    double step_length = x_swing_foot_end-x_swing_foot_start;

    t_posz[0] = 0.0*cycle_duration;
    t_posz[1] = 0.001*cycle_duration;
    t_posz[2] = 0.3*cycle_duration;
    t_posz[3] = 0.7*cycle_duration;
    t_posz[4] = cycle_duration - parameter_init.t_dsp;
    t_posz[5] = cycle_duration - parameter_init.t_dsp/2;
    t_posz[6] = 0.999*cycle_duration;
    t_posz[7] = 1*cycle_duration;

    z[0] = parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    z[1] = parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    z[2] = parameter_init.robot_ankle_to_foot+1.0*swing_foot_z_peak;
    z[3] = parameter_init.robot_ankle_to_foot+0.2*swing_foot_z_peak;
    z[4] = parameter_init.robot_ankle_to_foot*cos(q_f)+parameter_init.foot_length_back*sin(q_f);
    z[5] = parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    z[6] = parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    z[7] = parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;

    t_posx[0] = 0.0*cycle_duration;
    t_posx[1] = 0.001*cycle_duration;
    t_posx[2] = 0.5*cycle_duration;
    t_posx[3] = cycle_duration - parameter_init.t_dsp;
    t_posx[4] = cycle_duration - parameter_init.t_dsp/2;
    t_posx[5] = 0.999*cycle_duration;
    t_posx[6] = 1*cycle_duration;

    //if(x_swing_foot_start ! = x_swing_foot_end){
    x[0]                      = x_swing_foot_start;
    x[1]                      = x_swing_foot_start;
    x[2]                      = x_swing_foot_start + swing_foot_x_peak;
    x[3]                      = x_swing_foot_end - parameter_init.robot_ankle_to_foot*sin(q_f)-parameter_init.foot_length_back*(1-cos(q_f));
    x[4]                      = x_swing_foot_end;
    x[5]                      = x_swing_foot_end;
    x[6]                      = x_swing_foot_end;

    t_theta[0] = 0.0*cycle_duration;
    t_theta[1] = 0.001*cycle_duration;
    t_theta[2] = cycle_duration - parameter_init.t_dsp;
    t_theta[3] = cycle_duration - parameter_init.t_dsp/2;
    t_theta[4] = cycle_duration - parameter_init.t_dsp/2+0.001;
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

    for(int i=1; i<parameter_init.step_duration/parameter_init.Ts+1; i++){
        double t_increment=parameter_init.Ts*i;
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

    //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameter_init.t_ssp - parameter_init.Ts) << std::endl;
    //std::cout << "x_ref at t_ssp is: " << x_ref(parameter_init.t_ssp) << std::endl;
    //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;

}
void calculate_swing_foot_last_piece(parameters parameter_init, double cycle_duration,
        double x_swing_foot_start, double x_swing_foot_end, double z_swing_foot_start, double z_swing_foot_stop,
        std::vector<double> * x_ref_swing_foot_complete,std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete
        ){
    std::vector<double> t_pos(2), x(2), z(2);

    t_pos[0]=0.0*cycle_duration;
    t_pos[1]=1*cycle_duration;

    x[0]=x_swing_foot_start;
    x[1]=x_swing_foot_end;
    //std::cout << x_swing_foot_start << " " << x_swing_foot_end << std::endl;
    //std::cout << z_swing_foot_start << " " << z_swing_foot_stop << std::endl;
    z[0]=z_swing_foot_start;
    z[1]=z_swing_foot_stop;

    tk::spline x_ref;
    x_ref.set_points(t_pos,x);
    tk::spline z_ref;
    z_ref.set_points(t_pos,z);


    for(int i=1; i<cycle_duration/parameter_init.Ts+1; i++){
        double t_increment=parameter_init.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;
        // std::cout << x_ref(t_increment) << std::endl;
        x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        y_ref_swing_foot_complete->push_back(y_ref_swing_foot_complete->back());
        z_ref_swing_foot_complete->push_back(z_ref(t_increment));
    }

}
void calculate_swing_foot_piece_trajectory_swing_leg(parameters parameter_init, 
        int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y, double swing_foot_z_hold,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete,
        std::vector<double> * theta_ref_swing_foot_complete){
    double cycle_duration;
    if(flag == 1){
        cycle_duration = parameter_init.t_ssp;
    }
    else{
        cycle_duration = parameter_init.step_duration;
    }
    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory
    double q_f = 10*pi/180;

    std::vector<double> t_pos(6), t_theta(5), theta(5), x(6), z(6);
    // Calculate x
    double step_length = x_swing_foot_end-x_swing_foot_start;

    t_pos[0]=0.0*cycle_duration;
    t_pos[1]=0.001*cycle_duration;
    t_pos[2]=0.5*cycle_duration;
    t_pos[3]=cycle_duration - parameter_init.t_dsp;
    t_pos[4]=0.999*cycle_duration;
    t_pos[5]=1*cycle_duration;

    t_theta[0]=0.0*cycle_duration;
    t_theta[1]=0.001*cycle_duration;
    t_theta[2]=cycle_duration - parameter_init.t_dsp;
    t_theta[3]=0.999*cycle_duration;
    t_theta[4]=1*cycle_duration;

    //if(x_swing_foot_start != x_swing_foot_end){
    x[0]=x_swing_foot_start;
    x[1]=x_swing_foot_start;
    x[2]=x_swing_foot_start + swing_foot_x_peak;
    x[3]=x_swing_foot_end - parameter_init.robot_ankle_to_foot*sin(q_f)-parameter_init.foot_length_back*(1-cos(q_f));
    x[4]=x_swing_foot_end;
    x[5]=x_swing_foot_end;
    //std::cout << parameter_init.robot_ankle_to_foot*sin(q_f) << std::endl;
    //std::cout << parameter_init.foot_length_back*(1-cos(q_f)) << std::endl;

    z[0]=parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    z[1]=parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    z[2]=parameter_init.robot_ankle_to_foot+1.0*swing_foot_z_peak;
    z[3]=parameter_init.robot_ankle_to_foot*cos(q_f)+parameter_init.foot_length_back*sin(q_f)+swing_foot_z_hold;
    z[4]=parameter_init.robot_ankle_to_foot+swing_foot_z_hold;
    z[5]=parameter_init.robot_ankle_to_foot+swing_foot_z_hold;

    theta[0] = 0;
    theta[1] = 0;
    theta[2] = -q_f;
    theta[3] = 0;
    theta[4] = 0;


    tk::spline x_ref;
    x_ref.set_points(t_pos,x);
    tk::spline z_ref;
    z_ref.set_points(t_pos,z);
    tk::spline theta_ref;
    theta_ref.set_points(t_theta,theta);

    for(int i=1; i<parameter_init.step_duration/parameter_init.Ts+1; i++){
        double t_increment=parameter_init.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        y_ref_swing_foot_complete->push_back(y);
        z_ref_swing_foot_complete->push_back(z_ref(t_increment));
        theta_ref_swing_foot_complete->push_back(theta_ref(t_increment));
    }

    //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameter_init.t_ssp - parameter_init.Ts) << std::endl;
    //std::cout << "x_ref at t_ssp is: " << x_ref(parameter_init.t_ssp) << std::endl;
    //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;

}

void calculate_swing_foot_piece_trajectory_swing_leg_hold_position(parameters parameter_init, 
        int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y, double swing_foot_z_hold,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete,
        std::vector<double> * theta_ref_swing_foot_complete){
    double cycle_duration;
    if(flag == 1){
        cycle_duration = parameter_init.t_ssp;
    }
    else{
        cycle_duration = parameter_init.step_duration;
    }
    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory
    double q_f = 0*pi/180;

    std::vector<double> t_pos(6), t_theta(5), theta(5), x(6), z(6);
    // Calculate x
    double step_length = x_swing_foot_end-x_swing_foot_start;

    t_pos[0]=0.0*cycle_duration;
    t_pos[1]=0.001*cycle_duration;
    t_pos[2]=0.5*cycle_duration;
    t_pos[3]=cycle_duration - parameter_init.t_dsp;
    t_pos[4]=0.999*cycle_duration;
    t_pos[5]=1*cycle_duration;

    t_theta[0]=0.0*cycle_duration;
    t_theta[1]=0.001*cycle_duration;
    t_theta[2]=cycle_duration - parameter_init.t_dsp;
    t_theta[3]=0.999*cycle_duration;
    t_theta[4]=1*cycle_duration;

    //if(x_swing_foot_start != x_swing_foot_end){
    x[0]=x_swing_foot_start;
    x[1]=x_swing_foot_start;
    x[2]=x_swing_foot_start + swing_foot_x_peak;
    x[3]=x_swing_foot_end - parameter_init.robot_ankle_to_foot*sin(q_f)-parameter_init.foot_length_back*(1-cos(q_f));
    x[4]=x_swing_foot_end;
    x[5]=x_swing_foot_end;
    //std::cout << parameter_init.robot_ankle_to_foot*sin(q_f) << std::endl;
    //std::cout << parameter_init.foot_length_back*(1-cos(q_f)) << std::endl;

    z[0]=parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    z[1]=parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    z[2]=parameter_init.robot_ankle_to_foot+1.0*swing_foot_z_peak;
    z[3]=parameter_init.robot_ankle_to_foot*cos(q_f)+parameter_init.foot_length_back*sin(q_f)+swing_foot_z_hold;
    z[4]=parameter_init.robot_ankle_to_foot+swing_foot_z_hold;
    z[5]=parameter_init.robot_ankle_to_foot+swing_foot_z_hold;

    theta[0] = 0;
    theta[1] = 0;
    theta[2] = -q_f;
    theta[3] = -q_f;
    theta[4] = -q_f;


    tk::spline x_ref;
    x_ref.set_points(t_pos,x);
    tk::spline z_ref;
    z_ref.set_points(t_pos,z);
    tk::spline theta_ref;
    theta_ref.set_points(t_theta,theta);

    for(int i=1; i<parameter_init.step_duration/parameter_init.Ts+1; i++){
        double t_increment=parameter_init.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        y_ref_swing_foot_complete->push_back(y);
        z_ref_swing_foot_complete->push_back(z_ref(t_increment));
        theta_ref_swing_foot_complete->push_back(theta_ref(t_increment));
    }

    //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameter_init.t_ssp - parameter_init.Ts) << std::endl;
    //std::cout << "x_ref at t_ssp is: " << x_ref(parameter_init.t_ssp) << std::endl;
    //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;

}


void calculate_swing_foot_piece_trajectory_swing_leg_last_piece_no_theta(parameters parameter_init, 
        int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y, double swing_foot_z_hold,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete,
        std::vector<double> * theta_ref_swing_foot_complete){
    double cycle_duration;
    if(flag == 1){
        cycle_duration = parameter_init.t_ssp;
    }
    else{
        cycle_duration = parameter_init.step_duration;
    }
    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory
    double q_f = 10*pi/180;

    // Calculate x
    double step_length = x_swing_foot_end-x_swing_foot_start;
    // std::vector<double> t_pos(6), t_theta(5), theta(5), x(6), z(6);

    // t_pos[0]=0.0*cycle_duration;
    // t_pos[1]=0.001*cycle_duration;
    // t_pos[2]=0.5*cycle_duration;
    // t_pos[3]=cycle_duration - parameter_init.t_dsp;
    // t_pos[4]=0.999*cycle_duration;
    // t_pos[5]=1*cycle_duration;

    // t_theta[0]=0.0*cycle_duration;
    // t_theta[1]=0.001*cycle_duration;
    // t_theta[2]=cycle_duration - parameter_init.t_dsp;
    // t_theta[3]=0.999*cycle_duration;
    // t_theta[4]=1*cycle_duration;

    // x[0]=x_swing_foot_start;
    // x[1]=x_swing_foot_start;
    // x[2]=x_swing_foot_start + swing_foot_x_peak;
    // x[3]=x_swing_foot_end - parameter_init.robot_ankle_to_foot*sin(q_f)-parameter_init.foot_length_back*(1-cos(q_f));
    // x[4]=x_swing_foot_end;
    // x[5]=x_swing_foot_end;
    // std::cout << parameter_init.robot_ankle_to_foot*sin(q_f) << std::endl;
    // std::cout << parameter_init.foot_length_back*(1-cos(q_f)) << std::endl;

    // z[0]=parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // z[1]=parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // z[2]=parameter_init.robot_ankle_to_foot+1.0*swing_foot_z_peak;
    // z[3]=parameter_init.robot_ankle_to_foot*cos(q_f)+parameter_init.foot_length_back*sin(q_f)+swing_foot_z_hold;
    // z[4]=parameter_init.robot_ankle_to_foot+swing_foot_z_hold;
    // z[5]=parameter_init.robot_ankle_to_foot+swing_foot_z_hold;

    // theta[0] = 0;
    // theta[1] = 0;
    // theta[2] = -q_f;
    // theta[3] = 0;
    // theta[4] = 0;


    std::vector<double> t_pos(4), t_theta(3), theta(3), x(4), z(4);

    t_pos[0]=0.0*cycle_duration;
    t_pos[1]=0.5*cycle_duration;
    t_pos[2]=cycle_duration - parameter_init.t_dsp;
    t_pos[3]=1*cycle_duration;

    t_theta[0]=0.0*cycle_duration;
    t_theta[1]=cycle_duration - parameter_init.t_dsp;
    t_theta[2]=1*cycle_duration;


    x[0]=x_swing_foot_start;
    x[1]=x_swing_foot_start + swing_foot_x_peak;
    x[2]=x_swing_foot_end - parameter_init.robot_ankle_to_foot*sin(q_f)-parameter_init.foot_length_back*(1-cos(q_f));
    x[3]=x_swing_foot_end;


    z[0]=parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    z[1]=parameter_init.robot_ankle_to_foot+1.0*swing_foot_z_peak;
    z[2]=parameter_init.robot_ankle_to_foot*cos(q_f)+parameter_init.foot_length_back*sin(q_f)+swing_foot_z_hold;
    z[3]=parameter_init.robot_ankle_to_foot+swing_foot_z_hold;

    theta[0] = 0;
    theta[1] = -q_f;
    theta[2] = 0;


    tk::spline x_ref;
    x_ref.set_points(t_pos,x);
    tk::spline z_ref;
    z_ref.set_points(t_pos,z);
    tk::spline theta_ref;
    theta_ref.set_points(t_theta,theta);

    for(int i=1; i<parameter_init.step_duration/parameter_init.Ts+1; i++){
        double t_increment=parameter_init.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        y_ref_swing_foot_complete->push_back(y);
        z_ref_swing_foot_complete->push_back(z_ref(t_increment));
        if(i < (cycle_duration - parameter_init.t_dsp)/parameter_init.Ts){
            theta_ref_swing_foot_complete->push_back(theta_ref(t_increment));
        };
    }

    //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameter_init.t_ssp - parameter_init.Ts) << std::endl;
    //std::cout << "x_ref at t_ssp is: " << x_ref(parameter_init.t_ssp) << std::endl;
    //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;

}


void calculate_swing_foot_piece_trajectory_swing_leg_shortened(parameters parameter_init, 
        int flag, double x_swing_foot_start, double x_swing_foot_end, double swing_foot_z_peak, double swing_foot_x_peak, double y, double swing_foot_z_hold,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete,
        std::vector<double> * theta_ref_swing_foot_complete){
    double cycle_duration;
    if(flag == 1){
        cycle_duration = parameter_init.t_ssp;
    }
    else{
        cycle_duration = parameter_init.step_duration;
    }
    // This function appends the swing_foot_piece to the complete trajectory
    // First the SSP trajectory
    double q_f = 10*pi/180;

    // std::vector<double> t_pos(5), t_theta(4), theta(4), x(5), z(5);
    // // Calculate x
    // double step_length = x_swing_foot_end-x_swing_foot_start;

    // t_pos[0]=0.0*cycle_duration;
    // t_pos[1]=0.001*cycle_duration;
    // t_pos[2]=0.5*cycle_duration;
    // t_pos[3]=0.999*cycle_duration;
    // t_pos[4]=1*cycle_duration;

    // t_theta[0]=0.0*cycle_duration;
    // t_theta[1]=0.001*cycle_duration;
    // t_theta[2]=0.999*cycle_duration;
    // t_theta[3]=1*cycle_duration;


    // x[0]=x_swing_foot_start;
    // x[1]=x_swing_foot_start;
    // x[2]=x_swing_foot_start + swing_foot_x_peak;
    // x[3]=x_swing_foot_end - parameter_init.robot_ankle_to_foot*sin(q_f)-parameter_init.foot_length_back*(1-cos(q_f));
    // x[4]=x_swing_foot_end - parameter_init.robot_ankle_to_foot*sin(q_f)-parameter_init.foot_length_back*(1-cos(q_f));

    // std::cout << parameter_init.robot_ankle_to_foot*sin(q_f) << std::endl;
    // std::cout << parameter_init.foot_length_back*(1-cos(q_f)) << std::endl;

    // z[0]=parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // z[1]=parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    // z[2]=parameter_init.robot_ankle_to_foot+1.0*swing_foot_z_peak;
    // z[3]=parameter_init.robot_ankle_to_foot*cos(q_f)+parameter_init.foot_length_back*sin(q_f)+swing_foot_z_hold;
    // z[4]=parameter_init.robot_ankle_to_foot*cos(q_f)+parameter_init.foot_length_back*sin(q_f)+swing_foot_z_hold;

    // theta[0] = 0;
    // theta[1] = 0;
    // theta[2] = -q_f;
    // theta[3] = -q_f;

    std::vector<double> t_pos(3), t_theta(2), theta(2), x(3), z(3);
    // Calculate x
    double step_length = x_swing_foot_end-x_swing_foot_start;

    t_pos[0]=0.0*cycle_duration;
    t_pos[1]=0.5*cycle_duration;
    t_pos[2]=1*cycle_duration;


    t_theta[0]=0.0*cycle_duration;
    t_theta[1]=1*cycle_duration;


    x[0]=x_swing_foot_start;
    x[1]=x_swing_foot_start + swing_foot_x_peak;
    x[2]=x_swing_foot_end - parameter_init.robot_ankle_to_foot*sin(q_f)-parameter_init.foot_length_back*(1-cos(q_f));

    //std::cout << x[0] <<  x[1] <<  x[2] << std::endl;
    //std::cout << z[0] <<  z[1] <<  z[2] << std::endl;
    z[0]=parameter_init.robot_ankle_to_foot+0.0*swing_foot_z_peak;
    z[1]=parameter_init.robot_ankle_to_foot+1.0*swing_foot_z_peak;
    z[2]=parameter_init.robot_ankle_to_foot*cos(q_f)+parameter_init.foot_length_back*sin(q_f)+swing_foot_z_hold;

    theta[0] = 0;
    theta[1] = -q_f;



    tk::spline x_ref;
    x_ref.set_points(t_pos,x);
    tk::spline z_ref;
    z_ref.set_points(t_pos,z);
    tk::spline theta_ref;
    theta_ref.set_points(t_theta,theta);

    for(int i=1; i<parameter_init.step_duration/parameter_init.Ts+1; i++){
        double t_increment=parameter_init.Ts*i;
        //std::cout << "t_increment: " << t_increment << std::endl;

        x_ref_swing_foot_complete->push_back(x_ref(t_increment));
        y_ref_swing_foot_complete->push_back(y);
        z_ref_swing_foot_complete->push_back(z_ref(t_increment));
        theta_ref_swing_foot_complete->push_back(theta_ref(t_increment));
    }

    //std::cout << "x_ref at t_ssp - Ts is: " << x_ref(parameter_init.t_ssp - parameter_init.Ts) << std::endl;
    //std::cout << "x_ref at t_ssp is: " << x_ref(parameter_init.t_ssp) << std::endl;
    //std::cout << "x foot end is : " << x_swing_foot_end << std::endl;

}

// Calculate complete swing foot trajectory


void calculate_swing_foot_complete_trajectory(std::vector<double> p_ref_x, std::vector<double> p_ref_y, parameters parameter_init,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete){
    double x_swing_foot_start, x_swing_foot_end, y_follow;
    double amount_step_samples  = parameter_init.step_duration/parameter_init.Ts;
    double swing_foot_z_peak    = 0.05;
    //std::cout << "tdsp: " << t_dsp << " tssp: " << t_ssp << " step_duration: " << step_duration << std::endl;
    //double amount_first_step_samples = ceil(amount_step_samples/2-amount_step_samples*parameter_init.dsp_percentage/100);
    //double amount_last_step_samples = floor(amount_step_samples/2+amount_step_samples*parameter_init.dsp_percentage/100);

    double amount_first_step_samples = ceil(amount_step_samples*parameter_init.dsp_percentage/100);
    double amount_last_step_samples = floor(amount_step_samples);
    for (int i = 0; i < parameter_init.steps_real+1; i++){
        //std::cout << "============" << std::endl;
        //std::cout << "real i: " << i << std::endl;
        if ((i > 1) && (i < parameter_init.steps_real - 1)){
            // //std::cout << "used i: " << i << std::endl;
            // x_swing_foot_start = p_ref_x.at((i-1)*amount_step_samples-1 + amount_first_step_samples);
            // x_swing_foot_end   = p_ref_x.at((i)*amount_step_samples + amount_first_step_samples);
            // y_follow           = -p_ref_y.at((i-1)*amount_step_samples + amount_first_step_samples);
            // calculate_swing_foot_piece_trajectory(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, y_follow, 
            //         x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete);

            x_swing_foot_start = p_ref_x.at((i-1)*amount_step_samples-1);
            x_swing_foot_end   = p_ref_x.at((i)*amount_step_samples );
            y_follow           = -p_ref_y.at((i-1)*amount_step_samples);
            calculate_swing_foot_piece_trajectory(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, y_follow, 
                    x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete);
        }
        else if (i == 1){
            x_swing_foot_start  = p_ref_x.at(0);
            x_swing_foot_end    = p_ref_x.at(1*amount_step_samples + amount_first_step_samples);
            y_follow            = -p_ref_y.at(0);
            calculate_swing_foot_piece_trajectory(parameter_init, 1, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, y_follow, 
                    x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete);

        }
        else if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_first_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_first_step_samples, -p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_first_step_samples, 0);
        }
        else if (i == parameter_init.steps_real-1){
            x_swing_foot_start = p_ref_x.at((i-1)*amount_step_samples-1);
            x_swing_foot_end   = p_ref_x.at(i*amount_step_samples-1);
            y_follow           = -p_ref_y.at((i-1)*amount_step_samples + amount_first_step_samples);
            calculate_swing_foot_piece_trajectory(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, y_follow, 
                    x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete);
        }
        else if (i == parameter_init.steps_real){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_last_step_samples, x_ref_swing_foot_complete->back());
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_last_step_samples, -y_ref_swing_foot_complete->back());
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_last_step_samples, 0);
        }
        //std::cout << "i: " << i << " y_follow: " << y_follow << std::endl;

        //std::cout << i << std::endl;
        //std::cout << x_ref_swing_foot_complete->size() << std::endl;
        //std::cout << y_ref_swing_foot_complete->size() << std::endl;
        //std::cout << z_ref_swing_foot_complete->size() << std::endl;
    }
    // show_vector(p_ref_x);
    //show_vector(p_ref_y);
    //Gnuplot gp;
    //gp << "set term x11 0" << std::endl;
    //gp << "plot" << gp.file1d(*x_ref_swing_foot_complete) << std::endl;
    //gp << "set term x11 1" << std::endl;
    //gp << "plot" << gp.file1d(*y_ref_swing_foot_complete) << std::endl;
    //gp << "set term x11 2" << std::endl;
    //gp << "plot" << gp.file1d(*z_ref_swing_foot_complete) << std::endl;
    //gp << "set term x11 3" << std::endl;
    //gp << "plot" << gp.file1d(*theta_ref_swing_foot_complete) << std::endl;
    //printf("\n");

}

void calculate_swing_foot_complete_trajectory_only_ssp(std::vector<double> p_ref_x, std::vector<double> p_ref_y, parameters parameter_init,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete){
    double x_swing_foot_start, x_swing_foot_end, y_follow;
    double amount_step_samples  = parameter_init.step_duration/parameter_init.Ts;
    double swing_foot_z_peak    = 0.15;
    //std::cout << "tdsp: " << t_dsp << " tssp: " << t_ssp << " step_duration: " << step_duration << std::endl;
    //double amount_first_step_samples = ceil(amount_step_samples/2-amount_step_samples*parameter_init.dsp_percentage/100);
    //double amount_last_step_samples = floor(amount_step_samples/2+amount_step_samples*parameter_init.dsp_percentage/100);

    double amount_first_step_samples = ceil(amount_step_samples*(parameter_init.dsp_percentage/100));
    double amount_last_step_samples = floor(amount_step_samples);
    for (int i = 0; i < parameter_init.steps_real; i++){
        //std::cout << "============" << std::endl;
        //std::cout << "real i: " << i << std::endl;

        if (i == 1){
            x_swing_foot_start  = p_ref_x.at(0);
            x_swing_foot_end    = p_ref_x.at(1*amount_step_samples + amount_first_step_samples)/2;
            y_follow            = p_ref_y.at(0);
            calculate_swing_foot_piece_trajectory_stay_at_height(parameter_init, 1, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, y_follow, 
                    x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete);
        }
        else if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_step_samples, -p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_step_samples, 0);
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

void calculate_swing_foot_complete_trajectory_heel(std::vector<double> p_ref_x, std::vector<double> p_ref_y, parameters parameter_init,
        std::vector<double> * x_ref_swing_foot_complete, 
        std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, std::vector<double> * theta_ref_swing_foot_complete){
    double x_swing_foot_start, x_swing_foot_end, y_follow;
    double amount_step_samples  = parameter_init.step_duration/parameter_init.Ts;
    double swing_foot_z_peak    = 0.04;
    double swing_foot_x_peak    = parameter_init.step_length;
    //std::cout << "tdsp: " << t_dsp << " tssp: " << t_ssp << " step_duration: " << step_duration << std::endl;
    //double amount_first_step_samples = ceil(amount_step_samples/2-amount_step_samples*parameter_init.dsp_percentage/100);
    //double amount_last_step_samples = floor(amount_step_samples/2+amount_step_samples*parameter_init.dsp_percentage/100);

    double amount_first_step_samples = ceil(amount_step_samples*parameter_init.dsp_percentage/100);
    double amount_last_step_samples = floor(amount_step_samples-amount_step_samples*parameter_init.dsp_percentage/100);
    for (int i = 0; i < parameter_init.steps_real+1; i++){
        //std::cout << "============" << std::endl;
        //std::cout << "real i: " << i << std::endl;

        if ((i > 1) && (i < parameter_init.steps_real - 1)){
            //std::cout << "used i: " << i << std::endl;
            //x_swing_foot_start = p_ref_x.at((i-1)*amount_step_samples-1 + amount_first_step_samples);
            //x_swing_foot_end   = p_ref_x.at((i)*amount_step_samples + amount_first_step_samples);
            //y_follow           = -p_ref_y.at((i-1)*amount_step_samples + amount_first_step_samples);
            //calculate_swing_foot_piece_trajectory(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, y_follow, 
            //x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete);

            x_swing_foot_start = p_ref_x.at((i-1)*amount_step_samples-1 + amount_first_step_samples);
            x_swing_foot_end   = p_ref_x.at((i)*amount_step_samples + amount_first_step_samples );
            y_follow           = p_ref_y.at((i-1)*amount_step_samples);
            calculate_swing_foot_piece_trajectory_heel(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_follow, 
                    x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete);

        }
        else if (i == 1){
            x_swing_foot_start  = p_ref_x.at(0);
            x_swing_foot_end    = p_ref_x.at(1*amount_step_samples + amount_first_step_samples);
            y_follow            = -p_ref_y.at(0);
            calculate_swing_foot_piece_trajectory_heel(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak/2, y_follow, 
                    x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete);

        }
        else if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_first_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_first_step_samples, -p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_first_step_samples, parameter_init.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_first_step_samples, 0);

        }
        else if (i == parameter_init.steps_real-1){
            x_swing_foot_start = p_ref_x.at((i-1)*amount_step_samples-1 + amount_first_step_samples);
            x_swing_foot_end   = p_ref_x.at(i*amount_step_samples-1 + amount_first_step_samples);
            y_follow           = -p_ref_y.at((i-1)*amount_step_samples + amount_first_step_samples);
            calculate_swing_foot_piece_trajectory_heel(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak/2, y_follow, 
                    x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete);

        }
        else if (i == parameter_init.steps_real){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_last_step_samples, x_ref_swing_foot_complete->back());
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_last_step_samples, -y_ref_swing_foot_complete->back());
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_last_step_samples, parameter_init.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_last_step_samples, 0);

        }
        //std::cout << "i: " << i << " y_follow: " << y_follow << std::endl;

        //std::cout << i << std::endl;
        //std::cout << x_ref_swing_foot_complete->size() << std::endl;
        //std::cout << y_ref_swing_foot_complete->size() << std::endl;
        //std::cout << z_ref_swing_foot_complete->size() << std::endl;
    }
    // show_vector(p_ref_x);
    //show_vector(p_ref_y);
    // Gnuplot gp;
    // gp << "set term x11 0" << std::endl;
    // gp << "plot" << gp.file1d(*x_ref_swing_foot_complete) << std::endl;
    // gp << "set term x11 1" << std::endl;
    // gp << "plot" << gp.file1d(*y_ref_swing_foot_complete) << std::endl;
    // gp << "set term x11 2" << std::endl;
    // gp << "plot" << gp.file1d(*z_ref_swing_foot_complete) << std::endl;
    // gp << "set term x11 3" << std::endl;
    // gp << "plot" << gp.file1d(*theta_ref_swing_foot_complete) << std::endl;
    // show_vector(*x_ref_swing_foot_complete);
    // show_vector(*z_ref_swing_foot_complete);

    // show_vector(*theta_ref_swing_foot_complete);
    printf("\n");

}

void calculate_swing_foot_complete_trajectory_swing_leg_hold_position(std::vector<double> p_ref_x, std::vector<double> p_ref_y, parameters parameter_init, 
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, 
        std::vector<double> * theta_ref_swing_foot_complete, double swing_foot_z_hold, double swing_foot_z_peak){

    double x_swing_foot_start, x_swing_foot_end, y_follow;
    double amount_step_samples  = parameter_init.step_duration/parameter_init.Ts;

    double swing_foot_x_peak    = parameter_init.step_length*0.3;

    //std::cout << "tdsp: " << t_dsp << " tssp: " << t_ssp << " step_duration: " << step_duration << std::endl;
    //double amount_first_step_samples = ceil(amount_step_samples/2-amount_step_samples*parameter_init.dsp_percentage/100);
    //double amount_last_step_samples = floor(amount_step_samples/2+amount_step_samples*parameter_init.dsp_percentage/100);

    double amount_first_step_samples = ceil(amount_step_samples);//*(parameter_init.dsp_percentage/100));
    double amount_last_step_samples = floor(amount_step_samples);
    for (int i = 0; i < parameter_init.steps_real; i++){
        //std::cout << "============" << std::endl;
        //std::cout << "real i: " << i << std::endl;

        if (i == 1){
            x_swing_foot_start  = p_ref_x.at(0);
            x_swing_foot_end    = p_ref_x.at(1*amount_step_samples + amount_first_step_samples)/2;
            y_follow            = p_ref_y.at(0);
            calculate_swing_foot_piece_trajectory_swing_leg_hold_position(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_follow, swing_foot_z_hold,
                    x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete);
        }
        else if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_first_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_first_step_samples, p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_first_step_samples, parameter_init.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_first_step_samples, 0);

        }
        else {
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_last_step_samples, x_ref_swing_foot_complete->back());
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_last_step_samples, y_ref_swing_foot_complete->back());
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_last_step_samples, z_ref_swing_foot_complete->back());           
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_last_step_samples, theta_ref_swing_foot_complete->back());
        }
        //std::cout << "i: " << i << " y_follow: " << y_follow << std::endl;

        //std::cout << i << std::endl;
        // std::cout << x_ref_swing_foot_complete->size() << std::endl;
        // std::cout << y_ref_swing_foot_complete->size() << std::endl;
        // std::cout << z_ref_swing_foot_complete->size() << std::endl;
    }

}

void calculate_swing_foot_complete_trajectory_swing_leg_till_position(std::vector<double> p_ref_x, std::vector<double> p_ref_y, parameters parameter_init,  
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, 
        std::vector<double> * theta_ref_swing_foot_complete, double swing_foot_z_hold, double swing_foot_z_peak){

    double x_swing_foot_start, x_swing_foot_end, y_follow;
    double amount_step_samples  = parameter_init.step_duration/parameter_init.Ts;
    double swing_foot_x_peak    = parameter_init.step_length*0.3;

    if (swing_foot_z_peak < swing_foot_z_hold){
        std::cout << "The maximum value of z has to be higher than the hold value" << std::endl;
    }

    //std::cout << "tdsp: " << t_dsp << " tssp: " << t_ssp << " step_duration: " << step_duration << std::endl;
    //double amount_first_step_samples = ceil(amount_step_samples/2-amount_step_samples*parameter_init.dsp_percentage/100);
    //double amount_last_step_samples = floor(amount_step_samples/2+amount_step_samples*parameter_init.dsp_percentage/100);

    double amount_first_step_samples = ceil(amount_step_samples);//*(parameter_init.dsp_percentage/100));
    double amount_last_step_samples = floor(amount_step_samples);
    for (int i = 0; i < parameter_init.steps_real; i++){
        //std::cout << "============" << std::endl;
        //std::cout << "real i: " << i << std::endl;

        if (i == 1){
            x_swing_foot_start  = p_ref_x.at(0);
            x_swing_foot_end    = p_ref_x.at(1*amount_step_samples + amount_first_step_samples)/2;
            y_follow            = p_ref_y.at(0);
            calculate_swing_foot_piece_trajectory_swing_leg(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_follow, swing_foot_z_hold,
                    x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete);
        }
        else if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_first_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_first_step_samples, -p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_first_step_samples, parameter_init.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_first_step_samples, 0);
        }
        else {

        }
        //std::cout << "i: " << i << " y_follow: " << y_follow << std::endl;

        //std::cout << i << std::endl;
        // std::cout << x_ref_swing_foot_complete->size() << std::endl;
        // std::cout << y_ref_swing_foot_complete->size() << std::endl;
        // std::cout << z_ref_swing_foot_complete->size() << std::endl;
    }

}
void calculate_swing_foot_complete_trajectory_swing_leg_till_position_shortened(std::vector<double> p_ref_x, std::vector<double> p_ref_y, parameters parameter_init, double cycle_duration,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete, 
        std::vector<double> * theta_ref_swing_foot_complete, double swing_foot_z_hold, double swing_foot_z_peak){

    double x_swing_foot_start, x_swing_foot_end, y_follow;
    double amount_step_samples  = parameter_init.step_duration/parameter_init.Ts;
    double swing_foot_x_peak    = parameter_init.step_length*0.5;

    if (swing_foot_z_peak < swing_foot_z_hold){
        std::cout << "The maximum value of z has to be higher than the hold value" << std::endl;
    }

    //std::cout << "tdsp: " << t_dsp << " tssp: " << t_ssp << " step_duration: " << step_duration << std::endl;
    //double amount_first_step_samples = ceil(amount_step_samples/2-amount_step_samples*parameter_init.dsp_percentage/100);
    //double amount_last_step_samples = floor(amount_step_samples/2+amount_step_samples*parameter_init.dsp_percentage/100);

    //double amount_first_step_samples = ceil(amount_step_samples);/[>(parameter_init.dsp_percentage/100));
    double amount_first_step_samples = ceil(amount_step_samples+amount_step_samples*parameter_init.dsp_percentage/100);
    double amount_last_step_samples = floor(amount_step_samples);
    for (int i = 0; i < parameter_init.steps_real; i++){
        //std::cout << "============" << std::endl;
        //std::cout << "real i: " << i << std::endl;

        if (i == 1){
            x_swing_foot_start  = p_ref_x.at(0);
            x_swing_foot_end    = swing_foot_x_peak*2;
            //x_swing_foot_end    = p_ref_x.at(1*amount_step_samples + amount_first_step_samples)/2;
            y_follow            = p_ref_y.at(0);

            /*
               Let the natural trajectory move (no torque after heel touches)
               */
            // calculate_swing_foot_piece_trajectory_swing_leg_last_piece_no_theta(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_follow, swing_foot_z_hold, 
            //        x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete);

            /*
               Calculate just the swing leg (theta is not soft)
               */
            // calculate_swing_foot_piece_trajectory_swing_leg(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_follow, swing_foot_z_hold,
            //         x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete);

            /*
               Calculate till heel touches the ground. Then interpolate own trajectory
               */

            calculate_swing_foot_piece_trajectory_swing_leg_shortened(parameter_init, 0, x_swing_foot_start, x_swing_foot_end, swing_foot_z_peak, swing_foot_x_peak, y_follow, swing_foot_z_hold,
                    x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete, theta_ref_swing_foot_complete);
            // calculate_swing_foot_last_piece( parameter_init, cycle_duration, x_ref_swing_foot_complete->back(), x_swing_foot_end, z_ref_swing_foot_complete->back(), parameter_init.robot_ankle_to_foot,
            //     x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete);
        }
        else if (i == 0){
            x_ref_swing_foot_complete->insert(x_ref_swing_foot_complete->end(), amount_first_step_samples, p_ref_x.at(0));
            y_ref_swing_foot_complete->insert(y_ref_swing_foot_complete->end(), amount_first_step_samples, -p_ref_y.at(0));
            z_ref_swing_foot_complete->insert(z_ref_swing_foot_complete->end(), amount_first_step_samples, parameter_init.robot_ankle_to_foot);
            theta_ref_swing_foot_complete->insert(theta_ref_swing_foot_complete->end(), amount_first_step_samples, 0);

        }
        else {

        }
        //std::cout << "i: " << i << " y_follow: " << y_follow << std::endl;

        //std::cout << i << std::endl;
        // std::cout << x_ref_swing_foot_complete->size() << std::endl;
        // std::cout << y_ref_swing_foot_complete->size() << std::endl;
        // std::cout << z_ref_swing_foot_complete->size() << std::endl;
    }

}
// IK
void convert_foot2ankle(joint_state foot, joint_state * ankle, parameters parameter_init){
    dlib::matrix<double, 3, 1> foot_to_ankle;
    //foot_to_ankle = foot_length_front, 0, -robot_ankle_to_foot;
    foot_to_ankle = 0, 0, -parameter_init.robot_ankle_to_foot;
    ankle->p = foot.p - ankle->R*foot_to_ankle;
    ankle->R = foot.R;
}

dlib::matrix<double, 3, 3> Rroll(double phi){
    dlib::matrix<double, 3, 3> R_x;
    R_x     =   1, 0        , 0,
            0, cos(phi) , -sin(phi),
            0, sin(phi) , cos(phi);
    return R_x;
}

dlib::matrix<double, 3, 3> Rpitch(double theta){
    dlib::matrix<double, 3, 3> R_y;
    R_y     =   cos(theta),  0, sin(theta),
            0,           1, 0,
            -sin(theta), 0, cos(theta);
    return R_y;
}

dlib::matrix<double, 3, 3> Ryaw(double psi){
    dlib::matrix<double, 3, 3> R_z;
    R_z     =   cos(psi),   -sin(psi),  0,
            sin(psi),   cos(psi),   0,
            0,          0,          1;
    return R_z;
}


dlib::matrix<double, 6, 1> InverseKinematics_analytical(joint_state Foot, joint_state Body, parameters parameter_init){

    //std::cout << Body.p - Foot.p << std::endl;
    double D = Foot.p(1);
    double A = parameter_init.robot_knee_length;
    double B = parameter_init.robot_shin_length;
    double q2, q3, q4, q5, q6, q6a, q7;

    dlib::matrix<double, 3, 1> tmp_vector;
    tmp_vector = 0, D, -parameter_init.pz_com;
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
    //std::cout << q << std::endl;

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

void calculate_walking_pattern(joint_state left_foot, joint_state right_foot, joint_state body, parameters parameter_init,
        std::vector<dlib::matrix<double, 12, 1>> * q12_vector){
    dlib::matrix<double, 12, 1> q12;
    joint_state left_ankle;
    joint_state right_ankle;
    convert_foot2ankle(left_foot, &left_ankle, parameter_init);
    dlib::set_colm(q12, dlib::range(0, 5))   = InverseKinematics_analytical(left_ankle, body, parameter_init);

    convert_foot2ankle(right_foot, &right_ankle, parameter_init);
    dlib::set_colm(q12, dlib::range(6, 11))  = InverseKinematics_analytical(right_ankle, body, parameter_init);

    q12_vector->push_back(q12); 
}

void calculate_walking_pattern_ankle(joint_state left_foot, joint_state right_foot, joint_state body, parameters parameter_init,
        std::vector<dlib::matrix<double, 12, 1>> * q12_vector){
    dlib::matrix<double, 12, 1> q12;

    dlib::set_colm(q12, dlib::range(0, 5))   = InverseKinematics_analytical(left_foot, body, parameter_init);

    dlib::set_colm(q12, dlib::range(6, 11))  = InverseKinematics_analytical(right_foot, body, parameter_init);

    q12_vector->push_back(q12); 
}

int current_time_nanoseconds(){
    struct timespec tm;
    clock_gettime(CLOCK_REALTIME, &tm);
    return tm.tv_nsec;
}


void calculate_ZMP(float ZMP[3], float FORCE_DATA[3], float TORQUE_DATA[3], float d){

    ZMP[0] = (-TORQUE_DATA[1] - FORCE_DATA[0]*d) / FORCE_DATA[2];
    ZMP[1] = ( TORQUE_DATA[0] - FORCE_DATA[1]*d) / FORCE_DATA[2];
    ZMP[2] = 0; // EVEN GROUND
}


void calculateComToAnkle(parameters parameter_init, dlib::matrix<double, 3,1> COM, dlib::matrix<double, 3,1> ankle, dlib::matrix<double, 3,1> * direction){
    dlib::matrix<double, 3,1> tmp;

    tmp = ankle - COM;
    // std::cout << "tmp: " << tmp << std::endl;
    double height_distance_left = parameter_init.robot_ankle_to_foot - ankle(2);
    double ratio         = height_distance_left/tmp(2);
    // std::cout << "height left:  " << height_distance_left << std::endl;
    // std::cout << "ratio: " << ratio << std::endl;
    *direction = tmp*ratio;
}

void correctEndEffector(double * x_increment, double * z_increment, dlib::matrix <double, 3, 1> COM, dlib::matrix <double, 3, 1> ankle, double Fx, double Fz){
    Fz = -Fz;
    // Calculate angle between ground and line (CoM to ankle). Necessary for coodinate transformation
    double height = fabs(COM(2) - ankle(2));
    double length = fabs(COM(0) - ankle(0));
    double alpha = 90-atan(height/length)*180/pi;
    dlib::matrix<double, 2,2> R;
    dlib::matrix<double, 2,2> C;
    double Fx_ref = 0;
    double Fz_ref = 100;
    alpha = alpha*pi/180;
    R = cos(alpha), -sin(alpha), sin(alpha), cos(alpha);
    // The smaller C the stiffer. The bigger C the more compliant
    C = 0, 0, 0, 1*5e-6;
    //C = 0, 0, 0, 5e-5; soft

    Fx = Fx - fmod(Fx, 1);
    Fz = Fz - fmod(Fz, 1);
    if(fabs(Fx) < 10){
        Fx = 0;
    }
    if(fabs(Fz) < 10){
        Fz = 0;
    }
    dlib::matrix<double, 2,1> F;
    F = Fx-Fx_ref, Fz-Fz_ref;
    dlib::matrix<double, 2, 1> delta;
    //std::cout << "alpha: " << alpha*180/pi << std::endl;
    delta = R*C*F;
    //std::cout << "Fx: " << Fx << " Fz:" << Fz << std::endl;
    //std::cout << "dx adding: " << delta(0) << "dz adding: " << delta(1) << std::endl;


    if(abs(Fz) < 250){
        if(fabs(delta(0)) < 0.01){
            *x_increment = *x_increment + delta(0);
        }
        else{
            *x_increment = *x_increment + 0;
        } if(fabs(delta(1)) < 0.01){
            *z_increment = *z_increment + delta(1);
        }
        else{
            *z_increment = *z_increment + 0;
        }
    }
    else{
        std::cout << "The force in z-direction exceeds 250N: " << Fz << "N" << std::endl;
    }

}

int left_foot_is_support_leg(int i, std::vector<double> p_ref_y){
    return (sign(p_ref_y.at(i)) == -1);
}

int right_foot_is_support_leg(int i, std::vector<double> p_ref_y){
    return (sign(p_ref_y.at(i)) == 1);
}

void force_control_leg_length(int i, dlib::matrix<double, 3, 1> COM, dlib::matrix<double, 3, 1> ankle, 
        std::vector<double> F0, std::vector<double> F2,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete){

    double x_increment = 0;
    double z_increment = 0;

    correctEndEffector(&x_increment, &z_increment, COM, ankle, F0.back(), F2.back());

    if(fabs(x_increment) > 1e-3){
        std::cout << "x_increment too much: " << x_increment << std::endl;
        x_increment = 0;
    }
    if(fabs(z_increment) > 1e-3){
        std::cout << "z_increment too much: " << z_increment << std::endl;
        z_increment = 0;
    }
    //std::cout << "dx after: " << x_increment << ", dz after: " << z_increment << std::endl;

    x_ref_swing_foot_complete->at(i) = x_ref_swing_foot_complete->at(i-1) + x_increment;
    z_ref_swing_foot_complete->at(i) = z_ref_swing_foot_complete->at(i-1) + z_increment;
}



void modify_support_foot_state(int left, double y_ref_now, dlib::matrix<double, 6, 1> q_tmp, parameters parameter_init,
        joint_state * new_support_foot_state, double x_offset_previous, double z_offset_previous){
    dlib::matrix<double, 6, 1> state;
    ForwardKinematic(fabs(left-1), parameter_init, y_ref_now, q_tmp, &state);
    //std::cout << "Length: " << state(0) << std::endl;
    //std::cout << "Height: " << state(2) << std::endl;
    //new_support_foot_state->p = state(0)+x_offset_previous, state(1), state(2)+z_offset_previous; 
    new_support_foot_state->p = state(0), state(1), state(2); 
    new_support_foot_state->R = Ryaw(state(3))*Rpitch(state(4))*Rroll(state(5));
}

void determine_support_foot_offset(int left, double y_ref_now, dlib::matrix<double, 6, 1> q_tmp, dlib::matrix<double,6,1> support_foot_state, parameters parameter_init,
        dlib::matrix<double,6,1> * support_foot_offset){

    dlib::matrix<double,6,1> state;
    ForwardKinematic(left, parameter_init, y_ref_now,  q_tmp, &state);

    *support_foot_offset = support_foot_state - state;
    //std::cout << "State support foot (before): " 
    //<< support_foot_state << std::endl;
    //std::cout << "State (from FK): " 
    //<< state << std::endl;
    std::cout << "offset: " << dlib::trans(*support_foot_offset) << std::endl;
}








void add_offset_to_foot_trajectory(joint_state * foot, dlib::matrix<double,6,1> support_foot_offset){
    if((fabs(support_foot_offset(0)) < 0.03) && (fabs(support_foot_offset(1)) < 0.03) && (fabs(support_foot_offset(2)) < 0.03)){
        foot->p(0) += support_foot_offset(0);
        foot->p(1) += support_foot_offset(1);
        foot->p(2) += support_foot_offset(2);
    }
    else{
        std::cout << "The offset is too large: " << dlib::trans(support_foot_offset) << std::endl;
    }

    double phi, theta, psi;
    rotm2eul(foot->R, &phi,&theta,&psi);
    //std::cout << "The offset is:" << dlib::trans(support_foot_offset) << std::endl;
    //std::cout << "phi:" << phi << ", theta: " << theta << ", psi: " << psi << std::endl;

    //std::cout << "original theta: " << theta << " theta offset: " << support_foot_offset(4) << std::endl;
    if( (fabs(phi+support_foot_offset(3)) < 2*pi/180) && (fabs(theta+support_foot_offset(4)) < 15*pi/180) && (fabs(psi+support_foot_offset(5)) < 2*pi/180)){
        foot->R = Ryaw(phi)*Rpitch(theta+support_foot_offset(4))*Rroll(psi);
    }
    else{
        std::cout << "The euler angles are too large: " << phi << ", " << theta << ", " << psi << std::endl;
    }
}

int modify_swing_foot_trajectory(int i, dlib::matrix<double, 3, 1> COM, dlib::matrix<double, 3, 1> ankle, parameters parameter_init, 
        std::vector<double> F0, std::vector<double> F2, double ZMP_x, int in_foot_landing_control_phase,
        std::vector<double> * x_ref_swing_foot_complete, std::vector<double> * y_ref_swing_foot_complete, std::vector<double> * z_ref_swing_foot_complete){

    double time_passed, t_initial;
    if(in_foot_landing_control_phase == 1){
        std::cout << "difference: " << (z_ref_swing_foot_complete->at(i-1) - parameter_init.robot_ankle_to_foot) << std::endl;
        if( ((z_ref_swing_foot_complete->at(i-1) - parameter_init.robot_ankle_to_foot) < 0.04) 
            &&  ((z_ref_swing_foot_complete->at(i-1) - parameter_init.robot_ankle_to_foot) > -0.01) ){
            //std::cout << "Extending leg with difference: " << fabs(z_ref_swing_foot_complete->back() - parameter_init.robot_ankle_to_foot) << std::endl;
            force_control_leg_length(i, COM, ankle, F0, F2, x_ref_swing_foot_complete, y_ref_swing_foot_complete, z_ref_swing_foot_complete);
        }
        else{
            std::cout << "extended more than 4cm. Stopping to extend" << std::endl;
        }
        return 1;
    }
    else{
        return 0;
    }
}

void softening_ankle(std::vector<double> T0, std::vector<double> T1, int * t_x, int * t_y){

    double tau_convert_x = 4; // Scaling from Nm (ATI) to current (dynamixel)
    double tau_convert_y = 1.5; // Scaling from Nm (ATI) to current (dynamixel)

    //std::cout << "Setting torque to soft" << std::endl;

    if(sign(T1.back()) == 1){
        *t_x = 35 + tau_convert_x * (T1.back()-fmod(T1.back(),0.1));
    }
    else{
        //std::cout << "negative" << std::endl;
        *t_x = -35 + tau_convert_x * (T1.back()-fmod(T1.back(),0.1));
    }

    if (*t_x > 90){
        std::cout << "Exceed torque limit of 90 (x). Setting to 90" << std::endl;
        *t_x = 90;
    }
    else if (*t_x < -90){
        std::cout << "Exceed 4 torque limit of -90 (x). Setting to -90" << std::endl;
        *t_x = -90;
    }

    if(sign(T0.back()) == 1){
        *t_y = 25 + tau_convert_y * (T0.back()-fmod(T0.back(),0.1));
    }
    else{
        *t_y = -25 + tau_convert_y * (T0.back()-fmod(T0.back(),0.1));
    }

    if (*t_y > 50){
        *t_y = 50;
        std::cout << "Exceed torque limit of 50 (y). Setting to 50" << std::endl;
    }
    else if (*t_y < - 50){
        *t_y = -50;
        std::cout << "Exceed torque limit of -50 (y). Setting to -50" << std::endl;
    }
    //std::cout << "Torque goal x: " << *t_x << " Torque x: " << T0.back() << "Nm" << std::endl;
    //std::cout << "Torque goal y: " << *t_y << " Torque y: " << T1.back() << "Nm" << std::endl;
}
void ForwardKinematic_COM(int left, parameters parameter_init, dlib::matrix<double, 6,1> q, dlib::matrix<double, 6, 1> * state){
    // From CoM to leg
    double q1,q2,q3,q4,q5,q6;
    q1 = q(0)*pi/180        ;
    q2 = q(1)*pi/180+pi/2   ;
    q3 = -q(2)*pi/180       ;
    q4 = -q(3)*pi/180       ;
    q5 = -q(4)*pi/180       ;
    q6 = q(5)*pi/180-pi/2   ;

    double s1,s2,s3,s4,s5,s6,c1,c2,c3,c4,c5,c6;
    s1 = sin(q1);
    s2 = sin(q2);
    s3 = sin(q3);
    s4 = sin(q4);
    s5 = sin(q5);
    s6 = sin(q6);

    c1 = cos(q1);
    c2 = cos(q2); 
    c3 = cos(q3); 
    c4 = cos(q4);
    c5 = cos(q5);
    c6 = cos(q6);

    dlib::matrix<double, 4, 4> A;
    A= -c6*(s5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3))+c5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3)))-c1*s2*s6, 
        s5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3))-c5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3)), 
        c1*c6*s2-s6*(s5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3))+c5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3))), 
        0.275*c1*c2*c3-0.275*s4*(c3*s1+c1*c2*s3)-0.275*c4*(s1*s3-c1*c2*c3)-0.275*s1*s3,
        c6*(s5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))+c5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)))-s1*s2*s6,
        c5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))-s5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)),
        s6*(s5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))+c5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)))+c6*s1*s2,
        0.275*c1*s3+0.275*s4*(c1*c3-c2*s1*s3)+0.275*c4*(c1*s3+c2*c3*s1)+0.275*c2*c3*s1,
        c6*(c5*(s2*s3*s4-c3*c4*s2)+s5*(c3*s2*s4+c4*s2*s3))-c2*s6, 
        c5*(c3*s2*s4+c4*s2*s3)-s5*(s2*s3*s4-c3*c4*s2), 
        c2*c6+s6*(c5*(s2*s3*s4-c3*c4*s2)+s5*(c3*s2*s4+c4*s2*s3)), 
        0.275*s2*s3*s4-0.275*c3*s2-0.275*c3*c4*s2,
        0,
        0,
        0,
        1;

    dlib::matrix<double, 6, 1> state_tmp;
    double phi, theta, psi;
    rotm2eul(dlib::subm(A,dlib::range(0,2), dlib::range(0,2)),&phi, &theta, &psi);

    if(left){
        state_tmp = -A(1,3), -A(0,3), A(2,3), -phi, -psi, -theta;
    }
    else{
        state_tmp = -A(1,3), -A(0,3), A(2,3), -theta, -psi, -phi;
    }

    state_tmp(2) += parameter_init.z_com_mbm;

    *state = state_tmp;
}

void ForwardKinematic(int left, parameters parameter_init, double y_ref_now,  dlib::matrix<double, 6,1> q, dlib::matrix<double, 6, 1> * state){
    // From CoM to leg
    double q1,q2,q3,q4,q5,q6;
    q1 = q(0)*pi/180        ;
    q2 = q(1)*pi/180+pi/2   ;
    q3 = -q(2)*pi/180       ;
    q4 = -q(3)*pi/180       ;
    q5 = -q(4)*pi/180       ;
    q6 = q(5)*pi/180-pi/2   ;

    double s1,s2,s3,s4,s5,s6,c1,c2,c3,c4,c5,c6;
    s1 = sin(q1);
    s2 = sin(q2);
    s3 = sin(q3);
    s4 = sin(q4);
    s5 = sin(q5);
    s6 = sin(q6);

    c1 = cos(q1);
    c2 = cos(q2); 
    c3 = cos(q3); 
    c4 = cos(q4);
    c5 = cos(q5);
    c6 = cos(q6);

    dlib::matrix<double, 4, 4> A;
    A= -c6*(s5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3))+c5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3)))-c1*s2*s6, 
        s5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3))-c5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3)), 
        c1*c6*s2-s6*(s5*(c4*(c3*s1+c1*c2*s3)-s4*(s1*s3-c1*c2*c3))+c5*(s4*(c3*s1+c1*c2*s3)+c4*(s1*s3-c1*c2*c3))), 
        0.275*c1*c2*c3-0.275*s4*(c3*s1+c1*c2*s3)-0.275*c4*(s1*s3-c1*c2*c3)-0.275*s1*s3,
        c6*(s5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))+c5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)))-s1*s2*s6,
        c5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))-s5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)),
        s6*(s5*(c4*(c1*c3-c2*s1*s3)-s4*(c1*s3+c2*c3*s1))+c5*(s4*(c1*c3-c2*s1*s3)+c4*(c1*s3+c2*c3*s1)))+c6*s1*s2,
        0.275*c1*s3+0.275*s4*(c1*c3-c2*s1*s3)+0.275*c4*(c1*s3+c2*c3*s1)+0.275*c2*c3*s1,
        c6*(c5*(s2*s3*s4-c3*c4*s2)+s5*(c3*s2*s4+c4*s2*s3))-c2*s6, 
        c5*(c3*s2*s4+c4*s2*s3)-s5*(s2*s3*s4-c3*c4*s2), 
        c2*c6+s6*(c5*(s2*s3*s4-c3*c4*s2)+s5*(c3*s2*s4+c4*s2*s3)), 
        0.275*s2*s3*s4-0.275*c3*s2-0.275*c3*c4*s2,
        0,
        0,
        0,
        1;

    dlib::matrix<double, 6, 1> state_tmp;
    double phi, theta, psi;
    rotm2eul(dlib::subm(A,dlib::range(0,2), dlib::range(0,2)),&phi, &theta, &psi);

    //if(left){
        //state_tmp = -A(1,3), -A(0,3), A(2,3), -phi, -psi, -theta;
    //}
    //else{
        //state_tmp = -A(1,3), -A(0,3), A(2,3), -theta, -psi, -phi;
    //}

    if(left){
        state_tmp(1) = state_tmp(1) - parameter_init.robot_width/2 + y_ref_now;
    }
    else{
        state_tmp(1) = state_tmp(1) + parameter_init.robot_width/2 + y_ref_now;
    }
    state_tmp(2) += parameter_init.z_com_mbm;

    *state = state_tmp;
}


void rotm2eul(dlib::matrix<double, 3, 3> R, double * phi, double * theta, double * psi){
    *phi = atan2(R(1,0), R(0,0));
    *theta = atan2(-R(2,0),sqrt(pow(R(0,0),2)+pow(R(1,0),2)));
    //*theta = atan2(-R(2,0),sqrt(pow(R(2,1),2)+pow(R(2,2),2)));
    *psi = atan2(R(2,1),R(2,2));
}

double determineCOPfoot(double Fzl, double Fzr){
    return(((Fzl/(Fzl+Fzr))-0.5)*(-0.2));
}

void returnToReference(parameters parameter_init, dlib::matrix<double,6,1> current_state, dlib::matrix<double,3,1> * is_at_ref,
        dlib::matrix<double,3,1> orientation_offset, dlib::matrix<double,6,1> reference_state, double transition_time, joint_state * foot_state
        ){

    //std::cout   << "Current state: " << dlib::trans(subm(current_state, range(3,5)))*180/pi  << std::endl;
    dlib::matrix<double,3,1> state_difference;
    state_difference = dlib::rowm(reference_state, dlib::range(3,5)) - dlib::rowm(current_state, dlib::range(3,5));
    //state_difference = (dlib::rowm(reference_state, dlib::range(3,5))-orientation_offset)/(transition_time/parameter_init.Ts);
    //std::cout << "state difference (increment): " << dlib::trans(state_difference) << std::endl;
    double phi_ref, theta_ref, psi_ref;
    rotm2eul(foot_state->R, &phi_ref, &theta_ref, &psi_ref);
    double phi, theta, psi;
    double K_phi, K_theta, K_psi;

    K_phi   = 10/(transition_time/parameter_init.Ts);
    K_theta = 10/(transition_time/parameter_init.Ts);
    K_psi   = 10/(transition_time/parameter_init.Ts);
    double phi_corrected, theta_corrected, psi_corrected;
    //std::cout << "Real R:\n" << foot_state->R << std::endl;
    //rotm2eul(foot_state->R, &phi, &theta, &psi);
    phi   = current_state(3);
    theta = current_state(4);
    psi   = current_state(5);

    std::cout << "Real:      " << phi*180/pi << "," << theta*180/pi << "," << psi*180/pi << std::endl;
    //std::cout << "offset:    " << dlib::trans(orientation_offset*180/pi) << std::endl;
    std::cout << "Error: " << dlib::trans(state_difference*180/pi) << std::endl;

    dlib::matrix<double,3,1> tmp;
    tmp = *is_at_ref;
    //if( (fabs(phi) > 0.3*pi/180) && (tmp(0) == 0) ){
    //phi_corrected = phi + state_difference(0)*K_phi;
    //}
    //else{
    //phi_corrected = phi;
    phi_corrected = 0;
    tmp(0) = 1;

    //std::cout << "no more increment in phi" << std::endl;
    //}

    if( (fabs(theta) > 0.3*pi/180) && (tmp(1) == 0) ){
        theta_corrected = theta + state_difference(1)*K_theta;
    }
    else{
        //theta_corrected = theta;
        theta_corrected = 0;
        tmp(1) = 1;
        //std::cout << "no more increment in theta" << std::endl;
    }

    //if( (fabs(psi) > 0.3*pi/180) && (tmp(2) == 0) ){
    //psi_corrected = psi + state_difference(2)*K_psi;
    //}
    //else{
    //psi_corrected = psi;
    psi_corrected = 0;
    tmp(2) = 1;
    //std::cout << "no more increment in psi" << std::endl;
    //}
    //std::cout << "Corrected: " << phi_corrected*180/pi << "," << theta_corrected*180/pi << "," << psi_corrected*180/pi << std::endl;

    dlib::matrix<double,3,3> new_R;
    if( (psi_corrected < 5*pi/180) && (theta_corrected < 5*pi/180) && (phi_corrected < 5*pi/180)){
        new_R = Ryaw(psi_corrected)*Rpitch(theta_corrected)*Rroll(phi_corrected);
    }
    else{
        std::cout << "Problem with correction: " << phi_corrected*180/pi << "," << theta_corrected*180/pi << "," << psi_corrected*180/pi << std::endl;
        new_R = Ryaw(0)*Rpitch(0)*Rroll(0);
        *is_at_ref = 1,1,1;
    }
    *is_at_ref = tmp;
    foot_state->R = new_R;
}

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
        std::cout << "Time passed: " << *time_passed << "ms" << std::endl;
        *t_initial   = 0;
    }
}

void start_foot_landing_control(int i, int * in_foot_landing_control_phase, parameters parameter_init, std::vector<double> z_ref_swing_foot_complete,
        std::vector<double> p_ref_x, std::vector<double> p_ref_y, std::vector<double> p_ref_z, 
        std::vector<double> FzL, std::vector<double> FzR, double ZMP_xl, double ZMP_xr, double y_ref_now, 
        dlib::matrix<double, 6,1> q_tmp, 
        joint_state * left_support_foot_state, joint_state * right_support_foot_state,
        joint_state left_swing_foot_state, joint_state right_swing_foot_state,
        double *  xl_offset, double * zl_offset, double * xr_offset, double * zr_offset,
        dlib::matrix<double,3,1> * orientation_offset_l, dlib::matrix<double,3,1> * orientation_offset_r,
        int * ascended_before, dlib::matrix<double,3,1> * is_at_ref,
        dlib::matrix<double,3,1> hip_euler, double Tipping_Scale
        ){

    //The current state will be retrieved from FK
    dlib::matrix<double,6,1> current_state;                                              

    // Start control after initial phase (2 x Dsp)
    if(i >= 2*parameter_init.t_dsp/parameter_init.Ts){   
        // Flags for the different states in_foot_landing_control_phase: 
        // Ascending = -3
        // Descending = -2
        // Foot landing control active = 1
        // Finished FLC  = 0
        int ascending, descending, flc, finished, touching_ground; 

        double distance = 0.02; //This should be calculated from when the foot is supposed to hit with cos and sin. But for now, start with FLC when 2cm above the ground

        // Determine whether ascending or descending using the trajectory of the swing leg
        ascending       = (z_ref_swing_foot_complete.at(i)-z_ref_swing_foot_complete.at(i-1) > 0); 
        descending      = (z_ref_swing_foot_complete.at(i)-z_ref_swing_foot_complete.at(i-1) <= 0);
        if(ascending == descending){
            std::cout << "ERROR ascending and descending state at the same time" << std::endl;
        }

        // Determine whether the swing foot is touching the ground
        if(left_foot_is_support_leg(i, p_ref_y)){
            touching_ground = FzR.back() < -15;
        }
        else{
            touching_ground = FzL.back() < -15;
        }

        // Determine whether flc is active
        flc = (((z_ref_swing_foot_complete.at(i)-parameter_init.robot_ankle_to_foot) < distance) || (descending && touching_ground));

        // Determine whether flc finished
        if(left_foot_is_support_leg(i, p_ref_y)){
            finished = ((FzR.back() < -15) && (ZMP_xr < 0.15) && (ZMP_xr > -0.06));
        }
        else{
            finished = ((FzL.back() < -15) && (ZMP_xl < 0.15) && (ZMP_xl > -0.06));
        }


        if(ascending){
            dlib::matrix<double,3,1> orientation_offset_tmp;
            //
            //// Returning to original state
            //double x_increment, z_increment;
            //if(left_foot_is_support_leg(i, p_ref_y)){ // For left foot being support
            //x_increment = *xr_offset/((parameter_init.t_ssp/parameter_init.Ts)/4);
            //z_increment = *zr_offset/((parameter_init.t_ssp/parameter_init.Ts)/4);

            //if( (fabs(left_support_foot_state->p(2) - parameter_init.robot_ankle_to_foot) > 1e-4) && (x_increment < 1e-4) && (z_increment < 1e-4) ){
            //left_support_foot_state->p(0) -= x_increment;
            //left_support_foot_state->p(2) += z_increment;
            //} 
            //else if(x_increment > 1e-4 || z_increment > 1e-4){
            //std::cout << "The increment is too much" << std::endl;
            //}
            //else{
            //}
            //}
            //else{                                     // For right foot being support
            //x_increment = *xl_offset/((parameter_init.t_ssp/parameter_init.Ts)/3);
            //z_increment = *zl_offset/((parameter_init.t_ssp/parameter_init.Ts)/3);

            //if( (fabs(right_support_foot_state->p(2) - parameter_init.robot_ankle_to_foot) > 5e-4) && (x_increment < 1e-4) && (z_increment < 1e-4)){
            //right_support_foot_state->p(0) -= x_increment;
            //right_support_foot_state->p(2) += z_increment;
            //} 
            //else if(x_increment > 1e-4 || z_increment > 1e-4){
            //std::cout << "The increment is too much" << std::endl;
            //}
            //else{
            //}
            //}

            double adjustment_time = (parameter_init.t_ssp/parameter_init.Ts)/5;
            double phi_increment, theta_increment, psi_increment;
            double phi_offset, theta_offset, psi_offset;

            if(left_foot_is_support_leg(i, p_ref_y)){
                rotm2eul(right_support_foot_state->R, &phi_offset, &theta_offset, &psi_offset);
                phi_increment   = phi_offset/adjustment_time;
                theta_increment = theta_offset/adjustment_time;
                psi_increment   = psi_offset/adjustment_time;

                orientation_offset_tmp = *orientation_offset_r;

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
                *orientation_offset_r = orientation_offset_tmp(0), orientation_offset_tmp(1), orientation_offset_tmp(2);
            }
            else{
                rotm2eul(left_support_foot_state->R, &phi_offset, &theta_offset, &psi_offset);
                phi_increment = phi_offset/adjustment_time;
                theta_increment = theta_offset/adjustment_time;
                psi_increment = psi_offset/adjustment_time;

                orientation_offset_tmp = *orientation_offset_l;
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
                *orientation_offset_l = orientation_offset_tmp;
            }
            //std::cout << "Ascending" << std::endl;
            *ascended_before = 1; // For FSM
            *in_foot_landing_control_phase = 3;
        }
        else if(descending && !flc){
            *in_foot_landing_control_phase = 2;
            //std::cout << "i:" << i << " descending" << std::endl;
            //std::cout << "Descending" << std::endl;
        }
        else if(flc && !finished && *ascended_before){
            //ForwardKinematic(0, parameter_init, y_ref_now, q_tmp, &current_state); 
            //std::cout << "Current state: " << dlib::trans(current_state) << std::endl;
            *in_foot_landing_control_phase = 1;
            // The control is happening somewhere else
        }

        else if(flc && *ascended_before && finished){
            //else if(*in_foot_landing_control_phase == 1 && finished){
            *ascended_before = 0;
            *in_foot_landing_control_phase = 0;
            *is_at_ref = 0,0,0;

            // FLC finished, saving offsets and foot position

            if(left_foot_is_support_leg(i, p_ref_y)){
                *right_support_foot_state = right_swing_foot_state;
                ForwardKinematic(1, parameter_init, y_ref_now, q_tmp, &current_state); 
                right_support_foot_state->R = Ryaw(current_state(3))*Rpitch(current_state(4))*Rroll(current_state(5));
                *xr_offset = p_ref_x.at(i)+parameter_init.step_length - right_support_foot_state->p(0);
                *zr_offset = parameter_init.robot_ankle_to_foot - right_support_foot_state->p(2);
                //*xr_offset = 0 - right_support_foot_state->p(0);
                //*zr_offset = 0.109 - right_support_foot_state->p(2);
                *orientation_offset_r = current_state(5), current_state(4), current_state(3);
                //std::cout << "xr offset: " << *xr_offset << std::endl;
                //std::cout << "zr offset: " << *zr_offset << std::endl;
            }
            else{
                *left_support_foot_state = left_swing_foot_state;
                ForwardKinematic(0, parameter_init, y_ref_now, q_tmp, &current_state); 
                left_support_foot_state->R = Rroll(current_state(3))*Rpitch(current_state(4))*Ryaw(current_state(5));
                *xl_offset = p_ref_x.at(i)+parameter_init.step_length - left_support_foot_state->p(0);
                *zl_offset = parameter_init.robot_ankle_to_foot - left_support_foot_state->p(2);
                //std::cout << "xl offset: " << *xl_offset << std::endl;
                //std::cout << "zl offset: " << *zl_offset << std::endl;
                //*xl_offset = 0 - left_support_foot_state->p(0);
                //*zl_offset = 0.109 - left_support_foot_state->p(2);

                *orientation_offset_l = current_state(3), current_state(4), current_state(5);
            }
        }
        else if(*in_foot_landing_control_phase == 0){
            // Returning to original state
            double x_increment, z_increment;
            if(left_foot_is_support_leg(i, p_ref_y)){ // For left foot being support
                //std::cout << "xr offset: " << *xr_offset << std::endl;
                //std::cout << "zr offset: " << *zr_offset << std::endl;
                x_increment = *xr_offset/((parameter_init.t_ssp/parameter_init.Ts)/3);
                z_increment = *zr_offset/((parameter_init.t_ssp/parameter_init.Ts)/3);

                if( (fabs(right_support_foot_state->p(2) - parameter_init.robot_ankle_to_foot) > 5e-4) && (x_increment < 1e-4) && (z_increment < 1e-4) ){
                    right_support_foot_state->p(0) -= x_increment;
                    right_support_foot_state->p(2) += z_increment;
                } 
                else if(x_increment > 1e-4 || z_increment > 1e-4){
                    std::cout << "The increment is too much" << std::endl;
                    std::cout << "x: " << x_increment << std::endl;
                    std::cout << "z: " << z_increment << std::endl;
                }
                else{
                }
            }
            else{                                     // For right foot being support
                //std::cout << "xl offset: " << *xl_offset << std::endl;
                //std::cout << "zl offset: " << *zl_offset << std::endl;
                x_increment = *xl_offset/((parameter_init.t_ssp/parameter_init.Ts)/3);
                z_increment = *zl_offset/((parameter_init.t_ssp/parameter_init.Ts)/3);

                if( (fabs(left_support_foot_state->p(2) - parameter_init.robot_ankle_to_foot) > 5e-4) && (x_increment < 1e-4) && (z_increment < 1e-4)){
                    left_support_foot_state->p(0) -= x_increment;
                    left_support_foot_state->p(2) += z_increment;
                } 
                else if(x_increment > 1e-4 || z_increment > 1e-4){
                    std::cout << "The increment is too much (left leg)" << std::endl;
                    std::cout << "x: " << x_increment << std::endl;
                    std::cout << "z: " << z_increment << std::endl;

                }
                else{
                }
            }

        }
        else{
            std::cout << "shouldnt exist" << std::endl;
        }
        }
        else{
            if(left_foot_is_support_leg(i, p_ref_y)){
                left_support_foot_state->p = p_ref_x.at(i), p_ref_y.at(i), p_ref_z.at(i);
                left_support_foot_state->R = Rroll(0)*Rpitch(0)*Ryaw(0);
            }
            else{
                right_support_foot_state->p = p_ref_x.at(i), p_ref_y.at(i), p_ref_z.at(i);
                right_support_foot_state->R = Rroll(0)*Rpitch(0)*Ryaw(0);
            }
        }

    }
