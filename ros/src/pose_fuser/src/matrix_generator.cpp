#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigen>
#include <pose_fuser/matrix_generator.h>

Eigen::Matrix<double, 6, 6> mat_generator::get_transformation_matrix_map_to_world(Eigen::Matrix<double, 3, 1> attitude){
    Eigen::Matrix<double, 6, 6> T;
    double c_roll = std::cos(attitude(0));
    double c_pitch = std::cos(attitude(1));
    double c_yaw = std::cos(attitude(2));
    double s_roll = std::sin(attitude(0));
    double s_pitch = std::sin(attitude(1));
    double s_yaw = std::sin(attitude(2));
    double t_pitch = std::tan(attitude(2));

    T <<
        c_pitch * c_yaw,
        s_roll * s_pitch * c_yaw - c_roll * s_yaw,
        c_roll * s_pitch * c_yaw + s_roll * s_yaw,
        0, 0, 0,

        c_pitch * s_yaw,
        s_roll * s_pitch * s_yaw + c_roll * c_yaw,
        c_roll * s_pitch * s_yaw - s_roll * c_yaw,
        0, 0, 0,

        - s_pitch,
        s_roll * c_pitch,
        c_roll * c_pitch,
        0, 0, 0,

        0, 0, 0, 1, s_roll * t_pitch, - c_roll * t_pitch,

        0, 0, 0, 0, c_roll, - s_roll,

        0, 0, 0, 0, s_roll / c_pitch, c_roll / c_pitch;
        
    return T;
}

Eigen::Matrix<double, 6, 6> mat_generator::get_Jacobian_matrix_state(double deltaTime, Eigen::Matrix<double, 3, 1> attitude, Eigen::Matrix<double, 6, 1> u){
    Eigen::Matrix<double, 6, 6> G;
    double deltaX = u(0) * deltaTime;
    double deltaY = u(1) * deltaTime;
    double deltaZ = u(2) * deltaTime;
    double deltaRoll = u(3) * deltaTime;
    double deltaPitch = u(4) * deltaTime;
    double deltaYaw = u(5) * deltaTime;
    double c_roll = std::cos(attitude(0));
    double c_pitch = std::cos(attitude(1));
    double c_yaw = std::cos(attitude(2));
    double s_roll = std::sin(attitude(0));
    double s_pitch = std::sin(attitude(1));
    double s_yaw = std::sin(attitude(2));
    double t_pitch = std::tan(attitude(2));

    G <<
        1, 0, 0,
        deltaY * (c_roll * s_pitch * c_yaw + s_roll * s_yaw) + deltaZ * (-s_roll * s_pitch * c_yaw + c_roll * s_yaw),
        -deltaX * s_pitch * c_yaw + deltaY * s_roll * c_pitch * c_yaw + deltaZ * c_roll * c_pitch * c_yaw,
        -deltaX * c_pitch * s_yaw + deltaY * (-s_roll * s_pitch * s_yaw - c_roll * c_yaw) + deltaZ * (-c_roll * s_pitch * s_yaw + s_roll * c_yaw),

        0, 1, 0,
        deltaY * (c_roll * s_pitch * s_yaw - s_roll * c_yaw) + deltaZ * (-s_roll * s_pitch * s_yaw - c_roll * c_yaw),
        -deltaX * s_pitch * s_yaw + deltaY * s_roll * c_pitch * s_yaw + deltaZ * c_roll * c_pitch * s_yaw,
        deltaX * c_pitch * c_yaw + deltaY * (s_roll * s_pitch * c_yaw - c_roll * s_yaw) + deltaZ * (c_roll * s_pitch * c_yaw + s_roll * s_pitch),

        0, 0, 1,
        deltaY * c_roll * c_pitch - deltaZ * s_roll * c_pitch,
        -deltaX * c_pitch - deltaY * s_roll * s_pitch - deltaZ * c_roll * s_pitch,
        0,

        0, 0, 0,
        1 + deltaPitch * c_roll * t_pitch - deltaYaw * s_roll * t_pitch,
        deltaPitch * s_roll * (1 + std::pow(t_pitch, 2.0f)) + deltaYaw * c_roll * (1 + std::pow(t_pitch, 2.0f)),
        0,

        0, 0, 0,
        -deltaPitch * s_roll - deltaPitch * c_roll,
        1, 0,

        0, 0, 0,
        (deltaPitch * c_roll) / c_pitch - (deltaYaw * s_roll) / c_pitch,
        (deltaPitch * s_roll * t_pitch) / c_pitch + (deltaYaw * c_roll * t_pitch) / c_pitch,
        1;
    return G;
}

Eigen::Matrix<double, 6, 6> mat_generator::get_Jacobian_matrix_input(double deltaTime, Eigen::Matrix<double, 3, 1> attitude){
    Eigen::Matrix<double, 6, 6> V;
    
    V = deltaTime * mat_generator::get_transformation_matrix_map_to_world(attitude);

    return V;
}

Eigen::Matrix<double, 6, 6> mat_generator::get_input_error_matrix(Eigen::Matrix<double, 36, 1> errorPram, Eigen::Matrix<double, 6, 1> u){
    Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();

    for (int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){
            M(i,i) += errorPram(6 + i+j) * std::pow(u(j),2.0f);
        }
    }
    return M;
}