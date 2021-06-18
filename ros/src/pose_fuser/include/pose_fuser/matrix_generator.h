#ifndef MAT_GENERATOR_H
#define MAT_GENERATOR_H

#include <eigen3/Eigen/Core>
#include <array>

namespace mat_generator {
    Eigen::Matrix<double, 6, 6> get_transformation_matrix_map_to_world(Eigen::Matrix<double, 3, 1> attitude);
    Eigen::Matrix<double, 6, 6> get_Jacobian_matrix_state(double deltaTime, Eigen::Matrix<double, 3, 1> attitude, Eigen::Matrix<double, 6, 1> u);
    Eigen::Matrix<double, 6, 6> get_Jacobian_matrix_input(double deltaTime, Eigen::Matrix<double, 3, 1> attitude);
    Eigen::Matrix<double, 6, 6> get_input_error_matrix(std::vector<double> noisePram, Eigen::Matrix<double, 6, 1> u);
    Eigen::Matrix<double, 3, 3> get_transform_matrix_RPY(Eigen::Matrix<double, 3, 1> attitude);
} // namespace mat_generator
#endif // MAT_GENERATOR_H