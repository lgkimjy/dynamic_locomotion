#include "Trajectory.h"

Eigen::Matrix<double, 6, 1> quintic_trajectory_generation(Eigen::Vector3d initial_pos, Eigen::Vector3d final_pos, double initial_time, double final_time)
{
    Eigen::Matrix<double, 6, 6> mat;
    Eigen::Matrix<double, 6, 1> target, coeff;

    // initial_time = t_moving;
    // final_time = t_step;

    mat <<  1, initial_time, pow(initial_time,2),   pow(initial_time,3),    pow(initial_time,4),    pow(initial_time,5),
            0,            1,      2*initial_time, 3*pow(initial_time,2),  4*pow(initial_time,3),  5*pow(initial_time,4),
            0,            0,                   2,        6*initial_time, 12*pow(initial_time,2), 20*pow(initial_time,3),
            1,   final_time,   pow(final_time,2),     pow(final_time,3),      pow(final_time,4),      pow(final_time,5),
            0,            1,        2*final_time,   3*pow(final_time,2),    4*pow(final_time,3),    5*pow(final_time,4),
            0,            0,                   2,          6*final_time,   12*pow(final_time,2),   20*pow(final_time,3);

    target << initial_pos(0), initial_pos(1), initial_pos(2), final_pos(0), final_pos(1), final_pos(2);
    coeff = mat.inverse() * target;
    return coeff;
}