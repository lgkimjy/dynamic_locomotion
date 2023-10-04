#include "Trajectory.h"

void QuinticTrajecotryProfile::set_duration(double duration)
{
    final_time = duration;
}

void QuinticTrajecotryProfile::quintic_trajectory_generation(Eigen::Vector3d initial_state, Eigen::Vector3d final_state)
{
    Eigen::Matrix<double, 6, 6> mat;
    Eigen::Matrix<double, 6, 1> target;

    mat <<  1, initial_time, pow(initial_time,2),   pow(initial_time,3),    pow(initial_time,4),    pow(initial_time,5),
            0,            1,      2*initial_time, 3*pow(initial_time,2),  4*pow(initial_time,3),  5*pow(initial_time,4),
            0,            0,                   2,        6*initial_time, 12*pow(initial_time,2), 20*pow(initial_time,3),
            1,   final_time,   pow(final_time,2),     pow(final_time,3),      pow(final_time,4),      pow(final_time,5),
            0,            1,        2*final_time,   3*pow(final_time,2),    4*pow(final_time,3),    5*pow(final_time,4),
            0,            0,                   2,          6*final_time,   12*pow(final_time,2),   20*pow(final_time,3);

    target << initial_state(0), initial_state(1), initial_state(2), final_state(0), final_state(1), final_state(2);
    coeff = mat.inverse() * target;
    moving_time = 0.0;
    is_moving_flag = true;
}

void QuinticTrajecotryProfile::compute()
{
    if(moving_time <= final_time) {
        pos = coeff(0) + coeff(1)*moving_time + coeff(2)*pow(moving_time,2) + coeff(3)*pow(moving_time,3) + coeff(4)*pow(moving_time,4) + coeff(5)*pow(moving_time,5);
        vel = coeff(1) + 2*coeff(2)*moving_time + 3*coeff(3)*pow(moving_time,2) + 4*coeff(4)*pow(moving_time,3) + 5*coeff(5)*pow(moving_time,4);
        acc = 2*coeff(2) + 6*coeff(3)*moving_time + 12*coeff(4)*pow(moving_time,2) + 20*coeff(5)*pow(moving_time,3);
    }
    else {
        // moving_time = 0.0;
        is_moving_flag = false;
    }
}

double QuinticTrajecotryProfile::get_pos()
{
    return pos;
}

double QuinticTrajecotryProfile::get_vel()
{
    return vel;
}

double QuinticTrajecotryProfile::get_acc()
{
    return acc;
}