#include <Eigen/Dense>

class Trajectory
{
    Trajectory();
    ~Trajectory();

    Eigen::Matrix<double, 6, 1> quintic_trajectory_generation(Eigen::Vector3d initial_pos, Eigen::Vector3d final_pos, double initial_time, double final_time);
};