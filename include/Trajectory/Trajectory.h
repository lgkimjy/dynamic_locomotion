#include <Eigen/Dense>

class QuinticTrajecotryProfile
{
public:
    QuinticTrajecotryProfile() {};
    ~QuinticTrajecotryProfile() {};

    bool is_moving_flag = false;
    double initial_time = 0.0;
    double final_time = 0.0;
    double moving_time = 0.0;
    Eigen::Matrix<double, 6, 1> coeff;

    double pos = 0.0;
    double vel = 0.0;
    double acc = 0.0;

    void set_duration(double duration);
    void quintic_trajectory_generation(Eigen::Vector3d initial_state, Eigen::Vector3d final_state);
    void compute(double dt);
    double get_pos();
    double get_vel();
    double get_acc();
};