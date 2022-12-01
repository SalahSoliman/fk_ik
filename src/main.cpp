#include "robot_kinematics.hpp"
#include <iostream>
int main()
{
    Eigen::Matrix<double, Eigen::Dynamic, NUM> dh_parameters{
        {1.57, 10, 0, 0},
        {0, 5, 0, 0},
        {0, 5, 0, 0}};
    Robot test(dh_parameters);

    std::vector<double> joints{1.57, 1.57, 0.0};
    Eigen::Vector<double, 3> goal{20, 0, 0};

    // // Eigen::Matrix4d s = test.ForwardKinematics(joints);
    // auto s = test.InverseKinematics();
    // std::cout << s << std::endl;

    Eigen::MatrixXd Xd(3, 1);
    Xd(0, 0) = 0.2;
    Xd(1, 0) = 0.3;
    Xd(2, 0) = 0.4;

    auto x = test.Jacobian(Xd, 0.1);
    std::cout << x << std::endl;
}