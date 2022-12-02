#include <iostream>
#include <functional>
#include "robot_kinematics.hpp"
#define DEBUG_IK 0
Robot::Robot(Eigen::Matrix<double, Eigen::Dynamic, NUM> dh_parameters) : dh_parameters(dh_parameters)
{
    Eigen::Matrix4d temp{{1.0, 0, 0, 0},
                         {0, 1.0, 0, 0},
                         {0, 0, 1.0, 0},
                         {0, 0, 0, 1.0}};

    robot_transformation = temp;
    number_of_joints = dh_parameters.rows();
}
Eigen::Matrix4d Robot::Transformation(double twist, double link_length, double link_offset, double joint_angle)
{
    Eigen::Matrix4d T_j{{cos(joint_angle), -1.0 * sin(joint_angle) * cos(twist), sin(joint_angle) * sin(twist), link_length * cos(joint_angle)},
                        {sin(joint_angle), cos(joint_angle) * cos(twist), -1.0 * cos(joint_angle) * sin(twist), link_length * sin(joint_angle)},
                        {0.0, sin(twist), cos(twist), link_offset},
                        {0.0, 0.0, 0.0, 1.0}};

    return T_j;
}

Eigen::Matrix4d Robot::ForwardKinematics(Eigen::MatrixXd &angles)
{
    Eigen::Matrix4d temp{{1.0, 0, 0, 0},
                         {0, 1.0, 0, 0},
                         {0, 0, 1.0, 0},
                         {0, 0, 0, 1.0}};

    robot_transformation = temp;

    for (int i = 0; i < dh_parameters.rows(); i++)
    {
        robot_transformation = robot_transformation * this->Transformation(dh_parameters(i, 0), dh_parameters(i, 1), dh_parameters(i, 2), angles(i, 0));
    }

    return robot_transformation;
}

Eigen::MatrixXd Robot::Jacobian(Eigen::MatrixXd &angles, double dt)
{
    Eigen::MatrixXd R0_0{{1.0, 0, 0},
                         {0, 1.0, 0},
                         {0, 0, 1.0}};
    Eigen::Matrix4d temp{{1.0, 0, 0, 0},
                         {0, 1.0, 0, 0},
                         {0, 0, 1.0, 0},
                         {0, 0, 0, 1.0}};

    Eigen::Matrix4d transformation = temp;
    Eigen::Matrix<double, 3, 1>
        zero_zero_one{0, 0, 1};

    // first term

    Eigen::Matrix<double, 4, 4> T0_3 = ForwardKinematics(angles);
    Eigen::Matrix<double, 3, 3> R0_3 = T0_3.block(0, 0, 3, 3);

    Eigen::Matrix<double, 3, 1> D0_3 = T0_3.block(0, 0, 3, 1);
    Eigen::Matrix<double, 3, 1> first_term = zero_zero_one.cross(D0_3);
    // second term
    transformation = transformation * this->Transformation(dh_parameters(0, 0), dh_parameters(0, 1), dh_parameters(0, 2), angles(0, 0)); // transform from 0 to 1
    Eigen::Matrix<double, 3, 1> D0_1 = transformation.block(0, 0, 3, 1);
    Eigen::Matrix<double, 3, 3> R0_1 = transformation.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> second_term = R0_1 * zero_zero_one;
    second_term = second_term.cross(D0_3 - D0_1);

    // third term
    transformation = transformation * this->Transformation(dh_parameters(1, 0), dh_parameters(1, 1), dh_parameters(1, 2), angles(1, 0)); // transform from 0 to 1
    Eigen::Matrix<double, 3, 1> D0_2 = transformation.block(0, 0, 3, 1);
    Eigen::Matrix<double, 3, 3> R0_2 = transformation.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> third_term = R0_2 * zero_zero_one;
    third_term = third_term.cross(D0_3 - D0_2);

    return temp;
}

Eigen::VectorXd Robot::InverseKinematics()
{

    // initial values
    Eigen::MatrixXd Xd(3, 1);
    // calaculate initial values of forward kinematics

    // consider "initial" error
    Eigen::MatrixXd i_1_theta(3, 1);
    Eigen::MatrixXd i_theta(3, 1);
    Eigen::MatrixXd e(3, 1);

    i_theta(0, 0) = M_PI / 5;
    i_theta(1, 0) = M_PI / 4;
    i_theta(2, 0) = 0;
    Eigen::Matrix4d FWD = ForwardKinematics(i_theta);

    Xd(0, 0) = 0;
    Xd(1, 0) = 10;
    Xd(2, 0) = 10;

    e = Xd - FWD.block(0, 3, 3, 1);
    //  solve IK numerically
    while ((std::abs(e(0, 0)) > 0.1) || (std::abs(e(1, 0)) > 0.1) || (std::abs(e(2, 0)) > 0.1))
    {
        Eigen::MatrixXd invJ = this->Jacobian(i_theta, 0.1).completeOrthogonalDecomposition().pseudoInverse();

#if DEBUG_IK
        std::cout << invJ.rows() << "\t" << invJ.cols();
        return i_theta;
#endif
        // update
        i_1_theta = i_theta + ForwardKinematics(i_theta).block(0, 0, 3, 3) * e;
        // calulate new posion of EE
        FWD = ForwardKinematics(i_1_theta);
        // compute error to be evaluated
        e = Xd - FWD.block(0, 3, 3, 1);
        i_theta = i_1_theta;
    }
    return i_theta;
}