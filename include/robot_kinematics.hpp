
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>

#include <vector>
#include <math.h>

#define NUM 4
class Robot
{
public:
    Robot(Eigen::Matrix<double, Eigen::Dynamic, NUM> dh_parameters);

    /**
     * @brief add a link to the robot.
     *
     * @param twist
     * @param link_length
     * @param link_offset
     * @param joint_angle
     */
    // void add_link(double twist, double link_length, double link_offset, double joint_angle);
    void constructRobot(Eigen::Matrix<double, Eigen::Dynamic, 4> &dh_parameters);

    /**
     * @brief a function that receives joint angles and returns the position and orientation of the end effector in a SE(3) matrix
     *
     * @param angles
     * @return Eigen::Matrix4f
     */
    Eigen::Matrix4d ForwardKinematics(Eigen::MatrixXd &angles);

    Eigen::VectorXd InverseKinematics();
    Eigen::MatrixXd Jacobian(Eigen::MatrixXd &angles, double dt);

private:
    Eigen::Matrix4d Transformation(double twist, double link_length, double link_offset, double joint_angle);

    int number_of_joints;
    Eigen::Matrix4d robot_transformation;
    Eigen::Matrix<double, Eigen::Dynamic, NUM> dh_parameters;
};