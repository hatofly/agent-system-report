#include <cnoid/SimpleController>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <vector>
#include <cassert>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <cnoid/BodyItem>
#include <cnoid/RateGyroSensor>
#define LOWPASS_N 20

using namespace cnoid;
class DynaBoxController1 : public SimpleController
{
    bool onLand = true; // Flag to check if the robot is on the ground
    double dt;
    Link* joints[6]; // Array to hold multiple joints
    Link* center;
    RateGyroSensor* gyro; // Gyro sensor for orientation control
    const double LandHeightThreshold = 2; // Threshold for jump detection
    const double maxPower = 1000.0; // Maximum power limit
    const double maxForce = 10000.0; // Maximum force limit

    double prev_pitch = 0.0; // Previous pitch angle for control
    double target_pitch = 30*(3.14/180); // Target pitch angle for control
    const double pitch_Kp = 5;//60はでかすぎ 50は足りない
    const double pitch_Kd = pitch_Kp * 0.01; // Proportional and derivative gains for pitch control
    ros::Publisher pub_PT;
    ros::Publisher pub_IMU;
    Eigen::Vector3d w_sum[LOWPASS_N]; // Array to hold angular velocity for low-pass filtering
    Eigen::Vector3d w;
    sensor_msgs::Imu imu_msg; // IMU message for publishing orientation data
    Eigen::Quaterniond imu_quat; // Quaternion for orientation
    Eigen::Quaterniond imu_quat_der; // Previous quaternion for orientation
    double simtime = 0.0; // Simulation time
    const double jumptime = 5.0;
    const double landtime = 5.0;
    const double jumpcycle = jumptime + landtime; // Total time for one jump cycle
    int stepcount = 0;

public:
    virtual bool initialize(SimpleControllerIO *io) override
    {
        std::vector<std::string> jointnames = {
            "REACTION_BASE_LEFT",
            "REACTION_BASE_RIGHT",
            "REACTION_WHEEL_LEFT_1",
            "REACTION_WHEEL_LEFT_2",
            "REACTION_WHEEL_RIGHT_1",
            "REACTION_WHEEL_RIGHT_2",
        };
        for (size_t i = 0; i < 6; ++i)
        {
            joints[i] = io->body()->link(jointnames[i]);
            joints[i]->setActuationMode(Link::JointTorque);
            io->enableIO(joints[i]);
        }
        dt = io->timeStep();
        center = io->body()->link("CENTER");
        /*
        center->setActuationMode(Link::JointTorque);
        center->u() = 0;
        */
        io->enableIO(center);
        ros::NodeHandle nh;
        pub_PT = nh.advertise<std_msgs::Float64MultiArray>("PT", 10);
        pub_IMU = nh.advertise<sensor_msgs::Imu>("imu", 10);
        gyro = io->body()->findDevice<RateGyroSensor>("GYRO");
        io->enableInput(gyro);
        for(int i=0; i < 9; ++i){
            imu_msg.orientation_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
            imu_msg.linear_acceleration_covariance[i] = 0.0;
        }
        imu_msg.orientation_covariance[0] = -1.0;
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 0.0;

        for (int i = 0; i < LOWPASS_N; ++i) {
            w_sum[i].setZero(); // Initialize the angular velocity sum
        }
        return true;
    }
    virtual bool control() override
    {
        //joints[0]->u() = 100.0; // Set all joint torques to zero
        // MARK: Jump Function
        // Get Translation
        Eigen::Vector3d translation = center->position().translation();

        // Check if the robot is on the ground
        /*
        if (translation.z() < LandHeightThreshold)
        {
            onLand = true;
            // Apply Force to Joints
            for (size_t i = 0; i < 2; ++i)
            {
                apply_limitedforce(1000, i); // Apply a force of maxForce to each joint
            }
        }
        else
        {
            onLand = false;
            for (size_t i = 0; i < 2; ++i)
            {
                apply_limitedforce(-100, i); // Apply a force of -10 to each joint
            }
        }
        */
        // MARK: Jump Function(based on time)
        /*
        if (std::fmod(simtime, jumpcycle) < jumptime)
        {
            // Apply Force to Joints
            for (size_t i = 0; i < 2; ++i)
            {
                apply_limitedforce(500, i); // Apply a force of maxForce to each joint
            }
        }
        else
        {
            for (size_t i = 0; i < 2; ++i)
            {
                apply_limitedforce(-10, i); // Apply a force of -10 to each joint
            }
        }*/
        // MARK: Jump Function(constant force)
        /*
        for (size_t i = 0; i < 2; ++i)
        {
            apply_limitedforce(500, i); // Apply a force of maxForce to each joint
        }*/
        
        // MARK: Pose Control
        // MARK:仮のz制御　limitを超えないようにあとで何とかして組み込もう
        for (size_t i = 0; i < 2; ++i)
        {
            apply_limitedforce(100 - joints[i]->q()*100, i);
        }
        //Eigen::Matrix3d R_temp = center->position().rotation();
        //double pitch = std::asin(R_temp(2, 0)); // Get the pitch angle
        w_sum[stepcount] = gyro->w(); // Accumulate angular velocity
        w.setZero(); // Initialize angular velocity vector
        for (int i = 0; i < LOWPASS_N; ++i)
        {
            w += w_sum[i]; // Sum the angular velocities
        }
        w /= LOWPASS_N; // Average the angular velocities

        /// angular velocity
        double pitch = w(1); // Get the pitch angle

        double pitch_vel = (pitch - prev_pitch) / dt; // Get the pitch velocity
        double pitch_torque = - ( pitch_Kp * (pitch - target_pitch) + pitch_Kd * pitch_vel); // PD control
        for (size_t i = 2; i < 6; ++i)
        {
            apply_limitedforce(pitch_torque,i); // Reset the torque for the first four joints
        }
        // apply_limitedforce(pitch_torque, 1); // Apply torque to the first joint
        prev_pitch = pitch;

        // MARK: Publish Pitch and Translation
        std_msgs::Float64MultiArray msg;
        msg.data.clear();
        msg.data.push_back(pitch); // Add pitch angle to the message
        msg.data.push_back(translation.x()); // Add x translation to the message
        msg.data.push_back(translation.y()); // Add y translation to the message
        msg.data.push_back(translation.z()); // Add z translation to the message

        pub_PT.publish(msg);
        // MARK: Simulation Time
        simtime += dt; // Increment simulation time

        return true;
    }
    // MARK: Power Limitter
    void apply_limitedforce(double force, int index){
        double vel = joints[index]->dq(); // Get the joint velocity
        double limited_force = force; // Initialize limited force
        if(force * vel < 0){
            // 負の仕事ならlimit不要
            limited_force = force;
        }else if(std::fabs(vel) < maxPower / maxForce) {
            // Avoid division by zero
            if (force > maxForce){
                limited_force = maxForce;
            }else if (force < -maxForce){
                limited_force = -maxForce;
            }else{
                limited_force = force; // No limit needed
            }
        }else{
            double ForceLimit = maxPower / std::fabs(vel);
            if (force > ForceLimit) {
                limited_force = ForceLimit;
            } else if (force < -ForceLimit) {
                limited_force = -ForceLimit;
            } else {
                limited_force = force;
            }
        }
        //assert(limited_force * vel < maxPower); // Ensure the limited force does not exceed max power
        joints[index]->u() = limited_force; // Apply the limited force to the joint
        stepcount++;
        stepcount %= LOWPASS_N; // Keep stepcount within bounds
    }
    
};
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(DynaBoxController1)