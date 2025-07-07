#include <cnoid/SimpleController>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
using namespace cnoid;
class DynaBoxController1 : public SimpleController
{
    Link *joint;
    double q_ref;
    double q_prev;
    double dt;

public:
    virtual bool initialize(SimpleControllerIO *io) override
    {
        joint = io->body()->link("REACTION_BASE_FRONT");
        joint->setActuationMode(Link::JointTorque);
        io->enableIO(joint);
        q_ref =  0.0;
        dt = io->timeStep();
        return true;
    }
    virtual bool control() override
    {
        // PD gains
        static const double P = 200.0;
        static const double D = 50.0;
        double q = joint->q(); // input
        double dq = (q - q_prev) / dt;
        double dq_ref = 0.0;
        joint->u() = P * (q_ref - q) + D * (dq_ref - dq); // output
        q_prev = q;

        //Eigen::MatrixXd transform = joint->position().translation(); //4x4 matrix
        //std::cout << "Current position: " << transform(0, 3) << ", " << transform(1, 3) << ", " << transform(2, 3) << std::endl;

        return true;
    }
};
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(DynaBoxController1)