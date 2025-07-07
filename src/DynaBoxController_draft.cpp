#include <cnoid/SimpleController>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <vector>
#include <map>
#include <string>
#include <cmath>

using namespace std;
using namespace cnoid;

class DynaBoxController : public SimpleController
{
    vector<Link*> joints;
    map<string, int> jointIndexMap;

    // 状態
    vector<double> q_prev, dq, torque_cmd;

    // PID制御ゲイン
    const double Kp = 500.0;
    const double Ki = 0.0;
    const double Kd = 100.0;

    // 制限
    const double maxForce = 1000.0;
    const double maxSpeed = 100.0;
    const double maxPower = 1000.0;
    const double maxSpeed_prismatic = 1e12;
    const double maxForce_prismatic = 1000.0;

    // ジャンプ制御用
    Link* center;
    Link* reactionUnit;
    double dt;
    double timeAcc = 0.0;
    double landingTime = 0.0;

    bool isFallen = false;
    bool largeAngleError = false;
    bool onLand = false;

    // 目標ピッチ
    double targetPitch = M_PI * 40.0 / 180.0;
    double prevPitchError = 0.0;
    double integralPitchError = 0.0;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        Body* body = io->body();
        vector<string> jointNames = {"j0", "j1", "j2", "j3", "j4", "j5", "j6"};
        for (size_t i = 0; i < jointNames.size(); ++i) {
            Link* joint = body->link(jointNames[i]);
            if (!joint) {
                io->os() << "Joint " << jointNames[i] << " not found" << endl;
                return false;
            }
            joint->setActuationMode(Link::JointTorque);
            io->enableInput(joint);
            io->enableOutput(joint);
            joints.push_back(joint);
            jointIndexMap[jointNames[i]] = i;
        }

        center = body->link("CENTER");
        reactionUnit = body->link("REACTION_UNIT_BASE");
        if (!center || !reactionUnit) {
            io->os() << "CENTER or REACTION_UNIT_BASE not found" << endl;
            return false;
        }

        dt = io->timeStep();
        q_prev.resize(joints.size(), 0.0);
        dq.resize(joints.size(), 0.0);
        torque_cmd.resize(joints.size(), 0.0);

        return true;
    }

    virtual bool control() override
    {
        timeAcc += dt;

        // --- 各関節状態更新 ---
        for (size_t i = 0; i < joints.size(); ++i) {
            double q = joints[i]->q();
            dq[i] = (q - q_prev[i]) / dt;
            q_prev[i] = q;
        }

        // --- ジャンプ制御 ---
        double z = center->p().z();
        if (z < 0.5 || largeAngleError) {
            isFallen = true;
            torque_cmd[jointIndexMap["j0"]] = -maxForce_prismatic;
        } else if (z < 1.3) {
            isFallen = false;
            onLand = true;
            landingTime += dt;
            if (landingTime > 0.2)
                torque_cmd[jointIndexMap["j0"]] = maxForce_prismatic;
        } else {
            isFallen = false;
            onLand = false;
            landingTime = 0.0;
            torque_cmd[jointIndexMap["j0"]] = -1.0;
        }

        // --- 姿勢制御 ---
        Vector3 rpy = reactionUnit->R().eulerAngles(2, 1, 0).reverse(); // xyz順
        double pitch = rpy.y();

        double pitchError = targetPitch - pitch;
        if (pitchError < -M_PI * 0.2)
            largeAngleError = true;

        integralPitchError += pitchError * dt;
        double derivativePitchError = (pitchError - prevPitchError) / dt;
        prevPitchError = pitchError;

        double pitchControl = Kp * pitchError + Ki * integralPitchError + Kd * derivativePitchError;

        // j3, j4 にトルクを与える
        torque_cmd[jointIndexMap["j3"]] = pitchControl;
        torque_cmd[jointIndexMap["j4"]] = pitchControl;

        // --- トルク制限と出力 ---
        for (size_t i = 0; i < joints.size(); ++i) {
            double v = dq[i];
            double f = torque_cmd[i];

            if (v * f < 0) {
                joints[i]->u() = f;
            } else {
                double limited = min(maxForce, maxPower / max(1e-6, abs(v)));
                joints[i]->u() = std::clamp(f, -limited, limited);
            }
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(DynaBoxController)
