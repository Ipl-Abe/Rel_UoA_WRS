
#ifndef Jaco2AISTControllerRTC_H
#define Jaco2AISTControllerRTC_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include "Interpolator.h"
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <boost/format.hpp>
#include <cnoid/EigenUtil>
#include <cnoid/BodyIoRTC>
#include <cnoid/ExecutablePath>
#include <cnoid/BodyLoader>
#include <cnoid/FileUtil>
#include <cnoid/Link>


using namespace RTC;
using namespace cnoid;
using boost::format;
using namespace std;


class Jaco2AISTControllerRTC : public RTC::DataFlowComponentBase
{
public:
    Jaco2AISTControllerRTC(RTC::Manager* manager);
    ~Jaco2AISTControllerRTC();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

    // Additional Functions
    //virtual bool initialize(SimpleControllerIO* io) override;
  
    //virtual bool control() override;
    void updateTargetJointAngles();
    void controlJointsWithTorque();
    void controlJointsWithVelocity();
    void controlJointsWithPosition();

    double deg2rad(double degree);
    double rad2deg(double radian);
    Matrix3d RotateX(double radian);
    Matrix3d RotateY(double radian);
    Matrix3d RotateZ(double radian);
    int updateControlflg();
    void sequence();
    bool perform(int seqNum);
    Vector3 toRadianVector3(double x, double y, double z);

    void updateTargetJointAnglesBYInverseKinematicks();


protected:
    // DataInPort declaration
    RTC::TimedFloatSeq m_axis_L;
    RTC::InPort<RTC::TimedFloatSeq> m_axis_LIn;
  
    RTC::TimedBooleanSeq m_button_L;
    RTC::InPort<RTC::TimedBooleanSeq> m_button_LIn;
  
    RTC::TimedFloatSeq m_axis_R;
    RTC::InPort<RTC::TimedFloatSeq> m_axis_RIn;
  
    RTC::TimedBooleanSeq m_button_R;
    RTC::InPort<RTC::TimedBooleanSeq> m_button_RIn;

    // Arm Angle
    RTC::TimedDoubleSeq m_angle_L;
    RTC::InPort<RTC::TimedDoubleSeq> m_angle_LIn;

    RTC::TimedDoubleSeq m_angle_R;
    RTC::InPort<RTC::TimedDoubleSeq> m_angle_RIn;

    // DataInPort declaration 
    // Arm Torque
    RTC::TimedDoubleSeq m_torque_L;
    RTC::OutPort<RTC::TimedDoubleSeq> m_torque_LOut;

    RTC::TimedDoubleSeq m_torque_R;
    RTC::OutPort<RTC::TimedDoubleSeq> m_torque_ROut;

private:
    Body* body;
    Body* ikBody;
    double dt;
    VectorXd qref, qold, qref_old;
    VectorXd qref_L, qold_L, qref_old_L;
    VectorXd qref_R, qold_R, qref_old_R;

    BodyPtr bodyPointer;
    int numJoints;
    int leftArm_id;
    int rightArm_id;

    int time;
    int controlModeFlag = 0;

    Link* ikWrist;
    Link* ikWrist2;
    std::shared_ptr<cnoid::JointPath> baseToWrist;
    std::shared_ptr<cnoid::JointPath> baseToWrist2;
    Link* base; 
    Link::ActuationMode mainActuationMode;
    Interpolator<VectorXd> wristInterpolator;
    Interpolator<VectorXd> wristInterpolator2;
    
    int armFlag; // switch left arm and right arm
    double currentTime;
    int controlModeflg = 0;


};

extern "C"
{
    DLL_EXPORT void Jaco2AISTControllerRTCInit(RTC::Manager* manager);
};

#endif
