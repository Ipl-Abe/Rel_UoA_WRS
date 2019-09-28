
#ifndef JOYCON_RTC_H
#define JOYCON_H


#include <iostream>
#include <vector>
#include <fcntl.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>


// check your device number
#define JOYCON_L "/dev/input/js1"
#define JOYCON_R "/dev/input/js2"

using namespace RTC;
using namespace std;


class JOYCON_RTC : public RTC::DataFlowComponentBase
{
public:
    JOYCON_RTC(RTC::Manager* manager);
    ~JOYCON_RTC();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

    // Additional Functions
    //virtual bool initialize(SimpleControllerIO* io) override;
  
    //virtual bool control() override;
    //void updateTargetJointAngles();
    //void controlJointsWithTorque();
    //void controlJointsWithVelocity();
    //void controlJointsWithPosition();

    //double deg2rad(double degree);
    //double rad2deg(double radian);
    //Matrix3d RotateX(double radian);
    //Matrix3d RotateY(double radian);
    //Matrix3d RotateZ(double radian);
    //int updateControlflg();
    //void sequence();
    //bool perform(int seqNum);
    //Vector3 toRadianVector3(double x, double y, double z);

    //void updateTargetJointAnglesBYInverseKinematicks();


protected:
    // DataInPort declaration
    RTC::TimedFloatSeq m_axis_L;
    RTC::TimedBooleanSeq m_button_L;
    RTC::TimedFloatSeq m_axis_R;
    RTC::TimedBooleanSeq m_button_R;

    RTC::OutPort<RTC::TimedFloatSeq> m_axis_LOut;
    RTC::OutPort<RTC::TimedBooleanSeq> m_button_LOut;
    RTC::OutPort<RTC::TimedFloatSeq> m_axis_ROut;
    RTC::OutPort<RTC::TimedBooleanSeq> m_button_ROut;



private:

    std::string m_device_path;
    int joy_fdL;
    int joy_fdR;
    int num_of_axisL;
    int num_of_axisR;
    int num_of_buttonsL;
    int num_of_buttonsR;
    char name_of_joystickL[80];
    char name_of_joystickR[80];

    vector<char> joy_buttonL;
    vector<char> joy_buttonR;
    vector<int> joy_axisL;
    vector<int> joy_axisR;

};

extern "C"
{
    DLL_EXPORT void JOYCON_RTCInit(RTC::Manager* manager);
};

#endif
