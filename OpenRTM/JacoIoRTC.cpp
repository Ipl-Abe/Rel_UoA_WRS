/**
    Author : Fumiaki Abe
    this RTC can get joint angle data from choreonoid and
    can set torque data.
*/

#include <cnoid/BodyIoRTC>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

class JacoIoRTC : public BodyIoRTC
{
public:
    JacoIoRTC(RTC::Manager* manager);
    ~JacoIoRTC();

    virtual bool initializeIO(ControllerIO* io) override;
    virtual bool initializeSimulation(ControllerIO* io) override;
    virtual void inputFromSimulator() override;
    virtual void outputToSimulator() override;

    BodyPtr body;
    
    // DataInPort declaration
    RTC::TimedDoubleSeq torque_L;
    RTC::InPort<RTC::TimedDoubleSeq> torque_LIn;
    
    RTC::TimedDoubleSeq torque_R;
    RTC::InPort<RTC::TimedDoubleSeq> torque_RIn;

    // DataOutPort declaration
    RTC::TimedDoubleSeq angle_L;
    RTC::OutPort<RTC::TimedDoubleSeq> angle_LOut;

    RTC::TimedDoubleSeq angle_R;
    RTC::OutPort<RTC::TimedDoubleSeq> angle_ROut;
};
// "activity_type" , "DataFlowComponent"
const char* spec[] =
{
    "implementation_id", "JacoIoRTC",
    "type_name",         "JacoIoRTC",
    "description",       "Robot I/O",
    "version",           "1.0",
    "vendor",            "Fumiaki Abe",
    "category",          "Generic",
    "activity_type", "PERIODIC",    
    "kind",          "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};

}


JacoIoRTC::JacoIoRTC(RTC::Manager* manager)
    : BodyIoRTC(manager),
      torque_LIn("u_L", torque_L),
      torque_RIn("u_R", torque_R),
      angle_LOut("q_L", angle_L),
      angle_ROut("q_R", angle_R)
{
}


JacoIoRTC::~JacoIoRTC()
{
}

// call at starting simulator
bool JacoIoRTC::initializeIO(ControllerIO* io)
{
    // Set InPort buffers
    addInPort("u_L", torque_LIn);
    addInPort("u_R", torque_RIn);
    
    // Set OutPort buffer
    addOutPort("q_L", angle_LOut);
    addOutPort("q_R", angle_ROut);
    angle_L.data.length(9);
    angle_R.data.length(9);

    return true;
}


bool JacoIoRTC::initializeSimulation(ControllerIO* io)
{
    body = io->body();

    for(auto joint : body->joints()){
        if(joint->isRevoluteJoint() || joint->isPrismaticJoint()){
            joint->setActuationMode(Link::JOINT_VELOCITY);
        }
    }

    return true;
}


void JacoIoRTC::inputFromSimulator()
{
    // left arm's root joint id is start from 4
    // right arm's root joint id is start from 13
    
    for (int i=0; i<9;++i){
        angle_L.data[i] = body->joint(i+4)->q();
        angle_R.data[i] = body->joint(i+13)->q();  
    }

    angle_LOut.write();
    angle_ROut.write();
}


void JacoIoRTC::outputToSimulator()
{
    // torque data from left arm
    if(torque_LIn.isNew()){
        torque_LIn.read();
        int n = torque_L.data.length();
        for(int i=0; i < n; ++i){
                body->joint(i+4)->dq_target() = torque_L.data[i];
                //cout << "torque_data : "<< torque_L.data[i] << endl;
        }
    }
    // torque data from right arm
    if(torque_RIn.isNew()){
        torque_RIn.read();
        int n = torque_R.data.length();
        
        for(int i=0; i < n; ++i){
                body->joint(i+13)->dq_target() = torque_R.data[i];
        }
    }
}

extern "C"
{
    DLL_EXPORT void JacoIoRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(
            profile, RTC::Create<JacoIoRTC>, RTC::Delete<JacoIoRTC>);
    }
};

