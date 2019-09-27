#include "Jaco2ControllerRTC.h"
#include <iostream>
#include <vector>
#include <fcntl.h>


#define VEL_GAIN 0.1

static const char* Jaco2ControllerRTC_spec[] =
  {
    "implementation_id", "Jaco2ControllerRTC",
    "type_name",         "Jaco2ControllerRTC",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "Fumiaki Abe",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.device_path", """",

    // Widget
    "conf.__widget__.device_path", "text",
    // Constraints

    "conf.__type__.device_path", "string",

    ""
  };

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Jaco2ControllerRTC::Jaco2ControllerRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_axis_LIn("axis_L", m_axis_L),
    m_button_LIn("button_L", m_button_L),
    m_axis_RIn("axis_R", m_axis_R),
    m_button_RIn("button_R", m_button_R),
    m_angle_LIn("angle_L", m_angle_L),
    m_angle_RIn("angle_R", m_angle_R),
    m_torque_LOut("torque_L", m_torque_L),
    m_torque_ROut("torque_R" ,m_torque_R)
{
}

Jaco2ControllerRTC::~Jaco2ControllerRTC()
{
}



RTC::ReturnCode_t Jaco2ControllerRTC::onInitialize()
{
    std::cout << "onInitialize" << std::endl; 
    addInPort("axis_L", m_axis_LIn);
    addInPort("button_L", m_button_LIn);
    addInPort("axis_R", m_axis_RIn);
    addInPort("button_R", m_button_RIn);
    addInPort("angle_L", m_angle_LIn);
    addInPort("angle_R", m_angle_RIn);  

    addOutPort("torque_L", m_torque_LOut);
    addOutPort("torque_R", m_torque_ROut);  

    // load Jaco arm model
    string modelfile = getNativePathString(
        boost::filesystem::path(shareDirectory()) / "model/AizuSpider/AizuSpiderDA.body");

    BodyLoader loader;
    loader.setMessageSink(cout);
    loader.setShapeLoadingEnabled(false);
    body = loader.load(modelfile);
            
    if(!body){
        cout << modelfile << " cannot be loaded." << endl;
        return RTC::RTC_ERROR;
    }
    numJoints = body->numJoints(); // joint number : 0-21
    leftArm_id = body->link("ARM1_HAND")->jointId(); // id : 4
    rightArm_id = body->link("ARM2_HAND")->jointId(); // id : 13

    currentTime = 0.0; // recode simulation time to use interpolator functions
    return RTC::RTC_OK;
}


RTC::ReturnCode_t Jaco2ControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    // when you wanto to update joint angle
    // use following variables 
    qref_L.resize(9); // ARM1_SHOULDER ~ ARM1_FINGER3
    qold_L.resize(9);
   
    qref_R.resize(9);
    qold_R.resize(9);
    
    // target position variables for HAND
    VectorXd p(6);
    VectorXd p2(6);

    ikWrist = body->link("ARM1_HAND");
    ikWrist2 = body->link("ARM2_HAND");
    Link* base = body->rootLink();
    baseToWrist = getCustomJointPath(body, base, ikWrist);
    baseToWrist2 = getCustomJointPath(body, base, ikWrist2);
    base->p().setZero();
    base->R().setIdentity();

    // Joint data comes from simulator
    if(m_angle_LIn.isNew()){
        m_angle_LIn.read();
    }
    // cout << "come angleL data" << endl;
    for(int i=0; i<m_angle_L.data.length(); ++i){
        double q = m_angle_L.data[i];
        qold_L[i] = q;
        body->joint(i+4)->q() = q;
        //cout << "q :" << i << " : " << rad2deg(q) << endl;
    }


    if(m_angle_RIn.isNew()){
        m_angle_RIn.read();  
    }
    // cout << "come angleL data" << endl;
    for(int i=0; i<m_angle_R.data.length(); ++i){
        double q = m_angle_R.data[i];
        qold_L[i] = q;
        body->joint(i+13)->q() = q;
        //cout << "q :" << i << " : " << rad2deg(q) << endl;
    }
    cout << "Debuc Activate" << endl;
    qref_L = qold_L;
    qref_R = qold_R;
    
    for(int i = 0; i < 9;i++){
        cout << "qref_L : " << rad2deg(qref_L[i]) << endl;
    }

    baseToWrist->calcForwardKinematics();
    baseToWrist2->calcForwardKinematics();

    //set up joint torque
    m_torque_L.data.length(9);
    m_torque_R.data.length(9);
    for(int i=0; i<9; ++i)
    {
        m_torque_L.data[i] = 0.0;
        m_torque_R.data[i] = 0.0;
    }

    mainActuationMode = Link::JOINT_VELOCITY;
    std::string prefix;
    prefix = "velocity";

    cout << "End Activate" << endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t Jaco2ControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

// m_angle_L.data.length() が使用可能
RTC::ReturnCode_t Jaco2ControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    cout << "in Execute " << endl;
    if(m_axis_LIn.isNew()){
        m_axis_LIn.read();
      //  std::cout << "data : " << m_axis_L.data[0] << std::endl;
    }
    if(m_button_LIn.isNew()){
        m_button_LIn.read();
        //std::cout << "button data :" << m_button_L.data[0] << std::endl;
    }

    if(m_axis_RIn.isNew()){
        m_axis_RIn.read();
       // std::cout << "data : " << m_axis_R.data[0] << std::endl;
    }
    if(m_button_RIn.isNew()){
        m_button_RIn.read();
        //std::cout << "button data :" << m_button_R.data[0] << std::endl;
    }

    // time step
    dt = 0.005;
    cout << " debug 2" << endl;
    m_torque_LOut.write();
    m_torque_ROut.write();

    cout << "debug 3" << endl;
    if(m_angle_LIn.isNew()){
        m_angle_LIn.read();  
    }

    if(m_angle_RIn.isNew()){
        m_angle_RIn.read();
    }
    cout << "debug 4" << endl;
    //controlModeflg = updateControlflg();
    if(controlModeflg == 0){
       updateTargetJointAnglesBYInverseKinematicks();
    }
    else {
        sequence();
    }
    
    cout << "in Execute 2 " << endl;
    for(int i=0; i < 9; ++i){
        double q = m_angle_L.data[i];
       // std::cout << " qref_L : "<< rad2deg(qref_L[i]) << " q : " << rad2deg(q) << endl;        
        double gain = (qref_L[i] - q);
        if(fabs(gain)>10.0) gain = 0.0;
        m_torque_L.data[i] = gain * VEL_GAIN / dt;
        qold_L[i] = q;
    }
        cout << "in Execute 3 " << endl;
    for(int i=0; i < 9; ++i){
        double q = m_angle_R.data[i];
        // std::cout << " qref_R : "<<  qref_R[i] << " rad : " << q << endl;
         double gain = (qref_R[i] -q);
        // std::cout << "gain : " << gain << endl;
        if(fabs(gain)>10.0) gain = 0.0;
        m_torque_R.data[i] = gain * VEL_GAIN / dt;
        qold_R[i] = q;
    }
        cout << "in Execute 4 " << endl;
    m_torque_LOut.write();
    m_torque_ROut.write();
    cout << "in Execute 5 " << endl;
    currentTime += dt;
    return RTC::RTC_OK;
}

 RTC::ReturnCode_t Jaco2ControllerRTC::onError(RTC::UniqueId ec_id) 
{
   return RTC::RTC_OK;
}


extern "C"
{
 
  void Jaco2ControllerRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(Jaco2ControllerRTC_spec);
    manager->registerFactory(profile,
                             RTC::Create<Jaco2ControllerRTC>,
                             RTC::Delete<Jaco2ControllerRTC>);
  }
  
};


////////////////////////////////////////////////////////////////
// Controller Script


Matrix3d Jaco2ControllerRTC::RotateX(double radian){
    Matrix3d rotX = MatrixXd::Zero(3,3);
    rotX(0,0) = 1;
    rotX(1,1) = cos(radian);
    rotX(1,2) = -sin(radian);
    rotX(2,1) = sin(radian);
    rotX(2,2) = cos(radian);
    return rotX; 
}

Matrix3d Jaco2ControllerRTC::RotateY(double radian){
    Matrix3d rotY = MatrixXd::Zero(3,3);
    rotY(0,0) = cos(radian);
    rotY(0,2) = sin(radian);
    rotY(1,1) = 1;
    rotY(2,0) = -sin(radian);
    rotY(2,2) = cos(radian);
    return rotY; 
}

Matrix3d Jaco2ControllerRTC::RotateZ(double radian){
    Matrix3d rotZ = MatrixXd::Zero(3,3);
    rotZ(0,0) = cos(radian);
    rotZ(0,1) = -sin(radian);
    rotZ(1,0) = sin(radian);
    rotZ(1,1) = cos(radian);
    rotZ(2,2) = 1;
    return rotZ; 
}

Vector3 Jaco2ControllerRTC::toRadianVector3(double x, double y, double z)
{
    return Vector3(radian(x), radian(y), radian(z));
}



void Jaco2ControllerRTC::updateTargetJointAnglesBYInverseKinematicks()
{ 

        cout << "update_target " << endl;    
        // 手首座標
        Vector3d p =  ikWrist->p();
        Matrix3d R =  ikWrist->R();
        cout << " test 1 " << endl;  
        if(m_button_L.data[0]){
            if(m_axis_L.data[1] > 0){
                R *= RotateZ(deg2rad(-0.2));
            }
            if(m_axis_L.data[1] < 0){
                R *= RotateZ(deg2rad(0.2));
            }
        }
        else{
            p(0) -= (float)m_axis_L.data[0] /20000000;
            p(1) += (float)m_axis_L.data[1] /20000000;
        }      
        if(m_button_L.data[4]){
            p(2) += 0.001;
        }
        if(m_button_L.data[5]){
            p(2) -= 0.001;
        }
        cout << " test 2 " << endl; 
        // // 手首の操作
        if(m_button_L.data[1]){
            R *= RotateX(deg2rad(0.2));
        }
        if(m_button_L.data[2]){
            R *= RotateX(deg2rad(-0.2));
        }
        if(m_button_L.data[0]){
            R *= RotateY(deg2rad(0.2));
        }
        if(m_button_L.data[3]){
            R *= RotateY(deg2rad(-0.2));
        }
        cout << " test 3 " << endl; 
        baseToWrist->calcInverseKinematics(p, R);
         for(int i = 0; i <6; i++ )
         {
            Link* joint = baseToWrist->joint(i);
            qref_L[i] = joint->q();
            //cout << "InverseKinematics : " << rad2deg(qref_L[i]) << endl;
         }
        double dq_fingerL = 0.0;
        if(m_button_L.data[4]){
            dq_fingerL -= 0.004;
        }
                cout << " test 4 " << endl; 
        VectorXd RPY = rpyFromRot(ikWrist->attitude());
        // cout << "p(x) : " << p(0) << " p(y) : " << p(1) << " p(z) : "<< p(2) << endl;
        // cout << "Roll : " << RPY(0) << " Pitch : " << RPY(1) << " YAW : "<< RPY(2) << endl;
        if(m_button_L.data[5]){
            dq_fingerL += 0.004;
        }
        for(int i = 6; i < 9; ++i){
            qref_L[i] += dq_fingerL;
        }

        // rightArm
        // 手首座標
        Vector3d p2 =  ikWrist2->p();
        Matrix3d R2 =  ikWrist2->R();
cout << " test 5 " << endl; 
        if(m_button_R.data[1]){
            if(m_axis_R.data[1] > 0){
                R2 *= RotateX(deg2rad(-0.2));
            }
            if(m_axis_R.data[1] < 0){
                R2 *= RotateX(deg2rad(0.2));
            }
        }
        else{
            p2(0) += (float)m_axis_R.data[0] /20000000;
            p2(1) -= (float)m_axis_R.data[1] /20000000;
        }      
        if(m_button_R.data[4]){
            p2(2) += 0.001;
        }
        if(m_button_R.data[5]){
            p2(2) -= 0.001;
        }

        // // 手首の操作
        if(m_button_R.data[1]){
            R2 *= RotateZ(deg2rad(0.2));
        }
        if(m_button_R.data[2]){
            R2 *= RotateZ(deg2rad(-0.2));
        }
        if(m_button_R.data[3]){
            R2 *= RotateY(deg2rad(0.2));
        }
        if(m_button_R.data[0]){
            R2 *= RotateY(deg2rad(-0.2));
        }
cout << " test 6 " << endl; 
        baseToWrist2->calcInverseKinematics(p2, R2);
        
         for(int i = 0; i <6; i++ )
         {
             Link* joint = baseToWrist2->joint(i);
             qref_R[i] = joint->q();

         }
        VectorXd RPY2 = rpyFromRot(ikWrist2->attitude());
        // cout << "p(x) : " << p2(0) << " p(y) : " << p2(1) << " p(z) : "<< p2(2) << endl;
        // cout << "Roll : " << RPY2(0) << " Pitch : " << RPY2(1) << " YAW : "<< RPY2(2) << endl;
        double dq_fingerR = 0.0;
        if(m_button_R.data[4]){
            dq_fingerR -= 0.004;
        }
        if(m_button_R.data[5]){
            dq_fingerR += 0.004;
        }
        for(int i = 6; i < 9; ++i){
            qref_R[i] += dq_fingerR;
        }
                cout << "end_kinematics " << endl;  
}

double Jaco2ControllerRTC::deg2rad(double degree)
{
    return degree * M_PI / 180.0f;
}

double Jaco2ControllerRTC::rad2deg(double radian)
{
    return radian * 180.0f/ M_PI;
}

void Jaco2ControllerRTC::sequence(){
    //cout << "in sequence" << endl;
        static int seqNum = 0;
        static bool flag = false;
        static bool isActive =false;
        static bool buttonflag = false;
        static int count = 0;
        static bool rightArm = false;
        static bool leftArm = false;
        if(m_button_L.data[8] && !buttonflag ){
            seqNum = 1;
            flag = true;
            buttonflag = true;
            leftArm = true;
        }
        if(m_button_R.data[12] && !buttonflag ){
            seqNum = 4;
            flag = true;
            buttonflag = true;
            leftArm = true;
            rightArm = true;
        }
        // if(joystick->getButtonState(Joystick::R_BUTTON) && !buttonflag){
        //     seqNum = 2;
        //     flag = true;
        //     buttonflag = true;
        //     rightArm = true;
        // }
        // if(joystick->getButtonState(Joystick::Y_BUTTON) && !buttonflag){
        //     seqNum = 3;
        //     flag = true;
        //     buttonflag = true;
        //     leftArm = true;
        //     rightArm = true;
        // }
        // if(joystick->getPosition(targetMode, Joystick::L_TRIGGER_AXIS) && !buttonflag){
        //     seqNum = 4;
        //     flag = true;
        //     buttonflag = true;
        //     leftArm = true;
        //     rightArm = true;
        // }
       

        if(flag){
            cout << "Sequence : " << seqNum << endl;
            flag = perform(seqNum);
            isActive =true;
        } else{}
        if(isActive && !flag){
        VectorXd p(6);
        VectorXd p3(6);

        
        if(leftArm){
            p = wristInterpolator.interpolate(currentTime);
            if(baseToWrist->calcInverseKinematics(
                Vector3(p.head<3>()), ikWrist->calcRfromAttitude(rotFromRpy(Vector3(p.tail<3>()))))){
                for(int i=0; i < baseToWrist->numJoints(); ++i){
                    Link* joint = baseToWrist->joint(i);
                    qref_L[i] = joint->q();   
                }
            }
            if(currentTime > wristInterpolator.domainUpper()){
                cout << "domainUpper" << endl;
                isActive = false;
                buttonflag = false;
                leftArm = false;
            }
        }
        if(rightArm){
            p3 = wristInterpolator2.interpolate(currentTime);
            if(baseToWrist2->calcInverseKinematics(        
                Vector3(p3.head<3>()), ikWrist2->calcRfromAttitude(rotFromRpy(Vector3(p3.tail<3>()))))){
                for(int i=0; i < 6; i++){
                    Link* joint = baseToWrist2->joint(i);
                    std::cout << "joint Id : " << joint->jointId() << endl; 
                    qref_R[i] = joint->q();
                }
            }
            if(currentTime > wristInterpolator2.domainUpper()){
                cout << "domainUpper" << endl;
                isActive = false;
                buttonflag = false;
                rightArm = false;
            }
        }
        
        }
    
    }


bool Jaco2ControllerRTC::perform(int seqNum){

    cout << "in perform" << endl;
        if (seqNum == 1){
            ikBody = body->clone();
            ikWrist = ikBody->link("ARM1_HAND");
            Link* base = ikBody->rootLink();
            baseToWrist = getCustomJointPath(ikBody, base, ikWrist);
            base->p().setZero();
            base->R().setIdentity();
            baseToWrist->calcForwardKinematics();
            VectorXd position(3);
            position = ikWrist->p();
            //cout << "p(x) : " << position(0) << " p(y) : " << position(1) << " p(z) : "<< position(2) << endl;
    
            VectorXd p0(6);
            p0.head<3>() = ikWrist->p();
            p0.tail<3>() = rpyFromRot(ikWrist->attitude());
            VectorXd p1(6); 
            p1.head<3>() = Vector3(0.55, 0.2, 0.7);
            //p1.tail<3>() = toRadianVector3(0.0, 0.0, 0.0);
            p1.tail<3>() = toRadianVector3(89.9704, -5.2188, 88.9282);  
            wristInterpolator.clear();
            wristInterpolator.appendSample(currentTime + 2.0, p0);
            wristInterpolator.appendSample(currentTime + 7.0, p1);
            wristInterpolator.update();
        }      
        if (seqNum == 4){
            cout << "seq 4" << endl;
            // ikBody = body->clone();
            // ikWrist = ikBody->link("ARM1_HAND");
            // ikWrist2 = ikBody->link("ARM2_HAND");
            // Link* base = ikBody->rootLink();
            // baseToWrist = getCustomJointPath(ikBody,base, ikWrist);
            // baseToWrist2 = getCustomJointPath(ikBody, base, ikWrist2);
            // base->p().setZero();
            // base->R().setIdentity();

            // baseToWrist->calcForwardKinematics();
            // baseToWrist2->calcForwardKinematics();

            VectorXd position(3);
            VectorXd position2(3);
            position = ikWrist->p();
            position2 = ikWrist2->p();
            
            // Left Arm 
            VectorXd p0(6);
            VectorXd p1(6);

            p0.head<3>() = ikWrist->p();
            p0.tail<3>() = rpyFromRot(ikWrist->attitude()); 
            p1.head<3>() = Vector3(0.5214, 0.2164, 0.4957);
            p1.tail<3>() = toRadianVector3(165.0, -25.29, -130.74);  
            //p1.tail<3>() = Vector3(2.880,-0.4415,-2.2826);
            wristInterpolator.clear();
            wristInterpolator.appendSample(currentTime + 3.0, p0);
            wristInterpolator.appendSample(currentTime + 10.0, p1);
            wristInterpolator.update();

            // Right Arm
            p0.head<3>() = ikWrist2->p();
            p0.tail<3>() = rpyFromRot(ikWrist2->attitude());
            p1.tail<3>() = toRadianVector3(164.0, -14.88, 94.70); 
            //p1.head<3>() = Vector3(0.5837, -0.2210, 0.5254);
            
            //p1.tail<3>() = Vector3(2.2869, -0.2599, 1.6529);  
            wristInterpolator2.clear();
            wristInterpolator2.appendSample(currentTime + 3.0, p0);
            wristInterpolator2.appendSample(currentTime + 10.0, p1);
            wristInterpolator2.update();
        }      


        return false;
    }


int Jaco2ControllerRTC::updateControlflg()
{
    static int flag = controlModeflg;
    static int count = 0;
    static bool cflag = false;
     
    if(m_button_L.data[10]){
        cflag = true;
    }
    if(cflag){
        // increment the controlModeflg when the count become more than 200 
        if(++count > 100 ){
            cflag = false;
            count = 0;
            ++flag;
            if (flag == 2){flag = 0;}
            cout << " Change The State" << endl;
        }
    }
    return flag; 
}