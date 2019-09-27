// -*- C++ -*-
/*!
 * @file  JOYCON_RTC.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */
#include "JOYCON_RTC.h"
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>


#define JOYCON_L "/dev/input/js1"
#define JOYCON_R "/dev/input/js2"

using namespace std;

JOYCON_RTC::JOYCON_RTC(RTC::Manager* manager) 
  : RTC::DataFlowComponentBase(manager),
    m_axis_LOut("axis_L", m_axis_L),
    m_button_LOut("button_L", m_button_L),
    m_axis_ROut("axis_R",m_axis_R),
    m_button_ROut("button_R",m_button_R)
{
}

JOYCON_RTC::~JOYCON_RTC()
{
}

RTC::ReturnCode_t JOYCON_RTC::onInitialize()
{
  addOutPort("axis_L",m_axis_LOut);
  addOutPort("button_L", m_button_LOut);
  addOutPort("axis_R", m_axis_ROut);
  addOutPort("button_R", m_button_ROut);

  bindParameter("device_path", m_device_path, "");

  m_axis_L.data.length(4);
  m_button_L.data.length(16);


  m_axis_R.data.length(4);
  m_button_R.data.length(16);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t JOYCON_RTC::onActivated(RTC::UniqueId ec_id)
{

  // set up left joycon
  if((joy_fdL=open(JOYCON_L, O_RDONLY)) < 0){
    std::cout << "Falid to Open : " << JOYCON_L << std::endl;
    return RTC::RTC_OK;
  }
  else{
    std::cout << "Connected to Joycon" << std::endl;
  }
    ioctl(joy_fdL, JSIOCGAXES, &num_of_axisL);
    ioctl(joy_fdL, JSIOCGBUTTONS, &num_of_buttonsL);
    ioctl(joy_fdL, JSIOCGNAME(80), &name_of_joystickL);

    joy_buttonL.resize(num_of_buttonsL,0);
    joy_axisL.resize(num_of_axisL,0);

    cout<<"Joystick: "<<name_of_joystickL<<endl
    <<"  axis: "<<num_of_axisL<<endl
    <<"  buttons: "<<num_of_buttonsL<<endl;

    fcntl(joy_fdL, F_SETFL, O_NONBLOCK);   

  // set up right joycon
    if((joy_fdR=open(JOYCON_R, O_RDONLY)) < 0){
    std::cout << "Falid to Open : " << JOYCON_R << std::endl;
    return RTC::RTC_OK;
  }
  else{
    std::cout << "Connected to Joycon" << std::endl;
  }
    ioctl(joy_fdR, JSIOCGAXES, &num_of_axisR);
    ioctl(joy_fdR, JSIOCGBUTTONS, &num_of_buttonsR);
    ioctl(joy_fdR, JSIOCGNAME(80), &name_of_joystickR);

    joy_buttonR.resize(num_of_buttonsR,0);
    joy_axisR.resize(num_of_axisR,0);

    cout<<"Joystick: "<<name_of_joystickR<<endl
    <<"  axis: "<<num_of_axisR<<endl
    <<"  buttons: "<<num_of_buttonsR<<endl;

    fcntl(joy_fdR, F_SETFL, O_NONBLOCK);  


  return RTC::RTC_OK;
}

RTC::ReturnCode_t JOYCON_RTC::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t JOYCON_RTC::onExecute(RTC::UniqueId ec_id)
{
  
     js_event jsL, jsR;

   //-- JOYCON L
   read(joy_fdL, &jsL, sizeof(js_event));

   switch (jsL.type & ~JS_EVENT_INIT)
   {
   case JS_EVENT_AXIS:
     if((int)jsL.number>=joy_axisL.size())  {cerr<<"err:"<<(int)jsL.number<<endl;}
     joy_axisL[(int)jsL.number] = jsL.value;
     break;
   case JS_EVENT_BUTTON:
     if((int)jsL.number>=joy_buttonL.size())  {cerr<<"err:"<<(int)jsL.number<<endl;}
     joy_buttonL[(int)jsL.number]= (bool)jsL.value;
     break;
   }

    //-- JOYCON L
   read(joy_fdR, &jsR, sizeof(js_event));

   switch (jsR.type & ~JS_EVENT_INIT)
   {
   case JS_EVENT_AXIS:
     if((int)jsR.number>=joy_axisR.size())  {cerr<<"err:"<<(int)jsR.number<<endl;}
     joy_axisR[(int)jsR.number] = jsR.value;
     break;
   case JS_EVENT_BUTTON:
     if((int)jsR.number>=joy_buttonR.size())  {cerr<<"err:"<<(int)jsR.number<<endl;}
     joy_buttonR[(int)jsR.number]= (bool)jsR.value;
     break;
   }
  
  // joyconL axis data input
  m_axis_L.data[0] = joy_axisL[4];
  m_axis_L.data[1] = joy_axisL[5];
  
  // joyconL button data input
  m_button_L.data[0] = (bool)joy_buttonL[0];
  m_button_L.data[1] = (bool)joy_buttonL[1];
  m_button_L.data[2] = (bool)joy_buttonL[2];
  m_button_L.data[3] = (bool)joy_buttonL[3];
  m_button_L.data[4] = (bool)joy_buttonL[4];
  m_button_L.data[5] = (bool)joy_buttonL[5];
  m_button_L.data[6] = (bool)joy_buttonL[6];
  m_button_L.data[7] = (bool)joy_buttonL[7];
  m_button_L.data[8] = (bool)joy_buttonL[8];
  m_button_L.data[9] = (bool)joy_buttonL[9];
  m_button_L.data[10] =(bool)joy_buttonL[10];
  m_button_L.data[11] =(bool)joy_buttonL[11];
  m_button_L.data[12] =(bool)joy_buttonL[12];
  m_button_L.data[13] =(bool)joy_buttonL[13];
  m_button_L.data[14] =(bool)joy_buttonL[14];
  m_button_L.data[15] =(bool)joy_buttonL[15];
  m_axis_LOut.write();
  m_button_LOut.write();


  // joyconR axis data input
  m_axis_R.data[0] = joy_axisR[4];
  m_axis_R.data[1] = joy_axisR[5];
  
  // joyconR button data input
  m_button_R.data[0] = (bool)joy_buttonR[0];
  m_button_R.data[1] = (bool)joy_buttonR[1];
  m_button_R.data[2] = (bool)joy_buttonR[2];
  m_button_R.data[3] = (bool)joy_buttonR[3];
  m_button_R.data[4] = (bool)joy_buttonR[4];
  m_button_R.data[5] = (bool)joy_buttonR[5];
  m_button_R.data[6] = (bool)joy_buttonR[6];
  m_button_R.data[7] = (bool)joy_buttonR[7];
  m_button_R.data[8] = (bool)joy_buttonR[8];
  m_button_R.data[9] = (bool)joy_buttonR[9];
  m_button_R.data[10] =(bool)joy_buttonR[10];
  m_button_R.data[11] =(bool)joy_buttonR[11];
  m_button_R.data[12] =(bool)joy_buttonR[12];
  m_button_R.data[13] =(bool)joy_buttonR[13];
  m_button_R.data[14] =(bool)joy_buttonR[14];
  m_button_R.data[15] =(bool)joy_buttonR[15];
  m_axis_ROut.write();
  m_button_ROut.write();
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t JOYCON_RTC::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

extern "C"
{
  DLL_EXPORT void JOYCON_RTCInit(RTC::Manager* manager){
    static const char* joycon_rtc_spec[] =
  {
    "implementation_id", "JOYCON_RTC",
    "type_name",         "JOYCON_RTC",
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
    coil::Properties profile(joycon_rtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<JOYCON_RTC>,
                             RTC::Delete<JOYCON_RTC>);
  }
};

