option(BUILD_REL_UOA_WRS_OPENRTM "Building REL_UOA_WRS_OPENRTM" OFF)
if(NOT BUILD_REL_UOA_WRS_OPENRTM)
  return()
elseif(NOT BUILD_OPENRTM_PLUGIN)
  message(FATAL_ERROR "REL_UOA_WRS need to buid OPENRTM_PLUGIN.")
endif()

add_definitions(${OPENRTM_DEFINITIONS})
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})


# To read Interpolator.h 
include_directories(${CMAKE_SOURCE_DIR}/sample/OpenRTM)


#add_cnoid_body_io_rtc(Jaco2ControllerRTC Jaco2ControllerRTC.cpp)
add_cnoid_body_io_rtc(JacoIoRTC JacoIoRTC.cpp)

add_cnoid_body_io_rtc(JOYCON_RTC JOYCON_RTC.cpp)
add_cnoid_body_io_rtc(Jaco2ControllerRTC Jaco2ControllerRTC.cpp)
#target_include_directories(Jaco2ControllerRTC "PROJECT_SOURCE_DIR")

