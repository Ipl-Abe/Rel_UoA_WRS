
include_directories(${CMAKE_SOURCE_DIR}/sample/WRS2018/project)
include_directories(${CMAKE_SOURCE_DIR}/sample/WRS2018/script)

#set(REL_UOA_DIR ${CMAKE_CURRENT_SOURCE_DIR})

configure_file(AizuSpiderDS.cnoid AizuSpiderDS.cnoid COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/AizuSpiderDS.cnoid
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/project)

configure_file(AizuSpiderDA.cnoid AizuSpiderDA.cnoid COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/AizuSpiderDA.cnoid
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/project)

# Terminal (remote control UI for AizuSpider)
configure_file(UoASPiderTerminal.cnoid UoASPiderTerminal.cnoid COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/UoASPiderTerminal.cnoid
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/OpenRTM)


# for AIST
configure_file(T2-AizuSpiderDS-RTM.py T2-AizuSpiderDS-RTM.py COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/T2-AizuSpiderDS-RTM.py
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/script)

configure_file(T3-AizuSpiderDS-RTM.py T3-AizuSpiderDS-RTM.py COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/T3-AizuSpiderDS-RTM.py
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/script)

configure_file(T4-AizuSpiderDS-RTM.py T4-AizuSpiderDS-RTM.py COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/T4-AizuSpiderDS-RTM.py
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/script)

configure_file(T5-AizuSpiderDS-RTM.py T5-AizuSpiderDS-RTM.py COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/T5-AizuSpiderDS-RTM.py
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/script)

configure_file(T6-AizuSpiderDS-RTM.py T6-AizuSpiderDS-RTM.py COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/T6-AizuSpiderDS-RTM.py
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/script)

# for AGX
configure_file(T2-AizuSpiderDA-RTM.py T2-AizuSpiderDA-RTM.py COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/T2-AizuSpiderDA-RTM.py
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/script)

configure_file(T3-AizuSpiderDA-RTM.py T3-AizuSpiderDA-RTM.py COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/T3-AizuSpiderDA-RTM.py
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/script)

configure_file(T4-AizuSpiderDA-RTM.py T4-AizuSpiderDA-RTM.py COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/T4-AizuSpiderDA-RTM.py
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/script)

configure_file(T5-AizuSpiderDA-RTM.py T5-AizuSpiderDA-RTM.py COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/T5-AizuSpiderDA-RTM.py
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/script)

configure_file(T6-AizuSpiderDA-RTM.py T6-AizuSpiderDA-RTM.py COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/T6-AizuSpiderDA-RTM.py
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/script)



#Message("test")
#Message("${project_files}")

