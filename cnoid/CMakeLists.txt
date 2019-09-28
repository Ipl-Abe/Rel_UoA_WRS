
include_directories(${CMAKE_SOURCE_DIR}/sample/WRS2018/project)
include_directories(${CMAKE_SOURCE_DIR}/sample/WRS2018/script)

#set(REL_UOA_DIR ${CMAKE_CURRENT_SOURCE_DIR})

configure_file(AizuSpiderDA.cnoid AizuSpiderDA.cnoid COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/AizuSpiderDA.cnoid
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/WRS2018/project)


configure_file(UoASPiderTerminal.cnoid UoASPiderTerminal.cnoid COPYONLY)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/UoASPiderTerminal.cnoid
        DESTINATION ${CMAKE_SOURCE_DIR}/sample/OpenRTM)


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

