file(GLOB subdirs "*")

set(REL_UOA_DIR ${CMAKE_CURRENT_SOURCE_DIR})
Message(${REL_UOA_DIR})

foreach(subdir ${subdirs})
    if(EXISTS ${subdir}/CMakeLists.txt)
        add_subdirectory(${subdir})
    endif()
endforeach()
