cmake_minimum_required(VERSION 2.8.4)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif(NOT CMAKE_BUILD_TYPE)

project(ScorpioNodelet)

find_package(catkin REQUIRED COMPONENTS  message_generation message_runtime nodelet roscpp)
find_package(Boost QUIET REQUIRED COMPONENTS system thread filesystem)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS message_generation
    message_runtime nodelet roscpp) 

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++14 -fPIC -Wno-unused-result -Wno-overflow -Dunix")

SET(ROS_BUILD_TYPE MinSizeRel)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH "{CMAKE_SOURCE_DIR}/../cmake")
if(NOT EIGEN_FOUND)
  find_package(Eigen)
endif()
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(PnC)
include_directories(${PNC_INCLUDE_DIRS})

add_library(ScorpioNodelet src/ScorpioNodelet.cpp)
set_target_properties(ScorpioNodelet PROPERTIES OUTPUT_NAME ScorpioNodelet PREFIX "lib")
add_dependencies(ScorpioNodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(ScorpioNodelet ${catkin_LIBRARIES}
                                    ${Boost_LIBRARIES}
                                    rt
                                    ${PNC_LIBRARIES})



#install(TARGETS ScorpioNodelet 
    #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #LIBRARY DESTINATION
    #${CATKIN_PACKAGE_LIB_DESTINATION}
    #RUNTIME DESTINATION
    #${CATKIN_PACKAGE_BIN_DESTINATION}
    #)

#install(DIRECTORY include/
    #DESTINATION
    #${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    #)

#install(DIRECTORY
    #launch/
    #DESTINATION
    #${CATKIN_PACKAGE_SHARE_DESTINATION}/launch 
    #)

#install(DIRECTORY
    #plotjuggler/
    #DESTINATION
    #${CATKIN_PACKAGE_SHARE_DESTINATION}/plotjuggler 
    #)

#install(FILES
    #nodelet_plugins.xml 
    #DESTINATION
    #${CATKIN_PACKAGE_SHARE_DESTINATION}
    #)
