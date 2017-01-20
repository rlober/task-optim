cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(YARP REQUIRED)



include_directories(${GAZEBO_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    )

link_directories(${GAZEBO_LIBRARY_DIRS}
                ${YARP_LIBRARY_DIRS}
                )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(task_optimization_change_color SHARED TaskOptimizationChangeColor.cc)
target_link_libraries(task_optimization_change_color ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS task_optimization_change_color DESTINATION lib)
