cmake_minimum_required(VERSION 2.8.3)
project(map_controller)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            nav_msgs
            map_msgs
            tf2
        )

catkin_package(
    CATKIN_DEPENDS
        roscpp
        nav_msgs
        map_msgs
        tf2
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_executable(map_organizer src/map_organizer.cpp)
add_dependencies(map_organizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_organizer
    ${catkin_LIBRARIES}
)


## Install executables and/or libraries
install(TARGETS map_organizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

