cmake_minimum_required(VERSION 2.8.3)
project(tf03_driver)

add_compile_options(-std=c++11)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

if(SERIAL_VERSION VERSION_LESS 1.0.0)
  add_definitions(-DLIBSERIAL_0_X)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

find_package(catkin REQUIRED)
catkin_package( )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SERIAL_INCLUDE_DIRS}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## OLD NODE

add_executable(tf03_node src/tf03_node.cpp src/TF03.cpp)

target_link_libraries(tf03_node
  ${catkin_LIBRARIES}
  ${SERIAL_LDFLAGS}
)

install(TARGETS tf03_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


# TF03 CAN

add_executable(tf03_CAN_node src/tf03_CAN_node.cpp src/TF03_CAN.cpp src/TF03_Base.cpp)

target_link_libraries(tf03_CAN_node
  ${catkin_LIBRARIES}
  ${SERIAL_LDFLAGS}
)

install(TARGETS tf03_CAN_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

