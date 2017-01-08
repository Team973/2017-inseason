cmake_minimum_required(VERSION 2.8)
include(${CMAKE_TOOLCHAIN_FILE})

project(greybots-skeleton)
set(TEAM_NUMBER 973)

# Robot target
# For quick list of files run
# find src -iname "*.cpp"
# in project root
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(SOURCE_FILES src/Robot.cpp src/PoseManager.cpp
    src/lib/CoopMTRobot.cpp
    src/lib/util/Util.cpp src/lib/util/Matrix.cpp
    src/lib/jsoncpp.cpp
    src/lib/TaskMgr.cpp src/lib/ControllerBase.cpp src/lib/CoopTask.cpp
    src/lib/DriveBase.cpp src/lib/GreyCompressor.cpp src/lib/JoystickHelper.cpp
    src/lib/WrapDash.cpp src/lib/SPIGyro.cpp src/lib/StateSpaceController.cpp
    src/lib/logging/LogSpreadsheet.cpp src/lib/logging/AsynchLogCell.cpp
    src/lib/filters/BullshitFilter.cpp src/lib/filters/CascadingFilter.cpp
    src/lib/filters/DelaySwitch.cpp src/lib/filters/FilterBase.cpp
    src/lib/filters/MedianFilter.cpp src/lib/filters/MovingAverageFilter.cpp
    src/lib/filters/PID.cpp src/lib/filters/RampedOutput.cpp
    src/controllers/ArcadeDriveController.cpp src/controllers/CheesyDriveController.cpp
    src/controllers/PIDDriveController.cpp src/controllers/PixyVisionDriveController.cpp
    src/controllers/RampedPIDDriveController.cpp src/controllers/StateSpaceFlywheelController.cpp
    src/controllers/StraightDriveController.cpp src/controllers/VelocityTurnPID.cpp
    src/subsystems/Intake.cpp src/subsystems/Drive.cpp src/subsystems/Hanger.cpp
    src/subsystems/Shooter.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
find_package(WPILib)
include_directories(${WPILIB_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${WPILIB_LIBRARIES})
include_directories( "src" )

# Test target
include(ExternalProject)
ExternalProject_Add(check
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/test
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test
  INSTALL_COMMAND ""
  TEST_AFTER_INSTALL 1
  TEST_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/test/src/check-build/check)
set_target_properties(check PROPERTIES EXCLUDE_FROM_ALL TRUE)

# Deploy target
find_file(ROBOTCOMMAND ant/robotCommand "${CMAKE_SYSROOT}/usr/share/wpilib")
add_custom_target(deploy
  COMMAND sh -c "${CMAKE_CURRENT_SOURCE_DIR}/deploy ${TEAM_NUMBER} $<TARGET_FILE:${PROJECT_NAME}> ${ROBOTCOMMAND}"
  DEPENDS ${PROJECT_NAME})
set_target_properties(deploy PROPERTIES EXCLUDE_FROM_ALL TRUE)

# Doxygen target
find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(Doxyfile.in
    ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(doxygen
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
  set_target_properties(doxygen PROPERTIES EXCLUDE_FROM_ALL TRUE)
endif()
