cmake_minimum_required(VERSION 2.8)

project(check)

# For quick list run
# find src -iname "*.cpp"
set(SOURCE_FILES src/main.cpp src/TrapProfileTest.cpp src/UtilTest.cpp
                 ../src/lib/InterpLookupTable.cpp
                 ../src/lib/TrapProfile.cpp
                 #../src/Robot.cpp
                 )
include_directories(wpilib-harness ../src)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)

add_custom_target(run
  COMMAND sh -c "./check"
  DEPENDS ${PROJECT_NAME})
set_target_properties(run PROPERTIES EXCLUDE_FROM_ALL TRUE)
