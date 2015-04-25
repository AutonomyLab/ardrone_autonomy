cmake_minimum_required(VERSION 2.8.3)
project(ardrone_autonomy)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    sensor_msgs
    std_srvs
    tf
    camera_info_manager
    message_generation
    roslint
)

add_service_files(
    DIRECTORY srv
    FILES
    CamSelect.srv
    FlightAnim.srv
    LedAnim.srv
    RecordEnable.srv
)

add_message_files(
    DIRECTORY msg
    FILES
    matrix33.msg
    navdata_adc_data_frame.msg
    navdata_altitude.msg
    navdata_demo.msg
    navdata_euler_angles.msg
    navdata_games.msg
    navdata_gyros_offsets.msg
    navdata_hdvideo_stream.msg
    navdata_kalman_pressure.msg
    navdata_magneto.msg
    Navdata.msg
    navdata_phys_measures.msg
    navdata_pressure_raw.msg
    navdata_pwm.msg
    navdata_raw_measures.msg
    navdata_rc_references.msg
    navdata_references.msg
    navdata_time.msg
    navdata_trackers_send.msg
    navdata_trims.msg
    navdata_video_stream.msg
    navdata_vision_detect.msg
    navdata_vision.msg
    navdata_vision_of.msg
    navdata_vision_perf.msg
    navdata_vision_raw.msg
    navdata_watchdog.msg
    navdata_wifi.msg
    navdata_wind_speed.msg
    navdata_zimmu_3000.msg
    vector21.msg
    vector31.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  DEPENDS libsdl-dev
  CATKIN_DEPENDS
    roscpp
    image_transport
    message_runtime
    sensor_msgs
    std_srvs
    tf
    camera_info_manager
)

# Build [pathced] ardronelib from an external project
include(ExternalProject)
ExternalProject_Add(ardronelib
    GIT_REPOSITORY git://github.com/AutonomyLab/ardronelib.git
    GIT_TAG 2f987029c55531e4c0119c3600f9c57f935851ed
    PREFIX ${CATKIN_DEVEL_PREFIX}
    CONFIGURE_COMMAND echo "No configure"
    BUILD_COMMAND make
    INSTALL_COMMAND make install INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}/lib/ardrone
    BUILD_IN_SOURCE 1
)

# Find sdk directories for headers and libraries in devel space
set(SDK ${CATKIN_DEVEL_PREFIX}/src/ardronelib/ARDroneLib/)
include_directories(
    ${SDK}
    ${SDK}/FFMPEG/Includes
    ${SDK}/Soft/Common
    ${SDK}/Soft/Lib
    ${SDK}/VP_SDK
    ${SDK}/VP_SDK/VP_Os/linux
)
link_directories(${CATKIN_DEVEL_PREFIX}/lib/ardrone)

# Add CATKIN_INCLUDE_DIRS
include_directories(${catkin_INCLUDE_DIRS})

# Add Boost_INCLUDE_DIRS
include_directories(${Boost_INCLUDE_DIRS})

# Add headers directory
include_directories(include)

# Set SOURCE and HEADER files

set(AA_HEADERS
    include/ardrone_autonomy/ardrone_driver.h
    include/ardrone_autonomy/video.h
    include/ardrone_autonomy/ardrone_sdk.h
    include/ardrone_autonomy/teleop_twist.h
)

set(AA_SOURCES
    src/ardrone_driver.cpp
    src/video.cpp
    src/ardrone_sdk.cpp
    src/teleop_twist.cpp
)

# Build executable
add_executable(ardrone_driver ${AA_SOURCES})
target_link_libraries(
    ardrone_driver
    pc_ardrone
    avcodec
    swscale
    avutil
    vlib
    sdk
    SDL
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_dependencies(ardrone_driver ardronelib ${PROJECT_NAME}_gencpp)

# roslint
set(ROSLINT_CPP_OPTS "--filter=-runtime/references-runtime/threadsafe_fn")
roslint_cpp(${AA_HEADERS} ${AA_SOURCES})

# Install Rules
install(TARGETS ardrone_driver RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".git" EXCLUDE
)

