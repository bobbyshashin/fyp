# Install script for directory: /home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/ubuntu/fyp/catkin_ws/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  INCLUDE("/home/ubuntu/fyp/catkin_ws/build/ugv/n3/n3_sdk/catkin_generated/safe_execute_install.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk/msg" TYPE FILE FILES
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/A3GPS.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/A3RTK.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/Acceleration.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/AttitudeQuaternion.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/Compass.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/FlightControlInfo.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/Gimbal.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/GlobalPosition.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/LocalPosition.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/PowerStatus.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/RCChannels.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/Velocity.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/Waypoint.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/WaypointList.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/TransparentTransmissionData.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/TimeStamp.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionPushInfo.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionWaypointAction.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionWaypoint.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionWaypointTask.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionHotpointTask.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionFollowmeTask.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionFollowmeTarget.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionStatusWaypoint.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionStatusHotpoint.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionStatusFollowme.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionStatusOther.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionEventWpUpload.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionEventWpAction.msg"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/msg/MissionEventWpReach.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk/srv" TYPE FILE FILES
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/Activation.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/AttitudeControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/CameraActionControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/DroneTaskControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/GimbalAngleControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/GimbalSpeedControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/GlobalPositionControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/LocalPositionControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/SDKPermissionControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/SendDataToRemoteDevice.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/VelocityControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/VersionCheck.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/DroneArmControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/SyncFlagControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MessageFrequencyControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/VirtualRCEnableControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/VirtualRCDataControl.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionStart.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionPause.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionCancel.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionWpUpload.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionWpSetSpeed.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionWpGetSpeed.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionWpDownload.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionHpUpload.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionHpDownload.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionHpSetSpeed.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionHpSetRadius.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionHpResetYaw.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionFmUpload.srv"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/srv/MissionFmSetTarget.srv"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk/action" TYPE FILE FILES
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/action/GlobalPositionNavigation.action"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/action/LocalPositionNavigation.action"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/action/WaypointNavigation.action"
    "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/action/DroneTask.action"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk/msg" TYPE FILE FILES
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/GlobalPositionNavigationAction.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/GlobalPositionNavigationActionGoal.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/GlobalPositionNavigationActionResult.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/GlobalPositionNavigationActionFeedback.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/GlobalPositionNavigationGoal.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/GlobalPositionNavigationResult.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/GlobalPositionNavigationFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk/msg" TYPE FILE FILES
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/LocalPositionNavigationAction.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/LocalPositionNavigationActionGoal.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/LocalPositionNavigationActionResult.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/LocalPositionNavigationActionFeedback.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/LocalPositionNavigationGoal.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/LocalPositionNavigationResult.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/LocalPositionNavigationFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk/msg" TYPE FILE FILES
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/WaypointNavigationAction.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/WaypointNavigationActionGoal.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/WaypointNavigationActionResult.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/WaypointNavigationActionFeedback.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/WaypointNavigationGoal.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/WaypointNavigationResult.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/WaypointNavigationFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk/msg" TYPE FILE FILES
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/DroneTaskAction.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/DroneTaskActionGoal.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/DroneTaskActionResult.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/DroneTaskActionFeedback.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/DroneTaskGoal.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/DroneTaskResult.msg"
    "/home/ubuntu/fyp/catkin_ws/devel/share/n3_sdk/msg/DroneTaskFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk/cmake" TYPE FILE FILES "/home/ubuntu/fyp/catkin_ws/build/ugv/n3/n3_sdk/catkin_generated/installspace/n3_sdk-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ubuntu/fyp/catkin_ws/devel/include/n3_sdk")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ubuntu/fyp/catkin_ws/devel/share/common-lisp/ros/n3_sdk")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/ubuntu/fyp/catkin_ws/devel/lib/python2.7/dist-packages/n3_sdk")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/ubuntu/fyp/catkin_ws/devel/lib/python2.7/dist-packages/n3_sdk")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ubuntu/fyp/catkin_ws/build/ugv/n3/n3_sdk/catkin_generated/installspace/n3_sdk.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk/cmake" TYPE FILE FILES "/home/ubuntu/fyp/catkin_ws/build/ugv/n3/n3_sdk/catkin_generated/installspace/n3_sdk-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk/cmake" TYPE FILE FILES
    "/home/ubuntu/fyp/catkin_ws/build/ugv/n3/n3_sdk/catkin_generated/installspace/n3_sdkConfig.cmake"
    "/home/ubuntu/fyp/catkin_ws/build/ugv/n3/n3_sdk/catkin_generated/installspace/n3_sdkConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk" TYPE FILE FILES "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/n3_sdk" TYPE DIRECTORY FILES "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/include/n3_sdk/")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/n3_sdk/n3_sdk_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/n3_sdk/n3_sdk_node")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/n3_sdk/n3_sdk_node"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/n3_sdk" TYPE EXECUTABLE FILES "/home/ubuntu/fyp/catkin_ws/devel/lib/n3_sdk/n3_sdk_node")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/n3_sdk/n3_sdk_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/n3_sdk/n3_sdk_node")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/n3_sdk/n3_sdk_node")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/n3_sdk/n3_sdk_node")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/n3_sdk/launch" TYPE DIRECTORY FILES "/home/ubuntu/fyp/catkin_ws/src/ugv/n3/n3_sdk/launch")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

