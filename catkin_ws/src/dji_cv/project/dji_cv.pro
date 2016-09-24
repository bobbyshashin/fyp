#-------------------------------------------------
#
# Project created by QtCreator 2016-03-16T11:30:48
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = dji_cv
CONFIG   += console
CONFIG   -= app_bundle

HEADERS += opencvHeaders.h                     \
        arucoHeaders.h                         \
        stdHeaders.h                           \
        aruco_lib/*.h                          \
        aruco_lib/*.cpp                        \
        eigenHeaders.h                         \
        ../src/*.h                             \
        ../../Onboard_SDK/dji_sdk_lib/include/dji_sdk_lib/*.h   \
        ../../Onboard_SDK/dji_sdk/include/dji_sdk/*.h

TEMPLATE = app

SOURCES += ../src/*.cxx

INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
               /usr/local/include/opencv2 \
               /opt/ros/indigo/include    \

LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_legacy.so \
        /usr/local/lib/libopencv_ml.so \
        /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_nonfree.so \
        /usr/local/lib/libopencv_gpu.so \
        /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_objdetect.so \
        /usr/local/lib/libopencv_features2d.so \
        /usr/local/lib/libopencv_contrib.so \
        /usr/local/lib/libopencv_video.so \
        /usr/local/lib/libopencv_superres.so \
        /usr/local/lib/libopencv_stitching.so \
        /usr/local/lib/libopencv_photo.so \
        /usr/local/lib/libopencv_ocl.so \
        /usr/local/lib/libopencv_flann.so.2.4 \
        /usr/local/lib/libopencv_calib3d.so \
        /usr/local/lib/libaruco.so \
        /usr/local/lib/libopencv_calib3d.so \

