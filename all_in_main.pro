TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += C++11

SOURCES += main.cpp \
    slam.cpp \
    slampart.cpp
SOURCES += kinect.cpp
SOURCES += pioneer.cpp
SOURCES += global.cpp

HEADERS += global.h \
    slam.h
HEADERS += kinect.h
HEADERS += pioneer.h

unix:!macx: LIBS += -L /usr/local/Aria/lib/ -lAria

LIBS += -lpthread

LIBS += -ldl

unix:!macx: LIBS += -L$$PWD/../../freenect2/lib/ -lfreenect2

INCLUDEPATH += $$PWD/../freenect2/lib
DEPENDPATH += $$PWD/../freenect2/lib

INCLUDEPATH += $$PWD/../../freenect2/include/libfreenect2
INCLUDEPATH += $$PWD/../../freenect2/include

INCLUDEPATH +=  /usr/local/Aria/include
DEPENDPATH +=  /usr/local/Aria/include
