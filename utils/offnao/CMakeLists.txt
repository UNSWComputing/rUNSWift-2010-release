PROJECT(OFFNAO)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET (CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
FIND_PACKAGE(Qt4 REQUIRED)

SET ( DEFAULT_AL_DIR "/home/robocup/2010/aldebaran-sdk-1.6.13-linux-i386" )
IF( "x$ENV{AL_DIR}x" STREQUAL "xx" )
   IF( NOT IS_DIRECTORY "${DEFAULT_AL_DIR}" )
      MESSAGE( FATAL_ERROR "Please set the environment variable AL_DIR to the naoqi directory!" )
   ELSE ( NOT IS_DIRECTORY "${DEFAULT_AL_DIR}" )
      SET ( AL_DIR "${DEFAULT_AL_DIR}" )
   ENDIF ( NOT IS_DIRECTORY "${DEFAULT_AL_DIR}" )
ELSE("x$ENV{AL_DIR}x" STREQUAL "xx" )
   SET ( AL_DIR "$ENV{AL_DIR}" )
ENDIF("x$ENV{AL_DIR}x" STREQUAL "xx" )

SET ( DEFAULT_CTC_DIR "/home/robocup/2010/ctc-robocup-1.6.13" )
IF( NOT IS_DIRECTORY "$ENV{CTC_DIR}" )
   IF( NOT IS_DIRECTORY "${AL_DIR}/ctc" )
      IF( NOT IS_DIRECTORY "${DEFAULT_CTC_DIR}" )
         MESSAGE( FATAL_ERROR "Please place cross toolchain in ${AL_DIR}/ctc, or set enviroment variable $CTC_DIR!" )
      ELSE( NOT IS_DIRECTORY "${DEFAULT_CTC_DIR}" )
         SET ( CTC_DIR "${DEFAULT_CTC_DIR}" )
      ENDIF( NOT IS_DIRECTORY "${DEFAULT_CTC_DIR}" )
   ELSE ( NOT IS_DIRECTORY "${AL_DIR}/ctc" )
      SET( CTC_DIR "${AL_DIR}/ctc" )
   ENDIF( NOT IS_DIRECTORY "${AL_DIR}/ctc" )
ELSE ( NOT IS_DIRECTORY "$ENV{CTC_DIR}" )
   SET( CTC_DIR "$ENV{CTC_DIR}" )
ENDIF (NOT IS_DIRECTORY "$ENV{CTC_DIR}" )

SET ( DEFAULT_RUNSWIFT_CHECKOUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." )
IF( NOT IS_DIRECTORY "$ENV{RUNSWIFT_CHECKOUT_DIR}" )
   IF( NOT IS_DIRECTORY "${DEFAULT_RUNSWIFT_CHECKOUT_DIR}/robot" )
      MESSAGE( FATAL_ERROR "Please set the environment variable RUNSWIFT_CHECKOUT_DIR!" )
   ELSE ( NOT IS_DIRECTORY "${DEFAULT_RUNSWIFT_CHECKOUT_DIR}/robot" )
      SET ( RUNSWIFT_CHECKOUT_DIR "${DEFAULT_RUNSWIFT_CHECKOUT_DIR}" )
   ENDIF ( NOT IS_DIRECTORY "${DEFAULT_RUNSWIFT_CHECKOUT_DIR}/robot" )
ELSE ( NOT IS_DIRECTORY "$ENV{RUNSWIFT_CHECKOUT_DIR}" )
   SET( RUNSWIFT_CHECKOUT_DIR "$ENV{RUNSWIFT_CHECKOUT_DIR}" )
ENDIF (NOT IS_DIRECTORY "$ENV{RUNSWIFT_CHECKOUT_DIR}" )

SET(RUNSWIFT_ROBOT_DIR "${RUNSWIFT_CHECKOUT_DIR}/robot")
SET(RUNSWIFT_BUILD_DIR "${RUNSWIFT_CHECKOUT_DIR}/robot/build")
SET(CTC_INCLUDE_DIR "${CTC_DIR}/staging/geode-linux/usr/include")
SET(CTC_LINK_DIR "${CTC_DIR}/staging/geode-linux/usr/lib")

# Problem A: using AL_DIR/include & FIND_PACKAGE(Qt4) uses moc from the system and Qt4 headers from AL_DIR
# Solution A1: Use moc from AL_DIR
#SET(QT_MOC_EXECUTABLE "${AL_DIR}/bin/moc")
# Problem A1: need to link against AL_DIR
# Solution A2: Give precedence to system qt4 headers
SET(AL_INCLUDE_DIR /usr/include/qt4 ${AL_DIR}/include)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR} ${RUNSWIFT_ROBOT_DIR} ${AL_INCLUDE_DIR} ${CTC_INCLUDE_DIR})
LINK_DIRECTORIES(${RUNSWIFT_BUILD_DIR} ${CTC_LINK_DIR})

SET(QT_USE_QTSVG TRUE)
SET(QT_USE_QTNETWORK TRUE)
SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QTXML TRUE)

INCLUDE(${QT_USE_FILE})

SET(OFFNAO_CXX_SRCS
   main.cpp
   mediaPanel.cpp
   visualiser.cpp
   inverseVision/inverseVision.cpp
   readers/diskReader.cpp
   readers/networkReader.cpp
   readers/dumpReader.cpp
   readers/reader.cpp
   readers/localisationReader.cpp
   tabs/cameraTab.cpp
   tabs/PointCloud.cpp
   tabs/classifier.cpp
   tabs/variableView.cpp
   tabs/plots.cpp
   tabs/histogram.cpp
   tabs/overviewTab.cpp
   tabs/sensorTab.cpp
   tabs/graphTab.cpp
   tabs/visionTab.cpp
   tabs/jointsTab.cpp
   tabs/zmpTab.cpp
   tabs/fieldView.cpp
   tabs/cameraPoseTab.cpp
   tabs/calibrationTab.cpp
   tabs/tab.cpp
)

SET(OFFNAO_MOC_SRCS
   readers/reader.hpp 
   mediaPanel.hpp 
   visualiser.hpp
   tabs/tab.hpp
   tabs/calibrationTab.hpp
   tabs/cameraPoseTab.hpp
   tabs/cameraTab.hpp
   tabs/graphTab.hpp
   tabs/jointsTab.hpp
   tabs/overviewTab.hpp
   tabs/plots.hpp
   tabs/sensorTab.hpp
   tabs/visionTab.hpp
   tabs/zmpTab.hpp
   readers/dumpReader.hpp
)

SET(OFFNAO_UI
   visualiser.ui
)

SET(OFFNAO_RES
   resources/visualiser_resources.qrc
)

# build cxx files for resources
QT4_ADD_RESOURCES(OFFNAO_RES_SRCS ${OFFNAO_RES})
#QT4_WRAP_CPP(OFFNAO_CXX_SRCS ${OFFNAO_RES_SRCS})
 
# build ui_XXX files from the XML-style .ui files
QT4_WRAP_UI(OFFNAO_CXX_SRCS ${OFFNAO_UI})

# this moc's the above variable and appends to the cxx sources
QT4_WRAP_CPP(OFFNAO_CXX_SRCS ${OFFNAO_MOC_SRCS})

ADD_EXECUTABLE(offnao ${OFFNAO_CXX_SRCS} ${OFFNAO_RES_SRCS})

TARGET_LINK_LIBRARIES(offnao ${QT_LIBRARIES} ${QWT_LIBRARIES} qwt-qt4 qglviewer-qt4 boost_system-mt boost_regex-mt boost_thread-mt boost_program_options-mt boost_serialization-mt robot soccer)
