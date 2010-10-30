PROJECT( RUNSWIFT )

############################ INCLUDE DIRECTORY
# Define include directories here
INCLUDE_DIRECTORIES( "." ".." ${BOOST_INCLUDE_DIR} ${PTHREAD_INCLUDE_DIR} ${BOOST_INCLUDE_DIR} )

############################ PROJECT SOURCES FILES
# Add source files needed to compile this project

SET(SOCCER_SRCS
   soccer.cpp
   log.cpp # generated

   # Vision
   perception/vision/rle.cpp
   perception/vision/VisionDefs.cpp
   perception/vision/RobotCamera.cpp
   perception/vision/RegionBuilder.cpp
   perception/vision/BallDetection.cpp
   perception/vision/GoalDetection.cpp
   perception/vision/RobotDetection.cpp
   perception/vision/FieldEdgeDetection.cpp
   perception/vision/FieldLineDetection.cpp
   perception/vision/ImageToRR.cpp
   perception/vision/yuv.cpp

   # Localisation
   perception/localisation/LocalisationAdapter.cpp
   perception/localisation/LocalisationUtils.cpp
   perception/localisation/FieldLineLocalisation.cpp
   perception/localisation/PFilter.cpp
   perception/localisation/KFilter.cpp
   perception/localisation/BallFilter.cpp
   perception/localisation/RobotFilter.cpp
   perception/localisation/SonarMonitor.cpp

   # Kinematics
   perception/kinematics/KinematicsAdapter.cpp
   perception/kinematics/Kinematics.cpp
   perception/kinematics/Pose.cpp
   utils/matrix_helpers.cpp

   # Behaviour
   perception/behaviour/BehaviourAdapter.cpp
   perception/behaviour/StartStopSkill.cpp
   perception/behaviour/SafetySkill.cpp
   perception/behaviour/PoseTestSkill.cpp
   perception/behaviour/python/PythonSkill.cpp

   # Misc
   utils/Connection.cpp
   # utils/compress.cpp
   utils/options.cpp
   gamecontroller/GameController.cpp
   gamecontroller/RoboCupGameControlData.cpp
   transmitter/OffNao.cpp
   transmitter/Nao.cpp
   receiver/Nao.cpp
   blackboard/Blackboard.cpp
   thread/ThreadWatcher.cpp

   # Motion
   motion/generator/ActionGenerator.cpp
   motion/generator/ALWalkGenerator.cpp
   motion/generator/ClippedGenerator.cpp
   motion/generator/SlowWalkGenerator.cpp
   motion/generator/FastWalkGenerator.cpp
   motion/generator/DistributedGenerator.cpp
   motion/generator/HeadGenerator.cpp
   motion/generator/KickGenerator.cpp
   motion/generator/NullGenerator.cpp
   motion/generator/RefPickupGenerator.cpp
   motion/generator/WaveWalkGenerator.cpp
   motion/generator/StandGenerator.cpp
   motion/generator/DeadGenerator.cpp

   motion/touch/NullTouch.cpp
   motion/touch/FilteredTouch.cpp
   )

SET(ROBOT_SRCS
   # Perception
   perception/PerceptionThread.cpp #nothing robot specific, but we depend on visionadapter

   # Vision
   perception/vision/NaoCamera.cpp
   perception/vision/Vision.cpp
   perception/vision/VisionAdapter.cpp #nothing robot specific, but we depend on vision

   # Motion
   motion/MotionAdapter.cpp
   motion/effector/AgentEffector.cpp
   motion/touch/AgentTouch.cpp
   )

############################ CHECK LIBRARY / EXECUTABLE OPTION

ADD_LIBRARY(robot STATIC ${ROBOT_SRCS} )
ADD_LIBRARY(soccer-static STATIC ${SOCCER_SRCS})
ADD_LIBRARY(soccer SHARED version.cpp)
SET_TARGET_PROPERTIES(soccer-static PROPERTIES OUTPUT_NAME "soccer")
SET_TARGET_PROPERTIES(soccer-static PROPERTIES PREFIX "lib")
SET_TARGET_PROPERTIES(soccer PROPERTIES CLEAN_DIRECT_OUTPUT 1)
SET_TARGET_PROPERTIES(soccer-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
ADD_EXECUTABLE( runswift main.cpp version.cpp )
TARGET_LINK_LIBRARIES(soccer -Wl,-whole-archive soccer-static -Wl,-no-whole-archive)

ADD_CUSTOM_COMMAND ( OUTPUT version.cpp
   COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../bin/genversion.pl > version.cpp
)

ADD_CUSTOM_COMMAND ( OUTPUT log.cpp
   COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../bin/genlog.py ${CMAKE_CURRENT_SOURCE_DIR}/utils/log.cpp.template ${CMAKE_CURRENT_SOURCE_DIR} > log.cpp
)

SET_SOURCE_FILES_PROPERTIES( version.cpp PROPERTIES GENERATED TRUE )
SET_SOURCE_FILES_PROPERTIES( log.cpp PROPERTIES GENERATED TRUE )

ADD_CUSTOM_COMMAND ( TARGET runswift POST_BUILD
   COMMAND rm version.cpp log.cpp
)

############################ SET LIBRARIES TO LINK WITH
# Add any 3rd party libraries to link each target with here
SET ( RUNSWIFT_BOOST  -lboost_system-mt
                      -lboost_regex-mt
                      -lboost_thread-mt
                      -lboost_program_options-mt
                      -lboost_serialization-mt
                      -lpython2.6 )

TARGET_LINK_LIBRARIES( soccer ${PTHREAD_LIBRARIES} ${RUNSWIFT_BOOST} -lz )
TARGET_LINK_LIBRARIES( robot soccer-static )
TARGET_LINK_LIBRARIES( runswift robot ${PTHREAD_LIBRARIES} ${RUNSWIFT_BOOST} -lz )
