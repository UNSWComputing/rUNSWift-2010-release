############################ INCLUDE DIRECTORY
# Define include directories here
INCLUDE_DIRECTORIES( "." ".." ${BOOST_INCLUDE_DIR} ${PTHREAD_INCLUDE_DIR} ${BOOST_INCLUDE_DIR} )

ADD_EXECUTABLE( testreceiver testreceiver.cpp )

############################ SET LIBRARIES TO LINK WITH
# Add any 3rd party libraries to link each target with here
SET ( RUNSWIFT_BOOST  -lboost_system-mt
                      -lboost_regex-mt
                      -lboost_thread-mt
                      -lboost_program_options-mt 
                      -lboost_serialization-mt
                      -lpython2.6 )

TARGET_LINK_LIBRARIES( testreceiver soccer-static ${PTHREAD_LIBRARIES} ${RUNSWIFT_BOOST} -lz )

