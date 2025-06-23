cmake_minimum_required( VERSION 3.10 )
project( sample-reader LANGUAGES CXX )

set( comma_DIR "/usr/local/CMakeFiles" CACHE STRING "location of comma cmake files" )
set( _comma_DIR "${comma_DIR}" )
find_package( comma PATHS ${comma_DIR} )
if( comma_DIR STREQUAL "comma_DIR-NOTFOUND" )
    message( FATAL_ERROR "comma not found in ${_comma_DIR}" )
endif( comma_DIR STREQUAL "comma_DIR-NOTFOUND" )
message( "found comma ${comma_version}" )

set( snark_DIR "/usr/local/CMakeFiles" CACHE STRING "location of snark cmake files" )
set( _snark_DIR "${snark_DIR}" )
find_package( snark PATHS ${snark_DIR} )
if( snark_DIR STREQUAL "snark_DIR-NOTFOUND" )
    message( FATAL_ERROR "snark not found in ${_snark_DIR}" )
endif( snark_DIR STREQUAL "snark_DIR-NOTFOUND" )
message( "found snark ${snark_version}" )

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )

add_executable( sample-reader sample-reader.cpp )
target_link_libraries( sample-reader ${comma_ALL_LIBRARIES} )
#install( TARGETS sample-reader RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

add_executable( sample-reader-tcp sample-reader-tcp.cpp )
target_link_libraries( sample-reader-tcp ${comma_ALL_LIBRARIES} )

add_executable( tcp_client tcp_client.cpp )
set_target_properties( tcp_client PROPERTIES LINK_FLAGS_RELEASE -s )
target_link_libraries( tcp_client ${comma_ALL_LIBRARIES} ${snark_ALL_LIBRARIES} ${OpenCV_LIBS} ${Eigen_LIBS} snark_ouster)

add_executable( tcp_interface tcp_interface.cpp )
set_target_properties( tcp_interface PROPERTIES LINK_FLAGS_RELEASE -s )
target_link_libraries( tcp_interface ${comma_ALL_LIBRARIES} ${snark_ALL_LIBRARIES} ${OpenCV_LIBS} ${Eigen_LIBS} snark_ouster)