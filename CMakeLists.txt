cmake_minimum_required( VERSION 3.13 )

# set the default MCU model
set( MICROCHIP_MCU PIC18F25K83 )

if( NOT CMAKE_BUILD_TYPE )
	set( CMAKE_BUILD_TYPE Release )
endif( )

project( AquariumCAN C )
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wpedantic" )
#set( CMAKE_C_FLAGS_DEBUG "-g" )
set( CMAKE_C_FLAGS_RELEASE "-O3" )

#add_definitions( __XC8 __PICC18__ _18F25K83 )

add_library( FreeRTOS )
add_executable( RTRRec )

add_subdirectory( FreeRTOS )
add_subdirectory( RTRRec )
