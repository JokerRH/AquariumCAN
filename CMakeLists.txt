cmake_minimum_required( VERSION 3.13 )

# set the default MCU model
set( MICROCHIP_MCU PIC18F25K83 )

if( NOT CMAKE_BUILD_TYPE )
	set( CMAKE_BUILD_TYPE Release )
endif( )

project( AquariumCAN C )
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wpedantic -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=default -msummary=-psect,-class,+mem,-hex,-file -ginhx032 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits -gdwarf-3 -mstack=compiled:auto:auto:auto" )
#set( CMAKE_C_FLAGS_DEBUG "-g" )
set( CMAKE_C_FLAGS_RELEASE "-O3" )

#add_definitions( __XC8 __PICC18__ _18F25K83 )

add_library( FreeRTOS )
add_executable( RTRRec )

add_subdirectory( FreeRTOS )
add_subdirectory( RTRRec )
