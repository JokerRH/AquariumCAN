target_sources( RTRRec
	PUBLIC
		main.c
		stack.h
		measure.c
		measure.h
		../FreeRTOS/portasm.s
)

add_subdirectory( peripherals )

target_link_libraries( RTRRec PUBLIC FreeRTOS )
