target_sources( RTRRec
	PUBLIC
		main.c
		stack.h
		../FreeRTOS/portasm.s
)

add_subdirectory( peripherals )

target_link_libraries( RTRRec PUBLIC FreeRTOS )
