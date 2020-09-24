target_sources( RTRRec
	PUBLIC
		main.c
)

add_subdirectory( peripherals )

target_link_libraries( RTRRec PUBLIC FreeRTOS )
