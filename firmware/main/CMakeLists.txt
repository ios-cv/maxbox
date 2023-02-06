# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.c"
				   "vehicle.c"
				   "network.c"
				   "led.c"
				   "rc522.c"
				   "owb.c"
				   "owb_rmt.c")
				   
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()

target_compile_options(${COMPONENT_LIB} PRIVATE  -Wno-format)