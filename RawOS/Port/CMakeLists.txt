include_directories( ${CMAKE_SOURCE_DIR}/RawOS/Source/include 
${CMAKE_SOURCE_DIR}/RawOS/Port)

message(${CMAKE_SOURCE_DIR}/RawOS/Source/include)
add_library(Port  port.c  port_hook.c  raw_trace_port.c  user_extension.c  context_gcc.S )#cpu.asm

