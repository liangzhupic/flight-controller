include_directories(
CMSIS_RTOS
include
portable/GCC/ARM_CM4F
${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)


add_library( freertos
croutine.c       portable  tasks.c
event_groups.c    list.c   queue.c   timers.c

CMSIS_RTOS/cmsis_os.c
portable/GCC/ARM_CM4F/port.c
portable/MemMang/heap_4.c
)
