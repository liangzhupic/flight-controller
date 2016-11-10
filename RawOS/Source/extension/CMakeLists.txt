include_directories(
include/device
include/mm
include/mm/slab
include/posix
include/rf
include/ 
${CMAKE_SOURCE_DIR}/RawOs/Source/include/
${CMAKE_SOURCE_DIR}/RawOS/Port)

add_library(rtos_extension
driver_interface/raw_device.c
driver_interface/raw_driver_interface.c

lib/fifo.c
lib/fifo_lock.c
lib/raw_mlock.c
lib/raw_work_queue.c
lib/rb_tree.c
lib/simple_printf.c
lib/string.c

mm/heap_1.c
mm/heap_2.c
mm/heap_3.c
mm/heap_4.c
mm/heap_5.c
mm/raw_malloc.c
mm/raw_page.c
mm/raw_tlsf.c

mm/slab/raw_buddy.c
mm/slab/raw_frame.c
mm/slab/raw_slab.c
posix/mqueue.c
posix/pthread.c
posix/pthread_cond.c
posix/pthread_mutex.c
posix/pthread_rwlock.c
posix/semphore.c
posix/time.c

protothread/clock_port.c
protothread/etimer.c
protothread/process.c
protothread/stimer.c
protothread/timer.c
rf/active_memory.c
rf/active_object.c
rf/active_queue_broadcast.c
rf/active_time_event.c
shell/rsh.c

)
