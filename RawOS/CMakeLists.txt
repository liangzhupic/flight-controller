include_directories( ${CMAKE_SOURCE_DIR}/RawOS/Source/include
${CMAKE_SOURCE_DIR}/RawOS/Port)

add_subdirectory(Source/extension)
add_subdirectory(Port)

add_library(rtos  
	Source/raw_block.c       
	Source/raw_mutex.c       
	Source/raw_queue_size.c 
	Source/raw_task_0.c 
	Source/raw_byte.c  
	Source/raw_obj.c  
	Source/raw_sched.c  
	Source/raw_task.c 
	Source/raw_event.c    
	Source/raw-os.c     
	Source/raw_sem.c 
	Source/raw_task_queue_size.c 
	Source/raw_idle.c     
	Source/raw_pend.c     
	Source/raw_stat.c      
	Source/raw_task_sem.c 
	Source/raw_idle_event.c 
	Source/raw_queue_buffer.c 
	Source/raw_stm.c      
	Source/raw_tick.c 
	Source/raw_mqueue.c    
	Source/raw_queue.c       
	Source/raw_system.c    
	Source/raw_timer.c )


target_link_libraries(rtos Port )
