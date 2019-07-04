/*
 * Copyright (c) 2016 Zibin Zheng <znbin@qq.com>
 * All rights reserved
 */

#include "multi_timer.h"

//timer handle list head.
//#ifndef NULL
//#define NULL 0
//#endif
#ifndef NULL
#define NULL ((void *)0)
#endif
static struct Timer* head_handle = NULL;

//Timer ticks
static uint64_t _timer_ticks = 0;

/**
  * @brief  Initializes the timer struct handle.
  * @param  handle: the timer handle strcut.
  * @param  timeout_cb: timeout callback.
  * @param  repeat: repeat interval time.
  * @retval None
  */
void timer_init(struct Timer* handle, void(*timeout_cb)(), uint32_t timeout, uint32_t repeat)
{
	// memset(handle, sizeof(struct Timer), 0);
	handle->timeout_cb = timeout_cb;
	handle->timeout = _timer_ticks + timeout;
	handle->repeat = repeat;
}

/**
  * @brief  Start the timer work, add the handle into work list.
  * @param  btn: target handle strcut.
  * @retval 0: succeed. -1: already exist.
  */
int timer_start(struct Timer* handle)
{
	struct Timer* target = head_handle;
	while(target) {
		if(target == handle) return -1;	//already exist.
		target = target->next;
	}
	handle->next = head_handle;
	head_handle = handle;
	return 0;
}

/**
  * @brief  Stop the timer work, remove the handle off work list.
  * @param  handle: target handle strcut.
  * @retval None
  */
void timer_stop(struct Timer* handle)
{
	struct Timer** curr;
	for(curr = &head_handle; *curr; ) {
		struct Timer* entry = *curr;
		if (entry == handle) {
			*curr = entry->next;
//			free(entry);
		} else
			curr = &entry->next;
	}
}

/**
  * @brief  main loop.
  * @param  None.
  * @retval None
  */
void timer_loop()
{
	struct Timer* target;
	for(target=head_handle; target; target=target->next) {
		if(_timer_ticks >= target->timeout) 
		{
			if(target->repeat == 0) timer_stop(target);
			else  target->timeout = _timer_ticks + target->repeat;
			target->timeout_cb();
		}
	}
}

/**
  * @brief  background ticks, timer repeat invoking interval 1ms.
  * @param  None.
  * @retval None.
  */
void timer_ticks()
{
	_timer_ticks++;
}

/*
//Examples

#include "multi_timer.h"
struct Timer timer1;
struct Timer timer2;
void timer1_callback(){printf("timer1 timeout!\r\n");}

void timer2_callback(){printf("timer2 timeout!\r\n");}

int main()
{
    timer_init(&timer1, timer1_callback, 1000, 1000); //1s loop
    timer_start(&timer1);
    
    timer_init(&timer2, timer2_callback, 50, 0); //50ms delay
    timer_start(&timer2);
    
    while(1) {
        
        timer_loop();
    }
}

void HAL_SYSTICK_Callback(void)
{
    timer_ticks(); //1ms ticks
}

*/
