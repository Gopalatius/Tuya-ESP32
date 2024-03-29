#include "MultiTimer.h"
#include <stdio.h>

// Timer handle list head.
static MultiTimer* timerList = NULL;

// Timer tick
static PlatformTicksFunction_t platformTicksFunction = NULL;

/**
 * @brief 
 * 
 * @param ticksFunc 
 * @return int 
 */
int MultiTimerInstall(PlatformTicksFunction_t ticksFunc)
{
    platformTicksFunction = ticksFunc;
    return 0;
}

/**
  * @brief  Initializes the timer struct handle.
  * @param  handle: the timer handle strcut.
  * @param  timeout_cb: timeout callback.
  * @param  repeat: repeat interval time.
  * @retval None
  */
int MultiTimerInit(MultiTimer* timer, uint32_t period, MultiTimerCallback_t cb, void* userData)
{
    timer->callback = cb;
    timer->userData = userData;
    timer->period = period;
    return 0;
}

/**
  * @brief  Start the timer work, add the handle into work list.
  * @param  handle: target handle strcut.
  * @param  timeout: Set the start time.
  * @retval 0: succeed. -1: already exist.
  */
int MultiTimerStart(MultiTimer* timer, uint32_t startTime)
{
    timer->timeout = platformTicksFunction() + startTime;

    // Insert timer.
    MultiTimer** nextTimer = &timerList;
    for (;; nextTimer = &(*nextTimer)->next) {
        if (!*nextTimer) {
            timer->next = NULL;
            *nextTimer = timer;
            break;
        }
        if (timer->timeout < (*nextTimer)->timeout) {
            timer->next = *nextTimer;
            *nextTimer = timer;
            break;
        }
    }

    return 0;
}

/**
  * @brief  Stop the timer work, remove the handle off work list.
  * @param  handle: target handle strcut.
  * @retval None
  */
int MultiTimerStop(MultiTimer* timer)
{
    MultiTimer** nextTimer = &timerList;

    // Find and remove timer.
    for (; *nextTimer; nextTimer = &(*nextTimer)->next) {
        MultiTimer* entry = *nextTimer;
        if (entry == timer) {
            *nextTimer = timer->next;
            break;
        }
    }
    return 0;
}

/**
  * @brief  main loop.
  * @param  None.
  * @retval None
  */
void MultiTimerYield(void)
{
    MultiTimer* target;
    for (target = timerList; target; target = target->next) {
        if (platformTicksFunction() >= target->timeout) {
            if (target->period == 0) {
                MultiTimerStop(target);
            } else {
                target->timeout = platformTicksFunction() + target->period;
            }
            if (target->callback) {
                target->callback(target, target->userData);
            }
        } else {
            break;
        }
    }
}
