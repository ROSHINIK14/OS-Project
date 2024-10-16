#include "devices/timer.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>
#include "devices/pit.h"
#include "threads/interrupt.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include "list.h"
#include <stdlib.h>

/* See [8254] for hardware details of the 8254 timer chip. */

#if TIMER_FREQ < 19
#error 8254 timer requires TIMER_FREQ >= 19
#endif
#if TIMER_FREQ > 1000
#error TIMER_FREQ <= 1000 recommended
#endif

/* Number of timer ticks since OS booted. */
static int64_t ticks;
/* Number of loops per timer tick.
   Initialized by timer_calibrate(). */
static unsigned loops_per_tick;
struct sleep_thread 
{ // to hold the sleep of a thread, created a structure//
    struct thread *thr;          
    int64_t wake_tick;        
    struct semaphore semap;     
    struct list_elem lis;     
};
bool compare_wake_ticks(const struct list_elem *x1, const struct list_elem *y1, void *aux);

static intr_handler_func timer_interrupt;
static bool too_many_loops (unsigned loops);
static void busy_wait (int64_t loops);
static void real_time_sleep (int64_t num, int32_t denom);
static void real_time_delay (int64_t num, int32_t denom);
static struct list thread_sleep_list;

bool compare_wake_ticks(const struct list_elem *x1, const struct list_elem *y1, void *aux UNUSED) 
{
    const struct sleep_thread *st_x1 = list_entry(x1, struct sleep_thread, lis);
    const struct sleep_thread *st_y1 = list_entry(y1, struct sleep_thread, lis);
    if (st_x1->wake_tick == st_y1->wake_tick) 
    {
        return st_x1->thr->priority > st_y1->thr->priority;
    }
    return st_x1->wake_tick < st_y1->wake_tick;
}

/* Sets up the timer to interrupt TIMER_FREQ times per second,
   and registers the corresponding interrupt. */
void timer_init (void) 
{
    pit_configure_channel (0, 2, TIMER_FREQ);
    intr_register_ext (0x20, timer_interrupt, "8254 Timer");
    list_init(&thread_sleep_list);  
}

/* Calibrates loops_per_tick, used to implement brief delays. */
void timer_calibrate (void) {
    unsigned high_bit, test_bit;

    ASSERT (intr_get_level () == INTR_ON);
    printf ("Calibrating timer...  ");

    /* Approximate loops_per_tick as the largest power-of-two
       still less than one timer tick. */
    loops_per_tick = 1u << 10;
    while (!too_many_loops (loops_per_tick << 1)) {
        loops_per_tick <<= 1;
        ASSERT (loops_per_tick != 0);
    }

    /* Refine the next 8 bits of loops_per_tick. */
    high_bit = loops_per_tick;
    for (test_bit = high_bit >> 1; test_bit != high_bit >> 10; test_bit >>= 1)
        if (!too_many_loops (loops_per_tick | test_bit))
            loops_per_tick |= test_bit;

    printf("%" PRIu64 " loops/s.\n", (uint64_t) loops_per_tick * TIMER_FREQ);

}

/* Returns the number of timer ticks since the OS booted. */
int64_t timer_ticks (void) 
{
  enum intr_level old_level = intr_disable ();
  int64_t t = ticks;
  intr_set_level (old_level);
  return t;
}

/* Returns the number of timer ticks elapsed since THEN, which
   should be a value once returned by timer_ticks(). */
int64_t timer_elapsed (int64_t then) {
    return timer_ticks () - then;
}
/* Sleeps for approximately TICKS timer ticks.  Interrupts must
   be turned on. */
void timer_sleep (int64_t ticks) {
    if(ticks > 0)
    {
      struct sleep_thread sl_th;
      int64_t wake_tick = timer_ticks() + ticks;
      sl_th.thr = thread_current();
      sl_th.wake_tick = wake_tick;
      sema_init(&sl_th.semap, 0); 
    
    enum intr_level old_level = intr_disable();
    list_insert_ordered(&thread_sleep_list, &sl_th.lis, compare_wake_ticks, NULL);
    sema_down(&sl_th.semap);  // Block the current thread
    intr_set_level(old_level);  // Re-enable interrupts
    }
    if(ticks <= 0)
    {
      return;
    }
}

/* Timer interrupt handler. */
static void timer_interrupt(struct intr_frame *args UNUSED) {
    ticks++;
    thread_tick();

    struct list_elem *elem1 = list_begin(&thread_sleep_list);
    while (elem1 != list_end(&thread_sleep_list)) {
        struct sleep_thread *sl_th = list_entry(elem1, struct sleep_thread, lis);

        if (sl_th->wake_tick > ticks) {
            break;
        }

        elem1 = list_remove(elem1);  // Remove the thread from the sleep list
        sema_up(&sl_th->semap);  // Unblock the sleeping thread

        // Preemption: If the woken thread has higher priority, yield the CPU after the interrupt handler
        if (sl_th->thr->priority > thread_current()->priority) {
            intr_yield_on_return();  // Mark for yielding once the interrupt handler finishes
        }
    }
}

/* Busy-waits for approximately MS milliseconds. Interrupts need
   not be turned on. */
void timer_mdelay (int64_t ms) {
    real_time_delay (ms, 1000);
}

/* Busy-waits for approximately US microseconds. Interrupts need
   not be turned on. */
void timer_udelay (int64_t us) {
    real_time_delay (us, 1000 * 1000);
}

void timer_usleep(int64_t us) {
    real_time_sleep(us, 1000 * 1000);
}

void timer_nsleep(int64_t ns) {
    real_time_sleep(ns, 1000 * 1000 * 1000);
}

void timer_msleep(int64_t ms) {
    real_time_sleep(ms, 1000);
}


/* Busy-waits for approximately NS nanoseconds. Interrupts need
   not be turned on. */
void timer_ndelay (int64_t ns) {
    real_time_delay (ns, 1000 * 1000 * 1000);
}

/* Prints timer statistics. */
void timer_print_stats (void) {
    printf ("Timer: %"PRId64" ticks\n", timer_ticks ());
}

/* Returns true if LOOPS iterations waits for more than one timer
   tick, otherwise false. */
static bool too_many_loops (unsigned loops) {
    /* Wait for a timer tick. */
    int64_t start = ticks;
    while (ticks == start)
        barrier ();

    /* Run LOOPS loops. */
    start = ticks;
    busy_wait (loops);

    /* If the tick count changed, we iterated too long. */
    barrier ();
    return start != ticks;
}

/* Iterates through a simple loop LOOPS times, for implementing
   brief delays.

   Marked NO_INLINE because code alignment can significantly
   affect timings, so that if this function was inlined
   differently in different places the results would be difficult
   to predict. */
static void NO_INLINE busy_wait (int64_t loops) {
    while (loops-- > 0)
        barrier ();
}

/* Sleep for approximately NUM/DENOM seconds. */
static void real_time_sleep (int64_t num, int32_t denom) {
    /* Convert NUM/DENOM seconds into timer ticks, rounding down.
          
        (NUM / DENOM) s          
     ---------------------- = NUM * TIMER_FREQ / DENOM ticks. 
     1 s / TIMER_FREQ ticks
  */
    int64_t ticks = num * TIMER_FREQ / denom;

    if (ticks > 0) {
        /* We're waiting for at least one full timer tick. Use
           timer_sleep() because it will yield the CPU to other
           processes. */                
        timer_sleep (ticks); 
    } else {
        /* Otherwise, use a busy-wait loop for more accurate
           sub-tick timing. */
        real_time_delay (num, denom); 
    }
}

/* Busy-wait for approximately NUM/DENOM seconds. */
static void real_time_delay (int64_t num, int32_t denom) {
    /* Scale the numerator and denominator down by 1000 to avoid
       the possibility of overflow. */
    busy_wait (loops_per_tick * num / 1000 * TIMER_FREQ / (denom / 1000)); 
}
