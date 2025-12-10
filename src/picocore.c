#include "hardware/sync.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/scb.h"
#include "hardware/structs/timer.h"
#include "hardware/regs/psm.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(picocore, CONFIG_RP2350_COPROCESSOR_LOG_LEVEL);

void core1_busy_wait_us(uint32_t delay_us) {
  timer_hw_t *timer = timer0_hw;

  // we only allow 31 bits, otherwise we could have a race in the loop below with
  // values very close to 2^32
  uint32_t start = timer->timerawl;
  while (timer->timerawl - start < delay_us) {
    tight_loop_contents();
  }
}

static inline bool multicore_fifo_rvalid(void) {
    return sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS;
}

static inline bool multicore_fifo_wready(void) {
    return sio_hw->fifo_st & SIO_FIFO_ST_RDY_BITS;
}

static inline void multicore_fifo_drain(void) {
  while (multicore_fifo_rvalid())
    (void) sio_hw->fifo_rd;
}

static inline void multicore_fifo_push_blocking_inline(uint32_t data) {
    // We wait for the fifo to have some space
    while (!multicore_fifo_wready())
        tight_loop_contents();

    sio_hw->fifo_wr = data;

    // Fire off an event to the other core
    __sev();
}

static inline uint32_t multicore_fifo_pop_blocking_inline(void) {
    // If nothing there yet, we wait for an event first,
    // to try and avoid too much busy waiting
    while (!multicore_fifo_rvalid())
        __wfe();

    return sio_hw->fifo_rd;
}

void multicore_fifo_push_blocking(uint32_t data) {
  multicore_fifo_push_blocking_inline(data);
}

uint32_t multicore_fifo_pop_blocking(void) {
  return multicore_fifo_pop_blocking_inline();
}

bool multicore_fifo_pop(uint32_t *data) {
  if (!multicore_fifo_rvalid())
    return false;
  *data = sio_hw->fifo_rd;
  return true;
}

static void __attribute__ ((naked)) core1_trampoline(void) {
  pico_default_asm("pop {r0, r1, pc}");
}

int core1_wrapper(int (*entry)(void), void *stack_base) {
    (void)stack_base;
    return (*entry)();
}

void multicore_launch_core1_raw(void (*entry)(void), uint32_t *sp, uint32_t vector_table) {
  // Values to be sent in order over the FIFO from core 0 to core 1
  //
  // vector_table is value for VTOR register
  // sp is initial stack pointer (SP)
  // entry is the initial program counter (PC) (don't forget to set the thumb bit!)
  const uint32_t cmd_sequence[] =
    {0, 0, 1, (uintptr_t) vector_table, (uintptr_t) sp, (uintptr_t) entry};

  uint seq = 0;
  do {
    uint cmd = cmd_sequence[seq];
    // Always drain the READ FIFO (from core 1) before sending a 0
    if (!cmd) {
      multicore_fifo_drain();
      // Execute a SEV as core 1 may be waiting for FIFO space via WFE
      __sev();
    }
    multicore_fifo_push_blocking(cmd);
    uint32_t response = multicore_fifo_pop_blocking();
    // Move to next state on correct response (echo-d value) otherwise start over
    seq = cmd == response ? seq + 1 : 0;
  } while (seq < count_of(cmd_sequence));

  LOG_INF("launch_core1_raw ok: %p %p %x", (void *)entry, (void *)sp, vector_table);
}

void multicore_launch_core1_with_stack(void (*entry)(void), uint32_t *stack_bottom, size_t stack_size_bytes) {
  assert(!(stack_size_bytes & 3u));
  uint32_t *stack_ptr = stack_bottom + stack_size_bytes / sizeof(uint32_t);

  // Push values onto top of stack for core1_trampoline
  stack_ptr -= 3;
  stack_ptr[0] = (uintptr_t) entry;
  stack_ptr[1] = (uintptr_t) stack_bottom;
  stack_ptr[2] = (uintptr_t) core1_wrapper;

  uint32_t vector_table = scb_hw->vtor;
  multicore_launch_core1_raw(core1_trampoline, stack_ptr, vector_table);
}
