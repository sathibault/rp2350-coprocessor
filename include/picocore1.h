#ifndef PICOCORE_H
#define PICOCORE_H

#ifdef __cplusplus
extern "C" {
#endif

void core1_busy_wait_us(uint32_t delay_us);
void multicore_fifo_push_blocking(uint32_t data);
uint32_t multicore_fifo_pop_blocking(void);

#ifdef __cplusplus
}
#endif
  
#endif
