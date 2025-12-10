/**
 * @file rp2350_coprocessor.h
 * @brief Public API for the RP2350 Inter-Processor FIFO (IP FIFO) driver.
 */

#ifndef RP2350_COPROCESSOR_H
#define RP2350_COPROCESSOR_H

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>

typedef void (*coprocessor_entry_t)(void);

#ifdef __cplusplus
extern "C" {
#endif
  
/**
 * @brief Push a 32-bit word onto the IP FIFO.
 *
 * This function attempts to write a 32-bit word to the inter-processor FIFO
 * associated with the given device. It will fail if the FIFO is currently full.
 *
 * @param dev The device structure pointer for the IP FIFO instance.
 * @param data The 32-bit word to send to the other core.
 * @retval 0 on success.
 * @retval -EAGAIN if the FIFO is full.
 */
__syscall int rp2350_coprocessor_push(const struct device *dev, uint32_t data);

/**
 * @brief Pop a 32-bit word from the IP FIFO.
 *
 * This function attempts to read a 32-bit word from the inter-processor FIFO
 * associated with the given device. It will fail if the FIFO is currently empty.
 *
 * @param dev The device structure pointer for the IP FIFO instance.
 * @param data Pointer to a location to store the received 32-bit word.
 * @retval 0 on success.
 * @retval -EAGAIN if the FIFO is empty.
 */
__syscall int rp2350_coprocessor_pop(const struct device *dev, uint32_t *data);

__syscall int rp2350_coprocessor_launch(const struct device *dev, coprocessor_entry_t entry);

/**
 * @brief IP FIFO driver API structure.
 *
 * Defines the function table required by the Zephyr Device Model.
 */
struct rp2350_coprocessor_driver_api {
    int (*push)(const struct device *dev, uint32_t data);
    int (*pop)(const struct device *dev, uint32_t *data);
    // Add other API functions here (e.g., enabling interrupts)
};

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/rp2350_coprocessor.h>

#endif // RP2350_COPROCESSOR_H
