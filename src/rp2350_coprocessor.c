/**
 * @file rp2350_coprocessor.c
 * @brief RP2350 Inter-Processor FIFO (IP FIFO) driver implementation.
 *
 * This file contains the core logic for initializing and operating the
 * dual-core FIFO mechanism on the RP2350 processor using the Zephyr Device Model,
 * including System Call definitions for privileged (Secure Mode) access.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/interrupt-controller/arm-gic.h>
#include <zephyr/syscall_handler.h> // Mandatory for defining syscall handlers

#include "rp2350_coprocessor.h"

LOG_MODULE_REGISTER(rp2350_coprocessor, CONFIG_RP2350_COPROCESSOR_LOG_LEVEL);

/* Device Tree compatibility string defined in rp2350_coprocessor.dtsi */
#define DT_DRV_COMPAT rp_rp2350_coprocessor

/* --- Hardware Register Definitions (Based on user input) --- */

/* Note: The register base in DTS is 0xD0000050, which is SIO_BASE + 0x50 */
struct rp2350_coprocessor_regs {
    /* Offset 0x50 (0x00 from base) */
    volatile uint32_t write;        
    /* Offset 0x54 (0x04 from base) */
    volatile uint32_t read;         
    /* Offset 0x58 (0x08 from base) - Assuming this is the status/control register */
    volatile uint32_t status_ctrl;  
};

/* Status register masks (Placeholder values - typically FIFO_FULL/FIFO_EMPTY bits) */
#define RP2350_COPROCESSOR_STATUS_FULL_BIT     BIT(0)
#define RP2350_COPROCESSOR_STATUS_EMPTY_BIT    BIT(1)

/* --- Driver Context and Configuration Structures --- */

struct rp2350_coprocessor_data {
    struct k_mutex lock;
    // Add other dynamic data here
};

struct rp2350_coprocessor_config {
    struct rp2350_coprocessor_regs *base;
    void (*irq_config_func)(const struct device *dev);
};

/* --- Privileged Implementation Functions (Run in Kernel/Secure Mode) --- */

/**
 * @brief Privileged implementation of the rp2350_coprocessor_push API function.
 * This is the function that actually accesses the secure hardware registers.
 */
static int rp2350_coprocessor_push_impl(const struct device *dev, uint32_t data)
{
    const struct rp2350_coprocessor_config *cfg = dev->config;

    k_mutex_lock(&((struct rp2350_coprocessor_data *)dev->data)->lock, K_FOREVER);

    // Check if the FIFO is full
    if (cfg->base->status_ctrl & RP2350_COPROCESSOR_STATUS_FULL_BIT) {
        LOG_WRN("FIFO is full, failed to push data 0x%x", data);
        k_mutex_unlock(&((struct rp2350_coprocessor_data *)dev->data)->lock);
        return -EAGAIN;
    }

    // Write the data to the write register (Privileged access)
    cfg->base->write = data;

    k_mutex_unlock(&((struct rp2350_coprocessor_data *)dev->data)->lock);
    LOG_DBG("Pushed data: 0x%x", data);
    return 0;
}

/**
 * @brief Privileged implementation of the rp2350_coprocessor_pop API function.
 * This is the function that actually accesses the secure hardware registers.
 */
static int rp2350_coprocessor_pop_impl(const struct device *dev, uint32_t *data)
{
    const struct rp2350_coprocessor_config *cfg = dev->config;
    int ret = 0;

    k_mutex_lock(&((struct rp2350_coprocessor_data *)dev->data)->lock, K_FOREVER);

    // Check if the FIFO is empty
    if (cfg->base->status_ctrl & RP2350_COPROCESSOR_STATUS_EMPTY_BIT) {
        LOG_DBG("FIFO is empty.");
        ret = -EAGAIN;
        goto unlock_and_exit;
    }

    // Read the data from the read register (Privileged access)
    *data = cfg->base->read;

    // Acknowledge the read (if required by hardware, often done via a specific register write)
    // cfg->base->status_ctrl = RP2350_COPROCESSOR_READ_ACK_MASK;

    LOG_DBG("Popped data: 0x%x", *data);

unlock_and_exit:
    k_mutex_unlock(&((struct rp2350_coprocessor_data *)dev->data)->lock);
    return ret;
}

/* Placeholder for the Interrupt Service Routine (ISR) - MUST run in Secure/Privileged Mode */
static void rp2350_coprocessor_isr(const struct device *dev)
{
    // Implementation: Handle interrupts (e.g., read data, notify waiters)
    LOG_DBG("FIFO Interrupt triggered!");

    // Clear the interrupt (Privileged access required)
}


/**
 * @brief Driver initialization function. (Runs in Privileged Mode)
 */
static int rp2350_coprocessor_init(const struct device *dev)
{
    const struct rp2350_coprocessor_config *cfg = dev->config;
    struct rp2350_coprocessor_data *data = dev->data;

    LOG_INF("Initializing RP2350 IP FIFO at 0x%08x", (uint32_t)cfg->base);

    // Initialize context data
    k_mutex_init(&data->lock);

    // 1. Hardware initialization (Privileged access)

    // 2. Configure and enable interrupt (if enabled in DTS)
    if (cfg->irq_config_func) {
        cfg->irq_config_func(dev);
    }
    
    return 0;
}


/* --- System Call Definition Block --- */

/* 1. Define the system call implementations (z_impl_) */

#ifdef CONFIG_USERSPACE

/* System call handler for rp2350_coprocessor_push */
FUNC_ALIAS(rp2350_coprocessor_push_impl, z_impl_rp2350_coprocessor_push);
/* This ensures the public API is accessible even if CONFIG_USERSPACE is not set */
#else
static int z_impl_rp2350_coprocessor_push(const struct device *dev, uint32_t data)
{
    return rp2350_coprocessor_push_impl(dev, data);
}
#endif /* CONFIG_USERSPACE */

/* System call handler for rp2350_coprocessor_pop */
#ifdef CONFIG_USERSPACE
static int z_impl_rp2350_coprocessor_pop(const struct device *dev, uint32_t *data)
{
    // Important: Validate that the pointer 'data' is valid for writing in userspace
    Z_OOPS(z_user_from_kernel_generic_to_rw(data, sizeof(uint32_t)));
    return rp2350_coprocessor_pop_impl(dev, data);
}
#else
static int z_impl_rp2350_coprocessor_pop(const struct device *dev, uint32_t *data)
{
    return rp2350_coprocessor_pop_impl(dev, data);
}
#endif /* CONFIG_USERSPACE */

/* 2. Link the system call implementation to the public API declaration */
/* This is necessary to create the system call table entry */
K_SYSCALL_DEFINE2(rp2350_coprocessor_push, const struct device *, dev, uint32_t, data)
{
    return z_impl_rp2350_coprocessor_push(dev, data);
}

K_SYSCALL_DEFINE2(rp2350_coprocessor_pop, const struct device *, dev, uint32_t *, data)
{
    return z_impl_rp2350_coprocessor_pop(dev, data);
}


/* 3. Define the driver API structure to link to the privileged implementations */
static const struct rp2350_coprocessor_driver_api rp2350_coprocessor_api = {
    .push = z_impl_rp2350_coprocessor_push,
    .pop = z_impl_rp2350_coprocessor_pop,
};


/* 4. Interrupt configuration using DTS_INST macros */
#define RP2350_COPROCESSOR_IRQ_CONFIG(inst)                                  \
    static void rp2350_coprocessor_irq_config_##inst(const struct device *dev) \
    {                                                                 \
        IRQ_CONNECT(DT_INST_IRQN(inst),                               \
                    DT_INST_IRQ(inst, priority),                      \
                    rp2350_coprocessor_isr, DEVICE_DT_INST_GET(inst),        \
                    0);                                               \
        irq_enable(DT_INST_IRQN(inst));                               \
    }

/* 5. Define the device instance using DTS_INST macros */
#define RP2350_COPROCESSOR_DEFINE(inst)                                      \
    RP2350_COPROCESSOR_IRQ_CONFIG(inst)                                      \
    static struct rp2350_coprocessor_data rp2350_coprocessor_data_##inst;           \
    static const struct rp2350_coprocessor_config rp2350_coprocessor_config_##inst = { \
        .base = (struct rp2350_coprocessor_regs *)DT_INST_REG_ADDR(inst), \
        .irq_config_func = rp2350_coprocessor_irq_config_##inst,             \
    };                                                                \
    /* Zephyr device definition macro */                              \
    DEVICE_DT_INST_DEFINE(inst,                                       \
                          rp2350_coprocessor_init,                           \
                          NULL,                                       \
                          &rp2350_coprocessor_data_##inst,                   \
                          &rp2350_coprocessor_config_##inst,                 \
                          PRE_KERNEL_2,                               \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE,         \
                          &rp2350_coprocessor_api);

/* Instantiate the driver for all compatible nodes defined in the Device Tree */
DT_INST_FOREACH_STATUS_OKAY(RP2350_COPROCESSOR_DEFINE)
