/*
 * bouncebuffer.h — minimal pass-through stub for STM32H7 SDMMC IDMA.
 *
 * The ArduPilot-patched ChibiOS SDMMCv2 driver calls these helpers to
 * optionally copy DMA data through a cache-safe intermediate buffer.
 * On this firmware all FatFS I/O buffers live in SRAM3 (.nocache section),
 * so no bounce copy is needed — pass the pointer through as-is.
 */

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct bouncebuffer_t {
    uint8_t _unused;
};

static inline bool bouncebuffer_setup_read(struct bouncebuffer_t *bb,
                                            uint8_t **buf, size_t size)
{
    (void)bb; (void)buf; (void)size;
    return true;
}

static inline void bouncebuffer_finish_read(struct bouncebuffer_t *bb,
                                             uint8_t *buf, size_t size)
{
    (void)bb; (void)buf; (void)size;
}

static inline bool bouncebuffer_setup_write(struct bouncebuffer_t *bb,
                                             const uint8_t **buf, size_t size)
{
    (void)bb; (void)buf; (void)size;
    return true;
}

static inline void bouncebuffer_finish_write(struct bouncebuffer_t *bb,
                                              const uint8_t *buf)
{
    (void)bb; (void)buf;
}

#ifdef __cplusplus
}
#endif
