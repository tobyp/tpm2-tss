/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2020 Peter Huewe
 */
#ifndef TCTI_SPI_USB_H
#define TCTI_SPI_USB_H

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "tcti-common.h"

#define TCTI_SPI_USB_MAGIC 0x4D5C6E8BD4811477ULL

#include "tss2_tcti_spi.h"
#include <sys/time.h>
#include <stdint.h>
#include <libusb-1.0/libusb.h>
typedef struct {
uint32_t timeoutval;
    struct timeval timeout_expiry;
    libusb_device_handle *dev_handle;
    libusb_context *ctx;
    uint8_t *spi_dma_buffer;
} PLATFORM_USERDATA;

TSS2_TCTI_SPI_PLATFORM create_tcti_spi_platform();

#endif /* TCTI_SPI_USB_H */
