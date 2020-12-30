/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2020 Peter Huewe
 */
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <assert.h>

#include "tss2_tcti.h"
#include "tss2_tcti_spi.h"
#include "tss2_tcti_spi_usb.h"
#include "tss2_mu.h"
#include "tcti-common.h"
#include "tcti-spi.h"
#include "tcti-spi-usb.h"
#include "util/io.h"
#define LOGMODULE tcti
#include "util/log.h"


//------------------------------------------------------------------------------
#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#define VID_CYPRESS 0x04b4u
#define PID_CYUSBSPI 0x0004u
#define TIMEOUT 1000
#define CTRL_SET 0xC0u
#define CTRL_GET 0x40u
#define CY_CMD_SPI 0xCAu
#define CY_CMD_GPIO_SET 0xDBu
#define CY_SPI_WRITEREAD 0x03u
#define EP_OUT 0x01u
#define EP_IN 0x82u

#define SPI_MAX_TRANSFER (64+4)

//static const uint8_t read_access[]= {0x80, 0xD4, 0x00, 0x00, 0x00};
//static const uint8_t assert_locality[]= {0x00, 0xD4, 0x00, 0x00, 0x02};

#define DEBUG 0

void posix_usleep(uint32_t us){


struct timeval ts = {us/1000000, us%1000000};
select (0,NULL,NULL, NULL, &ts);

}


void hexdump (char *desc, const uint8_t *buf, size_t len){
#if DEBUG
    printf("%s:\t", desc);
    for (size_t i = 0; i <len; i++){
        printf("%02x ", buf[i]);
        if(i!= 0 && i%16==0)
            printf("\n\t");
    }
    printf("\n");
#else
(void) desc;
(void) buf;
(void) len;
#endif

}


int tpm_reset(libusb_device_handle *dev_handle){
    int ret;
    ret = libusb_control_transfer(dev_handle, CTRL_SET, CY_CMD_GPIO_SET, 8, 0, NULL, 0, TIMEOUT);
    if (ret) {
        fprintf(stderr, "preparing SPI FAILED\n");
        goto out;
    }

       struct timespec {
               time_t tv_sec;        /* seconds */
               long   tv_nsec;       /* nanoseconds */
           };

posix_usleep(30*1000);
    ret = libusb_control_transfer(dev_handle, CTRL_SET, CY_CMD_GPIO_SET, 8, 1, NULL, 0, TIMEOUT);
    if (ret) {
        fprintf(stderr, "preparing SPI FAILED\n");
        goto out;
    }
posix_usleep(30*1000);
out:
    return ret;



}


TSS2_RC platform_spi_acquire (void* user_data)
{
    // Cast our user data
    PLATFORM_USERDATA* platform_data = (PLATFORM_USERDATA*) user_data;
    TSS2_RC ret;
    ret = libusb_control_transfer(platform_data->dev_handle, CTRL_SET, CY_CMD_GPIO_SET, 1, 0, NULL, 0, TIMEOUT);
    if (ret) {
        //fprintf(stderr, "preparing SPI FAILED\n");
        goto out;
    }
out:
    return 0;


}

TSS2_RC platform_spi_release (void* user_data)
{
    // Cast our user data
    TSS2_RC ret;
    PLATFORM_USERDATA* platform_data = (PLATFORM_USERDATA*) user_data;
    ret = libusb_control_transfer(platform_data->dev_handle, CTRL_SET, CY_CMD_GPIO_SET, 1, 1, NULL, 0, TIMEOUT);
    if (ret) {
       // fprintf(stderr, "preparing SPI FAILED\n");
        goto out;
    }
out:
    // Release CS and release the bus for other devices
    return 0;
}

TSS2_RC platform_spi_transfer (void* user_data, const void *data_out, void *data_in, size_t cnt)
{
    // Cast our user data
    PLATFORM_USERDATA* platform_data = (PLATFORM_USERDATA*) user_data;

    // Maximum transfer size is 64 byte because we don't use DMA (and the TPM doesn't support more anyway)
    if (cnt > 64+4) {
        return TSS2_TCTI_RC_BAD_VALUE;
    }

    // At least one of the buffers has to be set
    if (data_out == NULL && data_in == NULL) {
        return TSS2_TCTI_RC_BAD_VALUE;
    }

    // Clear receive buffer
    if (data_in != NULL && data_in !=  data_out) {
        memset(data_in, 0, cnt);
    }


    size_t length = cnt;
    uint8_t *spi_dma_buffer = platform_data->spi_dma_buffer;
    libusb_device_handle *dev_handle = platform_data->dev_handle;
    memset(spi_dma_buffer, 0, SPI_MAX_TRANSFER);

    int act_len = 0;
    int retry = 0;

    size_t transfered = 0;
    int ret = 0;
    if (data_out != NULL){

    memcpy(spi_dma_buffer, data_out, length);
    hexdump("OUT", data_out, length);
    }
    ret = libusb_control_transfer(dev_handle, CTRL_SET, CY_CMD_SPI, CY_SPI_WRITEREAD, length, NULL, 0, TIMEOUT);
    if (ret) {
        fprintf(stderr, "preparing SPI FAILED\n");
        goto out;
    }
    while (transfered != length){
        ret = libusb_bulk_transfer(dev_handle, EP_OUT, spi_dma_buffer+transfered, length, &act_len, TIMEOUT);
        if (ret) {
            fprintf(stderr, "WRITING SPI FAILED %d %s\n",ret, libusb_strerror(ret));
            goto out;
        }
        transfered += act_len;

        if (transfered != length) {
	#if DEBUG > 2
            fprintf(stderr, "not all bytes written %zd %d < %zd\n", transfered, act_len, length);
	#endif
        }

    }


posix_usleep(30*1000);

    transfered = 0 ;
   retry = 0 ; 
    while(transfered != length){
        ret = libusb_bulk_transfer(dev_handle, EP_IN, spi_dma_buffer+transfered, length, &act_len, TIMEOUT);
        if (ret) {
            fprintf(stderr, "READ SPI FAILED %d  %s\n", ret, libusb_strerror(ret));
	    if (retry++ > 5)
            	goto out;
	    continue;

        }
        transfered += act_len;
        if (transfered != length) {
	#if DEBUG  > 2
            fprintf(stderr, "INFO: not all bytes read %zd %d < %zd\n", transfered, act_len, length);
	    #endif
        }
    }
if (data_in != NULL) {
    memcpy(data_in, spi_dma_buffer, length);
    hexdump("IN", data_in, length);
    }
	#if DEBUG  > 2
    printf("done\n");
    #endif
out:
    memset(spi_dma_buffer, 0, SPI_MAX_TRANSFER);
    return ret;




}

void platform_sleep_ms (void* user_data, int32_t milliseconds)
{
(void ) user_data;
  posix_usleep(milliseconds*1000);
}

void platform_start_timeout (void* user_data, int32_t milliseconds)
{
    // Cast our user data
    PLATFORM_USERDATA* platform_data = (PLATFORM_USERDATA*) user_data;
    struct timeval t1;
//    struct timeval t2 = {0, milliseconds*1000};
    
    gettimeofday(&t1, NULL);


platform_data->timeout_expiry.tv_sec=milliseconds/1000;
platform_data->timeout_expiry.tv_usec=milliseconds*1000;
    //timeradd(&t1,&t2, &platform_data->timeout_expiry);
}

bool platform_timeout_expired (void* user_data)
{
    // Cast our user data
    PLATFORM_USERDATA* platform_data = (PLATFORM_USERDATA*) user_data;
    struct timeval t1;
    gettimeofday(&t1, NULL);
    int res =0; //=  timercmp(&t1,&platform_data->timeout_expiry, >);

    printf("t1: %ld.%ld exp %ld.%ld %d\n", t1.tv_sec, t1.tv_usec,platform_data->timeout_expiry.tv_sec, platform_data->timeout_expiry.tv_usec, res);


    // Check if timeout already expired
    return res;
}

void platform_finalize(void* user_data)
{
    // Cast our user data
    PLATFORM_USERDATA* platform_data = (PLATFORM_USERDATA*) user_data;

    // Free resources inside user_data like SPI device handles here
    // ...
    
    // Free user_data
    free(platform_data);
}

TSS2_TCTI_SPI_PLATFORM create_tcti_spi_platform()
{
    TSS2_TCTI_SPI_PLATFORM platform= {};
    // Create required platform user data
    PLATFORM_USERDATA* platform_data = malloc(sizeof(PLATFORM_USERDATA));
    memset(platform_data, 0, sizeof(*platform_data));
    gettimeofday(&platform_data->timeout_expiry, NULL);


    int ret=0;
    platform_data->dev_handle = NULL;
    platform_data->ctx = NULL;
    ret = libusb_init(&platform_data->ctx);
    if (ret) {
        fprintf(stderr, "libusb init failed\n");
        goto out;
    }
    platform_data->dev_handle = libusb_open_device_with_vid_pid(platform_data->ctx, VID_CYPRESS, PID_CYUSBSPI); 
    if (!platform_data->dev_handle) {
        fprintf(stderr, "LetsTrust-TPM2Go not found \n");
        goto out;
    }

    platform_data->spi_dma_buffer = libusb_dev_mem_alloc(platform_data->dev_handle, SPI_MAX_TRANSFER);
    if (!platform_data->spi_dma_buffer){

        fprintf(stderr, "failed to allocate memory\n");
        goto out;
    }

#if 0
    tpm_reset(platform_data->dev_handle);
    uint8_t readbuf [64] ;
    memset(readbuf, 0xAA, 64);

    printf("Read Access Register:");
    ret = platform_spi_transfer(platform_data, read_access, readbuf, sizeof(read_access));
    if(ret){
        fprintf(stderr, "spi_transfer failed\n");
    }
    printf("%02x\n", readbuf[4]);

    printf("Assert locality \n");
    ret = platform_spi_transfer(platform_data, assert_locality, readbuf, sizeof(read_access));
    if(ret){
        fprintf(stderr, "spi_transfer failed\n");
    }


    memset(readbuf, 0xAA, 64);
    printf("Read Access Register again:");
    ret = platform_spi_transfer(platform_data, read_access, readbuf, sizeof(read_access));
    if(ret){
        fprintf(stderr, "spi_transfer failed\n");
    }
    printf("%02x", readbuf[4]);


    tpm_reset(platform_data->dev_handle);


#endif

    
    // Create TCTI SPI platform struct with custom platform methods
    platform.user_data = platform_data;
    platform.sleep_ms = platform_sleep_ms;
    platform.start_timeout = platform_start_timeout;
    platform.timeout_expired = platform_timeout_expired;
    platform.spi_acquire = platform_spi_acquire;
    platform.spi_release = platform_spi_release;
    platform.spi_transfer = platform_spi_transfer;
    platform.finalize = platform_finalize;

    printf("PLATFORM REG DONE\n--------------------\n");
    
    return platform;
out:
    if (platform_data->spi_dma_buffer)
        libusb_dev_mem_free(platform_data->dev_handle, platform_data->spi_dma_buffer, SPI_MAX_TRANSFER);


    if(platform_data->dev_handle)
        libusb_close(platform_data->dev_handle); //close the device we opened
    if (platform_data->ctx)
        libusb_exit(platform_data->ctx); //needs to be called to end the

 return platform;

}










//------------------------------------------------------------------------------

#if 0

static inline void spi_tpm_delay_ms(TSS2_TCTI_SPI_CONTEXT* ctx, int milliseconds)
{
    // Sleep a specified amount of milliseconds
    ctx->platform.sleep_ms(ctx->platform.user_data, milliseconds);
}

static inline void spi_tpm_start_timeout(TSS2_TCTI_SPI_CONTEXT* ctx, int milliseconds)
{
    // Start a timeout timer with the specified amount of milliseconds
    ctx->platform.start_timeout(ctx->platform.user_data, milliseconds);
}

static inline bool spi_tpm_timeout_expired(TSS2_TCTI_SPI_CONTEXT* ctx)
{
    // Check if the last started tiemout expired
    return ctx->platform.timeout_expired(ctx->platform.user_data);
}

static inline TSS2_RC spi_tpm_spi_acquire(TSS2_TCTI_SPI_CONTEXT* ctx)
{
    // Reserve SPI bus until transaction is over and keep pulling CS
    return ctx->platform.spi_acquire(ctx->platform.user_data);
} 

static inline TSS2_RC spi_tpm_spi_release(TSS2_TCTI_SPI_CONTEXT* ctx)
{
    // Release SPI bus and release CS
    return ctx->platform.spi_release(ctx->platform.user_data);
} 

static inline TSS2_RC spi_tpm_spi_transfer(TSS2_TCTI_SPI_CONTEXT* ctx, const void *data_out, void *data_in, size_t cnt)
{
    // Perform SPI transaction with cnt bytes
    return ctx->platform.spi_transfer(ctx->platform.user_data, data_out, data_in, cnt);
}

static inline void spi_tpm_platform_finalize(TSS2_TCTI_SPI_CONTEXT* ctx)
{
    // Free user_data and resources inside
    ctx->platform.finalize(ctx->platform.user_data);
}

static inline uint32_t spi_tpm_read_be32(const void *src)
{
    const uint8_t *s = src;
    return (((uint32_t)s[0]) << 24) | (((uint32_t)s[1]) << 16) | (((uint32_t)s[2]) << 8) | (((uint32_t)s[3]) << 0);
}

static TSS2_RC spi_tpm_start_transaction(TSS2_TCTI_SPI_CONTEXT* ctx, enum TCTI_SPI_REGISTER_ACCESS_TYPE access, size_t bytes, uint32_t addr)
{
    TSS2_RC rc;

    // Build spi header
    uint8_t header[4];
    
    // Transaction type and transfer size
    header[0] = ((access == TCTI_SPI_REGISTER_READ) ? 0x80 : 0x00) | (bytes - 1);
    
    // TPM register address
    header[1] = addr >> 16 & 0xff;
    header[2] = addr >> 8  & 0xff;
    header[3] = addr >> 0  & 0xff;

    // Reserve SPI bus until transaction is over and keep pulling CS
    rc = spi_tpm_spi_acquire(ctx);
    if (rc != TSS2_RC_SUCCESS) {
        return TSS2_TCTI_RC_IO_ERROR;
    }

    // Send header
    uint8_t header_response[4];
    rc = spi_tpm_spi_transfer(ctx, header, header_response, 4);
    if (rc != TSS2_RC_SUCCESS) {
        return TSS2_TCTI_RC_IO_ERROR;
    }

    // Wait until the TPM exits the wait state and sends a 1 bit
    uint8_t byte;
    
    // The 1 bit is often already set in the last byte of the transaction header
    byte = header_response[3];
    if (byte & 1) {
        return TSS2_RC_SUCCESS;
    }
    
    // With most current TPMs there shouldn't be any more waitstate at all, but according to
    // the spec, we have to retry until there is no more waitstate inserted. So we try again
    // a few times by reading only one byte at a time and waiting in between.
    uint8_t zero = 0;
    for (int retries = 256; retries > 0; retries--) {
        rc = spi_tpm_spi_transfer(ctx, &zero, &byte, 1);
        if (rc != TSS2_RC_SUCCESS) {
            return TSS2_TCTI_RC_IO_ERROR;
        }
        if (byte & 1) {
            return TSS2_RC_SUCCESS;
        }
        spi_tpm_delay_ms(ctx, 1);
    }

    // The TPM did not exit the wait state in time
    return TSS2_TCTI_RC_IO_ERROR;
}

static TSS2_RC spi_tpm_end_transaction(TSS2_TCTI_SPI_CONTEXT* ctx)
{
    // Release CS (ends the transaction) and release the bus for other devices
    return spi_tpm_spi_release(ctx);
}

static void spi_tpm_log_register_access(enum TCTI_SPI_REGISTER_ACCESS_TYPE access, uint32_t reg_number, const void *buffer, size_t cnt, char* err) {
    // Print register access debug information
    char* access_str = (access == TCTI_SPI_REGISTER_READ) ? "READ" : "WRITE";
    if (err != NULL) {
        LOG_ERROR("%s register %#02x (%zu bytes) %s", access_str, reg_number, cnt, err);
    } else {
        LOGBLOB_TRACE(buffer, cnt, "%s register %#02x (%zu bytes)", access_str, reg_number, cnt);
    }
}

static TSS2_RC spi_tpm_read_reg(TSS2_TCTI_SPI_CONTEXT* ctx, uint32_t reg_number, void *buffer, size_t cnt)
{
    TSS2_RC rc;
    enum TCTI_SPI_REGISTER_ACCESS_TYPE access = TCTI_SPI_REGISTER_READ;
    
    // Check maximum register transfer size is 64 byte
    assert(cnt <= 64);

    // Start read transaction
    rc = spi_tpm_start_transaction(ctx, access, cnt, reg_number);
    if (rc != TSS2_RC_SUCCESS) {
        spi_tpm_log_register_access(access, reg_number, NULL, cnt, "failed in transaction start");
        spi_tpm_end_transaction(ctx);
        return TSS2_TCTI_RC_IO_ERROR;
    }
    // Read register
    rc = spi_tpm_spi_transfer(ctx, NULL, buffer, cnt);
    if (rc != TSS2_RC_SUCCESS) {
        spi_tpm_log_register_access(access, reg_number, NULL, cnt, "failed in transfer");
        spi_tpm_end_transaction(ctx);
        return TSS2_TCTI_RC_IO_ERROR;
    }
    // End transaction
    rc = spi_tpm_end_transaction(ctx);
    if (rc != TSS2_RC_SUCCESS) {
        spi_tpm_log_register_access(access, reg_number, NULL, cnt, "failed ending the transaction");
        return TSS2_TCTI_RC_IO_ERROR;
    }

    // Print debug information and return success
    spi_tpm_log_register_access(access, reg_number, buffer, cnt, NULL);
    return TSS2_RC_SUCCESS;
}

static TSS2_RC spi_tpm_write_reg(TSS2_TCTI_SPI_CONTEXT* ctx, uint32_t reg_number, const void *buffer, size_t cnt)
{
    TSS2_RC rc;
    enum TCTI_SPI_REGISTER_ACCESS_TYPE access = TCTI_SPI_REGISTER_WRITE;
    
    // Check maximum register transfer size is 64 byte
    assert(cnt <= 64);

    // Start write transaction
    rc = spi_tpm_start_transaction(ctx, access, cnt, reg_number);
    if (rc != TSS2_RC_SUCCESS) {
        spi_tpm_end_transaction(ctx);
        spi_tpm_log_register_access(access, reg_number, buffer, cnt, "failed in transaction start");
        return TSS2_TCTI_RC_IO_ERROR;
    }
    // Write register
    rc = spi_tpm_spi_transfer(ctx, buffer, NULL, cnt);
    if (rc != TSS2_RC_SUCCESS) {
        spi_tpm_end_transaction(ctx);
        spi_tpm_log_register_access(access, reg_number, buffer, cnt, "failed in transfer");
        return TSS2_TCTI_RC_IO_ERROR;
    }
    // End transaction
    rc = spi_tpm_end_transaction(ctx);
    if (rc != TSS2_RC_SUCCESS) {
        spi_tpm_log_register_access(access, reg_number, NULL, cnt, "failed ending the transaction");
        return TSS2_TCTI_RC_IO_ERROR;
    }

    // Print debug information and return success
    spi_tpm_log_register_access(access, reg_number, buffer, cnt, NULL);
    return TSS2_RC_SUCCESS;
}

static uint32_t spi_tpm_read_sts_reg(TSS2_TCTI_SPI_CONTEXT* ctx)
{
    uint32_t status = 0;
    spi_tpm_read_reg(ctx, TCTI_SPI_TPM_STS_REG, &status, sizeof(status));
    return status;
}

static void spi_tpm_write_sts_reg(TSS2_TCTI_SPI_CONTEXT* ctx, uint32_t status)
{
    spi_tpm_write_reg(ctx, TCTI_SPI_TPM_STS_REG, &status, sizeof(status));
}

static uint32_t spi_tpm_get_burst_count(TSS2_TCTI_SPI_CONTEXT* ctx)
{
    uint32_t status = spi_tpm_read_sts_reg(ctx);
    return (status & TCTI_SPI_TPM_STS_BURST_COUNT_MASK) >> TCTI_SPI_TPM_STS_BURST_COUNT_SHIFT;
}
#endif
#if 0
static uint8_t spi_tpm_read_access_reg(TSS2_TCTI_SPI_CONTEXT* ctx)
{
    uint8_t access = 0;
    spi_tpm_read_reg(ctx, TCTI_SPI_TPM_ACCESS_REG, &access, sizeof(access));
    return access;
}

static void spi_tpm_write_access_reg(TSS2_TCTI_SPI_CONTEXT* ctx, uint8_t access_bit)
{
    // Writes to access register can set only 1 bit at a time
    assert (!(access_bit & (access_bit - 1)));
    spi_tpm_write_reg(ctx, TCTI_SPI_TPM_ACCESS_REG, &access_bit, sizeof(access_bit));
}
#endif
#if 0
static TSS2_RC spi_tpm_claim_locality(TSS2_TCTI_SPI_CONTEXT* ctx)
{
    uint8_t access;
    access = spi_tpm_read_access_reg(ctx);
    
    // Check if locality 0 is busy
    if (access & TCTI_SPI_TPM_ACCESS_ACTIVE_LOCALITY) {
        LOG_ERROR("Locality 0 is busy, status: %#x", access);
        return TSS2_TCTI_RC_IO_ERROR;
    }

    // Request locality 0
    spi_tpm_write_access_reg(ctx, TCTI_SPI_TPM_ACCESS_REQUEST_USE);
    access = spi_tpm_read_access_reg(ctx);
    if (access & (TCTI_SPI_TPM_ACCESS_VALID | TCTI_SPI_TPM_ACCESS_ACTIVE_LOCALITY)) {
        LOG_DEBUG("Claimed locality 0");
        return TSS2_RC_SUCCESS;
    }

    LOG_ERROR("Failed to claim locality 0, status: %#x", access);
    return TSS2_TCTI_RC_IO_ERROR;
}
static TSS2_RC spi_tpm_wait_for_status(TSS2_TCTI_SPI_CONTEXT* ctx, uint32_t status_mask, uint32_t status_expected, int32_t timeout)
{
    uint32_t status;
    bool blocking = (timeout == TSS2_TCTI_TIMEOUT_BLOCK);
    if (!blocking) {
        spi_tpm_start_timeout(ctx, timeout);
    }
    
    // Wait for the expected status with or without timeout
    do {
        status = spi_tpm_read_sts_reg(ctx);
        // Return success on expected status
        if ((status & status_mask) == status_expected) {
            return TSS2_RC_SUCCESS;
        }
        // Delay next poll by 8ms to avoid spamming the TPM
        spi_tpm_delay_ms(ctx, 8);
    } while (blocking || !spi_tpm_timeout_expired(ctx));

    // Timed out
    return TSS2_TCTI_RC_TRY_AGAIN;
}

static inline size_t spi_tpm_size_t_min(size_t a, size_t b) {
    if (a < b) {
        return a;
    }
    return b;
}

static void spi_tpm_fifo_transfer(TSS2_TCTI_SPI_CONTEXT* ctx, uint8_t* transfer_buffer, size_t transfer_size, enum TCTI_SPI_FIFO_TRANSFER_DIRECTION direction)
{
    size_t transaction_size;
    size_t burst_count;
    size_t handled_so_far = 0;

    do {
        do {
            // Can be zero when TPM is busy
            burst_count = spi_tpm_get_burst_count(ctx);
        } while (!burst_count);

        transaction_size = transfer_size - handled_so_far;
        transaction_size = spi_tpm_size_t_min(transaction_size, burst_count);
        transaction_size = spi_tpm_size_t_min(transaction_size, 64);

        if (direction == TCTI_SPI_FIFO_RECEIVE){
            spi_tpm_read_reg(ctx, TCTI_SPI_TPM_DATA_FIFO_REG, (void*)(transfer_buffer + handled_so_far), transaction_size);
        } else {
            spi_tpm_write_reg(ctx, TCTI_SPI_TPM_DATA_FIFO_REG, (const void*)(transfer_buffer + handled_so_far), transaction_size);
        }

        handled_so_far += transaction_size;

    } while (handled_so_far != transfer_size);
}

/*
 * This function wraps the "up-cast" of the opaque TCTI context type to the
 * type for the device TCTI context. The only safe-guard we have to ensure
 * this operation is possible is the magic number for the device TCTI context.
 * If passed a NULL context, or the magic number check fails, this function
 * will return NULL.
 */
TSS2_TCTI_SPI_CONTEXT* tcti_spi_context_cast (TSS2_TCTI_CONTEXT *tcti_ctx)
{
    if (tcti_ctx != NULL && TSS2_TCTI_MAGIC (tcti_ctx) == TCTI_SPI_USB_MAGIC) {
        return (TSS2_TCTI_SPI_CONTEXT*)tcti_ctx;
    }
LOG_ERROR("context cast failed");
    return NULL;
}

/*
 * This function down-casts the device TCTI context to the common context
 * defined in the tcti-common module.
 */
TSS2_TCTI_COMMON_CONTEXT* tcti_spi_down_cast (TSS2_TCTI_SPI_CONTEXT *tcti_spi)
{
    if (tcti_spi == NULL) {
        return NULL;
    }
    return &tcti_spi->common;
}

TSS2_RC tcti_spi_receive (TSS2_TCTI_CONTEXT* tcti_context, size_t *response_size, unsigned char *response_buffer, int32_t timeout)
{
    TSS2_RC rc;
    TSS2_TCTI_SPI_CONTEXT* tcti_spi = tcti_spi_context_cast (tcti_context);
    TSS2_TCTI_COMMON_CONTEXT* tcti_common = tcti_spi_down_cast (tcti_spi);
    
    if (tcti_spi == NULL) {
        return TSS2_TCTI_RC_BAD_CONTEXT;
    }
    
    rc = tcti_common_receive_checks (tcti_common, response_size, TCTI_SPI_USB_MAGIC);
    if (rc != TSS2_RC_SUCCESS) {
LOG_ERROR("common checks failed");
        return rc;
    }

    // Use ctx as a shorthand for tcti_spi
    TSS2_TCTI_SPI_CONTEXT* ctx = tcti_spi;

    // Expected status bits for valid status and data availabe
    uint32_t expected_status_bits = TCTI_SPI_TPM_STS_VALID | TCTI_SPI_TPM_STS_DATA_AVAIL;

    // Check if we already have received the header
    if (tcti_common->header.size == 0) {
        // Wait for response to be ready
        rc = spi_tpm_wait_for_status(ctx, expected_status_bits, expected_status_bits, timeout);
        if (rc != TSS2_RC_SUCCESS) {
            LOG_ERROR("Failed waiting for status");
            // Return rc from wait_for_status(). May be TRY_AGAIN after timeout.
            return rc;
        }

        // Read only response header into context header buffer
        rc = spi_tpm_read_reg(ctx, TCTI_SPI_TPM_DATA_FIFO_REG, ctx->header, TCTI_SPI_RESP_HEADER_SIZE);
        if (rc != TSS2_RC_SUCCESS) {
            LOG_ERROR("Failed reading response header");
            return TSS2_TCTI_RC_IO_ERROR;
        }

        // Find out the total payload size, skipping the two byte tag and update tcti_common
        tcti_common->header.size = spi_tpm_read_be32(ctx->header + 2);
        LOG_TRACE("Read response size from response header: %" PRIu32 " bytes", tcti_common->header.size);
    }

    // Check if response size is requested
    if (response_buffer == NULL) {
        *response_size = tcti_common->header.size;
        LOG_TRACE("Caller requested response size. Returning size of %zu bytes", *response_size);
        return TSS2_RC_SUCCESS;
    }

    // Check if response fits in buffer and update response size
    if (tcti_common->header.size > *response_size) {
        LOG_ERROR("TPM response too long (%" PRIu32 " bytes)", tcti_common->header.size);
        return TSS2_TCTI_RC_INSUFFICIENT_BUFFER;
    }
    *response_size = tcti_common->header.size;

    // Receive the TPM response
    LOG_TRACE("Reading response of size %" PRIu32, tcti_common->header.size);
    
    // Copy already received header into response buffer
    memcpy(response_buffer, ctx->header, TCTI_SPI_RESP_HEADER_SIZE);

    // Read all but the last byte in the FIFO
    size_t bytes_to_go = tcti_common->header.size - 1 - TCTI_SPI_RESP_HEADER_SIZE;
    spi_tpm_fifo_transfer(ctx, response_buffer + TCTI_SPI_RESP_HEADER_SIZE, bytes_to_go, TCTI_SPI_FIFO_RECEIVE);

    // Verify that there is still data to read
    uint32_t status = spi_tpm_read_sts_reg(ctx);
    if ((status & expected_status_bits) != expected_status_bits) {
        LOG_ERROR("Unexpected intermediate status %#x",status);
        return TSS2_TCTI_RC_IO_ERROR;
    }

    // Read the last byte
    rc = spi_tpm_read_reg(ctx, TCTI_SPI_TPM_DATA_FIFO_REG, response_buffer + tcti_common->header.size - 1, 1);
    if (rc != TSS2_RC_SUCCESS) {
        return TSS2_TCTI_RC_IO_ERROR;
    }

    // Verify that there is no more data available
    status = spi_tpm_read_sts_reg(ctx);
    if ((status & expected_status_bits) != TCTI_SPI_TPM_STS_VALID) {
        LOG_ERROR("Unexpected final status %#x", status);
        return TSS2_TCTI_RC_IO_ERROR;
    }

    LOGBLOB_DEBUG(response_buffer, tcti_common->header.size, "Response buffer received:");

    // Set the TPM back to idle state
    spi_tpm_write_sts_reg(ctx, TCTI_SPI_TPM_STS_COMMAND_READY);

    tcti_common->header.size = 0;
    tcti_common->state = TCTI_STATE_TRANSMIT;

    return TSS2_RC_SUCCESS;
}

void tcti_spi_finalize (TSS2_TCTI_CONTEXT* tcti_context)
{
    TSS2_TCTI_SPI_CONTEXT *tcti_spi = tcti_spi_context_cast (tcti_context);
    TSS2_TCTI_COMMON_CONTEXT *tcti_common = tcti_spi_down_cast (tcti_spi);
    
    if (tcti_spi == NULL) {
        return;
    }
    tcti_common->state = TCTI_STATE_FINAL;

    // Free platform struct user data and resources inside
    spi_tpm_platform_finalize(tcti_spi);
}

TSS2_RC tcti_spi_transmit (TSS2_TCTI_CONTEXT *tcti_ctx, size_t size, const uint8_t *cmd_buf)
{
    TSS2_RC rc;
    TSS2_TCTI_SPI_CONTEXT *tcti_spi = tcti_spi_context_cast (tcti_ctx);
    TSS2_TCTI_COMMON_CONTEXT *tcti_common = tcti_spi_down_cast (tcti_spi);
    tpm_header_t header;
    
    if (tcti_spi == NULL) {
        return TSS2_TCTI_RC_BAD_CONTEXT;
    }
    TSS2_TCTI_SPI_CONTEXT* ctx = tcti_spi;

    rc = tcti_common_transmit_checks (tcti_common, cmd_buf, TCTI_SPI_USB_MAGIC);
    if (rc != TSS2_RC_SUCCESS) {
LOG_ERROR("transmit checks failed");
        return rc;
    }
    rc = header_unmarshal (cmd_buf, &header);
    if (rc != TSS2_RC_SUCCESS) {
        return rc;
    }
    if (header.size != size) {
        LOG_ERROR("Buffer size parameter: %zu, and TPM2 command header size "
                  "field: %" PRIu32 " disagree.", size, header.size);
        return TSS2_TCTI_RC_BAD_VALUE;
    }

    LOGBLOB_DEBUG (cmd_buf, size, "Sending command with TPM_CC %#x and size %" PRIu32,
               header.code, header.size);

    // Tell TPM to expect command
    spi_tpm_write_sts_reg(ctx, TCTI_SPI_TPM_STS_COMMAND_READY);

    // Send command
    spi_tpm_fifo_transfer(ctx, (void*)cmd_buf, size, TCTI_SPI_FIFO_TRANSMIT);

    // Tell TPM to start processing the command
    spi_tpm_write_sts_reg(ctx, TCTI_SPI_TPM_STS_GO);

    tcti_common->state = TCTI_STATE_RECEIVE;
    return TSS2_RC_SUCCESS;
}

TSS2_RC tcti_spi_cancel (TSS2_TCTI_CONTEXT* tcti_context)
{
    (void)(tcti_context);
    return TSS2_TCTI_RC_NOT_IMPLEMENTED;
}

TSS2_RC tcti_spi_get_poll_handles (TSS2_TCTI_CONTEXT* tcti_context, TSS2_TCTI_POLL_HANDLE *handles, size_t *num_handles)
{
    (void)(tcti_context);
    (void)(handles);
    (void)(num_handles);
    return TSS2_TCTI_RC_NOT_IMPLEMENTED;
}

TSS2_RC tcti_spi_set_locality (TSS2_TCTI_CONTEXT* tcti_context, uint8_t locality)
{
    (void)(tcti_context);
    (void)(locality);
    return TSS2_TCTI_RC_NOT_IMPLEMENTED;
}
#endif

extern TSS2_RC tcti_spi_receive (TSS2_TCTI_CONTEXT* tcti_context, size_t *response_size, unsigned char *response_buffer, int32_t timeout);
extern TSS2_RC tcti_spi_transmit (TSS2_TCTI_CONTEXT *tcti_ctx, size_t size, const uint8_t *cmd_buf);
extern void tcti_spi_finalize (TSS2_TCTI_CONTEXT* tcti_context);
TSS2_RC tcti_spi_cancel (TSS2_TCTI_CONTEXT* tcti_context)
{
    (void)(tcti_context);
    return TSS2_TCTI_RC_NOT_IMPLEMENTED;
}

TSS2_RC tcti_spi_get_poll_handles (TSS2_TCTI_CONTEXT* tcti_context, TSS2_TCTI_POLL_HANDLE *handles, size_t *num_handles)
{
    (void)(tcti_context);
    (void)(handles);
    (void)(num_handles);
    return TSS2_TCTI_RC_NOT_IMPLEMENTED;
}

TSS2_RC tcti_spi_set_locality (TSS2_TCTI_CONTEXT* tcti_context, uint8_t locality)
{
    (void)(tcti_context);
    (void)(locality);
    return TSS2_TCTI_RC_NOT_IMPLEMENTED;
}
TSS2_TCTI_SPI_CONTEXT* tcti_spi_context_cast (TSS2_TCTI_CONTEXT *tcti_ctx)
{
    if (tcti_ctx != NULL && TSS2_TCTI_MAGIC (tcti_ctx) == TCTI_SPI_USB_MAGIC) {
        return (TSS2_TCTI_SPI_CONTEXT*)tcti_ctx;
    }
LOG_ERROR("context cast failed");
    return NULL;
}

/*
 * This function down-casts the device TCTI context to the common context
 * defined in the tcti-common module.
 */
TSS2_TCTI_COMMON_CONTEXT* tcti_spi_down_cast (TSS2_TCTI_SPI_CONTEXT *tcti_spi)
{
    if (tcti_spi == NULL) {
        return NULL;
    }
    return &tcti_spi->common;
}

TSS2_RC Tss2_Tcti_Spi_Usb_Init (TSS2_TCTI_CONTEXT* tcti_context, size_t* size, const char* config)
{
    TSS2_TCTI_SPI_CONTEXT* tcti_spi;
    TSS2_TCTI_COMMON_CONTEXT* tcti_common;
    TSS2_TCTI_SPI_PLATFORM tcti_platform = {}; 

    // Check if context size is requested
    if (tcti_context == NULL) {
        return  Tss2_Tcti_Spi_Init(NULL, size, tcti_platform);
    }

    tcti_platform = create_tcti_spi_platform();
//LOG_ERROR("INIT");
    // Init TCTI context
    TSS2_TCTI_MAGIC (tcti_context) = TCTI_SPI_USB_MAGIC;
    TSS2_TCTI_VERSION (tcti_context) = TCTI_VERSION;
    TSS2_TCTI_TRANSMIT (tcti_context) = tcti_spi_transmit;
    TSS2_TCTI_RECEIVE (tcti_context) = tcti_spi_receive;
    TSS2_TCTI_FINALIZE (tcti_context) = tcti_spi_finalize;
    TSS2_TCTI_CANCEL (tcti_context) = tcti_spi_cancel;
    TSS2_TCTI_GET_POLL_HANDLES (tcti_context) = tcti_spi_get_poll_handles;
    TSS2_TCTI_SET_LOCALITY (tcti_context) = tcti_spi_set_locality;
    TSS2_TCTI_MAKE_STICKY (tcti_context) = tcti_make_sticky_not_implemented;
    
    // Init SPI TCTI context
    tcti_spi = tcti_spi_context_cast (tcti_context);
    tcti_common = tcti_spi_down_cast (tcti_spi);
    tcti_common->state = TCTI_STATE_TRANSMIT;
    memset (&tcti_common->header, 0, sizeof (tcti_common->header));
    tcti_common->locality = 0;
    (void) config;


    // Initialize TCTI context
    return Tss2_Tcti_Spi_Init(tcti_context, size, tcti_platform);
}

const TSS2_TCTI_INFO tss2_tcti_info = {
    .version = TCTI_VERSION,
    .name = "tcti-spi-usb",
    .description = "Platform independent TCTI for communication with TPMs over SPI.",
    .config_help = "TSS2_TCTI_SPI_PLATFORM struct containing platform methods. See tss2_tcti_spi.h for more information.",
    
    /*
     * The Tss2_Tcti_Spi_Init method has a different signature than required by .init due too
     * our custom platform_conf parameter, so we can't expose it here and it has to be used directly.
     */
    .init = Tss2_Tcti_Spi_Usb_Init
};

const TSS2_TCTI_INFO* Tss2_Tcti_Info (void)
{
    return &tss2_tcti_info;
}
