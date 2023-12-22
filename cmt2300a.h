#ifndef __CMT2300A_H
#define __CMT2300A_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CMT2300_433FREQ_MODE,
    CMT2300_868FREQ_MODE,
} cmt2300_modes_t;

typedef enum {
    CMT2300A_SUCCESS,
    CMT2300A_FAILED,
} cmt2300_returns_t;

typedef enum {
    CTM2300A_STATE_IDLE = 0,
    CTM2300A_STATE_SLEEP = 1,
    CTM2300A_STATE_STBY = 2,
    CTM2300A_STATE_RFS = 3,
    CTM2300A_STATE_TFS = 4,
    CTM2300A_STATE_RX = 5,
    CTM2300A_STATE_TX = 6,
    CTM2300A_STATE_UNLOCKED_LOW_VDD = 8,
    CTM2300A_STATE_CAL = 9,
    CTM2300A_STATE_INVALID,
} cmt2300a_state_t;

typedef enum {
    CMT2300A_GO_EEPROM = 0x01,
    CMT2300A_GO_STBY = 0x02,
    CMT2300A_GO_RFS = 0x04,
    CMT2300A_GO_RX = 0x08,
    CMT2300A_GO_SLEEP = 0x10,
    CMT2300A_GO_TFS = 0x20,
    CMT2300A_GO_TX = 0x40,
    CMT2300A_GO_SWITCH = 0x80,
} cmt2300a_go_state_t;

/* Low level functions for radio chip pins managing */
typedef struct {
    void (*delay_us)(uint32_t us);

    /* chip select pin */
    void (*csb_pin_set)(void);
    void (*csb_pin_reset)(void);

    /* FIFO select pin */
    void (*fcsb_pin_set)(void);
    void (*fcsb_pin_reset)(void);

    /* SCLK */
    void (*sclk_pin_set)(void);
    void (*sclk_pin_reset)(void);

    /* DIO */
    void (*dio_pin_set)(void);
    void (*dio_pin_reset)(void);
    int (*dio_read_state)(void); /* must return 1, if pin set and 0, if reset state */
} cmt2300a_ll_t;

/* Device sample */
typedef struct {
    cmt2300a_ll_t *cmt2300a_ll;
} cmt2300a_dev_t;

/*
 * Default init IS:
 * - CFG_RETAIN enabled (settings not erased after soft reset)
 * - RSTN_IN_EN disabled
 * - LFOSC disabled, LFOSC disabled (to gpio3). Defult 25kHz, if you'll call 'cmt2300a_set_lfosc_output(dev, 1);'
 * - SLEEP timer disabled
 * - All IRQ flags cleared
 * - 32-byte FIFO for Tx and another 32-byte FIFO for Rx (It's a default work).
 *   Other words: your tx-buf-max-size = your rx-buf-max-size = 32 bytes
 * - Also this function checks the chip exists by reading special values from special registers.
 * 
 * Returns CMT2300A_SUCCESS in case of success.
 */
int cmt2300a_init(cmt2300a_dev_t *dev, int mode);

/* Returns CMT2300A_SUCCESS only in case if chip state is CTM2300A_STATE_SLEEP after soft reset command */
int cmt2300a_soft_reset(cmt2300a_dev_t *dev);

/* Returns CMT2300A_SUCCESS if yes */
int cmt2300a_is_chip_exist(cmt2300a_dev_t *dev);

/*
 * Change chip work state.
 * 
 * 'go_state_mask' is one of 'cmt2300a_go_state_t' value.
 * 'desired_chip_status_mask' is one of 'cmt2300a_state_t' value.
 * 
 * Returns CMT2300A_SUCCESS in case real chip state equal 'desired_chip_status_mask'.
 */
int cmt2300a_go_state(cmt2300a_dev_t *dev, int go_state_mask, int desired_chip_status_mask);

/*
 * Obtain current chip work state
 * Returns one of 'cmt2300a_state_t' value.
 */
uint8_t cmt2300a_get_state(cmt2300a_dev_t *dev);

void cmt2300a_set_lfosc(cmt2300a_dev_t *dev, bool enabled); /* Low frequency osc state */
void cmt2300a_set_lfosc_output(cmt2300a_dev_t *dev, bool enabled); /* LFOSC output via GPIO3 */

/* Clears all rised IRQ inside chip and returns IRQ masks have been rised */
uint8_t cmt2300a_clear_irq_flags(cmt2300a_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __CMT2300A_H */
