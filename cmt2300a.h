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

typedef enum {
    /* CMT2300A_MASK_GPIO4_SEL options */
    CMT2300A_GPIO4_SEL_RSTIN = 0x00,
    CMT2300A_GPIO4_SEL_INT1 = 0x40,
    CMT2300A_GPIO4_SEL_DOUT = 0x80,
    CMT2300A_GPIO4_SEL_DCLK = 0xC0,

    /* CMT2300A_MASK_GPIO3_SEL options */
    CMT2300A_GPIO3_SEL_CLKO = 0x00,
    CMT2300A_GPIO3_SEL_DOUT = 0x10,
    CMT2300A_GPIO3_SEL_DIN = 0x10,
    CMT2300A_GPIO3_SEL_INT2 = 0x20,
    CMT2300A_GPIO3_SEL_DCLK = 0x30,

    /* CMT2300A_MASK_GPIO2_SEL options */
    CMT2300A_GPIO2_SEL_INT1 = 0x00,
    CMT2300A_GPIO2_SEL_INT2 = 0x04,
    CMT2300A_GPIO2_SEL_DOUT = 0x08,
    CMT2300A_GPIO2_SEL_DIN = 0x08,
    CMT2300A_GPIO2_SEL_DCLK = 0x0C,

    /* CMT2300A_MASK_GPIO1_SEL options */
    CMT2300A_GPIO1_SEL_DOUT = 0x00,
    CMT2300A_GPIO1_SEL_DIN = 0x00,
    CMT2300A_GPIO1_SEL_INT1 = 0x01,
    CMT2300A_GPIO1_SEL_INT2 = 0x02,
    CMT2300A_GPIO1_SEL_DCLK = 0x03,
} cmt2300a_gpio_functions_t;

typedef enum {
    CMT2300A_INT_SEL_RX_ACTIVE = 0x00,
    CMT2300A_INT_SEL_TX_ACTIVE = 0x01,
    CMT2300A_INT_SEL_RSSI_VLD = 0x02,
    CMT2300A_INT_SEL_PREAM_OK = 0x03,
    CMT2300A_INT_SEL_SYNC_OK = 0x04,
    CMT2300A_INT_SEL_NODE_OK = 0x05,
    CMT2300A_INT_SEL_CRC_OK = 0x06,
    CMT2300A_INT_SEL_PKT_OK = 0x07,
    CMT2300A_INT_SEL_SL_TMO = 0x08,
    CMT2300A_INT_SEL_RX_TMO = 0x09,
    CMT2300A_INT_SEL_TX_DONE = 0x0A,
    CMT2300A_INT_SEL_RX_FIFO_NMTY = 0x0B,
    CMT2300A_INT_SEL_RX_FIFO_TH = 0x0C,
    CMT2300A_INT_SEL_RX_FIFO_FULL = 0x0D,
    CMT2300A_INT_SEL_RX_FIFO_WBYTE = 0x0E,
    CMT2300A_INT_SEL_RX_FIFO_OVF = 0x0F,
    CMT2300A_INT_SEL_TX_FIFO_NMTY = 0x10,
    CMT2300A_INT_SEL_TX_FIFO_TH = 0x11,
    CMT2300A_INT_SEL_TX_FIFO_FULL = 0x12,
    CMT2300A_INT_SEL_STATE_IS_STBY = 0x13,
    CMT2300A_INT_SEL_STATE_IS_FS = 0x14,
    CMT2300A_INT_SEL_STATE_IS_RX = 0x15,
    CMT2300A_INT_SEL_STATE_IS_TX = 0x16,
    CMT2300A_INT_SEL_LBD = 0x17,
    CMT2300A_INT_SEL_TRX_ACTIVE = 0x18,
    CMT2300A_INT_SEL_PKT_DONE = 0x19,
} cmt2300_gpio_irq_mappings_t;

typedef enum {
    CMT2300A_MASK_SL_TMO_EN = 0x80,
    CMT2300A_MASK_RX_TMO_EN = 0x40,
    CMT2300A_MASK_TX_DONE_EN = 0x20,
    CMT2300A_MASK_PREAM_OK_EN = 0x10,
    CMT2300A_MASK_SYNC_OK_EN = 0x08,
    CMT2300A_MASK_NODE_OK_EN = 0x04,
    CMT2300A_MASK_CRC_OK_EN = 0x02,
    CMT2300A_MASK_PKT_DONE_EN = 0x01,
} cmt2300_which_irq_enabled_t;

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

/* 
 * The function defines what pin what have to do.
 * 'mask' is 'cmt2300a_gpio_functions_t' enum.
 *
 *         GPIO1_SEL:
 *            CMT2300A_GPIO1_SEL_DOUT/DIN 
 *            CMT2300A_GPIO1_SEL_INT1
 *            CMT2300A_GPIO1_SEL_INT2
 *            CMT2300A_GPIO1_SEL_DCLK
 *
 *          GPIO2_SEL:
 *            CMT2300A_GPIO2_SEL_INT1
 *            CMT2300A_GPIO2_SEL_INT2
 *            CMT2300A_GPIO2_SEL_DOUT/DIN 
 *            CMT2300A_GPIO2_SEL_DCLK
 *
 *          GPIO3_SEL:
 *            CMT2300A_GPIO3_SEL_CLKO
 *            CMT2300A_GPIO3_SEL_DOUT/DIN
 *            CMT2300A_GPIO3_SEL_INT2 
 *            CMT2300A_GPIO3_SEL_DCLK
 *
 *          GPIO4_SEL:
 *            CMT2300A_GPIO4_SEL_RSTIN
 *            CMT2300A_GPIO4_SEL_INT1
 *            CMT2300A_GPIO4_SEL_DOUT
 *            CMT2300A_GPIO4_SEL_DCLK
 * 
 * Example:
 *      cmt2300a_select_gpio_pins_mode(&mydevive, CMT2300A_GPIO1_SEL_INT1 | CMT2300A_GPIO2_SEL_INT2);
 * After this call GPIO1 responces for the INT1, GPIO1 for the INT2.
 * 
 * Returns CMT2300A_SUCCESS in case of success.
 */
int cmt2300a_select_gpio_pins_mode(cmt2300a_dev_t *dev, uint32_t mask);

/*
 * Config int what event interrupt on INT1 and INT2 will trigger GPIO. Active - 1 is default.
 * 
 * 'int1_mapping' and 'int2_mapping' is 'cmt2300_gpio_irq_mappings_t' enum.
 *            CMT2300A_INT_SEL_RX_ACTIVE
 *            CMT2300A_INT_SEL_TX_ACTIVE
 *            CMT2300A_INT_SEL_RSSI_VLD
 *            CMT2300A_INT_SEL_PREAM_OK
 *            CMT2300A_INT_SEL_SYNC_OK
 *            CMT2300A_INT_SEL_NODE_OK
 *            CMT2300A_INT_SEL_CRC_OK
 *            CMT2300A_INT_SEL_PKT_OK
 *            CMT2300A_INT_SEL_SL_TMO
 *            CMT2300A_INT_SEL_RX_TMO
 *            CMT2300A_INT_SEL_TX_DONE
 *            CMT2300A_INT_SEL_RX_FIFO_NMTY
 *            CMT2300A_INT_SEL_RX_FIFO_TH
 *            CMT2300A_INT_SEL_RX_FIFO_FULL
 *            CMT2300A_INT_SEL_RX_FIFO_WBYTE
 *            CMT2300A_INT_SEL_RX_FIFO_OVF
 *            CMT2300A_INT_SEL_TX_FIFO_NMTY
 *            CMT2300A_INT_SEL_TX_FIFO_TH
 *            CMT2300A_INT_SEL_TX_FIFO_FULL
 *            CMT2300A_INT_SEL_STATE_IS_STBY
 *            CMT2300A_INT_SEL_STATE_IS_FS
 *            CMT2300A_INT_SEL_STATE_IS_RX
 *            CMT2300A_INT_SEL_STATE_IS_TX
 *            CMT2300A_INT_SEL_LED
 *            CMT2300A_INT_SEL_TRX_ACTIVE
 *            CMT2300A_INT_SEL_PKT_DONE
 * 
 * Example:
 *       cmt2300a_select_gpio_pins_mode(&mydevive, CMT2300A_GPIO1_SEL_INT1 | CMT2300A_GPIO2_SEL_INT2);
 *       cmt2300a_map_gpio_to_int(&muydev, CMT2300A_INT_SEL_TX_DONE, CMT2300A_INT_SEL_PKT_OK);
 * After this call:
 *    - GPIO1 will be triggered only after transmitting done
 *    - GPIO2 will be triggered only after receiving whole packet (right or wrong).
 * To see if packet write or wrong enable more interrupts (but they will not trigger gpio)
 * and read interrupt flags from chip. Function cmt2300a_clear_irq_flags() returns all IRQs
 * which have been enabled by user and triggered inside radio chip during receiving a packet.
 * 
 * Returns CMT2300A_SUCCESS in case of success.
 */
int cmt2300a_map_gpio_to_irq(cmt2300a_dev_t *dev, uint8_t int1_mapping, uint8_t int2_mapping);

/*
 * Set polarity active level.
 * 'level' is:
 *      true - active-high (default)
 *      false - active-low
 * 
 * Returns CMT2300A_SUCCESS in case of success. 
 */
int cmt2300a_set_irq_polar(cmt2300a_dev_t *dev, bool level);

/*
 * Which have to be enabled?
 * 'mask' is 'cmt2300_which_irq_enabled_t' enum.
 *            CMT2300A_MASK_SL_TMO_EN   |
 *            CMT2300A_MASK_RX_TMO_EN   |
 *            CMT2300A_MASK_TX_DONE_EN  |
 *            CMT2300A_MASK_PREAM_OK_EN |
 *            CMT2300A_MASK_SYNC_OK_EN  |
 *            CMT2300A_MASK_NODE_OK_EN  |
 *            CMT2300A_MASK_CRC_OK_EN   |
 *            CMT2300A_MASK_PKT_DONE_EN
 * 
 * Call this with needed mask to trigger corresponding flags in the chip.
 * But remember some of them will trigger GPIOs. See cmt2300a_map_gpio_to_int().
 * 
 * Returns CMT2300A_SUCCESS in case of success. 
 */
int cmt2300a_enable_irq(cmt2300a_dev_t *dev, uint8_t mask);

/*
 * 'is_merged' true: use a single 64-byte FIFO for either Tx or Rx
 *             false: use a 32-byte FIFO for Tx and another 32-byte FIFO for Rx(default)
 */
int cmt2300a_set_merge_fifo(cmt2300a_dev_t *dev, bool is_merged);
int cmt2300a_set_fifo_threshold(cmt2300a_dev_t *dev, uint32_t threshold);

#ifdef __cplusplus
}
#endif

#endif /* __CMT2300A_H */
