#include "cmt2300a.h"
#include "cmt2300a_defs.h"
#include "cmt2300a_params.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>

#define LINK_WITH_CHIP_ATTEMPTS 20
#define IF_DELAY_US 5

static void     write_reg(cmt2300a_dev_t *dev, uint8_t reg, uint8_t data);
static uint8_t  read_reg(cmt2300a_dev_t *dev, uint8_t reg);
static void     config_reg_bank(cmt2300a_dev_t *dev, uint8_t base_addr, const uint8_t *bank, size_t len);

int cmt2300a_init(cmt2300a_dev_t *dev, int mode)
{
    if (!dev || \
        !dev->cmt2300a_ll->delay_us || \
        !dev->cmt2300a_ll->csb_pin_set || \
        !dev->cmt2300a_ll->csb_pin_reset || \
        !dev->cmt2300a_ll->fcsb_pin_set || \
        !dev->cmt2300a_ll->fcsb_pin_reset || \
        !dev->cmt2300a_ll->sclk_pin_set || \
        !dev->cmt2300a_ll->sclk_pin_reset || \
        !dev->cmt2300a_ll->dio_pin_set || \
        !dev->cmt2300a_ll->dio_pin_reset || \
        !dev->cmt2300a_ll->dio_read_state) {
        return CMT2300A_FAILED;
    }

    /* Setup low level interface */
    dev->cmt2300a_ll->csb_pin_set();
    dev->cmt2300a_ll->sclk_pin_reset();
    dev->cmt2300a_ll->dio_pin_set();
    dev->cmt2300a_ll->fcsb_pin_set();
    dev->cmt2300a_ll->delay_us(20);

    /* 1. Ensure chip is in SLEEP state */
    if (cmt2300a_soft_reset(dev) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    /* 2. Go STBY and confirm it */
    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    /* 3. Config registers */
    config_reg_bank(dev, CMT2300A_CMT_BANK_ADDR, g_cmt2300aCmtBank, CMT2300A_CMT_BANK_SIZE);
    config_reg_bank(dev, CMT2300A_SYSTEM_BANK_ADDR, g_cmt2300aSystemBank, CMT2300A_SYSTEM_BANK_SIZE);
    config_reg_bank(dev, CMT2300A_FREQUENCY_BANK_ADDR, g_cmt2300aFrequencyBank, CMT2300A_FREQUENCY_BANK_SIZE);
    config_reg_bank(dev, CMT2300A_DATA_RATE_BANK_ADDR, g_cmt2300aDataRateBank, CMT2300A_DATA_RATE_BANK_SIZE);
    config_reg_bank(dev, CMT2300A_BASEBAND_BANK_ADDR, g_cmt2300aBasebandBank, CMT2300A_BASEBAND_BANK_SIZE);
    config_reg_bank(dev, CMT2300A_TX_BANK_ADDR, g_cmt2300aTxBank, CMT2300A_TX_BANK_SIZE);

    /* 4. Diable LFOSC (used for sleep timer) */
    cmt2300a_set_lfosc(dev, false);
    cmt2300a_set_lfosc_output(dev, false);

    /* 5, 6. Disable RSTN_IN, Enable CFG_RETAIN */
    uint8_t tmp = read_reg(dev, CMT2300A_CUS_MODE_STA);
    tmp &= ~CMT2300A_MASK_RSTN_IN_EN;
    tmp |= CMT2300A_MASK_CFG_RETAIN;
    write_reg(dev, CMT2300A_CUS_MODE_STA, tmp);

    /* Reset all IRQ flags */
    (void)cmt2300a_clear_irq_flags(dev);

    if (cmt2300a_is_chip_exist(dev) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    /* SLEEP mode to configuration to take effect */
    if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    return CMT2300A_SUCCESS;
}

int cmt2300a_soft_reset(cmt2300a_dev_t *dev)
{
    int attempts = LINK_WITH_CHIP_ATTEMPTS;
    uint8_t status = CTM2300A_STATE_INVALID;

    write_reg(dev, 0x7F, 0xFF);

    while ((status != CTM2300A_STATE_SLEEP) && (attempts > 0)) {
        dev->cmt2300a_ll->delay_us(20000);
        status = cmt2300a_get_state(dev);
        --attempts;
    }
    if (status != CTM2300A_STATE_SLEEP) {
        return CMT2300A_FAILED;
    }

    return CMT2300A_SUCCESS;
}

int cmt2300a_is_chip_exist(cmt2300a_dev_t *dev)
{
    uint8_t back = 0;
    uint8_t data = 0;

    back = read_reg(dev, CMT2300A_CUS_PKT17);
    write_reg(dev, CMT2300A_CUS_PKT17, 0xAA);

    data = read_reg(dev, CMT2300A_CUS_PKT17);
    write_reg(dev, CMT2300A_CUS_PKT17, back);

    if (data == 0xAA) {
        return CMT2300A_SUCCESS;
    }

    return CMT2300A_FAILED;
}

int cmt2300a_go_state(cmt2300a_dev_t *dev, int go_state_mask, int desired_chip_status_mask)
{
    int attempts = LINK_WITH_CHIP_ATTEMPTS;
    int status = CTM2300A_STATE_INVALID;

    write_reg(dev, CMT2300A_CUS_MODE_CTL, go_state_mask);

    while ((status != desired_chip_status_mask) && (attempts > 0)) {
        dev->cmt2300a_ll->delay_us(1000);
        status = cmt2300a_get_state(dev);
        --attempts;
    }
    if (status != desired_chip_status_mask) {
        return CMT2300A_FAILED;
    }

    return CMT2300A_SUCCESS;
}

uint8_t cmt2300a_get_state(cmt2300a_dev_t *dev)
{
    return read_reg(dev, CMT2300A_CUS_MODE_STA) & CMT2300A_MASK_CHIP_MODE_STA;
}

void cmt2300a_set_lfosc(cmt2300a_dev_t *dev, bool enabled)
{
    uint8_t tmp = read_reg(dev, CMT2300A_CUS_SYS2);

    if (enabled) {
        tmp |= CMT2300A_MASK_LFOSC_RECAL_EN;
        tmp |= CMT2300A_MASK_LFOSC_CAL1_EN;
        tmp |= CMT2300A_MASK_LFOSC_CAL2_EN;
    } else {
        tmp &= ~CMT2300A_MASK_LFOSC_RECAL_EN;
        tmp &= ~CMT2300A_MASK_LFOSC_CAL1_EN;
        tmp &= ~CMT2300A_MASK_LFOSC_CAL2_EN;
    }

    write_reg(dev, CMT2300A_CUS_SYS2, tmp);
}

void cmt2300a_set_lfosc_output(cmt2300a_dev_t *dev, bool enabled)
{
    uint8_t tmp = read_reg(dev, CMT2300A_CUS_INT2_CTL);

    if (enabled) {
        tmp |= CMT2300A_MASK_LFOSC_OUT_EN;
    } else {
        tmp &= ~CMT2300A_MASK_LFOSC_OUT_EN;
    }

    write_reg(dev, CMT2300A_CUS_INT2_CTL, tmp);
}

uint8_t cmt2300a_clear_irq_flags(cmt2300a_dev_t *dev)
{
    uint8_t nFlag1 = 0;
    uint8_t nFlag2 = 0;
    uint8_t nClr1 = 0;
    uint8_t nClr2 = 0;
    uint8_t nRet  = 0;
    uint8_t nIntPolar = 0;
    
    nIntPolar = read_reg(dev, CMT2300A_CUS_INT1_CTL);
    nIntPolar = (nIntPolar & CMT2300A_MASK_INT_POLAR) ? 1 :0;

    nFlag1 = read_reg(dev, CMT2300A_CUS_INT_FLAG);
    nFlag2 = read_reg(dev, CMT2300A_CUS_INT_CLR1);
    
    if (nIntPolar) { /* Interrupt flag active-low */
        nFlag1 = ~nFlag1;
        nFlag2 = ~nFlag2;
    }

    if (CMT2300A_MASK_LBD_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_LBD_CLR; /* Clear LBD_FLG */
    }

    if (CMT2300A_MASK_COL_ERR_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR; /* Clear COL_ERR_FLG by PKT_DONE_CLR */
    }

    if (CMT2300A_MASK_PKT_ERR_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR; /* Clear PKT_ERR_FLG by PKT_DONE_CLR */
    }

    if (CMT2300A_MASK_PREAM_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PREAM_OK_CLR; /* Clear PREAM_OK_FLG */
        nRet  |= CMT2300A_MASK_PREAM_OK_FLG; /* Return PREAM_OK_FLG */
    }

    if (CMT2300A_MASK_SYNC_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_SYNC_OK_CLR; /* Clear SYNC_OK_FLG */
        nRet  |= CMT2300A_MASK_SYNC_OK_FLG; /* Return SYNC_OK_FLG */
    }

    if (CMT2300A_MASK_NODE_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_NODE_OK_CLR; /* Clear NODE_OK_FLG */
        nRet  |= CMT2300A_MASK_NODE_OK_FLG; /* Return NODE_OK_FLG */
    }

    if (CMT2300A_MASK_CRC_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_CRC_OK_CLR; /* Clear CRC_OK_FLG */
        nRet  |= CMT2300A_MASK_CRC_OK_FLG; /* Return CRC_OK_FLG */
    }

    if (CMT2300A_MASK_PKT_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR; /* Clear PKT_OK_FLG */
        nRet  |= CMT2300A_MASK_PKT_OK_FLG; /* Return PKT_OK_FLG */
    }

    if (CMT2300A_MASK_SL_TMO_FLG & nFlag2) {
        nClr1 |= CMT2300A_MASK_SL_TMO_CLR; /* Clear SL_TMO_FLG */
        nRet  |= CMT2300A_MASK_SL_TMO_EN; /* Return SL_TMO_FLG by SL_TMO_EN */
    }

    if (CMT2300A_MASK_RX_TMO_FLG & nFlag2) {
        nClr1 |= CMT2300A_MASK_RX_TMO_CLR; /* Clear RX_TMO_FLG */
        nRet  |= CMT2300A_MASK_RX_TMO_EN; /* Return RX_TMO_FLG by RX_TMO_EN */
    }

    if (CMT2300A_MASK_TX_DONE_FLG & nFlag2) {
        nClr1 |= CMT2300A_MASK_TX_DONE_CLR; /* Clear TX_DONE_FLG */
        nRet  |= CMT2300A_MASK_TX_DONE_EN; /* Return TX_DONE_FLG by TX_DONE_EN */
    }
    
    write_reg(dev, CMT2300A_CUS_INT_CLR1, nClr1);
    write_reg(dev, CMT2300A_CUS_INT_CLR2, nClr2);

    if (nIntPolar) { /* Interrupt flag active-low */
        nRet = ~nRet;
    }

    return nRet;
}

static void if_send_byte(cmt2300a_dev_t *dev, uint8_t data8)
{
    for (int i = 0; i < 8; ++i) {
        dev->cmt2300a_ll->sclk_pin_reset();

        /* Send byte on the rising edge of SCL */
        if (data8 & 0x80) {
            dev->cmt2300a_ll->dio_pin_set();
        } else {
            dev->cmt2300a_ll->dio_pin_reset();
        }
        data8 <<= 1;

        dev->cmt2300a_ll->delay_us(IF_DELAY_US);
        dev->cmt2300a_ll->sclk_pin_set();
        dev->cmt2300a_ll->delay_us(IF_DELAY_US);
    }
}

static void write_reg(cmt2300a_dev_t *dev, uint8_t reg, uint8_t data)
{
    dev->cmt2300a_ll->dio_pin_set();
    dev->cmt2300a_ll->sclk_pin_reset();
    dev->cmt2300a_ll->fcsb_pin_set();
    dev->cmt2300a_ll->csb_pin_reset();
    dev->cmt2300a_ll->delay_us(2 * IF_DELAY_US); /* > 0.5 SCL cycle */

    if_send_byte(dev, reg & 0x7F); /* r/w = 0 */
    if_send_byte(dev, data);

    dev->cmt2300a_ll->sclk_pin_reset();
    dev->cmt2300a_ll->delay_us(2 * IF_DELAY_US); /* > 0.5 SCL cycle */

    dev->cmt2300a_ll->csb_pin_set();
    dev->cmt2300a_ll->dio_pin_set();
    dev->cmt2300a_ll->fcsb_pin_set();
}

static uint8_t if_read_byte(cmt2300a_dev_t *dev)
{
    uint8_t data8 = 0xFF;

    for (int i = 0; i < 8; ++i) {
        dev->cmt2300a_ll->sclk_pin_reset();
        dev->cmt2300a_ll->delay_us(IF_DELAY_US);
        data8 <<= 1;

        dev->cmt2300a_ll->sclk_pin_set();

        /* Read byte on the rising edge of SCL */
        if (dev->cmt2300a_ll->dio_read_state()) {
            data8 |= 0x01;
        } else {
            data8 &= ~0x01;
        }

        dev->cmt2300a_ll->delay_us(IF_DELAY_US);
    }

    return data8;
}

static uint8_t read_reg(cmt2300a_dev_t *dev, uint8_t reg)
{
    uint8_t value = 0;

    dev->cmt2300a_ll->dio_pin_set();
    dev->cmt2300a_ll->sclk_pin_reset(); 
    dev->cmt2300a_ll->fcsb_pin_set();
    dev->cmt2300a_ll->csb_pin_reset();
    dev->cmt2300a_ll->delay_us(2 * IF_DELAY_US); /* > 0.5 SCL cycle */

    /* r/w = 1 */
    if_send_byte(dev, reg | 0x80);

    /* Must set SDA to input before the falling edge of SCL */
    value = if_read_byte(dev);

    dev->cmt2300a_ll->sclk_pin_reset();
    dev->cmt2300a_ll->delay_us(2 * IF_DELAY_US); /* > 0.5 SCL cycle */

    dev->cmt2300a_ll->csb_pin_set();
    dev->cmt2300a_ll->dio_pin_set();
    dev->cmt2300a_ll->fcsb_pin_set();

    return value;
}

static void config_reg_bank(cmt2300a_dev_t *dev, uint8_t base_addr, const uint8_t *bank, size_t len)
{
    for (int i = 0; i < len; ++i) {
        write_reg(dev, i + base_addr, bank[i]);
    }
}
