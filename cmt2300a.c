#include "cmt2300a.h"
#include "cmt2300a_defs.h"
#include "cmt2300a_params.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#define LINK_WITH_CHIP_ATTEMPTS 20
#define IF_DELAY_US 1

static void     write_reg(cmt2300a_dev_t *dev, uint8_t reg, uint8_t data);
static void     write_fifo(cmt2300a_dev_t *dev, uint8_t *buf, size_t len);
static void     read_fifo(cmt2300a_dev_t *dev, uint8_t *buf, size_t len);
static uint8_t  read_reg(cmt2300a_dev_t *dev, uint8_t reg);
static void     fifo_write_enable(cmt2300a_dev_t *dev);
static void     fifo_read_enable(cmt2300a_dev_t *dev);
static uint8_t  fifo_clear_tx(cmt2300a_dev_t *dev);
static uint8_t  fifo_clear_rx(cmt2300a_dev_t *dev);
static void     config_reg_bank(cmt2300a_dev_t *dev, uint8_t base_addr, const uint8_t *bank, size_t len);

int cmt2300a_init(cmt2300a_dev_t *dev, int mode)
{
    if (!dev || \
        !dev->cmt2300a_ll->delay_us || \
        !dev->cmt2300a_ll->get_tick_ms || \
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

    dev->rx_packet_received_flag = 0;
    dev->tx_done_flag = 0;
    dev->is_fifo_merged = 0;
    dev->rx_fifo_size = 32;
    dev->tx_fifo_size = 32;
    dev->tx_rx_state = RF_STATE_IDLE;
    dev->tx_desired_tick = 0;
    dev->rx_desired_tick = 0;
    dev->rx_buf = NULL;
    dev->rx_buf_len = 0;

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
    (void)cmt2300a_clear_irq_flags(dev, NULL);

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

uint8_t cmt2300a_clear_irq_flags(cmt2300a_dev_t *dev, int *is_col_detected)
{
    uint8_t flag1 = 0;
    uint8_t flag2 = 0;
    uint8_t clr1 = 0;
    uint8_t clr2 = 0;
    uint8_t ret  = 0;
    uint8_t polar = 0;

    polar = read_reg(dev, CMT2300A_CUS_INT1_CTL);
    polar = (polar & CMT2300A_MASK_INT_POLAR) ? 1 : 0;

    flag1 = read_reg(dev, CMT2300A_CUS_INT_FLAG);
    flag2 = read_reg(dev, CMT2300A_CUS_INT_CLR1);

    if (polar) { /* Interrupt flag active-low */
        flag1 = ~flag1;
        flag2 = ~flag2;
    }

    if (CMT2300A_MASK_LBD_FLG & flag1) {
        clr2 |= CMT2300A_MASK_LBD_CLR; /* Clear LBD_FLG */
    }

    if (CMT2300A_MASK_COL_ERR_FLG & flag1) {
        clr2 |= CMT2300A_MASK_PKT_DONE_CLR; /* Clear COL_ERR_FLG by PKT_DONE_CLR */
        if (is_col_detected) {
            *is_col_detected = 1;
        }
    }

    if (CMT2300A_MASK_PKT_ERR_FLG & flag1) {
        clr2 |= CMT2300A_MASK_PKT_DONE_CLR; /* Clear PKT_ERR_FLG by PKT_DONE_CLR */
    }

    if (CMT2300A_MASK_PREAM_OK_FLG & flag1) {
        clr2 |= CMT2300A_MASK_PREAM_OK_CLR; /* Clear PREAM_OK_FLG */
        ret  |= CMT2300A_MASK_PREAM_OK_FLG; /* Return PREAM_OK_FLG */
    }

    if (CMT2300A_MASK_SYNC_OK_FLG & flag1) {
        clr2 |= CMT2300A_MASK_SYNC_OK_CLR; /* Clear SYNC_OK_FLG */
        ret  |= CMT2300A_MASK_SYNC_OK_FLG; /* Return SYNC_OK_FLG */
    }

    if (CMT2300A_MASK_NODE_OK_FLG & flag1) {
        clr2 |= CMT2300A_MASK_NODE_OK_CLR; /* Clear NODE_OK_FLG */
        ret  |= CMT2300A_MASK_NODE_OK_FLG; /* Return NODE_OK_FLG */
    }

    if (CMT2300A_MASK_CRC_OK_FLG & flag1) {
        clr2 |= CMT2300A_MASK_CRC_OK_CLR; /* Clear CRC_OK_FLG */
        ret  |= CMT2300A_MASK_CRC_OK_FLG; /* Return CRC_OK_FLG */
    }

    if (CMT2300A_MASK_PKT_OK_FLG & flag1) {
        clr2 |= CMT2300A_MASK_PKT_DONE_CLR; /* Clear PKT_OK_FLG */
        ret  |= CMT2300A_MASK_PKT_OK_FLG; /* Return PKT_OK_FLG */
    }

    if (CMT2300A_MASK_SL_TMO_FLG & flag2) {
        clr1 |= CMT2300A_MASK_SL_TMO_CLR; /* Clear SL_TMO_FLG */
        ret  |= CMT2300A_MASK_SL_TMO_EN; /* Return SL_TMO_FLG by SL_TMO_EN */
    }

    if (CMT2300A_MASK_RX_TMO_FLG & flag2) {
        clr1 |= CMT2300A_MASK_RX_TMO_CLR; /* Clear RX_TMO_FLG */
        ret  |= CMT2300A_MASK_RX_TMO_EN; /* Return RX_TMO_FLG by RX_TMO_EN */
    }

    if (CMT2300A_MASK_TX_DONE_FLG & flag2) {
        clr1 |= CMT2300A_MASK_TX_DONE_CLR; /* Clear TX_DONE_FLG */
        ret  |= CMT2300A_MASK_TX_DONE_EN; /* Return TX_DONE_FLG by TX_DONE_EN */
    }

    write_reg(dev, CMT2300A_CUS_INT_CLR1, clr1);
    write_reg(dev, CMT2300A_CUS_INT_CLR2, clr2);

    if (polar) { /* Interrupt flag active-low */
        ret = ~ret;
    }

    return ret;
}

int cmt2300a_select_gpio_pins_mode(cmt2300a_dev_t *dev, uint32_t mask)
{
    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    write_reg(dev, CMT2300A_CUS_IO_SEL, mask);

    if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    return CMT2300A_SUCCESS;
}

int cmt2300a_map_gpio_to_irq(cmt2300a_dev_t *dev, uint8_t int1_mapping, uint8_t int2_mapping)
{
    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    int1_mapping &= CMT2300A_MASK_INT1_SEL;
    int1_mapping |= (~CMT2300A_MASK_INT1_SEL) & read_reg(dev, CMT2300A_CUS_INT1_CTL);
    write_reg(dev, CMT2300A_CUS_INT1_CTL, int1_mapping);

    int2_mapping &= CMT2300A_MASK_INT2_SEL;
    int2_mapping |= (~CMT2300A_MASK_INT2_SEL) & read_reg(dev, CMT2300A_CUS_INT2_CTL);
    write_reg(dev, CMT2300A_CUS_INT2_CTL, int2_mapping);

    if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    return CMT2300A_SUCCESS;
}

int cmt2300a_set_irq_polar(cmt2300a_dev_t *dev, bool level)
{
    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    uint8_t tmp = read_reg(dev, CMT2300A_CUS_INT1_CTL);

    if (level) {
        tmp &= ~CMT2300A_MASK_INT_POLAR;
    } else {
        tmp |= CMT2300A_MASK_INT_POLAR;
    }

    write_reg(dev, CMT2300A_CUS_INT1_CTL, tmp);

    if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    return CMT2300A_SUCCESS;
}

int cmt2300a_enable_irq(cmt2300a_dev_t *dev, uint8_t mask)
{
    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    write_reg(dev, CMT2300A_CUS_INT_EN, mask);

    if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    return CMT2300A_SUCCESS;
}

int cmt2300a_set_merge_fifo(cmt2300a_dev_t *dev, bool is_merged)
{
    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    uint8_t tmp = read_reg(dev, CMT2300A_CUS_FIFO_CTL);

    if (is_merged) {
        tmp |= CMT2300A_MASK_FIFO_MERGE_EN;
        dev->is_fifo_merged = 1;
        dev->rx_fifo_size = 64;
        dev->tx_fifo_size = 64;
    } else {
        tmp &= ~CMT2300A_MASK_FIFO_MERGE_EN;
        dev->is_fifo_merged = 0;
        dev->rx_fifo_size = 32;
        dev->tx_fifo_size = 32;
    }

    write_reg(dev, CMT2300A_CUS_FIFO_CTL, tmp);

    if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    return CMT2300A_SUCCESS;
}

int cmt2300a_set_fifo_threshold(cmt2300a_dev_t *dev, uint32_t threshold)
{
    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    uint8_t tmp = read_reg(dev, CMT2300A_CUS_PKT29);

    tmp &= ~CMT2300A_MASK_FIFO_TH;
    tmp |= threshold & CMT2300A_MASK_FIFO_TH;

    write_reg(dev, CMT2300A_CUS_PKT29, tmp);

    if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    return CMT2300A_SUCCESS;
}

int cmt2300a_process(cmt2300a_dev_t *dev)
{
    switch (dev->tx_rx_state) {
    case RF_STATE_TX_WAIT:
    {
        if (dev->tx_done_flag) {
            dev->tx_done_flag = 0;
            dev->tx_rx_state = RF_STATE_IDLE;

            cmt2300a_clear_irq_flags(dev, NULL);
            if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
                return RF_STATE_ERROR;
            }
            return RF_STATE_TX_DONE;
        } else {
            if (dev->cmt2300a_ll->get_tick_ms() > dev->tx_desired_tick) {
                dev->tx_rx_state = RF_STATE_IDLE;
                if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
                    return RF_STATE_ERROR;
                }
                return RF_STATE_TIMEOUT;
            }
        }
        return RF_STATE_TX_WAIT;
    }

    case RF_STATE_RX_WAIT:
    {
        if (dev->rx_packet_received_flag) {
            dev->rx_packet_received_flag = 0;
            dev->tx_rx_state = RF_STATE_IDLE;

            if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
                dev->tx_rx_state = RF_STATE_ERROR;
            }

            read_fifo(dev, dev->rx_buf, dev->rx_buf_len);

            (void)cmt2300a_clear_irq_flags(dev, NULL);
            if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
                dev->tx_rx_state = RF_STATE_ERROR;
            }

            if (dev->tx_rx_state == RF_STATE_ERROR) {
                dev->tx_rx_state = RF_STATE_IDLE;
                return RF_STATE_ERROR;
            }

            return RF_STATE_RX_DONE;
        } else {
            if (dev->cmt2300a_ll->get_tick_ms() > dev->rx_desired_tick) {
                dev->tx_rx_state = RF_STATE_IDLE;
                if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
                    return RF_STATE_ERROR;
                }
                return RF_STATE_TIMEOUT;
            }
        }
        return RF_STATE_RX_WAIT;
    }

    case RF_STATE_IDLE:
        break;
    default:
        break;
    }

    return RF_STATE_IDLE;
}

int cmt2300a_transmit_packet(cmt2300a_dev_t *dev, uint8_t *data_to_tx, size_t data_to_tx_len, uint32_t timeout_ms)
{
    if ((data_to_tx_len > dev->tx_fifo_size) || data_to_tx == NULL) {
        dev->tx_rx_state = RF_STATE_ERROR;
        return CMT2300A_FAILED;
    }

    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        dev->tx_rx_state = RF_STATE_ERROR;
        return CMT2300A_FAILED;
    }

    (void)cmt2300a_clear_irq_flags(dev, NULL);

    /* Transmitting approach is:
     *  - Fill in len register (CMT2300A_CUS_PKT15)
     *  - Fill in tx fifo
     *  - Transmit data
     * 
     * Therefore only length not more than 'tx_fifo_size' is supported for now.
     */
    write_reg(dev, CMT2300A_CUS_PKT15, data_to_tx_len);

    /* Must clear FIFO after enable SPI to read or write the FIFO */
    fifo_write_enable(dev);
    (void)fifo_clear_tx(dev);

    write_fifo(dev, data_to_tx, data_to_tx_len);
    dev->tx_done_flag = 0;

    if (cmt2300a_go_state(dev, CMT2300A_GO_TX, CTM2300A_STATE_TX) != CMT2300A_SUCCESS) {
        dev->tx_rx_state = RF_STATE_ERROR;
        return CMT2300A_FAILED;
    }

    dev->tx_desired_tick = dev->cmt2300a_ll->get_tick_ms() + timeout_ms;
    dev->tx_rx_state = RF_STATE_TX_WAIT;

    return CMT2300A_SUCCESS;
}

int cmt2300a_receive_packet(cmt2300a_dev_t *dev, uint8_t *place_to_rx, size_t place_to_rx_len, uint32_t timeout_ms)
{
    if ((place_to_rx_len > dev->rx_fifo_size) || place_to_rx == NULL) {
        dev->tx_rx_state = RF_STATE_ERROR;
        return CMT2300A_FAILED;
    }

    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        dev->tx_rx_state = RF_STATE_ERROR;
        return CMT2300A_FAILED;
    }

    (void)cmt2300a_clear_irq_flags(dev, NULL);

    /* Must clear FIFO after enable SPI to read or write the FIFO */
    fifo_read_enable(dev);
    (void)fifo_clear_rx(dev);

    dev->rx_packet_received_flag = 0;
    dev->rx_buf = place_to_rx;
    dev->rx_buf_len = place_to_rx_len;

    if (cmt2300a_go_state(dev, CMT2300A_GO_RX, CTM2300A_STATE_RX) != CMT2300A_SUCCESS) {
        dev->tx_rx_state = RF_STATE_ERROR;
        return CMT2300A_FAILED;
    }

    dev->rx_desired_tick = dev->cmt2300a_ll->get_tick_ms() + timeout_ms;
    dev->tx_rx_state = RF_STATE_RX_WAIT;

    return CMT2300A_SUCCESS;
}

void cmt2300a_abort_any_operations(cmt2300a_dev_t *dev)
{
    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY); /* just try one more time */
    }

    dev->cmt2300a_ll->delay_us(20);
    dev->rx_packet_received_flag = 0;
    dev->tx_done_flag = 0;
    dev->tx_rx_state = RF_STATE_IDLE;
}

int cmt2300a_set_node_id(cmt2300a_dev_t *dev, uint32_t node_id)
{
    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    write_reg(dev, 0x48, (uint8_t)node_id);
    write_reg(dev, 0x49, (uint8_t)(node_id >> 8));
    write_reg(dev, 0x4A, (uint8_t)(node_id >> 16));
    write_reg(dev, 0x4B, (uint8_t)(node_id >> 24));

    if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    return CMT2300A_SUCCESS;
}

int cmt2300a_allow_receiving_any_nodeid(cmt2300a_dev_t *dev, bool is_allowed)
{
    if (cmt2300a_go_state(dev, CMT2300A_GO_STBY, CTM2300A_STATE_STBY) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    uint8_t reg = read_reg(dev, 0x47);
    if (!is_allowed) {
        reg &= ~CMT2300A_MASK_NODE_ERR_MASK;
    } else {
        reg |= CMT2300A_MASK_NODE_ERR_MASK;
    }

    write_reg(dev, 0x47, reg);

    if (cmt2300a_go_state(dev, CMT2300A_GO_SLEEP, CTM2300A_STATE_SLEEP) != CMT2300A_SUCCESS) {
        return CMT2300A_FAILED;
    }

    return CMT2300A_SUCCESS;
}

void cmt2300a_tx_done_isr(cmt2300a_dev_t *dev)
{
    dev->tx_done_flag = 1;
}

void cmt2300a_rx_packet_isr(cmt2300a_dev_t *dev)
{
    dev->rx_packet_received_flag = 1;
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

static void write_fifo(cmt2300a_dev_t *dev, uint8_t *buf, size_t len)
{
    dev->cmt2300a_ll->fcsb_pin_set();
    dev->cmt2300a_ll->csb_pin_set();
    dev->cmt2300a_ll->sclk_pin_reset();

    for (int i = 0; i < len; ++i) {
        dev->cmt2300a_ll->fcsb_pin_reset();

        /* > 1 SCL cycle */
        dev->cmt2300a_ll->delay_us(3 * IF_DELAY_US);

        if_send_byte(dev, buf[i]);
        dev->cmt2300a_ll->sclk_pin_reset();

        /* > 2 us */
        dev->cmt2300a_ll->delay_us(4 * IF_DELAY_US);
        dev->cmt2300a_ll->fcsb_pin_set();

        /* > 4 us */
        dev->cmt2300a_ll->delay_us(6 * IF_DELAY_US);
    }

    dev->cmt2300a_ll->fcsb_pin_set();
}

static void read_fifo(cmt2300a_dev_t *dev, uint8_t *buf, size_t len)
{
    dev->cmt2300a_ll->fcsb_pin_set();
    dev->cmt2300a_ll->csb_pin_set();
    dev->cmt2300a_ll->sclk_pin_reset();

    for (int i = 0; i < len; ++i) {
        dev->cmt2300a_ll->fcsb_pin_reset();

        /* > 1 SCL cycle */
        dev->cmt2300a_ll->delay_us(3 * IF_DELAY_US);

        buf[i] = if_read_byte(dev);
        dev->cmt2300a_ll->sclk_pin_reset();

        /* > 2 us */
        dev->cmt2300a_ll->delay_us(4 * IF_DELAY_US);
        dev->cmt2300a_ll->fcsb_pin_set();

        /* > 4 us */
        dev->cmt2300a_ll->delay_us(6 * IF_DELAY_US);
    }

    dev->cmt2300a_ll->fcsb_pin_set();
}

static void fifo_write_enable(cmt2300a_dev_t *dev)
{
    uint8_t tmp = read_reg(dev, CMT2300A_CUS_FIFO_CTL);
    tmp |= CMT2300A_MASK_SPI_FIFO_RD_WR_SEL;
    if (dev->is_fifo_merged) {
        tmp |= CMT2300A_MASK_FIFO_RX_TX_SEL;
    }
    write_reg(dev, CMT2300A_CUS_FIFO_CTL, tmp);
}

static void fifo_read_enable(cmt2300a_dev_t *dev)
{
    uint8_t tmp = read_reg(dev, CMT2300A_CUS_FIFO_CTL);
    tmp &= ~CMT2300A_MASK_SPI_FIFO_RD_WR_SEL;
    if (dev->is_fifo_merged) {
        tmp &= ~CMT2300A_MASK_FIFO_RX_TX_SEL;
    }
    write_reg(dev, CMT2300A_CUS_FIFO_CTL, tmp);
}

static uint8_t fifo_clear_tx(cmt2300a_dev_t *dev)
{
    uint8_t tmp = read_reg(dev, CMT2300A_CUS_FIFO_FLAG);
    write_reg(dev, CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_TX);
    return tmp;
}

static uint8_t fifo_clear_rx(cmt2300a_dev_t *dev)
{
    uint8_t tmp = read_reg(dev, CMT2300A_CUS_FIFO_FLAG);
    write_reg(dev, CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_RX);
    return tmp;
}

static void config_reg_bank(cmt2300a_dev_t *dev, uint8_t base_addr, const uint8_t *bank, size_t len)
{
    for (int i = 0; i < len; ++i) {
        write_reg(dev, i + base_addr, bank[i]);
    }
}
