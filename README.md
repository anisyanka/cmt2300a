#  CMT2300A Radio tranciever library (RFM300W module)

Radio communication hardware independent library for cmt2300a chip.

Just implement low level functions for your microcontroller and use the lib.
```c
typedef struct {
    void (*delay_us)(uint32_t us);
    uint32_t (*get_tick_ms)(void);

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
```

**Other features:**
 - Supports the ability to setup device address (NODE ID)
```c
/*
 * is_allowed = false - obtain packets only with our node_id
 * is_allowed = true - obtain any node_id
 */
int cmt2300a_allow_receiving_any_nodeid(cmt2300a_dev_t *dev, bool is_allowed);

/* Radio chip will receive or send data ONLY from/with this node_id */
int cmt2300a_set_node_id(cmt2300a_dev_t *dev, uint32_t node_id);
```

 - Supports for async communication. Just call these functions in appropriate interrupt handlers:
```c
void cmt2300a_tx_done_isr(cmt2300a_dev_t *dev); /* must be called in gpio TX_DONE isr */
void cmt2300a_rx_packet_isr(cmt2300a_dev_t *dev); /* must be called in gpio PKT_OK isr */
```
But remember, first of all you have to setup the reasons when the IRQs will be triggered inside radio chip.
See `cmt2300a_map_gpio_to_irq()`, `cmt2300a_set_irq_polar()` and `cmt2300a_enable_irq()`.
After that you can just use `cmt2300a_process()` in your radio thread or while loop.
The function will return one of the next states:
```c
typedef enum {
    RF_STATE_IDLE = 0, /* Initial state after cmt2300a_init() */
    RF_STATE_RX_WAIT, /* Receiving started but not finished */
    RF_STATE_RX_DONE, /* Packet received and check crc-ok flag is OK. Remember you have to use GPIO PKT_OK interrupt and call cmt2300a_rx_packet_isr() in your ISR */
    RF_STATE_TX_WAIT, /* Transmitting started but not finished */
    RF_STATE_TX_DONE, /* Transmitting fully done with no errors. Remember you have to use GPIO TX_DONE interrupt and call cmt2300a_tx_done_isr() in your ISR */
    RF_STATE_TIMEOUT, /* Transmitting or receiving timve exceeded timeout passed to cmt2300a_transmit_packet() or cmt2300a_receive_packet() functions  */
    RF_STATE_CRC_ERROR, /* Packet received, but crc calculated by the chip is failed */
    RF_STATE_ERROR, /* Something bad happend. For instance chip didn't change your state from STDBY to TX or RX. Probably something happend in you SCK\DIO lines. Use cmt2300a_soft_reset() after that event */
} cmt2300a_tx_rx_states_t;
```

# Example of usage
```c
static cmt2300a_ll_t cmt2300a_ll = { 0 };
static cmt2300a_dev_t cmt2300a_dev = { 0 };

void radio_thread_create(void)
{
    static StaticTask_t radio_tcb;
    static StackType_t radio_stack[RADIO_TASK_STACK_SIZE_WORDS];

    cmt2300a_ll.delay_us = no_os_radio_timer_delay_us; /* this func pointer is implemented by user */
    cmt2300a_ll.get_tick_ms = get_ms_common_tick; /* this func pointer is implemented by user */
    cmt2300a_ll.csb_pin_set = radio_csb_pin_set; /* this func pointer is implemented by user */
    cmt2300a_ll.csb_pin_reset = radio_csb_pin_reset; /* this func pointer is implemented by user */
    cmt2300a_ll.dio_pin_set = radio_dio_pin_set; /* this func pointer is implemented by user */
    cmt2300a_ll.dio_pin_reset = radio_dio_pin_reset; /* this func pointer is implemented by user */
    cmt2300a_ll.dio_read_state = radio_dio_read_state; /* this func pointer is implemented by user */
    cmt2300a_ll.fcsb_pin_set = radio_fcsb_pin_set; /* this func pointer is implemented by user */
    cmt2300a_ll.fcsb_pin_reset = radio_fcsb_pin_reset; /* this func pointer is implemented by user */
    cmt2300a_ll.sclk_pin_set = radio_sclk_pin_set; /* this func pointer is implemented by user */
    cmt2300a_ll.sclk_pin_reset = radio_sclk_pin_reset; /* this func pointer is implemented by user */
    cmt2300a_dev.cmt2300a_ll = &cmt2300a_ll;

    radio_task_handle = xTaskCreateStatic(radio_thread, "Radio", RADIO_TASK_STACK_SIZE_WORDS, NULL, RADIO_TASK_PRIO, radio_stack, &radio_tcb);
    if (radio_task_handle == NULL) {
        error_handler(/* err code */);
    }

    if (cmt2300a_init(&cmt2300a_dev, CMT2300_433FREQ_MODE) != CMT2300A_SUCCESS) {
        error_handler(/* err code */);
    }

    /*
        RF_GPIO2_Pin(EXTI2_IRQn) --> INT1; --> CMT2300A_INT_SEL_TX_DONE
        RF_GPIO3_Pin(EXTI1_IRQn) --> INT2; --> CMT2300A_INT_SEL_PKT_OK
    */
    if (cmt2300a_select_gpio_pins_mode(&cmt2300a_dev, CMT2300A_GPIO2_SEL_INT1 | CMT2300A_GPIO3_SEL_INT2) != CMT2300A_SUCCESS) {
        error_handler(/* err code */);
    }

    if (cmt2300a_map_gpio_to_irq(&cmt2300a_dev, CMT2300A_INT_SEL_TX_DONE, CMT2300A_INT_SEL_PKT_OK) != CMT2300A_SUCCESS) {
        error_handler(/* err code */);
    }

    if (cmt2300a_set_irq_polar(&cmt2300a_dev, true) != CMT2300A_SUCCESS) {
        error_handler(/* err code */);
    }

    /* Recieve PKT_DONE and check CRC flag */
    if (cmt2300a_enable_irq(&cmt2300a_dev,
                            CMT2300A_MASK_TX_DONE_EN |
                            /* CMT2300A_MASK_PREAM_OK_EN | */
                            /* CMT2300A_MASK_SYNC_OK_EN | */
                            /* CMT2300A_MASK_NODE_OK_EN | */
                            CMT2300A_MASK_CRC_OK_EN |
                            CMT2300A_MASK_PKT_DONE_EN) != CMT2300A_SUCCESS) {
        error_handler(/* err code */);
    }

    if (cmt2300a_set_merge_fifo(&cmt2300a_dev, false) != CMT2300A_SUCCESS) {
        error_handler(/* err code */);
    }

    if (cmt2300a_set_fifo_threshold(&cmt2300a_dev, 32) != CMT2300A_SUCCESS) {
        error_handler(/* err code */);
    }

    if (cmt2300a_allow_receiving_any_nodeid(&cmt2300a_dev, false) != CMT2300A_SUCCESS) {
        error_handler(/* err code */);
    }

    uint32_t node_id = 0; /* GET YOUR RADIO NODE ID HERE */
    if (cmt2300a_set_node_id(&cmt2300a_dev, node_id) != CMT2300A_SUCCESS) {
        error_handler(/* err code */);
    }

    if (cmt2300a_is_chip_exist(&cmt2300a_dev) != CMT2300A_SUCCESS) {
        error_handler(/* err code */);
    } else {
        logger_dgb_print("[RF] CMT2300A radio chip is ready to use\n");
    }
}

static void radio_thread(void *params)
{
    (void)params;
    int radio_state = RF_STATE_IDLE;

    for (;;) {
        radio_state = cmt2300a_process(&cmt2300a_dev);
        if ((radio_state == RF_STATE_TIMEOUT) || (radio_state == RF_STATE_ERROR) || (radio_state == RF_STATE_CRC_ERROR)) {
            logger_dgb_print("[RF] ERROR OCCURED\n");
        } else {
            if (radio_state == RF_STATE_TX_DONE) {
                logger_dgb_print("[RF] Tx done successfully!\n");
            }
            if (radio_state == RF_STATE_RX_DONE) {
                logger_dgb_print("[RF] Rx done successfully!\n");
            }
        }
    }
}


/* <------------------> Some code in other place <------------------ */
cmt2300a_transmit_packet(&cmt2300a_dev, data_to_send, len_to_send, 200); <--- Non BLOCKING function. You receive event in thread when Tx will be finished.


/* <------------------> Some code in other place <------------------ */
static uint8_t data_to_recv[] = { ..... }; /* the array must be in scope */
cmt2300a_receive_packet(&cmt2300a_dev, data_to_recv, len_to_recv, 200); <--- Non BLOCKING function. You receive event in thread when Rx will be finished.
```