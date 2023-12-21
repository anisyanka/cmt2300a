#ifndef __CMT2300_H
#define __CMT2300_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    CMT2300_433FREQ_MODE,
    CMT2300_868FREQ_MODE,
} cmt2300_modes_t;

typedef enum {
    CMT2300_SUCCESS,
    CMT2300_FAILED,
} cmt2300_returns_t;

/* Low level functions for radio chip pins managing */
typedef struct {
    void (*delay_us)(void);

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
    int (*dio_read_state)(void);
} cmt2300_ll_t;

int cmt2300_init(int mode);

#ifdef __cplusplus
}
#endif

#endif /* __CMT2300_H */
