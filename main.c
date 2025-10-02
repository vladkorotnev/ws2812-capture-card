#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "ws2812recv.pio.h"

#define LED_PER_STRIP  8
#define STRIP_PER_WEDGE  5
#define WEDGE_PER_MACHINE 12
#define WORDS_IN_FRAMEBUFFER (LED_PER_STRIP * STRIP_PER_WEDGE * WEDGE_PER_MACHINE)
#define LED_FREQ 2000000
// The threshold of L time to determine we are definitely at start of a new frame
#define DWELL_TIME 2048
#define LED_PIN 1

uint _pio_offset = 0;

void ws2812_pio_init(PIO pio, uint sm, uint freq, uint pin) {
    pio_gpio_init(pio, pin);
    pio_gpio_init(pio, pin + 1);
    pio_gpio_init(pio, pin + 2);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, pin + 1, 2, true);

    uint offset = pio_add_program(pio, &ws2812in_program);
    _pio_offset = offset;

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pin_base(&c, pin);
    sm_config_set_in_pin_count(&c, 1);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_sideset_pin_base(&c, pin + 1);
    sm_config_set_sideset(&c, 2, true, false);
    sm_config_set_wrap(&c, offset + ws2812in_wrap_target, offset + ws2812in_wrap);
    sm_config_set_in_shift(&c, false, true, 24);
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);

    unsigned int cycles_per_bit = T1 + T2 + T3;
    float div = clock_get_hz(clk_sys) / (freq * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
}

void ws2812_pio_capture_frame(PIO pio, uint sm, uint dma_channel, uint32_t* dst, size_t count) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    dma_channel_config d = dma_channel_get_default_config(dma_channel);
    channel_config_set_read_increment(&d, false);
    channel_config_set_write_increment(&d, true);
    channel_config_set_dreq(&d, pio_get_dreq(pio, sm, false));
    channel_config_set_high_priority(&d, true);

    dma_channel_configure(
        dma_channel,
        &d,
        dst,
        &pio->rxf[sm],
        count,
        true
    );

    pio_sm_put(pio, sm, DWELL_TIME);
    pio_sm_exec(pio, sm, pio_encode_jmp(_pio_offset));
    pio_sm_set_enabled(pio, sm, true);
}

void transfer_frame(const uint32_t* buffer, size_t count) {
    FILE * f = stdout;
    // send marker
    fwrite("\x55\xEE\xAA", 3, 1, f);
    // send size, 3 bytes per LED
    size_t count_bytes = count * sizeof(uint32_t);
    putc(count_bytes >> 8, f);
    putc(count_bytes & 0xFF, f);
    // send data bytes
    fwrite(buffer, sizeof(uint32_t), count, f);
}

int main()
{
    stdio_init_all();

    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint dma_channel = dma_claim_unused_channel(true);
    uint pin = LED_PIN;
    uint freq = LED_FREQ;
    ws2812_pio_init(pio, sm, freq, pin);

    static uint32_t framebuffer[WORDS_IN_FRAMEBUFFER] = { 0 };
    static uint32_t framebuffer2[WORDS_IN_FRAMEBUFFER] = { 0 };
    bool in_side = false;

    while (true) {
        ws2812_pio_capture_frame(pio, sm, dma_channel, in_side ? framebuffer2 : framebuffer, WORDS_IN_FRAMEBUFFER);
        transfer_frame(in_side ? framebuffer : framebuffer2, WORDS_IN_FRAMEBUFFER);
        dma_channel_wait_for_finish_blocking(dma_channel);
        in_side = !in_side;
    }
}
