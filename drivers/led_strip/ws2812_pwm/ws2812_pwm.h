#ifndef __WS2812_PWM_H__
#define __WS2812_PWM_H__

/**
 * TODO
 */
#define DUTY_CYCLE_HIGH_BIT 76 // unit: ticks, 0.7us
#define DUTY_CYCLE_LOW_BIT 38 // 0.35us

// used to record values in phandle properties
typedef struct ws2812_dma_pha_config {
    uint32_t channel;
    uint32_t slot;
    uint32_t channel_config;
    uint32_t features;
} ws2812_dma_pha_config_t;

// used to record values in phandle properties
typedef struct ws2812_pwm_pha_config {
    uint32_t channel;
    uint32_t period; // in nsec
    uint32_t flags;
} ws2812_pwm_pha_config_t;

#define WS2812_DMA_GET_SOURCE_ADDR_ADJ(channel_config) \
    STM32_DMA_CONFIG_MEMORY_ADDR_INC(channel_config) ? \
    0b00 : 0b01

#define WS2812_DMA_GET_DEST_ADDR_ADJ(channel_config) \
    STM32_DMA_CONFIG_PERIPHERAL_ADDR_INC(channel_config) ? \
    0b00 : 0b10

#define WS2812_DMA_GET_TIM_CCR(CHANNEL, TIM) \
    (CHANNEL == 1 ? (uint32_t)&TIM->CCR1 : \
     CHANNEL == 2 ? (uint32_t)&TIM->CCR2 : \
     CHANNEL == 3 ? (uint32_t)&TIM->CCR3 : \
     CHANNEL == 4 ? (uint32_t)&TIM->CCR4 : 0)

#endif