#ifndef __WS2812_PWM_H__
#define __WS2812_PWM_H__

typedef struct ws2812_dma_config {
    uint32_t channel;
    uint32_t slot;
    uint32_t channel_config;
    uint32_t features;
} ws2812_dma_config_t;

#define WS2812_DMA_GET_SOURCE_ADDR_ADJ(channel_config) \
    STM32_DMA_CONFIG_MEMORY_ADDR_INC(channel_config) ? \
    0b00 : 0b01

#define WS2812_DMA_GET_DEST_ADDR_ADJ(channel_config) \
    STM32_DMA_CONFIG_PERIPHERAL_ADDR_INC(channel_config) ? \
    0b00 : 0b01

#endif