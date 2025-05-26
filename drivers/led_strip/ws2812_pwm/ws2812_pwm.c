#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ws2812_pwm, CONFIG_WS2812_PWM_LOG_LEVEL);

#include "ws2812_pwm.h"

#define DT_DRV_COMPAT worldsemi_ws2812_pwm

// check if the devicetree contains any devices with the driver compatible
#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#error "Custom BME280 driver enabled without any devices"
#endif

static struct dma_block_config block_cfg[2];

/* data */
struct ws2812_pwm_data {
    uint8_t flags;
};

/* configuration */
struct ws2812_pwm_config {
    const struct device *dev;
    const struct device *dma_dev;
    const ws2812_dma_config_t dma_config;
    const size_t num_leds; // can be configured in the devicetree
                           // by the chain_length property
};

/* function to initialize the driver */
static int ws2812_pwm_init(const struct device *dev)
{
    // const struct ws2812_pwm_config *config = dev->config;
    struct ws2812_pwm_data *data = dev->data;
    const struct ws2812_pwm_config *ws2812_cfg = dev->config;
    const struct device *dma_dev = ws2812_cfg->dma_dev;
    const char *dma_dev_name = dma_dev->name;
    // const struct dma_stm32_config *stm32_dma_cfg = dma_dev->config;

    LOG_DBG("Initializing WS2812 PWM driver for %s", dma_dev_name);
    // Check if the DMA device is ready
    if (!device_is_ready(dma_dev)) {
        LOG_WRN("DMA device %s is not ready", dma_dev_name);
        return -ENODEV;
    }

    // configure DMA
    const ws2812_dma_config_t dma_cfg = ws2812_cfg->dma_config;
    LOG_DBG("Configuring DMA: channel=%d, slot=%d, channel_config=0x%x, features=0x%x",
            dma_cfg.channel, dma_cfg.slot, dma_cfg.channel_config, dma_cfg.features);
    block_cfg[0] = {
        .source_address = ,
        .dest_address = ,
        .block_size = sizeof(),
        .next_block = &block_cfg[1],
        .source_addr_adj = WS2812_DMA_GET_SOURCE_ADDR_ADJ(dma_cfg.channel_config),
        .dest_addr_adj = WS2812_DMA_GET_DEST_ADDR_ADJ(dma_cfg.channel_config),
        .fifo_mode_control = STM32_DMA_FEATURES_FIFO_THRESHOLD(dma_cfg.features),
    };
    struct dma_config dma_cfg = {
        .dma_slot = dma_cfg.slot,
        .channel_direction = MEMORY_TO_PERIPHERAL,
        .complete_callback_en = true,
        .channel_priority = STM32_DMA_CONFIG_PRIORITY(dma_cfg.channel_config),
        .source_data_size = STM32_DMA_CONFIG_MEMORY_DATA_SIZE(dma_cfg.channel_config),
        .dest_data_size = STM32_DMA_CONFIG_PERIPHERAL_DATA_SIZE(dma_cfg.channel_config),
        .block_count = 2,
        .head_block = &block_cfg[0],
    };

    data->flags = 0;

    return 0;
}

/* define the driver APIs */
static int ws2812_pwm_update_rgb(const struct device *dev, 
                                struct led_rgb *rgb, 
                                size_t num_leds)
{
    // const struct ws2812_pwm_config *config = dev->config;
    // struct ws2812_pwm_data *data = dev->data;

    return 0;
}

static size_t ws2812_pwm_length(const struct device *dev)
{
    const struct ws2812_pwm_config *config = dev->config;
    
    return config->num_leds;
}

// assign the driver APIs to the led_strip class
static DEVICE_API(led_strip, ws2812_pwm_api) = {
    .update_rgb = ws2812_pwm_update_rgb,
    .length = ws2812_pwm_length,
};

/* helper macro to define a new driver instance */
#define WS2812_PWM_DEFINE(inst) \
    static struct ws2812_pwm_data ws2812_pwm_data_##inst; \
    static const struct ws2812_pwm_config ws2812_pwm_config_##inst = { \
        .dev = DEVICE_DT_GET(DT_DRV_INST(inst)), \
        .dma_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, dmas)), \
        .dma_config = { \
            .slot = DT_INST_PHA(inst, dmas, slot), \
            .channel = DT_INST_PHA(inst, dmas, channel), \
            .channel_config = DT_INST_PHA(inst, dmas, channel_config), \
            .features = DT_INST_PHA(inst, dmas, features), \
        }, \
        .num_leds = DT_INST_PROP(inst, chain_length), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, &ws2812_pwm_init, NULL, \
        &ws2812_pwm_data_##inst, &ws2812_pwm_config_##inst, \
        POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY, &ws2812_pwm_api);

/* Create the struct device for every status "okay" node in the devicetree */
DT_INST_FOREACH_STATUS_OKAY(WS2812_PWM_DEFINE)