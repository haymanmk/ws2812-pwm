#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ws2812_pwm, CONFIG_WS2812_PWM_LOG_LEVEL);

#define DT_DRV_COMPAT worldsemi_ws2812_pwm

// check if the devicetree contains any devices with the driver compatible
#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#error "Custom BME280 driver enabled without any devices"
#endif

/* data */
struct ws2812_pwm_data {
    uint8_t flags;
};

/* configuration */
struct ws2812_pwm_config {
    const struct device *dev;
    const size_t num_leds;
};

/* function to initialize the driver */
static int ws2812_pwm_init(const struct device *dev)
{
    // const struct ws2812_pwm_config *config = dev->config;
    struct ws2812_pwm_data *data = dev->data;

    // if (!device_is_ready(config->dev)) {
    //     LOG_ERR("Device %s is not ready", config->dev->name);
    //     return -ENODEV;
    // }

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
        .num_leds = DT_INST_PROP(inst, chain_length), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, &ws2812_pwm_init, NULL, \
        &ws2812_pwm_data_##inst, &ws2812_pwm_config_##inst, \
        POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY, &ws2812_pwm_api);

/* Create the struct device for every status "okay" node in the devicetree */
DT_INST_FOREACH_STATUS_OKAY(WS2812_PWM_DEFINE)