#include <soc.h>
#include <math.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ws2812_pwm, CONFIG_WS2812_PWM_LOG_LEVEL);

#include "ws2812_pwm.h"

#define DT_DRV_COMPAT worldsemi_ws2812_pwm

// check if the devicetree contains any devices with the driver compatible
#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#error "Custom WS2812 PWM driver enabled without any devices"
#endif

/* define control flags */
#define WS2812_PWM_FLAG_BUSY BIT(0) // a lock to indicate that the driver is busy
#define WS2812_PWM_FLAG_DMA_ENABLED BIT(1) // DMA is enabled
#define WS2812_PWM_FLAG_DMA_ACTIVE BIT(2) // DMA is currently active
#define WS2812_PWM_FLAG_TASK_STOP_PWM BIT(3) // a flag to signal that the PWM task should be stopped
#define WS2812_PWM_FLAG_TASK_SEND_RESET BIT(4) // a flag to signal that the reset signal should be sent
                                               // this is used to reset the LED strip after the last update
#define WS2812_PWM_FLAG_TASK_UPDATE_BUFFER BIT(5) // a flag to signal that the PWM buffer should be updated

#define COLOR_SIZE_BYTES 3 // RGB
#define BATCH_UPDATE_SIZE 5 // 5 LEDs per DMA block
#define TIMER_MAX_CHANNELS 4 // Number of channels in the timer

// Each pulse populates 32 bits for the duty cycle, each color contains 8 bits,
// and we have 3 colors (RGB), so each LED requires 24 bits.
// note: 2 means two buffers, 8 means 8 bits per byte
#define HALF_BLOCK_SIZE_WORDS (BATCH_UPDATE_SIZE * COLOR_SIZE_BYTES * 8)
#define BLOCK_SIZE_WORDS (2 * HALF_BLOCK_SIZE_WORDS) // 2 buffers for DMA transfer
// lock to prevent concurrent access
#define __WS2812_PWM_LOCK(dev) \
    do { \
        struct ws2812_pwm_data *data = dev->data; \
        if (data->flags & WS2812_PWM_FLAG_BUSY) { \
            LOG_ERR("WS2812 PWM driver is busy"); \
            return -EBUSY; \
        } \
        data->flags |= WS2812_PWM_FLAG_BUSY; \
    } while (0)
#define __WS2812_PWM_UNLOCK(dev) \
    do { \
        struct ws2812_pwm_data *data = dev->data; \
        data->flags &= ~WS2812_PWM_FLAG_BUSY; \
    } while (0)


/* private function prototype */
static int ws2812_pwm_start(const struct device *dev);
static int ws2812_pwm_stop(const struct device *dev);
static int ws2812_pwm_update_buffer(const struct device *dev, 
                                uint16_t index_led,
                                size_t num_leds);
static void ws2812_pwm_reset_signal(const struct device *dev);


/**
 * @brief Create degug output pin to reveal certain information
 */
static const struct gpio_dt_spec debug_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(usr_dbg), gpios);

/* data */
struct ws2812_pwm_data {
    volatile uint8_t flags;
    struct led_rgb *led_data; // pointer to the led RGB data
    uint32_t pwm_buffer[BLOCK_SIZE_WORDS]; // buffer for the PWM data
    struct dma_config dma_cfg; // DMA configuration
    struct dma_block_config block_cfg; // DMA block configuration
    size_t num_leds; // number of LEDs in the chain
    volatile uint16_t num_led_buffer_updated; // number of LED whose PWM buffer has been updated
    // Note: the buffer is updated in batches of BATCH_UPDATE_SIZE LEDs
    volatile uint16_t num_reset_isr_issued; // number of reset ISR issued
    // Note: this is used to count the number of ISR issued for the reset signal
    //       the reset signal is sent after a certain number of ISR.
    //       during the reset signal, the PWM buffer is reset to zero.
    uint16_t total_num_reset_isr; // total number of reset ISR to be issued
    uint64_t pwm_freq; // PWM clock frequency in Hz
    uint32_t period; // PWM period in ticks, be written in auto-reload register
    uint32_t delay_t1h; // delay for the high pulse in ticks
    uint32_t delay_t0h; // delay for the low pulse in ticks
};

/* configuration */
struct ws2812_pwm_config {
    const struct device *dma_dev;
    const ws2812_dma_pha_config_t dma_config;
    const struct device *pwm_dev;
    const ws2812_pwm_pha_config_t pwm_config;
    const size_t num_leds; // can be configured in the devicetree
                           // by the chain_length property
    const uint8_t color_mapping[3]; // RGB order
    const uint32_t reset_delay_us; // reset signal duration in microseconds
    const uint32_t delay_t1h_ns; // delay for the high pulse in nsec
    const uint32_t delay_t0h_ns; // delay for the low pulse in nsec
};

/* PWM driver configuration copied from pwm_stm32.c */
struct pwm_stm32_config {
	TIM_TypeDef *timer;
	uint32_t prescaler;
	uint32_t countermode;
	struct stm32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
};

// channel to LL mapping
static const uint32_t channel_to_ll[TIMER_MAX_CHANNELS] = {
    LL_TIM_CHANNEL_CH1,
    LL_TIM_CHANNEL_CH2,
    LL_TIM_CHANNEL_CH3,
    LL_TIM_CHANNEL_CH4,
};

// set compare register for the timer channel
static void (*const set_timer_compare[TIMER_MAX_CHANNELS])(TIM_TypeDef *,
						     uint32_t) = {
	LL_TIM_OC_SetCompareCH1, LL_TIM_OC_SetCompareCH2,
	LL_TIM_OC_SetCompareCH3, LL_TIM_OC_SetCompareCH4,
};

// enable the DMA request for the timer channel
static void (*const enable_dma_request[TIMER_MAX_CHANNELS])(TIM_TypeDef *) = {
    LL_TIM_EnableDMAReq_CC1, LL_TIM_EnableDMAReq_CC2,
    LL_TIM_EnableDMAReq_CC3, LL_TIM_EnableDMAReq_CC4,
};

// disable the DMA request for the timer channel
static void (*const disable_dma_request[TIMER_MAX_CHANNELS])(TIM_TypeDef *) = {
    LL_TIM_DisableDMAReq_CC1, LL_TIM_DisableDMAReq_CC2,
    LL_TIM_DisableDMAReq_CC3, LL_TIM_DisableDMAReq_CC4,
};

// clear interrupt flags for the timer channel
static void (*const clear_interrupt_flags[TIMER_MAX_CHANNELS])(TIM_TypeDef *) = {
    LL_TIM_ClearFlag_CC1, LL_TIM_ClearFlag_CC2,
    LL_TIM_ClearFlag_CC3, LL_TIM_ClearFlag_CC4,
};

/* DMA transfer complete callback */
static void ws2812_pwm_dma_tc_callback(const struct device *dev, 
                                       void *user_data,
                                       uint32_t channel,
                                       int status)
{
    // LOG_DBG("DMA transfer complete callback for channel %d with status %d", channel, status);

    // create atomic operation by locking the IRQ
    // int key = irq_lock();
    __disable_irq();
    
    /**
     * Turn on the debug pin to reveal certain information
     */
    gpio_pin_set_dt(&debug_pin, 1);

    const struct device *ws2812_dev = (const struct device *)user_data;
    struct ws2812_pwm_data *ws2812_data = ws2812_dev->data;
    uint8_t op_flags = ws2812_data->flags; // get the operation flags

    // disable timer channel to prevent further updates

    // check if DMA transfer should be stopped
    if (op_flags & WS2812_PWM_FLAG_TASK_STOP_PWM)
    {
        // stop the DMA transfer
        if (ws2812_pwm_stop(ws2812_dev) != 0)
        {
            LOG_ERR("Failed to stop DMA transfer");
            goto exit;
        };

        // clear the flag for the operation of the LED strip
        ws2812_data->flags &= ~(WS2812_PWM_FLAG_TASK_UPDATE_BUFFER | WS2812_PWM_FLAG_TASK_STOP_PWM | WS2812_PWM_FLAG_TASK_SEND_RESET);

        // clear the count of the reset signal ISRs
        ws2812_data->num_reset_isr_issued = 0;

        // clear the number of LED buffer updated
        ws2812_data->num_led_buffer_updated = 0;
    }
    // check if the reset signal should be sent
    else if (op_flags & WS2812_PWM_FLAG_TASK_SEND_RESET)
    {
        ws2812_pwm_reset_signal(ws2812_dev);

        // increment the count of ISR for the reset signal
        ws2812_data->num_reset_isr_issued++;

        if (ws2812_data->num_reset_isr_issued >= ws2812_data->total_num_reset_isr)
        {
            // set the flag for the operation of the LED strip
            ws2812_data->flags |= WS2812_PWM_FLAG_TASK_STOP_PWM;
        }
    }
    else
    {
        // update the buffer for the PWM data
        ws2812_pwm_update_buffer(ws2812_dev, ws2812_data->num_led_buffer_updated, BATCH_UPDATE_SIZE);

        // increment the number of LED buffer updated
        ws2812_data->num_led_buffer_updated += BATCH_UPDATE_SIZE;

        // check if all the LED buffers have been updated
        if (ws2812_data->num_led_buffer_updated >= ws2812_data->num_leds)
        {
            // set the flag for the operation of the LED strip
            ws2812_data->flags |= WS2812_PWM_FLAG_TASK_SEND_RESET;
        }
    }

exit:
    /**
     * Turn off the debug pin to reveal certain information
     */
    gpio_pin_set_dt(&debug_pin, 0);

    // enable irq after the operation
    // irq_unlock(key);
    __enable_irq();
}

/**
 * @brief Setup PWM and enable DMA request
 * @param dev Pointer to the device structure
 * @return 0 on success, negative error code on failure
 * @note This function configures the PWM peripheral to generate the required
 *       signal for WS2812 LEDs and enables the DMA request.
 */
static int ws2812_pwm_setup_pwm(const struct device *dev)
{
    const struct ws2812_pwm_config *config = dev->config;
    const struct ws2812_pwm_data *data = dev->data;
    const struct device *pwm_dev = config->pwm_dev;
    const struct pwm_stm32_config *pwm_cfg = pwm_dev->config;
    const ws2812_pwm_pha_config_t *pwm_pha_cfg = &config->pwm_config;

    // Check if the PWM device is ready
    if (!device_is_ready(pwm_dev)) {
        LOG_ERR("PWM device %s is not ready", pwm_dev->name);
        return -ENODEV;
    }

    // lock the driver to prevent concurrent access
    __WS2812_PWM_LOCK(dev);

    // Retrieve Timer handler
    TIM_TypeDef *tim = pwm_cfg->timer;
    uint32_t ll_channel = channel_to_ll[pwm_pha_cfg->channel - 1];

    // disable the timer channel before configuration
    LL_TIM_CC_DisableChannel(tim, ll_channel);
    // diable the timer
    LL_TIM_DisableCounter(tim);

    // Configure the timer for PWM mode
    LL_TIM_SetCounterMode(tim, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetAutoReload(tim, data->period);
    LL_TIM_EnableARRPreload(tim);
    LL_TIM_OC_EnablePreload(tim, ll_channel); // the CCR shadow register is updated only when the update event occurs
    LL_TIM_OC_SetMode(tim, ll_channel, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(tim, ll_channel, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_DisableFast(tim, ll_channel);
    set_timer_compare[pwm_pha_cfg->channel - 1](tim, 0);

    // reset master slave mode
    LL_TIM_SetTriggerOutput(tim, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(tim);
    
    // Enable the timer channel
    LL_TIM_CC_EnableChannel(tim, ll_channel);

    // Enable the timer
    // LL_TIM_EnableCounter(tim);

    LOG_DBG("PWM setup complete for channel %d with period %d", 
            pwm_pha_cfg->channel, pwm_pha_cfg->period);

    // Unlock the driver after setup
    __WS2812_PWM_UNLOCK(dev);

    return 0;
}

/* set timer capture/compare register */
// static void ws2812_pwm_set_ccr(const struct device *dev, uint32_t value)
// {
//     const struct ws2812_pwm_config *config = dev->config;
//     const struct pwm_stm32_config *pwm_cfg = config->pwm_dev->config;

//     // Retrieve Timer handler
//     TIM_TypeDef *tim = pwm_cfg->timer;

//     // set the compare register for the timer channel
//     set_timer_compare[config->pwm_config.channel - 1](tim, value);
// }

/**
 * @brief Start the PWM signal generation using DMA
 * @param dev Pointer to the device structure
 * @note This function starts the DMA transfer and enables the timer to generate
 *       the PWM signal for WS2812 LEDs.
 */
static int ws2812_pwm_start(const struct device *dev)
{
    int ret = 0;
    const struct ws2812_pwm_config *config = dev->config;
    struct ws2812_pwm_data *data = dev->data;
    const struct device *pwm_dev = config->pwm_dev;
    const struct pwm_stm32_config *pwm_cfg = pwm_dev->config;
    const ws2812_dma_pha_config_t *dma_pha_cfg = &config->dma_config;

    // configure the DMA transfer
    ret = dma_config(config->dma_dev, dma_pha_cfg->channel, &data->dma_cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure DMA: %d", ret);
        goto exit;
    }

    // Retrieve Timer handler
    TIM_TypeDef *tim = pwm_cfg->timer;

    // reset timer counter
    LL_TIM_SetCounter(tim, 0);

    // start dma transfer
    ret = dma_start(config->dma_dev, dma_pha_cfg->channel);
    if (ret < 0) {
        LOG_ERR("Failed to start DMA: %d", ret);
        goto exit;
    }

    // enable the DMA request for the timer channel
    enable_dma_request[config->pwm_config.channel - 1](tim);

    // enable the timer channel
    LL_TIM_CC_EnableChannel(tim, channel_to_ll[config->pwm_config.channel - 1]);

    // Enable the timer
    LL_TIM_EnableCounter(tim);
    // LOG_DBG("PWM started for %s", pwm_dev->name);

exit:
    return ret;
}

static int ws2812_pwm_stop(const struct device *dev)
{
    const struct ws2812_pwm_config *config = dev->config;
    const struct device *pwm_dev = config->pwm_dev;
    const struct pwm_stm32_config *pwm_cfg = pwm_dev->config;
    const ws2812_dma_pha_config_t *dma_pha_cfg = &config->dma_config;

    // Retrieve Timer handler
    TIM_TypeDef *tim = pwm_cfg->timer;

    // Diable the timer channel
    LL_TIM_CC_DisableChannel(tim, channel_to_ll[config->pwm_config.channel - 1]);

    // Disable the timer
    LL_TIM_DisableCounter(tim);

    // Disable timer DMA request
    disable_dma_request[config->pwm_config.channel - 1](tim);

    // clear the interrupt flags for the timer channel
    clear_interrupt_flags[config->pwm_config.channel - 1](tim);

    // Stop dma transfer
    int ret = dma_stop(config->dma_dev, dma_pha_cfg->channel);

    // Reset the compare register
    set_timer_compare[config->pwm_config.channel - 1](tim, 0);

    // reset the timer counter
    LL_TIM_SetCounter(tim, 0);

    // Enable the timer
    // LOG_DBG("PWM stopped for %s", pwm_dev->name);

    return ret;
}

static void ws2812_pwm_reset_signal(const struct device *dev)
{
    struct ws2812_pwm_data *data = dev->data;
    uint16_t len_data = HALF_BLOCK_SIZE_WORDS; // 8 bits per color, 3 colors (RGB), 5 LEDs per batch
    uint16_t start_index = ((data->num_led_buffer_updated + data->num_reset_isr_issued) * len_data) % BLOCK_SIZE_WORDS;
    // set memory to zero for the reset signal
    memset(data->pwm_buffer+start_index, 0, len_data * sizeof(uint32_t));
}

/* function to initialize the driver */
static int ws2812_pwm_init(const struct device *dev)
{
    /**
     * Create a debug output pin to reveal certain information
     * Delete this if not needed.
     */
    if (!device_is_ready(debug_pin.port)) {
        LOG_ERR("Debug pin %s is not ready", debug_pin.port->name);
        return -ENODEV;
    }
    if (gpio_pin_configure_dt(&debug_pin, GPIO_OUTPUT_INACTIVE) < 0) {
        LOG_ERR("Failed to configure debug pin %s", debug_pin.port->name);
        return -EIO;
    }

    int ret = 0;
    const struct ws2812_pwm_config *ws2812_cfg = dev->config;
    const struct device *dma_dev = ws2812_cfg->dma_dev;
    const ws2812_dma_pha_config_t *dma_pha_cfg = &ws2812_cfg->dma_config;
    const char *dma_dev_name = dma_dev->name;
    const struct device *pwm_dev = ws2812_cfg->pwm_dev;
    const struct pwm_stm32_config *pwm_cfg = pwm_dev->config;
    const ws2812_pwm_pha_config_t *pwm_pha_cfg = &ws2812_cfg->pwm_config;   

    LOG_DBG("Initializing WS2812 PWM driver for %s", dma_dev_name);
    // Check if the DMA device is ready
    if (!device_is_ready(dma_dev)) {
        LOG_WRN("DMA device %s is not ready", dma_dev_name);
        return -ENODEV;
    }

    // get data storage
    struct ws2812_pwm_data *ws2812_data = dev->data;
    // get pwm clock
    uint64_t pwm_cycles_per_sec = 0;
    if ((ret = pwm_get_cycles_per_sec(pwm_dev, pwm_pha_cfg->channel, &pwm_cycles_per_sec)) < 0) {
        LOG_ERR("Failed to get PWM cycles per second: %d", ret);
        return ret;
    }
    // compute the final clock frequency with prescaler
    ws2812_data->pwm_freq = pwm_cycles_per_sec / (pwm_cfg->prescaler + 1);
    // compute the delay for the high and low pulses. [pulse width(ns)] * [pwm_freq(Hz)] / 10^9
    float delay_t1h_f = (float)ws2812_cfg->delay_t1h_ns * (float)(ws2812_data->pwm_freq / 1e9);
    float delay_t0h_f = (float)ws2812_cfg->delay_t0h_ns * (float)(ws2812_data->pwm_freq / 1e9);
    // round the delay to the nearest integer
    ws2812_data->delay_t1h = (uint32_t)roundf(delay_t1h_f);
    ws2812_data->delay_t0h = (uint32_t)roundf(delay_t0h_f);
    // compute the PWM period in ticks
    ws2812_data->period = (uint32_t)roundf(((float)pwm_pha_cfg->period * (float)(ws2812_data->pwm_freq / 1e9)));
    // compute the total number of reset ISR to be issued
    // reset ticks = [reset_delay (us)] / [pwm_period (ns)] * 10^3
    uint32_t reset_ticks = ws2812_cfg->reset_delay_us * 1000 / pwm_pha_cfg->period + 1;
    ws2812_data->total_num_reset_isr = 1 + ( reset_ticks / (BLOCK_SIZE_WORDS / 2)) + 
                                        (reset_ticks % (BLOCK_SIZE_WORDS / 2) > 0);

    /* Retrieve Timer handler */
    TIM_TypeDef *tim = pwm_cfg->timer;

    /* configure DMA */
    LOG_DBG("Configuring DMA: channel=%d, slot=%d, channel_config=0x%x, features=0x%x",
            dma_pha_cfg->channel, dma_pha_cfg->slot, dma_pha_cfg->channel_config, dma_pha_cfg->features);

    // Initialize each block configuration
    ws2812_data->block_cfg = (struct dma_block_config){
        .source_address = (uint32_t)ws2812_data->pwm_buffer,
        .dest_address = WS2812_DMA_GET_TIM_CCR(pwm_pha_cfg->channel, tim), // address of timer peripheral
        .block_size = BLOCK_SIZE_WORDS * sizeof(uint32_t), // size in bytes
        .next_block = NULL,
        .source_addr_adj = WS2812_DMA_GET_SOURCE_ADDR_ADJ(dma_pha_cfg->channel_config),
        .dest_addr_adj = WS2812_DMA_GET_DEST_ADDR_ADJ(dma_pha_cfg->channel_config),
        .source_reload_en = true, // this will enable the DMA cyclic mode for stm32,
                                  // which is required for half-complete interrupts
        .dest_reload_en = true, // source_reload_en and dest_reload_en must be the same
        .fifo_mode_control = STM32_DMA_FEATURES_FIFO_THRESHOLD(dma_pha_cfg->features),
    };
    
    ws2812_data->dma_cfg = (struct dma_config){
        .dma_slot = dma_pha_cfg->slot,
        .channel_direction = MEMORY_TO_PERIPHERAL,
        .complete_callback_en = true,
        .channel_priority = STM32_DMA_CONFIG_PRIORITY(dma_pha_cfg->channel_config),
        .source_data_size = STM32_DMA_CONFIG_MEMORY_DATA_SIZE(dma_pha_cfg->channel_config),
        .dest_data_size = STM32_DMA_CONFIG_PERIPHERAL_DATA_SIZE(dma_pha_cfg->channel_config),
        .source_burst_length = 1, // single bust, no fifo
        .dest_burst_length = 1, // single burst, no fifo
        .block_count = 1,
        .head_block = &ws2812_data->block_cfg,
        .user_data = (void *)dev, // pass the device pointer as user data
        .dma_callback = &ws2812_pwm_dma_tc_callback,
    };

    // set the DMA configuration
    // ret = dma_config(dma_dev, dma_pha_cfg->channel, &dma_cfg);
    // if (ret < 0) {
    //     LOG_ERR("Failed to configure DMA: %d", ret);
    //     return ret;
    // }
    // LOG_DBG("DMA configured successfully for %s", dma_dev_name);

    /* Configure PWM */
    ws2812_pwm_setup_pwm(dev);
    if (ret < 0) {
        LOG_ERR("Failed to setup PWM: %d", ret);
        return ret;
    }

    return 0;
}

/* update pwm buffer which is a DMA source memory */
static int ws2812_pwm_update_buffer(const struct device *dev, 
                                uint16_t index_led,
                                size_t num_leds)
{
    struct ws2812_pwm_data *ws2812_data = dev->data;
    uint32_t *pwm_buffer = ws2812_data->pwm_buffer;
    // check if the index is valid
    if (index_led >= ws2812_data->num_leds) {
        LOG_ERR("Index LED out of bounds: %u >= %zu", index_led, ws2812_data->num_leds);
        return -EINVAL;
    }
    // check if the number of LEDs is valid
    if (num_leds > ws2812_data->num_leds) {
        LOG_ERR("Number of LEDs exceeds the configured limit: %zu > %zu", num_leds, ws2812_data->num_leds);
        return -EINVAL;
    }

    // lock the driver to prevent concurrent access
    __WS2812_PWM_LOCK(dev);

    int ret = 0;
    const struct ws2812_pwm_config *ws2812_cfg = dev->config;
    struct led_rgb *rgb = ws2812_data->led_data;
    // check if the RGB data is valid
    if (rgb == NULL) {
        LOG_ERR("RGB data is NULL");
        ret = -EINVAL;
        goto exit;
    }

    // uint8_t dts_color_map[3];
    // memcpy(dts_color_map, ws2812_cfg->color_mapping, sizeof(dts_color_map));
    uint8_t color_mapping[LED_COLOR_ID_MAX];

    uint16_t end_index = index_led + num_leds;
    uint16_t len_data = 8 * COLOR_SIZE_BYTES;

    // set the PWM data for the LED
    for (uint16_t i = index_led; i < end_index; i++)
    {
        // Get the RGB values for the current LED
        color_mapping[LED_COLOR_ID_RED] = rgb[i].r;
        color_mapping[LED_COLOR_ID_GREEN] = rgb[i].g;
        color_mapping[LED_COLOR_ID_BLUE] = rgb[i].b;

        // calculate the start index for the PWM data
        uint16_t start_index = (i * len_data) % BLOCK_SIZE_WORDS;

        // set the PWM data for each bit of RGB color code
        // NOTE: the PWM data is set in the order of GRB,
        // and the data is set in the order of MSB first.
        for (uint8_t j = 0; j < 8; j++)
        {
            pwm_buffer[start_index + j] = (color_mapping[ws2812_cfg->color_mapping[0]] & (1 << (7 - j))) ? 
                ws2812_data->delay_t1h : ws2812_data->delay_t0h;
            pwm_buffer[start_index + j + 8] = (color_mapping[ws2812_cfg->color_mapping[1]] & (1 << (7 - j))) ?
                ws2812_data->delay_t1h : ws2812_data->delay_t0h;
            pwm_buffer[start_index + j + 16] = (color_mapping[ws2812_cfg->color_mapping[2]] & (1 << (7 - j))) ?
                ws2812_data->delay_t1h : ws2812_data->delay_t0h;
        }
    }
    // LOG_DBG("WS2812 PWM buffer updated for %zu LEDs starting from index %u", num_leds, index_led);
    
exit:
    // unlock the driver after updating the buffer
    __WS2812_PWM_UNLOCK(dev);

    return ret;
}

/* define the driver APIs */
static int ws2812_pwm_update_rgb(const struct device *dev, 
                                struct led_rgb *rgb, 
                                size_t num_leds)
{
    struct ws2812_pwm_data *ws2812_data = dev->data;

    // save the RGB data pointer and number of LEDs
    ws2812_data->led_data = rgb;
    ws2812_data->num_leds = num_leds;

    // check if update is already in progress
    if (ws2812_data->flags & WS2812_PWM_FLAG_TASK_UPDATE_BUFFER) {
        LOG_ERR("WS2812 PWM update is already in progress");
        return -EBUSY;
    }

    // write the RGB values to the pwm_buffer
    ws2812_pwm_update_buffer(dev, 0, 2 * BATCH_UPDATE_SIZE);

    // update the number of LED buffer updated
    ws2812_data->num_led_buffer_updated = 2 * BATCH_UPDATE_SIZE;

    // set flag to indicate that the buffer is being updated
    ws2812_data->flags |= WS2812_PWM_FLAG_TASK_UPDATE_BUFFER;


    // set CCRx register with the first value of the first block
    // ws2812_pwm_set_ccr(dev, pwm_buffer[0]);
    // Enable the timer to start sending the PWM signal with the help of DMA
    int ret = ws2812_pwm_start(dev);
    LOG_DBG("WS2812 PWM update RGB called for %zu LEDs", num_leds);

    return ret;
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
        .dma_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, dmas)), \
        .dma_config = { \
            .slot = DT_INST_PHA(inst, dmas, slot), \
            .channel = DT_INST_PHA(inst, dmas, channel), \
            .channel_config = DT_INST_PHA(inst, dmas, channel_config), \
            .features = DT_INST_PHA(inst, dmas, features), \
        }, \
        .pwm_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, pwms)), \
        .pwm_config = { \
            .channel = DT_INST_PHA(inst, pwms, channel), \
            .period = DT_INST_PHA(inst, pwms, period), \
            .flags = DT_INST_PHA(inst, pwms, flags), \
        }, \
        .num_leds = DT_INST_PROP(inst, chain_length), \
        .color_mapping = DT_INST_PROP(inst, color_mapping), \
        .reset_delay_us = DT_INST_PROP(inst, reset_delay), \
        .delay_t1h_ns = DT_INST_PROP(inst, delay_t1h), \
        .delay_t0h_ns = DT_INST_PROP(inst, delay_t0h), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, &ws2812_pwm_init, NULL, \
        &ws2812_pwm_data_##inst, &ws2812_pwm_config_##inst, \
        POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY, &ws2812_pwm_api);

/* Create the struct device for every status "okay" node in the devicetree */
DT_INST_FOREACH_STATUS_OKAY(WS2812_PWM_DEFINE)
