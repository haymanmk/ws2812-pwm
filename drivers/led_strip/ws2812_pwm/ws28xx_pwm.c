#include <soc.h>
#include "stm32f7xx_remote_io.h"
#include "ws28xx_pwm.h"

// buffer for the PWM data
static uint32_t ws28xx_pwm_buffer[WS28XX_PWM_BUFFER_SIZE] = {0};

// color data for each LED
ws_color_t ws28xx_pwm_color[NUMBER_OF_LEDS] = {0};

static TIM_HandleTypeDef *htim;
static uint32_t tim_channel;
volatile uint16_t num_led_buffer_updated = 0; // number of LED whose PWM buffer has been updated
volatile uint16_t flag_operation = 0;         // flag for the operation of the LED strip

// minimum number of ISRs should be issued for the reset signal
uint16_t num_isr_for_reset = 0;

// counting how many ISRs have been executed for the reset signal
volatile uint16_t count_isr_for_reset = 0;

/* Function Prototype */
HAL_StatusTypeDef __ws28xx_pwm_dma_stop(void);
HAL_StatusTypeDef __ws28xx_pwm_update_buffer(uint16_t led, uint16_t length);
void __ws28xx_pwm_reset(void);

void ws28xx_pwm_init(TIM_HandleTypeDef *_htim, uint32_t _tim_channel)
{
    // save the timer handle
    htim = _htim;

    // save the timer channel
    tim_channel = _tim_channel;

    // assert the size of the buffer is a multiple of 2
    ASSERT((WS28XX_PWM_BUFFER_SIZE % 2) == 0);

    // assert the total number of LEDs is a multiple of the number of LEDs updated per ISR
    ASSERT((NUMBER_OF_LEDS % NUMBER_OF_LEDS_UPDATED_PER_ISR) == 0);

    // initialize buffer for the PWM data
    for (uint16_t i = 0; i < WS28XX_PWM_BUFFER_SIZE; i++)
    {
        ws28xx_pwm_buffer[i] = 0;
    }

    // clear number of LED buffer updated
    num_led_buffer_updated = 0;

    // clear the flag for the operation of the LED strip
    flag_operation = 0;

    // calculate the minimum number of ISRs should be issued for the reset signal
    num_isr_for_reset = 1 + (NUM_PWM_CYCLES_RESET / (WS28XX_PWM_BUFFER_SIZE / 2)) + (NUM_PWM_CYCLES_RESET % (WS28XX_PWM_BUFFER_SIZE / 2) > 0);

    // in case LEDs are affected by the noise once the system is powered on,
    // the update signal is sent to the LED strip to reset the color of the LEDs.
    ws28xx_pwm_update();
}

HAL_StatusTypeDef ws28xx_pwm_set_color(uint8_t r, uint8_t g, uint8_t b, uint16_t led)
{
    // check if the LED index is valid
    if (led >= NUMBER_OF_LEDS)
    {
        return HAL_ERROR;
    }

    // set the color of the LED
    ws28xx_pwm_color[led].r = r;
    ws28xx_pwm_color[led].g = g;
    ws28xx_pwm_color[led].b = b;

    return HAL_OK;
}

void ws28xx_pwm_set_color_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t i = 0; i < NUMBER_OF_LEDS; i++)
    {
        ws28xx_pwm_set_color(r, g, b, i);
    }
}

void ws28xx_pwm_set_color_all_off(void)
{
    ws28xx_pwm_set_color_all(0, 0, 0);
}

HAL_StatusTypeDef ws28xx_pwm_update(void)
{
    // check if the DMA transfer is ongoing
    if (flag_operation & FLAG_OPERATION_UPDATING)
    {
        return HAL_BUSY;
    }

    // set the flag for the operation of the LED strip
    // Note: this flag is set first to avoid the
    //       multiple calls of this function.
    flag_operation |= FLAG_OPERATION_UPDATING;

    // update the buffer for the PWM data
    (void)__ws28xx_pwm_update_buffer(0, 2 * NUMBER_OF_LEDS_UPDATED_PER_ISR);

    // start the DMA transfer
    if (HAL_TIM_PWM_Start_DMA(htim, tim_channel, ws28xx_pwm_buffer, WS28XX_PWM_BUFFER_SIZE) != HAL_OK)
    {
        // clear the flag for the operation of the LED strip
        flag_operation &= ~FLAG_OPERATION_UPDATING;

        return HAL_ERROR;
    }

    // increment the number of LED buffer updated
    num_led_buffer_updated += 2 * NUMBER_OF_LEDS_UPDATED_PER_ISR;

    return HAL_OK;
}

void ws28xx_pwm_dma_half_complete_callback(void)
{
    uint16_t _flag_operation = flag_operation;

    // atomic operation
    __disable_irq();
        
    // check if DMA transfer should be stopped
    if (_flag_operation & FLAG_OPERATION_DMA_STOP)
    {
        // stop the DMA transfer
        if (__ws28xx_pwm_dma_stop() != HAL_OK)
        {
            // error handling
            // enable the interrupt
            __enable_irq();

            return;
        };

        // clear the flag for the operation of the LED strip
        flag_operation &= ~(FLAG_OPERATION_UPDATING | FLAG_OPERATION_DMA_STOP | FLAG_OPERATION_RESET_SIGNAL);

        // clear the count of ISR for the reset signal
        count_isr_for_reset = 0;

        // clear the number of LED buffer updated
        num_led_buffer_updated = 0;
    }
    // check if the reset signal should be sent
    else if (_flag_operation & FLAG_OPERATION_RESET_SIGNAL)
    {
        // reset the buffer for the PWM data
        __ws28xx_pwm_reset();

        // increment the count of ISR for the reset signal
        count_isr_for_reset++;

        if (count_isr_for_reset >= num_isr_for_reset)
        {
            // set the flag for the operation of the LED strip
            flag_operation |= FLAG_OPERATION_DMA_STOP;
        }
    }
    else
    {

        // update the buffer for the PWM data
        __ws28xx_pwm_update_buffer(num_led_buffer_updated, NUMBER_OF_LEDS_UPDATED_PER_ISR);

        // increment the number of LED buffer updated
        num_led_buffer_updated += NUMBER_OF_LEDS_UPDATED_PER_ISR;

        // check if all the LED buffers have been updated
        if (num_led_buffer_updated >= NUMBER_OF_LEDS)
        {
            // set the flag for the operation of the LED strip
            flag_operation |= FLAG_OPERATION_RESET_SIGNAL;
        }
    }

    // enable the interrupt
    __enable_irq();
}

void ws28xx_pwm_dma_complete_callback(void)
{
    uint16_t _flag_operation = flag_operation;

    // atomic operation
    __disable_irq();
        
    // check if DMA transfer should be stopped
    if (_flag_operation & FLAG_OPERATION_DMA_STOP)
    {
        // stop the DMA transfer
        if (__ws28xx_pwm_dma_stop() != HAL_OK)
        {
            // error handling
            // enable the interrupt
            __enable_irq();

            return;
        };

        // clear the flag for the operation of the LED strip
        flag_operation &= ~(FLAG_OPERATION_UPDATING | FLAG_OPERATION_DMA_STOP | FLAG_OPERATION_RESET_SIGNAL);

        // clear the count of ISR for the reset signal
        count_isr_for_reset = 0;

        // clear the number of LED buffer updated
        num_led_buffer_updated = 0;
    }
    // check if the reset signal should be sent
    else if (_flag_operation & FLAG_OPERATION_RESET_SIGNAL)
    {
        // reset the buffer for the PWM data
        __ws28xx_pwm_reset();

        // increment the count of ISR for the reset signal
        count_isr_for_reset++;

        if (count_isr_for_reset >= num_isr_for_reset)
        {
            // set the flag for the operation of the LED strip
            flag_operation |= FLAG_OPERATION_DMA_STOP;
        }
    }
    else
    {
        // update the buffer for the PWM data
        __ws28xx_pwm_update_buffer(num_led_buffer_updated, NUMBER_OF_LEDS_UPDATED_PER_ISR);

        // increment the number of LED buffer updated
        num_led_buffer_updated += NUMBER_OF_LEDS_UPDATED_PER_ISR;

        // check if all the LED buffers have been updated
        if (num_led_buffer_updated >= NUMBER_OF_LEDS)
        {
            // set the flag for the operation of the LED strip
            flag_operation |= FLAG_OPERATION_RESET_SIGNAL;
        }
    }

    // enable the interrupt
    __enable_irq();
}

int ws28xx_pwm_get_color(uint8_t *r, uint8_t *g, uint8_t *b, uint16_t led)
{
    // check if the LED index is valid
    if (led >= NUMBER_OF_LEDS)
    {
        return -1;
    }

    // get the color of the LED
    *r = ws28xx_pwm_color[led].r;
    *g = ws28xx_pwm_color[led].g;
    *b = ws28xx_pwm_color[led].b;

    return 0;
}

HAL_StatusTypeDef __ws28xx_pwm_dma_stop(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    if (htim)
    {
        // stop the DMA transfer
        if (HAL_TIM_PWM_Stop_DMA(htim, tim_channel) != HAL_OK)
        {
            status = HAL_ERROR;
        }
    }

    return status;
}

/** */
HAL_StatusTypeDef __ws28xx_pwm_update_buffer(uint16_t led, uint16_t length)
{
    // assert led index is valid
    if (led >= NUMBER_OF_LEDS) return HAL_ERROR;

    // assert length is valid
    if (length > NUMBER_OF_LEDS) return HAL_ERROR;

    uint16_t end_index = led + length;
    uint16_t len_data = 8 * NUMBER_OF_BASIC_COLORS;

    // set the PWM data for the LED
    for (uint16_t i = led; i < end_index; i++)
    {
        // get the color of the LED
        ws_color_t color = ws28xx_pwm_color[i];

        // calculate the start index for the PWM data
        uint16_t start_index = (i * len_data) % WS28XX_PWM_BUFFER_SIZE;

        // set the PWM data for each bit of RGB color code
        // NOTE: the PWM data is set in the order of GRB,
        // and the data is set in the order of MSB first.
        for (uint8_t j = 0; j < 8; j++)
        {
            // set the PWM data for the green color
            ws28xx_pwm_buffer[start_index + j] = (color.g & (1 << (7 - j))) ? DUTY_CYCLE_HIGH_BIT : DUTY_CYCLE_LOW_BIT;

            // set the PWM data for the red color
            ws28xx_pwm_buffer[start_index + j + 8] = (color.r & (1 << (7 - j))) ? DUTY_CYCLE_HIGH_BIT : DUTY_CYCLE_LOW_BIT;

            // set the PWM data for the blue color
            ws28xx_pwm_buffer[start_index + j + 16] = (color.b & (1 << (7 - j))) ? DUTY_CYCLE_HIGH_BIT : DUTY_CYCLE_LOW_BIT;
        }
    }

    return HAL_OK;
}

void __ws28xx_pwm_reset(void)
{
    uint16_t len_data = 8 * NUMBER_OF_BASIC_COLORS * NUMBER_OF_LEDS_UPDATED_PER_ISR;
    uint16_t start_index = ((num_led_buffer_updated + count_isr_for_reset) * len_data) % WS28XX_PWM_BUFFER_SIZE;
    // set the PWM data for the reset signal
    for (uint16_t i = 0; i < len_data; i++)
    {
        ws28xx_pwm_buffer[start_index+ i] =  0;
    }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  ws28xx_pwm_dma_half_complete_callback();
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  ws28xx_pwm_dma_complete_callback();
}