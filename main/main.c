// pan_tilt_freertos.c

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#define MOTOR_C   14
#define PAN_GPIO  15
#define TILT_GPIO 16

typedef struct {
    int16_t x;  // leitura ADC X (0–4095)
    int16_t y;  // leitura ADC Y (0–4095)
} Joystick_t;

static QueueHandle_t joystick_queue;
static QueueHandle_t motorC_queue;


// ----- Conversão ADC → duty cycle PWM  -----
static uint16_t adc_to_pwm(uint16_t adc_val) {
    // adc_val ∈ [0..4095] → pulse ∈ [1000..2000] µs
    //const uint16_t min_us = 1000, max_us = 2000;
    const uint16_t min_us = 550, max_us = 2300;

    return min_us + ((uint32_t)(adc_val) * (max_us - min_us) / 4095);
}

// ----- Task de leitura do joystick -----
void joystick_task(void *pvParameters) {
    Joystick_t js;
    while (1) {
        adc_select_input(0);
        js.x = adc_read();       // canal 0
        adc_select_input(1);
        js.y = adc_read();       // canal 1
        xQueueOverwrite(joystick_queue, &js);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ----- Task de leitura do potenciometro/LDR -----
void pot_task(void *pvParameters) {
    while (1) {
        adc_select_input(2);
        int leitura = adc_read();       // canal 2
        xQueueOverwrite(motorC_queue, &leitura);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ----- Task de controle de servos -----
void servo_task(void *p) {
    Joystick_t js;
    int16_t pot;
    const uint slice_pan   = pwm_gpio_to_slice_num(PAN_GPIO);
    const uint chan_pan    = pwm_gpio_to_channel(PAN_GPIO);
    const uint slice_tilt  = pwm_gpio_to_slice_num(TILT_GPIO);
    const uint chan_tilt   = pwm_gpio_to_channel(TILT_GPIO);
    const uint slice_C  = pwm_gpio_to_slice_num(MOTOR_C);
    const uint chan_C   = pwm_gpio_to_channel(MOTOR_C);

    while (1) {
        if (xQueueReceive(joystick_queue, &js, portMAX_DELAY) == pdTRUE) {
            uint16_t pulse_pan  = adc_to_pwm(js.x);
            uint16_t pulse_tilt = adc_to_pwm(js.y);

            pwm_set_chan_level(slice_pan,  chan_pan,  pulse_pan);
            pwm_set_chan_level(slice_tilt, chan_tilt, pulse_tilt);
        }
        if (xQueueReceive(motorC_queue, &pot, portMAX_DELAY) == pdTRUE) {
            uint16_t pulse_C  = adc_to_pwm(pot);

            printf("raw X=%4d  → pulse=%4dµs\n", pot, pulse_C);

            pwm_set_chan_level(slice_C,  chan_C,  pulse_C);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

int main() {
    stdio_init_all();

    // 1) ADC
    adc_init();
    adc_gpio_init(28);
    adc_gpio_init(26);
    adc_gpio_init(27);
    //adc_select_input(0);

    // 2) PWM para servos
    const uint servos[3] = { PAN_GPIO, TILT_GPIO, MOTOR_C };
    for (int i = 0; i < 3; i++) {
        uint gpio    = servos[i];
        uint slice   = pwm_gpio_to_slice_num(gpio);
        uint channel = pwm_gpio_to_channel(gpio);

        gpio_set_function(gpio, GPIO_FUNC_PWM);
        pwm_set_wrap(slice, 20000 - 1);       // 20 ms período
        pwm_set_clkdiv(slice, 125.0f);        // tick = 1 µs

        // centra no neutro (1.5 ms)
        pwm_set_chan_level(slice, channel, 1500);
        pwm_set_enabled(slice, true);
    }

    // 3) queue e tasks (igual a antes) …
    joystick_queue = xQueueCreate(1, sizeof(Joystick_t));
    motorC_queue = xQueueCreate(1, sizeof(int16_t));
    xTaskCreate(pot_task,    "POT", 256, NULL, 1, NULL);
    xTaskCreate(joystick_task, "JOY", 256, NULL, 2, NULL);
    xTaskCreate(servo_task,    "SRV", 256, NULL, 1, NULL);

    // 4) start
    vTaskStartScheduler();
    while (1) {}
    return 0;
}