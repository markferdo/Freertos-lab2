#include <algorithm>
#include <iostream>
#include <sstream>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware/gpio.h"
#include "PicoOsUart.h"
#include "ssd1306.h"
#include "hardware/pwm.h"

#include "hardware/timer.h"

#define BUTTON_PIN2 7 //sw_2
#define BUTTON_PIN1 8 //sw_1
#define BUTTON_PIN0 9 //sw_0
#define LED_PIN2 20 //D2
#define LED_PIN1 21 //D1
#define LED_PIN0 22 //D0
#define ROTARY_SW 12
#define ROTARY_A 10
#define ROTARY_B 11
#define MIN_FREQ    2
#define MAX_FREQ    200
#define DEBOUNCE_TIME_MS 200

extern "C" {
uint32_t read_runtime_ctr(void) {
    return timer_hw->timerawl;
}
}

#include "blinker.h"

SemaphoreHandle_t gpio_sem;
SemaphoreHandle_t charactor_sem; //lab_2

void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // signal task that a button was pressed
    xSemaphoreGiveFromISR(gpio_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

struct led_params{
    uint pin;
    uint delay;
};

void blink_task(void *param)
{
    auto lpr = (led_params *) param;
    const uint led_pin = lpr->pin;
    const uint delay = pdMS_TO_TICKS(lpr->delay);
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    while (true) {
        gpio_put(led_pin, true);
        vTaskDelay(delay);
        gpio_put(led_pin, false);
        vTaskDelay(delay);
    }
}

void gpio_task(void *param) {
    (void) param;
    const uint button_pin = 9;
    const uint led_pin = 22;
    const uint delay = pdMS_TO_TICKS(250);
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    gpio_init(button_pin);
    gpio_set_dir(button_pin, GPIO_IN);
    gpio_set_pulls(button_pin, true, false);
    gpio_set_irq_enabled_with_callback(button_pin, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    while(true) {
        if(xSemaphoreTake(gpio_sem, portMAX_DELAY) == pdTRUE) {
            //std::cout << "button event\n";
            gpio_put(led_pin, 1);
            vTaskDelay(delay);
            gpio_put(led_pin, 0);
            vTaskDelay(delay);
        }
    }
}

void serial_task(void *param)
{
    PicoOsUart u(0, 0, 1, 115200);
    Blinker blinky(20);
    uint8_t buffer[64];
    std::string line;
    while (true) {
        if(int count = u.read(buffer, 63, 30); count > 0) {
            u.write(buffer, count);
            buffer[count] = '\0';
            line += reinterpret_cast<const char *>(buffer);
            if(line.find_first_of("\n\r") != std::string::npos){
                u.send("\n");
                std::istringstream input(line);
                std::string cmd;
                input >> cmd;
                if(cmd == "delay") {
                    uint32_t i = 0;
                    input >> i;
                    blinky.on(i);
                }
                else if (cmd == "off") {
                    blinky.off();
                }
                line.clear();
            }
        }
    }
}

void modbus_task(void *param);
void display_task(void *param);
void i2c_task(void *param);
extern "C" {
    void tls_test(void);
}
void tls_task(void *param)
{
    tls_test();
    while(true) {
        vTaskDelay(100);
    }
}

//lab_2 starts
typedef enum {
    rot_rotate_clockwise,
    rot_rotate_counterclockwise
}rotation_event_t;

typedef enum {
    event_button,
    event_encoder
}event_type_t;

typedef struct {
    rotation_event_t direction;
    event_type_t type;
    uint32_t timestamp;
}gpio_event_s;

typedef struct {
    bool led_state;
    int led_frequency;
    uint32_t last_button_time;
    TaskHandle_t blink_task_handle;
    QueueHandle_t queue;
    QueueHandle_t gpio_semaphore;
} led_data_s;

void gpio_event_task(void *param);
void led_blink_task(void *param);
//void encoder_isr(uint gpio, uint32_t event_mask);
//void button_isr(uint gpio, uint32_t event_mask);
void gpio_isr(uint gpio, uint32_t event_mask);

void assign_pin(const uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

static led_data_s *led_data_for_isr = NULL;

//lab_2 ends
int main()
{
    stdio_init_all();
    printf("\nBoot\n");
    gpio_init(LED_PIN1);
    gpio_set_dir(LED_PIN1, true);

    //lab_2 part 2
    assign_pin(ROTARY_SW); // Rotary switch
    assign_pin(ROTARY_A); // R_A --> clockwise
    assign_pin(ROTARY_B); // R_B --> counterclockwise

    static led_data_s led_data = {false, 5, 0, NULL, NULL};
    led_data.queue = xQueueCreate(10, sizeof(gpio_event_s));
    vQueueAddToRegistry(led_data.queue, "Queue");
    led_data_for_isr = &led_data;

    gpio_set_irq_enabled_with_callback(ROTARY_A, GPIO_IRQ_EDGE_FALL, true, &gpio_isr);
    gpio_set_irq_enabled(ROTARY_SW, GPIO_IRQ_EDGE_FALL, true);

    xTaskCreate(gpio_event_task, "EventTask", 512, &led_data, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(led_blink_task, "BlinkTask", 512, &led_data, tskIDLE_PRIORITY + 1, &led_data.blink_task_handle);

    vTaskStartScheduler();

    while(true){};
}

//lab_2 starts

void gpio_isr(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    gpio_event_s event;
    led_data_s *led_data = led_data_for_isr;

    if (gpio == ROTARY_A) {
        bool rotary_B_state = gpio_get(ROTARY_B);
        event.type = event_encoder;
        if(rotary_B_state) {
            event.direction = rot_rotate_counterclockwise;
        } else {
            event.direction = rot_rotate_clockwise;
        }
    }
    else if (gpio == ROTARY_SW) {
        event.type = event_button;
    }
    event.timestamp = to_ms_since_boot(get_absolute_time());
    xQueueSendFromISR(led_data->queue, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void gpio_event_task(void *param) {
    led_data_s *led = (led_data_s *)param;

    while (true) {
        gpio_event_s event;
        if (xQueueReceive(led->queue, &event, portMAX_DELAY)) {
            if (event.type == event_button) {
                if (event.timestamp-led->last_button_time >= 250) {
                    led->last_button_time = event.timestamp;
                    led->led_state = !led->led_state;
                    if (led->led_state) {
                        printf("LED: ON, frequency: %d Hz \n", led->led_frequency);

                    }else {
                        printf("LED: OFF\n");
                        gpio_put(LED_PIN1, false);
                    }
                }
            }else if (event.type == event_encoder && led->led_state) {
                if (event.direction == rot_rotate_clockwise) {
                    led->led_frequency--;
                }else {
                    led->led_frequency++;
                }
                if (led->led_frequency<MIN_FREQ) {
                    led->led_frequency = MIN_FREQ;
                }
                if (led->led_frequency>MAX_FREQ) {
                    led->led_frequency = MAX_FREQ;
                }
                printf("frequency: %d Hz \n", led->led_frequency);
            }
        }
    }
}

void led_blink_task(void *param) {
    led_data_s *led = (led_data_s *)param;

    while (true) {
        if (led->led_state) {
            int half_ms = 500/led->led_frequency;
            gpio_put(LED_PIN1, 1);
            vTaskDelay(pdMS_TO_TICKS(half_ms));
            gpio_put(LED_PIN1, 0);
            vTaskDelay(pdMS_TO_TICKS(half_ms));
        } else {
            gpio_put(LED_PIN1, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

//lab_2 ends

#include <cstdio>
#include "ModbusClient.h"
#include "ModbusRegister.h"

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#if 0
#define UART_NR 0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#else
#define UART_NR 1
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#endif

#define BAUD_RATE 9600
#define STOP_BITS 2 // for real system (pico simualtor also requires 2 stop bits)

#define USE_MODBUS

void modbus_task(void *param) {

    const uint led_pin = 22;
    const uint button = 9;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    gpio_init(button);
    gpio_set_dir(button, GPIO_IN);
    gpio_pull_up(button);

    // Initialize chosen serial port
    //stdio_init_all();

    //printf("\nBoot\n");

#ifdef USE_MODBUS
    auto uart{std::make_shared<PicoOsUart>(UART_NR, UART_TX_PIN, UART_RX_PIN, BAUD_RATE, STOP_BITS)};
    auto rtu_client{std::make_shared<ModbusClient>(uart)};
    ModbusRegister rh(rtu_client, 241, 256);
    ModbusRegister t(rtu_client, 241, 257);
    ModbusRegister produal(rtu_client, 1, 0);
    produal.write(100);
    vTaskDelay((100));
    produal.write(100);
#endif

    while (true) {
#ifdef USE_MODBUS
        gpio_put(led_pin, !gpio_get(led_pin)); // toggle  led
        printf("RH=%5.1f%%\n", rh.read() / 10.0);
        vTaskDelay(5);
        printf("T =%5.1f%%\n", t.read() / 10.0);
        vTaskDelay(3000);
#endif
    }


}

#include "ssd1306os.h"
void display_task(void *param)
{
    auto i2cbus{std::make_shared<PicoI2C>(1, 400000)};
    ssd1306os display(i2cbus);
    display.fill(0);
    display.text("Boot", 0, 0);
    display.show();
    while(true) {
        vTaskDelay(100);
    }

}

void i2c_task(void *param) {
    auto i2cbus{std::make_shared<PicoI2C>(0, 100000)};

    const uint led_pin = 21;
    const uint delay = pdMS_TO_TICKS(250);
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    uint8_t buffer[64] = {0};
    i2cbus->write(0x50, buffer, 2);

    auto rv = i2cbus->read(0x50, buffer, 64);
    printf("rv=%u\n", rv);
    for(int i = 0; i < 64; ++i) {
        printf("%c", isprint(buffer[i]) ? buffer[i] : '_');
    }
    printf("\n");

    buffer[0]=0;
    buffer[1]=64;
    rv = i2cbus->transaction(0x50, buffer, 2, buffer, 64);
    printf("rv=%u\n", rv);
    for(int i = 0; i < 64; ++i) {
        printf("%c", isprint(buffer[i]) ? buffer[i] : '_');
    }
    printf("\n");

    while(true) {
        /*
        gpio_put(led_pin, 1);
        vTaskDelay(delay);
        gpio_put(led_pin, 0);
        vTaskDelay(delay);
        */
    }


}




/*
void led_blink_task(void *param) {
led_data_t *led = (led_data_t *)param;
while(true) {
if (led->led_state) {
gpio_put(LED_PIN1, 1);
vTaskDelay(pdMS_TO_TICKS(1000 / (2 * led->led_frequency)));
gpio_put(LED_PIN1, 0);
vTaskDelay(pdMS_TO_TICKS(1000 / (2 * led->led_frequency)));
} else {
gpio_put(LED_PIN1, 0);
vTaskDelay(pdMS_TO_TICKS(50));
}
}
}
*/