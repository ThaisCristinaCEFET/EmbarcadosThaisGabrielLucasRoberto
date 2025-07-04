/*
 * SPDX-FileCopyrightText: 2020-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "MyModule";
static const char *TAG_GPIO = "GPIO";
static const char *TAG_Timer = "GPTimer";
static const char *TAG_pwm = "PWM";
static const char *TAG_adc = "ADC";
static const char *TAG_led = "OLED";
static const char *TAG_wifi = "mqtt";



/*========================================================================================================
ÁREA DE FUNÇÃO: botões
SEÇÃO DE CONFIGURAÇÃO DE PINOS DE ENTRADA E SAÍDA
========================================================================================================*/

#define BOTAO_0 21
#define BOTAO_1 22
#define BOTAO_2 23
#define GPIO_INPUT_PIN_SEL ((1ULL << BOTAO_0) | (1ULL << BOTAO_1) | (1ULL << BOTAO_2))
#define LED 2
#define GPIO_OUTPUT_PIN_SEL (1ULL << LED)
#define ESP_INTR_FLAG_DEFAULT 0

/*========================================================================================================
SEÇÃO DE CONFIGURAÇÃO DE LEDS DO PWM
========================================================================================================*/
#define LEDC_TIMER LEDC_TIMER_0         // tipo do timer
#define LEDC_MODE LEDC_LOW_SPEED_MODE   // velocide do pwm
#define LEDC_OUTPUT_IO_16 (16)          // Define the output GPIO VERDE
#define LEDC_OUTPUT_IO_17 (17)          // Define the output GPIO
#define LEDC_OUTPUT_IO_26 (26)          // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0     // oq o canal faz
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (4096)                // Set duty to 50%. (2 ** 13) * 50% = 4096 muda a intencidade do brilho
#define LEDC_FREQUENCY (5000)           // Frequency in Hertz. Set frequency at 4 kHz

#define LEDC_TIMER2 LEDC_TIMER_0         // tipo do timer
#define LEDC_MODE2 LEDC_LOW_SPEED_MODE   // velocide do pwm
#define LEDC_OUTPUT_IO_32 (32)           // Define the output GPIO
#define LEDC_OUTPUT_IO_33 (33)           // Define the output GPIO
#define LEDC_CHANNEL2 LEDC_CHANNEL_1     // oq o canal faz
#define LEDC_DUTY_RES2 LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY2 (4096)                // Set duty to 50%. (2 ** 13) * 50% = 4096 muda a intencidade do brilho
#define LEDC_FREQUENCY2 (5000)           // Frequency in Hertz. Set frequency at 4 kHz

/*========================================================================================================
SEÇÃO DE CONFIGURAÇÃO DO ADC
========================================================================================================*/
#define ADC1_CHAN0 ADC_CHANNEL_3
#define ADC_ATTEN ADC_ATTEN_DB_12

/*========================================================================================================
SEÇÃO DE CONFIGURAÇÃO DO I2C
========================================================================================================*/
#define I2C_BUS_PORT  0
#define LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define PIN_NUM_SDA           19
#define PIN_NUM_SCL           18
#define PIN_NUM_RST           -1
#define I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical

#define LCD_H_RES              128
#define LCD_V_RES              64

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8


static QueueHandle_t evento_botao = NULL;
static QueueHandle_t evento_timer = NULL;
static QueueHandle_t pwm_queue = NULL;
static SemaphoreHandle_t semaphore_pwm = NULL;
static QueueHandle_t evento_adc = NULL;
static QueueHandle_t evento_oled = NULL;

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);


//teste do label
lv_obj_t *label = NULL;
lv_obj_t *label2 = NULL;

// Mesa de trabalho do botão
// ISR -> interrupt service routine
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(evento_botao, &gpio_num, NULL);
}

// Mesa de trabalho do timer, separando espaço de memória para o mesmo
typedef struct
{
    uint64_t evento_contador;
} propriedades_fila_timer_t;

// Declaração do formato estrutural do tempo
typedef struct
{
    uint8_t minuto;
    uint8_t segundo;
    uint8_t hora;
} relogio_t;

typedef struct
{
    bool automatizado; // true = modo automático; false = modo manual
    int16_t duty;      // valor de incremento (em modo manual, >0 significa incrementar)
} PWM_elements_t;

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG_wifi, "Last error %s: 0x%x", message, error_code);
    }
}

static void gpio_task_led_botao(void *arg)
{
    uint32_t io_num;
    int ESTADO_LED = 0;

    // Configuração zerada para botões.
    gpio_config_t io_conf = {};
    // interrupção borda de descida
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // setando como entrada apenas
    io_conf.mode = GPIO_MODE_INPUT;
    // mascara para os pinos 21, 22 e 23
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // disabilitando o pull down
    io_conf.pull_down_en = 0;
    // habilitando pull up
    io_conf.pull_up_en = 1;
    // Configura o GPIO
    gpio_config(&io_conf);

    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set GPIO2
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(BOTAO_0, gpio_isr_handler, (void *)BOTAO_0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(BOTAO_1, gpio_isr_handler, (void *)BOTAO_1);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(BOTAO_2, gpio_isr_handler, (void *)BOTAO_2);

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    PWM_elements_t pwm_config = {
        .automatizado = false,
        .duty = 0,
    };
    bool repique = false;
    for (;;)
    {
        repique = false;

        while (xQueueReceive(evento_botao, &io_num, pdMS_TO_TICKS(20)))
        {
            if (!repique)
            {
                switch (io_num)
                {
                case 21:
                    ESTADO_LED = 1;
                    gpio_set_level(LED, ESTADO_LED);
                    ESP_LOGI(TAG_GPIO, "LIGA LED");

                    pwm_config.automatizado = true;
                    pwm_config.duty = 0;
                    xQueueSendToBack(pwm_queue, &pwm_config, portMAX_DELAY);
                    ESP_LOGI(TAG_pwm, "Selecionando Modo automatico");
                    // vTaskDelay(1000 / portTICK_PERIOD_MS);
                    break;

                case 22:
                    ESTADO_LED = 0;
                    gpio_set_level(LED, ESTADO_LED);
                    ESP_LOGI(TAG_GPIO, "DESLIGA LED");

                    pwm_config.automatizado = false;
                    pwm_config.duty = 0;
                    xQueueSendToBack(pwm_queue, &pwm_config, portMAX_DELAY);
                    ESP_LOGI(TAG_pwm, "Selecionando Modo manual");
                    // vTaskDelay(1000 / portTICK_PERIOD_MS);
                    break;

                case 23:
                    ESTADO_LED = !ESTADO_LED;
                    gpio_set_level(LED, ESTADO_LED);
                    ESP_LOGI(TAG_GPIO, "INVERTE LED LED");

                    if (!pwm_config.automatizado)
                    {
                        pwm_config.duty = pwm_config.duty + 819;
                        if (pwm_config.duty >= 8100)
                        {
                            pwm_config.duty = 0;
                        }
                        xQueueSendToBack(pwm_queue, &pwm_config, portMAX_DELAY);
                        ESP_LOGI(TAG_pwm, "Incrementa modo manual");
                    }
                    else
                    {
                        ESP_LOGI(TAG_pwm, "Incrementa modo manual desabilitado");
                    }
                    // vTaskDelay(1000 / portTICK_PERIOD_MS);
                    break;

                default:
                    ESP_LOGI(TAG_GPIO, "Falha da fila");
                    break;
                }
            }

            repique = true;
        }
    }
}

/*========================================================================================================
ÁREA DE FUNÇÃO: INTERRUPÇÃO FIM
========================================================================================================*/

// Tratador de interrupção (CALLBACK) do GPTimer - ou seja, é executada sempre que o timer dispara
// Parâmetros: gptimer_handle_t timer: identificador do timer que chamou essa função.
// const gptimer_alarm_event_data_t *edata: estrutura que contém o valor atual do timer no momento da interrupção.
// void *user_fila: ponteiro que aponta para a fila que a tarefa de relógio está usando.
static bool IRAM_ATTR timer_relogio(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_fila)
{
    // Verifica se alguma tarefa de maior prioridade
    BaseType_t high_task_awoken = pdFALSE;
    // Setando a fila do relógio
    QueueHandle_t fila_timer = (QueueHandle_t)user_fila;

    // cria uma instancia de relogio_t e penas sinaliza que 1 segundo se passou
    relogio_t evento_relogio = {0};

    // envio da fila da interrupção
    xQueueSendFromISR(fila_timer, &evento_relogio, &high_task_awoken);

    // Reconfigura o alarme para o próximo segundo (1.000.000 us = 1 s)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + 100000,
    };
    gptimer_set_alarm_action(timer, &alarm_config);

    xSemaphoreGive(semaphore_pwm);
    xSemaphoreGive(evento_adc);

    return (high_task_awoken == pdTRUE);
}

// inicia e configura um temporizador; cria uma fila para os sinais do timer.
static void gptimer_task(void *arg)
{

    propriedades_fila_timer_t elemento_fila;
    QueueHandle_t fila_timer = xQueueCreate(10, sizeof(propriedades_fila_timer_t));
    relogio_t relogio;

    relogio.segundo = 0;
    relogio.minuto = 0;
    relogio.hora = 0;

    // retorno caso a criação da fila nao dê certo
    if (!fila_timer)
    {
        ESP_LOGE(TAG_Timer, "Criação fila_timer falho");
        return;
    }

    ESP_LOGI(TAG_Timer, "Create timer handle");

    gptimer_handle_t gptimer = NULL;

    // configurações do gptimer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // fonte clock
        .direction = GPTIMER_COUNT_UP,      // contagem crescente
        .resolution_hz = 1000000,           // 1MHz, 1 tick=1us
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    // função de interrupção callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_relogio, // chamada de atribuição de dados
    };

    // Cria um timer com resolução de 1 MHz (microsegundo)
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, fila_timer));
    // Ativa e inicializa o timer;
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    ESP_LOGI(TAG_Timer, "Start timer CV1, stop it at alarm event");
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 100000, // period = 100ms
    };

    // Define o primeiro alarme para ocorrer após 100 ms
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    while (1) {
    if (xQueueReceive(fila_timer, &elemento_fila, pdMS_TO_TICKS(2000))) {

        /* conta quantos alarmes de 100 ms já chegaram */
        static uint8_t cont100ms = 0;
        cont100ms++;

        /* a cada 10 alarmes = 1 s */
        if (cont100ms >= 10) {
            cont100ms = 0;          // zera para o próximo segundo

            /* --- lógica de relógio --- */
            relogio.segundo++;
            if (relogio.segundo == 60) {
                relogio.segundo = 0;
                relogio.minuto++;
                if (relogio.minuto == 60) {
                    relogio.minuto = 0;
                    relogio.hora = (relogio.hora + 1) % 24;
                }
            }

            /* atualiza o label a cada segundo */
            if (label2) {
                char buf[32];
                snprintf(buf, sizeof(buf),
                         "Relogio: %02d:%02d:%02d",
                         relogio.hora, relogio.minuto, relogio.segundo);
                lv_label_set_text(label2, buf);
            }
        }
    } else {
        ESP_LOGW(TAG_Timer, "Missed one count event");
    }
}

}

static void ledc_task(void *arg)
{
    int duty_max = 8191;
    // Prepare and then apply the LEDs Conectados
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC conctados
    ledc_channel_config_t ledc_channel_led0 = {
        .speed_mode = LEDC_MODE,        // config o modo de operação
        .channel = LEDC_CHANNEL,        // aplica o canal
        .timer_sel = LEDC_TIMER,        // não sei
        .intr_type = LEDC_INTR_DISABLE, // desabilita interrupção
        .gpio_num = LEDC_OUTPUT_IO_17,  // led de saida do pwm
        .duty = 4000,                   // Set duty to 0%  //escolhe o duty fora da do gatilho de duty
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_led0)); // avisa se der problema
    // ==============================================================================

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_led1 = {
        .speed_mode = LEDC_MODE2,       // config o modo de operação
        .channel = LEDC_CHANNEL2,       // aplica o canal
        .timer_sel = LEDC_TIMER,        // não sei
        .intr_type = LEDC_INTR_DISABLE, // desabilita interrupção
            .gpio_num = LEDC_OUTPUT_IO_33,  // led de saida do pwm
        .duty = 0,                      // Set duty to 0%  //escolhe o duty fora da do gatilho de duty
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_led1)); // avisa se der problema

    PWM_elements_t pwm_config;

    while (1)
    {
        if (xSemaphoreTake(semaphore_pwm, portMAX_DELAY))
        {
            if (xQueueReceive(pwm_queue, &pwm_config, 0)) // pdMS_TO_TICKS(2000)))
            {
            }
            if (pwm_config.automatizado)
            {
                pwm_config.duty = pwm_config.duty + 819;
                ledc_channel_led0.duty = pwm_config.duty;
                // ESP_LOGI(TAG_pwm, "Modo automatico ativo");

                ledc_set_duty(ledc_channel_led0.speed_mode, ledc_channel_led0.channel, ledc_channel_led0.duty);
                ledc_update_duty(ledc_channel_led0.speed_mode, ledc_channel_led0.channel);

                ledc_channel_led1.duty = ledc_channel_led0.duty;
                ledc_set_duty(ledc_channel_led1.speed_mode, ledc_channel_led1.channel, ledc_channel_led1.duty);
                ledc_update_duty(ledc_channel_led1.speed_mode, ledc_channel_led1.channel);
            }
            else
            {
                //ESP_LOGI(TAG_pwm, "Modo manual: %d", pwm_config.duty);
                ledc_channel_led0.duty = (uint32_t)pwm_config.duty;

                ledc_set_duty(ledc_channel_led0.speed_mode, ledc_channel_led0.channel, ledc_channel_led0.duty);
                ledc_update_duty(ledc_channel_led0.speed_mode, ledc_channel_led0.channel);

                ledc_channel_led1.duty = ledc_channel_led0.duty;
                ledc_set_duty(ledc_channel_led1.speed_mode, ledc_channel_led1.channel, ledc_channel_led1.duty);
                ledc_update_duty(ledc_channel_led1.speed_mode, ledc_channel_led1.channel);
            }
            if (pwm_config.duty >= duty_max)
            {
                pwm_config.duty = 0;
            }
        }
    }
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG_adc, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG_adc, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG_adc, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG_adc, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG_adc, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG_adc, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG_adc, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

static void adc_task(void *arg)
{

    static int adc_raw[1][10];
    static int voltage[1][10];

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, ADC1_CHAN0, ADC_ATTEN, &adc1_cali_chan0_handle);

    while (1)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw[0][0]));
        //ESP_LOGI(TAG_adc, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN0, adc_raw[0][0]);
        if (do_calibration1_chan0)
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
            //ESP_LOGI(TAG_adc, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC1_CHAN0, voltage[0][0]);
        }
        char x[100];
        sprintf(x,"voltage: %d",voltage[0][0]);
        if(label){
            lv_label_set_text(label, x);
        }
        
    
       xSemaphoreTake(evento_adc,portMAX_DELAY);

    }

    // Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0)
    {
        adc_calibration_deinit(adc1_cali_chan0_handle);
    }
}

static void lvgl_escrita(lv_disp_t *disp)
{
    //escrita
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    label = lv_label_create(scr);

    label2 = lv_label_create(scr);

    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, "EMBARCADOS");
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    //char x[100];
    //int cont = 0;
    //sprintf(x,"teste: %d",cont);
    //lv_label_set_text(label2, "Tulio aprova!");
    //cont++;
    
    lv_obj_align(label2, LV_ALIGN_BOTTOM_MID, 0, 0);
}

static void oled_task(void *arg)
{
    ESP_LOGI(TAG_led, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = PIN_NUM_SDA,
        .scl_io_num = PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    ESP_LOGI(TAG_led, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_HW_ADDR,
        .scl_speed_hz = LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    ESP_LOGI(TAG_led, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = PIN_NUM_RST,
    };

    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


    ESP_LOGI(TAG_led, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LCD_H_RES * LCD_V_RES,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    ESP_LOGI(TAG_led, "Display LVGL Scroll Text");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(0)) {
        lvgl_escrita(disp);
        // Release the mutex
        lvgl_port_unlock();
    }

    while(1){
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_wifi, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_wifi, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG_wifi, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG_wifi, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG_wifi, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG_wifi, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_wifi, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_wifi, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG_wifi, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_wifi, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_wifi, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_wifi, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_wifi, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG_wifi, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG_wifi, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://g1device:g1device@node02.myqtthub.com:1883",
        // CONFIG_BROKER_URL: username, senha
        .credentials.client_id = "g1device", // ID

    };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG_wifi, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

static void task_uaifi(void)
{
    ESP_LOGI(TAG_wifi, "[APP] Startup..");
    ESP_LOGI(TAG_wifi, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG_wifi, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();

  
}



void app_main(void)
{
    /* Print chip information */

    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG_adc, "This is %s chip with %d CPU core(s), WiFi%s%s%s, ",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;

    ESP_LOGI(TAG_adc, "silicon revision v%d.%d, ", major_rev, minor_rev);

    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        ESP_LOGI(TAG_adc, "Get flash size failed");
        return;
    }

    ESP_LOGI(TAG_adc, "%" PRIu32 "MB %s flash", flash_size / (uint32_t)(1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGW(TAG_adc, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());

    for (int i = 3; i >= 0; i--)
    {
        ESP_LOGI(TAG_adc, "Restarting in %d seconds...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    task_uaifi();

    // create a queue to handle gpio event from isr
    evento_botao = xQueueCreate(10, sizeof(uint32_t));
    evento_timer = xQueueCreate(10, sizeof(propriedades_fila_timer_t));
    // Cria fila de comunicação GPIO → PWM
    pwm_queue = xQueueCreate(10, sizeof(PWM_elements_t));

    // Cria semáforo binário para sincronização a cada 100 ms
    semaphore_pwm = xSemaphoreCreateBinary();

    // Cria fila ADC
    evento_adc = xSemaphoreCreateBinary();

    //cria fila oled
    evento_oled = xQueueCreate(10, sizeof(uint64_t));

    // start gpio task
    xTaskCreate(gpio_task_led_botao, "gpio_task_intr", 2048, NULL, 10, NULL);
    xTaskCreate(gptimer_task, "gptimer_task_intr", 2048, NULL, 10, NULL);
    xTaskCreate(ledc_task, "ledc_task", 2048, NULL, 10, NULL);
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 10, NULL);
    xTaskCreate(oled_task, "oled_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Restarting now.");
    fflush(stdout);
    /* esp_restart();*/
}
