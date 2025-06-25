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
#include "esp_log.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_err.h"

static const char *TAG = "MyModule";
static const char *TAG_GPIO = "GPIO";
static const char *TAG_Timer = "GPTimer";
static const char *TAG_pwm = "PWM";

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

static QueueHandle_t evento_botao = NULL;
static QueueHandle_t evento_timer = NULL;
static QueueHandle_t pwm_queue = NULL;
static SemaphoreHandle_t semaphore_pwm = NULL;

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
                        if (pwm_config.duty >= 8000)
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

    while (1)
    {
        if (xQueueReceive(fila_timer, &elemento_fila, pdMS_TO_TICKS(2000)))
        {
            relogio.segundo++;
            if (relogio.segundo >= 60)
            {
                relogio.segundo = 0;
                relogio.minuto++;
                if (relogio.minuto >= 60)
                {
                    relogio.minuto = 0;
                    relogio.hora++;
                    if (relogio.hora >= 24)
                    {
                        relogio.hora = 0;
                    }
                }
            }
            // ESP_LOGI(TAG_Timer, "Relógio: %02d:%02d:%02d", relogio.hora, relogio.minuto, relogio.segundo);
        }
        else
        {
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
                ESP_LOGI(TAG_pwm, "Modo manual: %d", pwm_config.duty);
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

void app_main(void)
{
    /* Print chip information */

    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), WiFi%s%s%s, ",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;

    ESP_LOGI(TAG, "silicon revision v%d.%d, ", major_rev, minor_rev);

    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        ESP_LOGI(TAG, "Get flash size failed");
        return;
    }

    ESP_LOGI(TAG, "%" PRIu32 "MB %s flash", flash_size / (uint32_t)(1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGW(TAG, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--)
    {
        ESP_LOGI(TAG, "Restarting in %d seconds...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // create a queue to handle gpio event from isr
    evento_botao = xQueueCreate(10, sizeof(uint32_t));
    evento_timer = xQueueCreate(10, sizeof(propriedades_fila_timer_t));
    // Cria fila de comunicação GPIO → PWM
    pwm_queue = xQueueCreate(10, sizeof(PWM_elements_t));

    // Cria semáforo binário para sincronização a cada 100 ms
    semaphore_pwm = xSemaphoreCreateBinary();

    // start gpio task
    xTaskCreate(gpio_task_led_botao, "gpio_task_intr", 2048, NULL, 10, NULL);
    xTaskCreate(gptimer_task, "gptimer_task_intr", 2048, NULL, 10, NULL);
    xTaskCreate(ledc_task, "ledc_task", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG, "Restarting now.");
    fflush(stdout);
    /* esp_restart();*/
}
