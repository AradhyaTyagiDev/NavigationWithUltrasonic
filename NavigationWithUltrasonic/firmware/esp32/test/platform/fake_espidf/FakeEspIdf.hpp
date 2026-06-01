#pragma once

#include <stddef.h>
#include <stdint.h>

#include <deque>
#include <map>
#include <string>
#include <vector>

using esp_err_t = int;

constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_FAIL = -1;

#ifndef ESP_IDF_VERSION_MAJOR
#define ESP_IDF_VERSION_MAJOR 5
#endif

using BaseType_t = int;
using UBaseType_t = unsigned int;
using TickType_t = uint32_t;
using TaskFunction_t = void (*)(void *);
using TaskHandle_t = void *;
using QueueHandle_t = void *;
using SemaphoreHandle_t = void *;

constexpr BaseType_t pdTRUE = 1;
constexpr BaseType_t pdFALSE = 0;
constexpr BaseType_t pdPASS = 1;

#define pdMS_TO_TICKS(ms) static_cast<TickType_t>(ms)
#define portYIELD_FROM_ISR() fake_espidf::yieldFromIsr()

using gpio_num_t = int;
constexpr gpio_num_t GPIO_NUM_NC = -1;
constexpr int GPIO_MODE_OUTPUT = 1;
constexpr int GPIO_PULLUP_DISABLE = 0;
constexpr int GPIO_PULLDOWN_DISABLE = 0;
constexpr int GPIO_INTR_DISABLE = 0;

struct gpio_config_t
{
    uint64_t pin_bit_mask = 0;
    int mode = 0;
    int pull_up_en = 0;
    int pull_down_en = 0;
    int intr_type = 0;
};

using ledc_mode_t = int;
using ledc_timer_bit_t = int;
using ledc_timer_t = int;
using ledc_channel_t = int;

constexpr ledc_timer_bit_t LEDC_TIMER_10_BIT = 10;
constexpr ledc_timer_t LEDC_TIMER_0 = 0;
constexpr ledc_channel_t LEDC_CHANNEL_0 = 0;
constexpr ledc_channel_t LEDC_CHANNEL_1 = 1;
constexpr ledc_mode_t LEDC_LOW_SPEED_MODE = 0;
constexpr int LEDC_AUTO_CLK = 0;
constexpr int LEDC_INTR_DISABLE = 0;
constexpr int LEDC_SLEEP_MODE_NO_ALIVE_NO_PD = 0;

struct ledc_timer_config_t
{
    ledc_mode_t speed_mode = 0;
    ledc_timer_bit_t duty_resolution = 0;
    ledc_timer_t timer_num = 0;
    uint32_t freq_hz = 0;
    int clk_cfg = 0;
    bool deconfigure = false;
};

struct ledc_channel_config_t
{
    gpio_num_t gpio_num = GPIO_NUM_NC;
    ledc_mode_t speed_mode = 0;
    ledc_channel_t channel = 0;
    int intr_type = 0;
    ledc_timer_t timer_sel = 0;
    uint32_t duty = 0;
    int hpoint = 0;
    int sleep_mode = 0;
    struct
    {
        uint32_t output_invert = 0;
    } flags;
};

using rmt_channel_handle_t = void *;
constexpr int RMT_CLK_SRC_DEFAULT = 0;

struct rmt_symbol_word_t
{
    uint32_t duration0 = 0;
    uint32_t level0 = 0;
    uint32_t duration1 = 0;
    uint32_t level1 = 0;
};

struct rmt_rx_channel_config_t
{
    gpio_num_t gpio_num = GPIO_NUM_NC;
    int clk_src = 0;
    uint32_t resolution_hz = 0;
    uint32_t mem_block_symbols = 0;
    struct
    {
        bool with_dma = false;
        bool invert_in = false;
    } flags;
};

struct rmt_receive_config_t
{
    uint32_t signal_range_min_ns = 0;
    uint32_t signal_range_max_ns = 0;
};

struct rmt_rx_done_event_data_t
{
    rmt_symbol_word_t *received_symbols = nullptr;
    size_t num_symbols = 0;
};

using rmt_rx_done_callback_t =
    bool (*)(rmt_channel_handle_t,
             const rmt_rx_done_event_data_t *,
             void *);

struct rmt_rx_event_callbacks_t
{
    rmt_rx_done_callback_t on_recv_done = nullptr;
};

namespace fake_espidf
{
struct TaskRecord
{
    std::string name;
    TaskFunction_t function = nullptr;
    void *arg = nullptr;
    uint32_t stackSize = 0;
    UBaseType_t priority = 0;
    BaseType_t core = 0;
    bool deleted = false;
};

struct QueueRecord
{
    size_t itemSize = 0;
    std::vector<uint8_t> latest;
    bool hasItem = false;
};

struct RmtRecord
{
    rmt_rx_channel_config_t config{};
    rmt_rx_done_callback_t callback = nullptr;
    void *userContext = nullptr;
    bool enabled = false;
    bool receiveStarted = false;
};

struct State
{
    esp_err_t gpioConfigResult = ESP_OK;
    esp_err_t gpioSetLevelResult = ESP_OK;
    esp_err_t ledcTimerConfigResult = ESP_OK;
    esp_err_t ledcChannelConfigResult = ESP_OK;
    esp_err_t ledcSetDutyResult = ESP_OK;
    esp_err_t ledcUpdateDutyResult = ESP_OK;
    esp_err_t rmtNewRxChannelResult = ESP_OK;
    esp_err_t rmtRegisterCallbacksResult = ESP_OK;
    esp_err_t rmtEnableResult = ESP_OK;
    esp_err_t rmtReceiveResult = ESP_OK;
    bool queueCreateFails = false;
    bool semaphoreCreateFails = false;
    int failTaskCreateAtCall = 0;

    uint64_t espTimerUs = 0;
    TickType_t tickCount = 0;
    uint32_t romDelayUsTotal = 0;
    uint32_t yieldedFromIsrCount = 0;

    std::vector<gpio_config_t> gpioConfigs;
    std::map<gpio_num_t, int> gpioLevels;

    std::vector<ledc_timer_config_t> ledcTimerConfigs;
    std::vector<ledc_channel_config_t> ledcChannelConfigs;
    std::map<ledc_channel_t, uint32_t> ledcDuty;
    std::map<ledc_channel_t, uint32_t> ledcUpdateCount;

    std::deque<TaskRecord *> tasks;
    uint32_t taskCreateCount = 0;
    TaskHandle_t currentTask = nullptr;

    std::deque<QueueRecord *> queues;
    std::deque<void *> semaphores;
    std::deque<RmtRecord *> rmtChannels;
};

State &state();
void reset();
void yieldFromIsr();
void setEspTimerUs(uint64_t timestampUs);
bool emitRmtRx(rmt_channel_handle_t channel, uint32_t pulseWidthUs);
} // namespace fake_espidf

extern "C" {
esp_err_t gpio_config(const gpio_config_t *config);
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level);

esp_err_t ledc_timer_config(const ledc_timer_config_t *config);
esp_err_t ledc_channel_config(const ledc_channel_config_t *config);
esp_err_t ledc_set_duty(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t duty);
esp_err_t ledc_update_duty(ledc_mode_t speed_mode, ledc_channel_t channel);

rmt_channel_handle_t fake_current_rmt_channel();
esp_err_t rmt_new_rx_channel(const rmt_rx_channel_config_t *config, rmt_channel_handle_t *ret_channel);
esp_err_t rmt_rx_register_event_callbacks(rmt_channel_handle_t channel, const rmt_rx_event_callbacks_t *callbacks, void *user_data);
esp_err_t rmt_enable(rmt_channel_handle_t channel);
esp_err_t rmt_disable(rmt_channel_handle_t channel);
esp_err_t rmt_del_channel(rmt_channel_handle_t channel);
esp_err_t rmt_receive(rmt_channel_handle_t channel, void *buffer, size_t buffer_size, const rmt_receive_config_t *config);

QueueHandle_t xQueueCreate(UBaseType_t queueLength, UBaseType_t itemSize);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticksToWait);
BaseType_t xQueueOverwriteFromISR(QueueHandle_t queue, const void *item, BaseType_t *higherPriorityTaskWoken);
void vQueueDelete(QueueHandle_t queue);

SemaphoreHandle_t xSemaphoreCreateMutex();
void vSemaphoreDelete(SemaphoreHandle_t semaphore);

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t taskFunction, const char *name, uint32_t stackDepth, void *parameter, UBaseType_t priority, TaskHandle_t *createdTask, BaseType_t coreId);
void vTaskDelete(TaskHandle_t taskToDelete);
TickType_t xTaskGetTickCount();
TaskHandle_t xTaskGetCurrentTaskHandle();
void vTaskDelay(TickType_t ticksToDelay);
void vTaskDelayUntil(TickType_t *previousWakeTime, TickType_t timeIncrement);

int64_t esp_timer_get_time();
void esp_rom_delay_us(uint32_t delayUs);
}
