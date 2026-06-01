#include "FakeEspIdf.hpp"

#include <algorithm>
#include <cstring>

namespace fake_espidf
{
namespace
{
    State g_state;

    template <typename T>
    void erasePointer(std::deque<T *> &items, T *item)
    {
        items.erase(
            std::remove(
                items.begin(),
                items.end(),
                item),
            items.end());
    }
}

State &state()
{
    return g_state;
}

void reset()
{
    for (TaskRecord *task : g_state.tasks)
    {
        delete task;
    }

    for (QueueRecord *queue : g_state.queues)
    {
        delete queue;
    }

    for (void *semaphore : g_state.semaphores)
    {
        delete static_cast<int *>(semaphore);
    }

    for (RmtRecord *channel : g_state.rmtChannels)
    {
        delete channel;
    }

    g_state = {};
}

void yieldFromIsr()
{
    g_state.yieldedFromIsrCount++;
}

void setEspTimerUs(uint64_t timestampUs)
{
    g_state.espTimerUs = timestampUs;
}

bool emitRmtRx(
    rmt_channel_handle_t channel,
    uint32_t pulseWidthUs)
{
    auto *record =
        static_cast<RmtRecord *>(channel);

    if (record == nullptr ||
        record->callback == nullptr)
    {
        return false;
    }

    rmt_symbol_word_t symbol{};
    symbol.duration0 = pulseWidthUs;

    rmt_rx_done_event_data_t event{};
    event.received_symbols = &symbol;
    event.num_symbols = 1;

    return record->callback(
        channel,
        &event,
        record->userContext);
}
} // namespace fake_espidf

extern "C" esp_err_t gpio_config(
    const gpio_config_t *config)
{
    if (config != nullptr)
    {
        fake_espidf::state()
            .gpioConfigs.push_back(*config);
    }

    return fake_espidf::state()
        .gpioConfigResult;
}

extern "C" esp_err_t gpio_set_level(
    gpio_num_t gpio_num,
    uint32_t level)
{
    fake_espidf::state()
        .gpioLevels[gpio_num] =
        static_cast<int>(level);

    return fake_espidf::state()
        .gpioSetLevelResult;
}

extern "C" esp_err_t ledc_timer_config(
    const ledc_timer_config_t *config)
{
    if (config != nullptr)
    {
        fake_espidf::state()
            .ledcTimerConfigs.push_back(*config);
    }

    return fake_espidf::state()
        .ledcTimerConfigResult;
}

extern "C" esp_err_t ledc_channel_config(
    const ledc_channel_config_t *config)
{
    if (config != nullptr)
    {
        fake_espidf::state()
            .ledcChannelConfigs.push_back(*config);

        fake_espidf::state()
            .ledcDuty[config->channel] =
            config->duty;
    }

    return fake_espidf::state()
        .ledcChannelConfigResult;
}

extern "C" esp_err_t ledc_set_duty(
    ledc_mode_t speed_mode,
    ledc_channel_t channel,
    uint32_t duty)
{
    (void)speed_mode;

    fake_espidf::state()
        .ledcDuty[channel] = duty;

    return fake_espidf::state()
        .ledcSetDutyResult;
}

extern "C" esp_err_t ledc_update_duty(
    ledc_mode_t speed_mode,
    ledc_channel_t channel)
{
    (void)speed_mode;

    fake_espidf::state()
        .ledcUpdateCount[channel]++;

    return fake_espidf::state()
        .ledcUpdateDutyResult;
}

extern "C" rmt_channel_handle_t fake_current_rmt_channel()
{
    if (fake_espidf::state()
            .rmtChannels.empty())
    {
        return nullptr;
    }

    return fake_espidf::state()
        .rmtChannels.back();
}

extern "C" esp_err_t rmt_new_rx_channel(
    const rmt_rx_channel_config_t *config,
    rmt_channel_handle_t *ret_channel)
{
    if (fake_espidf::state()
            .rmtNewRxChannelResult != ESP_OK)
    {
        return fake_espidf::state()
            .rmtNewRxChannelResult;
    }

    auto *record =
        new fake_espidf::RmtRecord();

    if (config != nullptr)
    {
        record->config = *config;
    }

    fake_espidf::state()
        .rmtChannels.push_back(record);

    if (ret_channel != nullptr)
    {
        *ret_channel = record;
    }

    return ESP_OK;
}

extern "C" esp_err_t rmt_rx_register_event_callbacks(
    rmt_channel_handle_t channel,
    const rmt_rx_event_callbacks_t *callbacks,
    void *user_data)
{
    if (fake_espidf::state()
            .rmtRegisterCallbacksResult != ESP_OK)
    {
        return fake_espidf::state()
            .rmtRegisterCallbacksResult;
    }

    auto *record =
        static_cast<fake_espidf::RmtRecord *>(channel);

    if (record != nullptr &&
        callbacks != nullptr)
    {
        record->callback = callbacks->on_recv_done;
        record->userContext = user_data;
    }

    return ESP_OK;
}

extern "C" esp_err_t rmt_enable(
    rmt_channel_handle_t channel)
{
    if (fake_espidf::state()
            .rmtEnableResult != ESP_OK)
    {
        return fake_espidf::state()
            .rmtEnableResult;
    }

    auto *record =
        static_cast<fake_espidf::RmtRecord *>(channel);

    if (record != nullptr)
    {
        record->enabled = true;
    }

    return ESP_OK;
}

extern "C" esp_err_t rmt_disable(
    rmt_channel_handle_t channel)
{
    auto *record =
        static_cast<fake_espidf::RmtRecord *>(channel);

    if (record != nullptr)
    {
        record->enabled = false;
    }

    return ESP_OK;
}

extern "C" esp_err_t rmt_del_channel(
    rmt_channel_handle_t channel)
{
    auto *record =
        static_cast<fake_espidf::RmtRecord *>(channel);

    if (record != nullptr)
    {
        fake_espidf::erasePointer(
            fake_espidf::state().rmtChannels,
            record);

        delete record;
    }

    return ESP_OK;
}

extern "C" esp_err_t rmt_receive(
    rmt_channel_handle_t channel,
    void *buffer,
    size_t buffer_size,
    const rmt_receive_config_t *config)
{
    (void)buffer;
    (void)buffer_size;
    (void)config;

    auto *record =
        static_cast<fake_espidf::RmtRecord *>(channel);

    if (record != nullptr)
    {
        record->receiveStarted = true;
    }

    return fake_espidf::state()
        .rmtReceiveResult;
}

extern "C" QueueHandle_t xQueueCreate(
    UBaseType_t queueLength,
    UBaseType_t itemSize)
{
    (void)queueLength;

    if (fake_espidf::state()
            .queueCreateFails)
    {
        return nullptr;
    }

    auto *queue =
        new fake_espidf::QueueRecord();

    queue->itemSize = itemSize;
    queue->latest.resize(itemSize);

    fake_espidf::state()
        .queues.push_back(queue);

    return queue;
}

extern "C" BaseType_t xQueueReceive(
    QueueHandle_t queueHandle,
    void *item,
    TickType_t ticksToWait)
{
    (void)ticksToWait;

    auto *queue =
        static_cast<fake_espidf::QueueRecord *>(queueHandle);

    if (queue == nullptr ||
        item == nullptr ||
        !queue->hasItem)
    {
        return pdFALSE;
    }

    std::memcpy(
        item,
        queue->latest.data(),
        queue->itemSize);

    queue->hasItem = false;

    return pdTRUE;
}

extern "C" BaseType_t xQueueOverwriteFromISR(
    QueueHandle_t queueHandle,
    const void *item,
    BaseType_t *higherPriorityTaskWoken)
{
    auto *queue =
        static_cast<fake_espidf::QueueRecord *>(queueHandle);

    if (queue == nullptr ||
        item == nullptr)
    {
        return pdFALSE;
    }

    std::memcpy(
        queue->latest.data(),
        item,
        queue->itemSize);

    queue->hasItem = true;

    if (higherPriorityTaskWoken != nullptr)
    {
        *higherPriorityTaskWoken = pdFALSE;
    }

    return pdTRUE;
}

extern "C" void vQueueDelete(
    QueueHandle_t queueHandle)
{
    auto *queue =
        static_cast<fake_espidf::QueueRecord *>(queueHandle);

    if (queue != nullptr)
    {
        fake_espidf::erasePointer(
            fake_espidf::state().queues,
            queue);

        delete queue;
    }
}

extern "C" SemaphoreHandle_t xSemaphoreCreateMutex()
{
    if (fake_espidf::state()
            .semaphoreCreateFails)
    {
        return nullptr;
    }

    auto *semaphore =
        new int(1);

    fake_espidf::state()
        .semaphores.push_back(semaphore);

    return semaphore;
}

extern "C" void vSemaphoreDelete(
    SemaphoreHandle_t semaphore)
{
    if (semaphore != nullptr)
    {
        fake_espidf::erasePointer(
            fake_espidf::state().semaphores,
            semaphore);

        delete static_cast<int *>(semaphore);
    }
}

extern "C" BaseType_t xTaskCreatePinnedToCore(
    TaskFunction_t taskFunction,
    const char *name,
    uint32_t stackDepth,
    void *parameter,
    UBaseType_t priority,
    TaskHandle_t *createdTask,
    BaseType_t coreId)
{
    fake_espidf::state()
        .taskCreateCount++;

    const int failAt =
        fake_espidf::state()
            .failTaskCreateAtCall;

    if (failAt > 0 &&
        fake_espidf::state()
                .taskCreateCount ==
            static_cast<uint32_t>(failAt))
    {
        if (createdTask != nullptr)
        {
            *createdTask = nullptr;
        }

        return pdFALSE;
    }

    auto *task =
        new fake_espidf::TaskRecord();

    task->name =
        name == nullptr ? "" : name;
    task->function = taskFunction;
    task->arg = parameter;
    task->stackSize = stackDepth;
    task->priority = priority;
    task->core = coreId;

    fake_espidf::state()
        .tasks.push_back(task);

    if (createdTask != nullptr)
    {
        *createdTask = task;
    }

    return pdPASS;
}

extern "C" void vTaskDelete(
    TaskHandle_t taskToDelete)
{
    if (taskToDelete == nullptr)
    {
        fake_espidf::state()
            .currentTask = nullptr;

        return;
    }

    auto *task =
        static_cast<fake_espidf::TaskRecord *>(taskToDelete);

    task->deleted = true;

    fake_espidf::erasePointer(
        fake_espidf::state().tasks,
        task);

    delete task;
}

extern "C" TickType_t xTaskGetTickCount()
{
    return fake_espidf::state()
        .tickCount;
}

extern "C" TaskHandle_t xTaskGetCurrentTaskHandle()
{
    return fake_espidf::state()
        .currentTask;
}

extern "C" void vTaskDelay(
    TickType_t ticksToDelay)
{
    fake_espidf::state()
        .tickCount += ticksToDelay;
}

extern "C" void vTaskDelayUntil(
    TickType_t *previousWakeTime,
    TickType_t timeIncrement)
{
    if (previousWakeTime != nullptr)
    {
        *previousWakeTime += timeIncrement;
        fake_espidf::state()
            .tickCount = *previousWakeTime;
    }
    else
    {
        fake_espidf::state()
            .tickCount += timeIncrement;
    }
}

extern "C" int64_t esp_timer_get_time()
{
    return static_cast<int64_t>(
        fake_espidf::state()
            .espTimerUs);
}

extern "C" void esp_rom_delay_us(
    uint32_t delayUs)
{
    fake_espidf::state()
        .romDelayUsTotal += delayUs;

    fake_espidf::state()
        .espTimerUs += delayUs;
}
