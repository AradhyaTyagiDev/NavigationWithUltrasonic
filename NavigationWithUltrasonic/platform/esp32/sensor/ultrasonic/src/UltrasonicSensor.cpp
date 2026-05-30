#include "sensor/ultrasonic/include/UltrasonicSensor.hpp"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>

static const char *TAG = "UltrasonicSensor";

static constexpr uint32_t TRIGGER_PULSE_US = 10;

static constexpr uint32_t RMT_RESOLUTION_HZ = 1000000;

static constexpr float SOUND_SPEED_DIVIDER = 58.0f;

UltrasonicSensor::UltrasonicSensor(const Config &config)
    : m_config(config)
{
}

UltrasonicSensor::~UltrasonicSensor()
{
    if (m_taskHandle)
    {
        vTaskDelete(m_taskHandle);
    }

    if (m_rxChannel)
    {
        rmt_disable(m_rxChannel);

        rmt_del_channel(m_rxChannel);
    }

    if (m_sensorQueue)
    {
        vQueueDelete(m_sensorQueue);
    }

    if (m_dataMutex)
    {
        vSemaphoreDelete(m_dataMutex);
    }

    shutdown();
}

bool UltrasonicSensor::initialize()
{
    m_sensorQueue = xQueueCreate(m_config.queueSize, sizeof(UltrasonicSensorData));

    if (!m_sensorQueue)
    {
        ESP_LOGE(TAG, "Failed to create queue");

        return false;
    }

    m_dataMutex = xSemaphoreCreateMutex();

    if (!m_dataMutex)
    {
        ESP_LOGE(TAG, "Failed to create mutex");

        return false;
    }

    if (!configureGPIO())
    {
        return false;
    }

    if (!configureRMT())
    {
        return false;
    }

    ESP_LOGI(TAG, "Ultrasonic initialized");

    return true;
}

bool UltrasonicSensor::start()
{
    BaseType_t result = xTaskCreatePinnedToCore(
        sensorTaskEntry,
        "UltrasonicTask",
        m_config.taskStackSize,
        this,
        m_config.taskPriority,
        &m_taskHandle,
        m_config.taskCore);

    return result == pdPASS;
}

// Shutdown
void UltrasonicSensor::shutdown()
{
    // Stop sensor task
    if (m_taskHandle != nullptr)
    {
        vTaskDelete(m_taskHandle);

        m_taskHandle = nullptr;
    }

    // Disable RMT channel
    if (m_rxChannel != nullptr)
    {
        rmt_disable(m_rxChannel);

        rmt_del_channel(m_rxChannel);

        m_rxChannel = nullptr;
    }

    // Reset trigger pin
    gpio_set_level(
        m_config.trigPin,
        0);

    // Delete queue
    if (m_sensorQueue != nullptr)
    {
        vQueueDelete(
            m_sensorQueue);

        m_sensorQueue = nullptr;
    }

    // Delete mutex
    if (m_dataMutex != nullptr)
    {
        vSemaphoreDelete(
            m_dataMutex);

        m_dataMutex = nullptr;
    }

    //-----------------------------------------

    //-----------------------------------------
    // Runtime flags
    m_echoReceived = false;

    ESP_LOGI(
        TAG,
        "Ultrasonic sensor shutdown complete");
}

bool UltrasonicSensor::configureGPIO()
{
    gpio_config_t config{};

    config.pin_bit_mask = (1ULL << m_config.trigPin);

    config.mode = GPIO_MODE_OUTPUT;

    config.pull_down_en = GPIO_PULLDOWN_DISABLE;

    config.pull_up_en = GPIO_PULLUP_DISABLE;

    config.intr_type = GPIO_INTR_DISABLE;

    if (gpio_config(&config) != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO config failed");

        return false;
    }

    gpio_set_level(m_config.trigPin, 0);

    return true;
}

bool UltrasonicSensor::configureRMT()
{
    rmt_rx_channel_config_t rxConfig{};

    rxConfig.gpio_num = m_config.echoPin;

    rxConfig.clk_src = RMT_CLK_SRC_DEFAULT;

    rxConfig.resolution_hz = RMT_RESOLUTION_HZ;

    rxConfig.mem_block_symbols = 64;

    rxConfig.flags.with_dma = false;

    rxConfig.flags.invert_in = false;

    if (rmt_new_rx_channel(
            &rxConfig,
            &m_rxChannel) != ESP_OK)
    {
        ESP_LOGE(TAG, "RMT RX create failed");

        return false;
    }

    rmt_rx_event_callbacks_t callbacks{};

    callbacks.on_recv_done = rmtRxCallback;

    if (rmt_rx_register_event_callbacks(
            m_rxChannel,
            &callbacks,
            this) != ESP_OK)
    {
        ESP_LOGE(TAG, "RMT callback registration failed");

        return false;
    }

    m_rxConfig.signal_range_min_ns = 1000;

    m_rxConfig.signal_range_max_ns = 30000000;

    if (rmt_enable(m_rxChannel) != ESP_OK)
    {
        ESP_LOGE(TAG, "RMT enable failed");

        return false;
    }

    return true;
}

bool UltrasonicSensor::sendTriggerPulse()
{
    gpio_set_level(m_config.trigPin, 0);

    esp_rom_delay_us(2);

    gpio_set_level(m_config.trigPin, 1);

    esp_rom_delay_us(TRIGGER_PULSE_US);

    gpio_set_level(m_config.trigPin, 0);

    return true;
}

bool UltrasonicSensor::startEchoReceive()
{
    m_echoReceived = false;

    esp_err_t result = rmt_receive(
        m_rxChannel,
        m_rxBuffer,
        sizeof(m_rxBuffer),
        &m_rxConfig);

    return result == ESP_OK;
}

bool UltrasonicSensor::rmtRxCallback(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *edata,
    void *user_ctx)
{
    UltrasonicSensor *sensor =
        static_cast<UltrasonicSensor *>(user_ctx);

    sensor->processEchoData(edata);

    return false;
}

void UltrasonicSensor::processEchoData(
    const rmt_rx_done_event_data_t *eventData)
{
    if (!eventData)
    {
        return;
    }

    const rmt_symbol_word_t *symbols =
        eventData->received_symbols;

    if (!symbols)
    {
        return;
    }

    uint32_t pulseWidthUs =
        symbols[0].duration0;

    if (pulseWidthUs == 0)
    {
        return;
    }

    UltrasonicSensorData data;

    data.pulseWidthUs = pulseWidthUs;

    data.timestampMs =
        static_cast<uint32_t>(
            esp_timer_get_time() / 1000ULL);

    BaseType_t higherPriorityTaskWoken =
        pdFALSE;

    // always keep newest frame. xQueueSendFromISR keeps older frames
    xQueueOverwriteFromISR(
        m_sensorQueue,
        &data,
        &higherPriorityTaskWoken);

    if (higherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }

    m_echoReceived = true;
}

void UltrasonicSensor::sensorTaskEntry(
    void *param)
{
    UltrasonicSensor *sensor =
        static_cast<UltrasonicSensor *>(param);

    sensor->sensorTaskLoop();
}

void UltrasonicSensor::sensorTaskLoop()
{
    const TickType_t delayTicks =
        pdMS_TO_TICKS(
            1000 / m_config.sensorFrequencyHz);

    while (true)
    {
        // Start RMT receiver and Now hardware waits for echo pulse.
        startEchoReceive();
        // Send trigger pulse after RMT receiver is ready.
        sendTriggerPulse();

        vTaskDelay(delayTicks);
    }
}

QueueHandle_t UltrasonicSensor::getQueueHandle() const
{
    return m_sensorQueue;
}

//====================================================
// Fetch latest sensor data
//====================================================
bool UltrasonicSensor::fetchLatestData(UltrasonicSensorData &outData)
{
    // Queue valid
    if (m_sensorQueue == nullptr)
    {
        return false;
    }

    // Read latest frame
    return (
        xQueueReceive(
            m_sensorQueue,
            &outData,
            0) == pdTRUE);
}