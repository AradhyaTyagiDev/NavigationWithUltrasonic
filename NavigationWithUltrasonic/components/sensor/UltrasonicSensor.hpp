#pragma once

#include <stdint.h>

#include <driver/gpio.h>
#include <driver/rmt_rx.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "UltrasonicTypes.hpp"

class UltrasonicSensor
{
public:
    struct Config
    {
        gpio_num_t trigPin;

        gpio_num_t echoPin;

        uint32_t taskStackSize = 4096;

        UBaseType_t taskPriority = 4;

        // Core 0 reserved for I/O
        BaseType_t taskCore = 0;

        // 20Hz = every 50ms
        uint32_t sensorFrequencyHz = 20;
    };

public:
    explicit UltrasonicSensor(const Config &config);

    ~UltrasonicSensor();

    bool initialize();

    bool start();

    bool getLatestData(UltrasonicSensorData &outData);

    QueueHandle_t getQueueHandle() const;

private:
    static void sensorTaskEntry(void *param);

    void sensorTaskLoop();

    bool configureGPIO();

    bool configureRMT();

    bool sendTriggerPulse();

    bool startEchoReceive();

    void processEchoData(
        const rmt_rx_done_event_data_t *eventData);

    static bool rmtRxCallback(
        rmt_channel_handle_t channel,
        const rmt_rx_done_event_data_t *edata,
        void *user_ctx);

private:
    Config m_config;

    TaskHandle_t m_taskHandle = nullptr;

    QueueHandle_t m_sensorQueue = nullptr;

    SemaphoreHandle_t m_dataMutex = nullptr;

    UltrasonicSensorData m_latestData{};

    rmt_channel_handle_t m_rxChannel = nullptr;

    rmt_receive_config_t m_rxConfig{};

    rmt_symbol_word_t m_rxBuffer[64];

    volatile bool m_echoReceived = false;
};