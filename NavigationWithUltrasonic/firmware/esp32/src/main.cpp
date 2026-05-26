#include <stdio.h>

#include "sensor/ultrasonic/UltrasonicSensor.hpp"

extern "C" void app_main()
{
    UltrasonicSensor::Config config;

    //     config.trigPin = GPIO_NUM_25;

    //     config.echoPin = GPIO_NUM_32;

    //     config.taskPriority = 3;

    //     config.taskCore = 0;

    //     /// 20 measurements/sec
    //     config.sensorFrequencyHz = 20;

    //     static IUltrasonicSensor ultrasonic(config);

    //     if (!ultrasonic.initialize())
    //     {
    //         printf("Ultrasonic init failed\n");

    //         return;
    //     }

    //     if (!ultrasonic.start())
    //     {
    //         printf("Ultrasonic start failed\n");

    //         return;
    //     }

    //     UltrasonicSensorData data;

    //     while (true)
    //     {
    //         if (xQueueReceive(
    //                 ultrasonic.getQueueHandle(),
    //                 &data,
    //                 pdMS_TO_TICKS(100)) == pdTRUE)
    //         {
    //             float distanceCm =
    //                 static_cast<float>(data.pulseWidthUs) / 58.0f;

    //             printf(
    //                 "Distance: %.2f cm\n",
    //                 distanceCm);
    //         }
    //     }
}