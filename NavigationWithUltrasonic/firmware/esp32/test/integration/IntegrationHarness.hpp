#pragma once

#include "fakes/Fakes.hpp"

#include "filter/include/UltrasonicFilter.hpp"
#include "motion/include/MotionPlanner.hpp"
#include "motor/controller/include/MotorController.hpp"
#include "navigation/include/NavigationManager.hpp"
#include "obstacle/include/ObstacleManager.hpp"
#include "robot/include/RobotController.hpp"

struct IntegrationConfig
{
    UltrasonicFilterConfig filter;
    ObstacleManagerConfig obstacle;
    NavigationManagerConfig navigation;
    MotionPlannerConfig motion;
    MotorControllerConfig motor;
    RobotControllerConfig robot;
};

class IntegrationHarness
{
public:
    explicit IntegrationHarness(
        const IntegrationConfig &config = IntegrationConfig{})
        : filter(config.filter),
          obstacleManager(config.obstacle),
          navigationManager(config.navigation),
          motionPlanner(config.motion),
          motorController(
              motorDriver,
              motorMutex,
              logger,
              timer,
              config.motor),
          robotConfig(config.robot),
          robotController(
              ultrasonicSensor,
              filter,
              obstacleManager,
              navigationManager,
              motionPlanner,
              motorController,
              robotMutex,
              logger,
              timer,
              robotConfig)
    {
    }

    bool start()
    {
        return (
            robotController.initialize() &&
            robotController.start());
    }

    void publishDistanceCm(
        float distanceCm)
    {
        const uint32_t pulseWidthUs =
            static_cast<uint32_t>(
                distanceCm * 58.0f);

        ultrasonicSensor.publishPulse(
            pulseWidthUs,
            Timer::milliseconds(timer));
    }

    void stepMs(
        uint32_t deltaMs,
        bool publishSensorFrame = true,
        float distanceCm = 150.0f)
    {
        const uint32_t startMs =
            Timer::milliseconds(timer);

        const uint32_t endMs =
            startMs + deltaMs;

        while (Timer::milliseconds(timer) < endMs)
        {
            timer.advanceMs(5);

            const uint32_t nowMs =
                Timer::milliseconds(timer);

            if ((nowMs % 20U) == 0U)
            {
                if (publishSensorFrame)
                {
                    publishDistanceCm(distanceCm);
                }

                robotController.update();
            }

            motorController.update(nowMs);
        }
    }

    void runDistanceSequence(
        const float *distancesCm,
        uint32_t count)
    {
        for (uint32_t index = 0; index < count; ++index)
        {
            stepMs(
                20,
                true,
                distancesCm[index]);
        }
    }

    FakeUltrasonicSensor ultrasonicSensor;
    FakeMotorDriver motorDriver;
    FakeMutex robotMutex;
    FakeMutex motorMutex;
    FakeLogger logger;
    FakeTimer timer;

    UltrasonicFilter filter;
    ObstacleManager obstacleManager;
    NavigationManager navigationManager;
    MotionPlanner motionPlanner;
    MotorController motorController;
    RobotControllerConfig robotConfig;
    RobotController robotController;
};

inline IntegrationConfig productionLikeIntegrationConfig()
{
    IntegrationConfig config;

    config.filter.requiredStableFrames = 1;
    config.filter.largeJumpThresholdCm = 1000.0f;
    config.filter.maxRealisticVelocityCmPerSec = 10000.0f;
    config.filter.deadZoneCm = 0.0f;
    config.filter.confidenceDecayAlpha = 1.0f;

    config.obstacle.enableConfidenceGating = false;
    config.obstacle.enableStabilityValidation = false;

    config.navigation.minimumStateDurationMs = 0;
    config.navigation.avoidanceCooldownMs = 0;

    config.motion.minimumMotionDurationMs = 0;
    config.motion.motionCooldownMs = 0;
    config.motion.maxAccelerationPerSec = 100.0f;
    config.motion.maxDecelerationPerSec = 100.0f;
    config.motion.wheelDeadzone = 0.0f;

    config.motor.maximumAccelerationPercentPerSec = 100.0f;
    config.motor.maximumDecelerationPercentPerSec = 100.0f;
    config.motor.minimumEffectiveSpeedPercent = 0.0f;
    config.motor.enableStartupBoost = false;

    return config;
}
