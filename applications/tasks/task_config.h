/*
 * Application task inventory: one descriptor per task file.
 *
 * This layer owns only RT-Thread thread stacks, priorities and periods. Every
 * application scheduling detail stays here. Each task owns its cycle.
 */
#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

#include <rtthread.h>

typedef struct {
    const char *name;
    void (*entry)(void *parameter);
    rt_uint8_t *stack;
    rt_uint32_t stack_size;
    rt_uint8_t priority;
} TaskDescriptor;

/* RT-Thread: a numerically lower value runs first. */
#define TASK_ACTUATOR_PRIORITY 6u
#define TASK_SENSOR_PRIORITY 7u
#define TASK_MISSION_PRIORITY 12u
#define TASK_MPC_PRIORITY 13u
#define TASK_SIM_PRIORITY 17u
#define TASK_COMMUNICATION_PRIORITY 18u
#define TASK_SUPERVISOR_PRIORITY 20u
#define TASK_LOGGER_PRIORITY 24u
#define TASK_FLASH_PRIORITY 25u

/* Periods / event waits in milliseconds. */
#define TASK_SENSOR_PERIOD_MS 1u
#define TASK_MISSION_PERIOD_MS 20u
#define TASK_MPC_PERIOD_MS 20u
#define TASK_ACTUATOR_PERIOD_MS 1u
#define TASK_SIM_PERIOD_MS 1u
#define TASK_COMMUNICATION_PERIOD_MS 4u
#define TASK_SUPERVISOR_PERIOD_MS 100u
#define TASK_LOGGER_PERIOD_MS 50u
#define TASK_FLASH_PERIOD_MS 50u

extern const TaskDescriptor task_sensor;
extern const TaskDescriptor task_mission;
extern const TaskDescriptor task_mpc;
extern const TaskDescriptor task_actuator;
extern const TaskDescriptor task_sim;
extern const TaskDescriptor task_communication;
extern const TaskDescriptor task_supervisor;
#ifdef FLIGHT_LOGGER_ENABLE
extern const TaskDescriptor task_logger;
#endif
#ifdef FLIGHT_NOR_LOG_ENABLE
extern const TaskDescriptor task_flash;
#endif

#endif /* TASK_CONFIG_H */
