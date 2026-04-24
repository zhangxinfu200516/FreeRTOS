/**
 * @file tsk_config_and_callback.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象: 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象.
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象: 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象. 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象: 比如裁判系统对象, 底盘类要调用它做功率控制, 发射机构要调用它做出膛速度与射击频率的控制, 因此裁判系统是通用对象.
 * 这种对象以指针形式进行指定
 *
 */

#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H


/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "buzzer.h"
/* Exported macros -----------------------------------------------------------*/
#ifdef QUEUE
/* 定义消息类型枚举 */
typedef enum {
    MSG_TYPE_TEMPERATURE,
    MSG_TYPE_HUMIDITY,
    MSG_TYPE_PRESSURE,
    MSG_TYPE_LIGHT
} MessageType_t;

/* 定义通用消息结构 */
typedef struct {
    MessageType_t type;     // 消息类型
    uint8_t sensorID;       // 传感器ID
    uint32_t timestamp;     // 时间戳
    float value;            // 测量值
} SensorMessage_t;
#endif
/* 系统配置 */
#define SENSOR_QUEUE_LENGTH    10
#define MAX_SENSORS            3
#define SAMPLE_INTERVAL_MS     1000

/* 传感器数据类型定义 */
typedef enum {
    SENSOR_TEMPERATURE,
    SENSOR_HUMIDITY,
    SENSOR_PRESSURE
} SensorType_t;

typedef struct {
    SensorType_t type;          // 传感器类型
    uint32_t timestamp;         // 时间戳
    float value;                // 测量值
    uint8_t sensor_id;          // 传感器ID
    uint8_t error_code;         // 错误代码
} SensorData_t;

/* 处理后的数据类型定义 */
typedef struct {
    SensorType_t type;          // 传感器类型
    uint32_t timestamp;         // 时间戳
    float processed_value;      // 处理后的值
    uint8_t sensor_id;          // 传感器ID
    uint8_t quality;            // 数据质量指标
} ProcessedData_t;

/* 系统状态结构 */
typedef struct {
    uint32_t total_samples;
    uint32_t failed_samples;
    uint32_t last_sample_time;
    uint8_t system_status;
} SystemStatus_t;

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

extern uint16_t pwmVal;
/* Exported function declarations --------------------------------------------*/


#ifdef __cplusplus
extern "C" {
#endif
 
  void Task_Init();
  void Task_Loop(); 
 
#ifdef __cplusplus
}
#endif


#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
