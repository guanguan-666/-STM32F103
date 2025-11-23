#ifndef _Fuzzy_pid_H
#define _Fuzzy_pid_H
#include "./SYSTEM/sys/sys.h"

// 控制参数宏定义
#define FUZZY_TABLE_SIZE 13
#define FUZZY_MIN_INDEX -6
#define FUZZY_MAX_INDEX 6

#define TARGET_TEMP  6.0f   // 目标温度25度
//#define Q_FACTOR     0.4f    // 量化因子：6 / 15 = 0.4 (假设最大误差15度)
#define Q_FACTOR     2.0f    // 量化因子：6 / 15 = 0.4 (假设最大误差15度)

/* ----------------- 外部变量引用 ----------------- */
extern UART_HandleTypeDef g_uart1_handle; // 引用 main.c 定义的串口句柄
extern uint8_t rx_buffer[10];             // 引用 main.c 定义的缓冲区

// PID 结构体
typedef struct {
    float target_val;   // 目标值
    float actual_val;   // 实际值
    float err;          // 当前误差
    float err_last;     // 上一次误差
    float integral;     // 积分累计值
    float output;       // PID输出结果
    float last_derivative;//微分值
} PID_Controller;


// 共用体声明（保持不变）
typedef union {
    float f_val;
    uint8_t b_val[4];
} FloatUnion;       

extern UART_HandleTypeDef g_uart1_handle; // 确保能引用串口句柄
extern uint8_t rx_buffer[10];
/* ----------------- 外部可调用的函数声明 ----------------- */

// 初始化 PID
void PID_Init(void);

// 核心计算函数
float Fuzzy_PID_Compute(PID_Controller *pid, float target, float measure, float q_factor);

// 处理数据的主逻辑
void Process_LoRa_Or_Serial_Data(void);

// 发送数据
void Send_To_Matlab(float val);
//标准pid计算函数
float Standard_PID_Compute(PID_Controller *pid, float target, float measure);
#endif
