#include "./BSP/Fuzzy_pid/fuzzy_pid.h"
#include <stdio.h>



#define TABLE_OFFSET 6   // 因为索引 -6 对应数组下标 0
#define MAX_IDX      6
#define MIN_IDX     -6

/* ----------------- 外部变量引用 ----------------- */
extern UART_HandleTypeDef g_uart1_handle; // 引用 main.c 定义的串口句柄
extern uint8_t rx_buffer[10];             // 引用 main.c 定义的缓冲区

/* ----------------- 全局变量定义 ----------------- */
PID_Controller g_pid; // 全局 PID 对象

// 基础 PID 参数
float Base_Kp = 8.7f; 
float Base_Ki = 0.18f;
float Base_Kd = 25.4f;

// 与 Simulink 保持一致的参数
float Kp = 6.0f;
float Ki = 0.04f;
float Kd = 350.0f;


/* ----------------- 模糊规则表 (放在 .c 文件中) ----------------- */
const float Fuzzy_Kp_Table[13][13] = {
    {-5.372, -5.253, -5.372, -5.253, -5.370, -4.292, -4.003, -3.998, -3.999, -3.000, -2.001, -2.002, -2.001}, // E = -6
    {-5.253, -5.253, -5.253, -4.280, -4.267, -3.273, -3.005, -3.000, -3.000, -1.999, -1.001, -1.001, -1.001}, // E = -5
    {-5.372, -5.253, -5.372, -4.280, -3.999, -3.026, -2.099, -2.002, -2.001, -1.001,  0.000,  0.000,  0.000}, // E = -4
    {-4.267, -4.267, -4.267, -4.267, -3.998, -3.000, -2.131, -1.001, -1.001, -0.000,  1.001,  1.001,  1.001}, // E = -3
    {-3.999, -3.998, -3.999, -3.998, -3.999, -3.026, -2.099, -1.001,  0.000,  1.001,  2.001,  2.002,  2.001}, // E = -2
    {-3.998, -3.998, -3.998, -4.241, -4.267, -2.274, -1.114, -0.000,  1.001,  1.999,  3.000,  3.000,  3.000}, // E = -1
    {-3.999, -3.998, -3.999, -4.241, -5.370, -1.855, -0.173,  1.001,  2.001,  3.000,  3.999,  3.998,  3.999}, // E = 0
    {-3.000, -3.000, -3.000, -2.286, -1.797, -0.486,  0.834,  1.001,  2.002,  3.000,  3.998,  4.267,  4.267}, // E = 1
    {-2.001, -2.002, -2.001, -1.030,  0.000,  0.974,  1.899,  2.002,  2.001,  3.000,  3.999,  4.267,  5.372}, // E = 2
    {-2.002, -1.001, -1.001, -0.042,  1.001,  1.001,  1.001,  1.001,  1.001,  1.999,  3.998,  3.273,  3.114}, // E = 3
    {-2.001, -1.001,  0.000,  0.970,  2.001,  1.026,  0.099,  0.000,  0.000,  1.999,  3.999,  3.000,  2.001}, // E = 4
    {-1.001, -0.000,  1.001,  1.958,  3.000,  1.999,  1.999,  1.999,  1.999,  2.316,  4.267,  3.273,  3.114}, // E = 5
    { 0.000,  1.001,  2.001,  2.970,  3.999,  3.999,  3.999,  3.998,  3.999,  4.267,  5.372,  5.253,  5.372}, // E = 6
};
const float Fuzzy_Ki_Table[13][13] = {
    {-5.372, -5.253, -5.372, -4.280, -3.999, -3.999, -3.999, -2.987, -1.976, -1.976, -1.976, -1.976, -1.976}, // E = -6
    {-4.267, -4.267, -4.267, -3.295, -2.987, -2.987, -2.987, -2.987, -1.976, -0.988, -0.988, -0.988, -0.988}, // E = -5
    {-3.999, -3.998, -3.999, -3.018, -1.976, -1.976, -1.976, -1.976, -1.976, -0.988,  0.000,  0.000,  0.000}, // E = -4
    {-4.267, -4.267, -3.998, -3.018, -1.976, -1.976, -1.976, -0.988, -0.988,  0.024,  1.012,  1.012,  1.012}, // E = -3
    {-5.372, -4.267, -3.999, -3.018, -1.976, -1.976, -1.976, -0.988,  0.000,  1.012,  2.024,  2.024,  2.024}, // E = -2
    {-5.253, -4.267, -3.998, -3.018, -1.976, -0.988, -0.988,  0.024,  1.012,  1.999,  3.011,  3.011,  3.011}, // E = -1
    {-5.372, -4.267, -3.999, -3.018, -1.976, -1.014, -0.097,  1.012,  2.024,  3.011,  3.999,  3.998,  3.999}, // E = 0
    {-3.097, -3.261, -2.987, -2.039, -0.988,  0.024,  0.908,  1.012,  2.024,  3.011,  3.998,  3.998,  3.998}, // E = 1
    {-1.976, -1.976, -1.976, -1.017,  0.000,  0.986,  1.921,  2.024,  2.024,  3.011,  3.999,  3.998,  3.999}, // E = 2
    {-0.988, -0.988, -0.988, -0.018,  1.012,  1.012,  1.889,  2.024,  2.024,  3.011,  3.998,  3.998,  3.998}, // E = 3
    { 0.000,  0.000,  0.000,  0.982,  2.024,  2.023,  2.024,  2.024,  2.024,  3.011,  3.999,  3.998,  3.999}, // E = 4
    { 0.000,  0.000,  0.000,  0.982,  1.012,  1.012,  1.012,  1.012,  1.012,  1.999,  1.999,  1.999,  1.999}, // E = 5
    { 0.000,  0.000,  0.000,  0.000,  0.000, -0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000}, // E = 6
};
const float Fuzzy_Kd_Table[13][13] = {
    {-5.372, -4.267, -3.999, -3.030, -2.001, -2.001, -2.001, -2.002, -2.001, -2.002, -2.001, -2.002, -2.001}, // E = -6
    {-5.253, -4.267, -4.267, -3.307, -3.000, -3.000, -3.000, -3.000, -2.002, -2.002, -2.002, -1.001, -1.001}, // E = -5
    {-5.372, -5.253, -5.372, -4.280, -3.999, -3.999, -3.999, -3.000, -2.001, -2.002, -2.001, -1.001,  0.000}, // E = -4
    {-5.253, -5.253, -5.253, -4.280, -3.998, -3.998, -3.998, -3.000, -2.002, -2.002, -2.002, -1.001,  0.000}, // E = -3
    {-5.372, -5.253, -5.372, -4.280, -3.999, -3.999, -3.999, -3.000, -2.001, -2.002, -2.001, -1.001,  0.000}, // E = -2
    {-4.267, -3.273, -3.114, -3.255, -3.000, -3.000, -3.000, -3.000, -2.002, -2.002, -2.002, -1.001,  0.000}, // E = -1
    {-3.999, -3.000, -2.001, -2.002, -2.001, -2.001, -2.001, -2.002, -2.001, -2.002, -2.001, -1.001,  0.000}, // E = 0
    {-4.267, -3.273, -3.000, -3.244, -3.114, -3.273, -3.005, -3.273, -3.114, -3.114, -3.114, -2.012, -1.797}, // E = 1
    {-5.372, -4.267, -3.999, -4.241, -5.370, -4.292, -4.003, -4.267, -5.372, -5.253, -5.372, -5.253, -5.372}, // E = 2
    {-3.114, -3.273, -3.000, -3.244, -4.267, -3.273, -3.005, -3.273, -4.267, -3.273, -3.114, -3.114, -3.114}, // E = 3
    {-2.001, -2.002, -2.001, -2.970, -3.999, -3.026, -2.099, -3.000, -3.999, -3.000, -2.001, -2.002, -2.001}, // E = 4
    {-3.000, -3.000, -3.000, -3.000, -3.998, -3.000, -3.000, -1.999, -1.999, -1.999, -1.001, -0.000, -0.000}, // E = 5
    {-3.999, -3.998, -3.999, -3.998, -3.999, -3.999, -3.999, -1.999,  0.000,  0.000,  0.000,  1.001,  2.001}, // E = 6
};

#define FUZZY_TABLE_SIZE 13
#define FUZZY_MIN_INDEX -6
#define FUZZY_MAX_INDEX 6

/* ----------------- 函数实现 ----------------- */

// 初始化函数 (建议加上)
void PID_Init(void) {
    g_pid.err = 0;
    g_pid.err_last = 0;
    g_pid.integral = 0;
    g_pid.output = 0;
}

/**
 * @brief  模糊PID计算函数 (STM32核心算法)
 * @param  pid: PID结构体指针
 * @param  target: 目标值 (如设定温度)
 * @param  measure: 实际值 (如传感器温度)
 * @param  q_factor: 量化因子 (把实际误差缩放到 -6~+6 的范围)
 *         例如：如果允许最大温差是 15度，那么 q_factor = 6.0 / 15.0 = 0.4
 */
float Fuzzy_PID_Compute(PID_Controller *pid, float target, float measure, float q_factor)
{
    // 1. 计算基础误差
    pid->target_val = target;
    pid->actual_val = measure;
    pid->err = measure - target;
    float err_change = pid->err - pid->err_last;
    
    // 2. 模糊化 (量化到 -6 ~ +6 的整数索引)
    int E_idx  = (int)(pid->err * q_factor);
    int EC_idx = (int)(err_change * q_factor);
    
    // 3. 限幅 (防止数组越界，这是嵌入式查表必须做的)
    if (E_idx > MAX_IDX)  E_idx = MAX_IDX;
    if (E_idx < MIN_IDX)  E_idx = MIN_IDX;
    
    if (EC_idx > MAX_IDX) EC_idx = MAX_IDX;
    if (EC_idx < MIN_IDX) EC_idx = MIN_IDX;
    
    // 4. 查表 (获取 PID 参数的修正量 Delta)
    // 注意：数组下标 = 索引 + 偏移量
    int row = E_idx + TABLE_OFFSET;
    int col = EC_idx + TABLE_OFFSET;
    
    float delta_Kp = Fuzzy_Kp_Table[row][col];
    float delta_Ki = Fuzzy_Ki_Table[row][col];
    float delta_Kd = Fuzzy_Kd_Table[row][col];
    
    // 5. 计算最终 PID 参数 (基础值 + 模糊修正值)
    //    注意：这里加入了一个系数 0.1，因为FIS输出范围是-6到6，直接加可能太大了
    //    这个系数需要在实验中微调
    float final_Kp = Base_Kp + delta_Kp*0.005; 
    float final_Ki = Base_Ki + delta_Ki*0.0165;
    float final_Kd = Base_Kd + delta_Kd*5.6;
    
    // 防止参数变成负数 (PID系数通常不能为负)
    if(final_Kp < 0) final_Kp = 0;
    if(final_Ki < 0) final_Ki = 0;
    if(final_Kd < 0) final_Kd = 0;
    
    // 6. 常规 PID 计算公式
    pid->integral += pid->err;
    
    // 积分限幅 (防止积分饱和)
    if(pid->integral > 1000) pid->integral = 1000;
    if(pid->integral < -1000) pid->integral = -1000;
    
    pid->output = (final_Kp * pid->err) + 
                  (final_Ki * pid->integral) + 
                  (final_Kd * err_change);
                  
    pid->err_last = pid->err;
    
    return pid->output;
}

/**
 * @brief       发送数据给Matlab
 * @param       val 浮点型数据
 * @retval      无
 */
void Send_To_Matlab(float val) {
    FloatUnion u_out;
    u_out.f_val = val;
    
    uint8_t tx_buf[6];
    tx_buf[0] = 0xBB;
    tx_buf[1] = u_out.b_val[0];
    tx_buf[2] = u_out.b_val[1];
    tx_buf[3] = u_out.b_val[2];
    tx_buf[4] = u_out.b_val[3];
    tx_buf[5] = 0x0D;
    
    // 即使这里阻塞10ms，也不会影响数据的接收，
    // 因为接收是由硬件和中断后台处理的。
    HAL_UART_Transmit(&g_uart1_handle, tx_buf, 6, 10);
}

void Process_LoRa_Or_Serial_Data(void) {
    // 1. 再次校验头尾，确保数据完整性

    if (rx_buffer[0] == 0xAA && rx_buffer[9] == 0x55) {
        FloatUnion u_temp, u_pres;
        float rcv_temp, rcv_pres;

        // 2. 提取数据 (和之前一样)
        u_temp.b_val[0] = rx_buffer[1];
        u_temp.b_val[1] = rx_buffer[2];
        u_temp.b_val[2] = rx_buffer[3];
        u_temp.b_val[3] = rx_buffer[4];
        
        rcv_temp = u_temp.f_val;

//        // 3. 运行 PID (耗时操作)
//        float pid_out = Fuzzy_PID_Compute(&g_pid, TARGET_TEMP, rcv_temp, Q_FACTOR);
        
        // 3. 运行 PID (耗时操作)
        float pid_out = Standard_PID_Compute(&g_pid, TARGET_TEMP, rcv_temp);
        // 4. 发送回传 (耗时操作 - 阻塞发送)
        // 现在在这里调用 Send_To_Matlab 是安全的，
        // 因为我们在主循环里，不会卡死其他中断。
        Send_To_Matlab(pid_out);
    }
}

float Standard_PID_Compute(PID_Controller *pid, float target, float measure) {

    
    // 1. 误差计算 (制冷模式)
    // 实际 30 > 目标 6 -> 误差 +24 -> 输出加大 -> 制冷
    pid->target_val = target;
    pid->actual_val = measure;
    pid->err = measure - target;

    // 2. 比例项 (P)
    float P_term = Kp * pid->err;

    // 3. 积分项 (I) - 【核心调整：积分分离】
    // 策略：只有当温度快到了（误差 < 3度）才开始积分
    // 原因：误差大时(24度)全靠P项就够了(30*24=720)，这时候积分只会帮倒忙
    if (pid->err > -4.0f && pid->err < 4.0f) {
        pid->integral += pid->err;
        
        // 积分限幅 (防止数值溢出)
        if (pid->integral > 4000) pid->integral = 4000;
        if (pid->integral < -4000) pid->integral = -4000;
    } else {
        // 误差很大时，清空积分，保证“轻装上阵”
        pid->integral = 0; 
    }
    float I_term = Ki * pid->integral;

    // 4. 微分项 (D)
    // 简单的低通滤波微分 (Lpf_Factor = 0.1 ~ 0.5)
    float derivative = (pid->err - pid->err_last);
    pid->last_derivative = pid->last_derivative * 0.7 + derivative * 0.3;
    float D_term = Kd * pid->last_derivative;
    // 5. 计算总输出
    // 临时变量，先不直接赋值给 output，方便做抗饱和判断
    float temp_output = P_term + I_term + D_term;

    // 6. 输出限幅 & 抗饱和 (Anti-Windup)
    // 这是为了防止阀门满载后，积分项还在空转
    if (temp_output > 800.0f) {
        pid->output = 800.0f;
    } else if (temp_output < 0.0f) {
        pid->output = 0.0f;
    } else {
        pid->output = temp_output;
    }

    // 更新历史误差
    pid->err_last = pid->err;

    return pid->output;

}



//float Standard_PID_Compute(PID_Controller *pid, float target, float measure) {

//    pid->actual_val = measure;
//    pid->target_val = target;
//    pid->err = pid->actual_val - pid->target_val;   //计算误差
//    
//    float Kp_term = Kp * pid->err;
//    if(pid->err < 20.0f && pid->err > -20.0f){
//        pid->integral = pid->integral + pid->err;
//        if(pid->integral > 4000) pid->integral = 4000;  //积分限幅
//        if(pid->integral < -4000) pid->integral = -4000;
//    }else{
//        pid->integral = 0;  //积分分离
//    }
//    float Ki_term = Ki * pid->integral;
//    float Kd_term = Kd * (pid->err - pid->err_last);
//    float Temp_output = Kp_term + Ki_term + Kd_term;
//    if(pid->output > 1000.0f){ //输出限幅
//        pid->output = 1000.0f;
//        pid->integral = pid->integral - pid->err;   //抗积分饱和
//    }
//    else if(pid->output < 0.0f){
//        pid->output = 0.0f;
//    }
//    else{
//        pid->output = Temp_output;
//    }
//    pid->err_last = pid->err;
//    return pid->output;
//}