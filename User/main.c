/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-20
 * @brief       跑马灯 实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/Fuzzy_pid/fuzzy_pid.h"
#include "./BSP/LED/led.h"
#include <rthw.h>
#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h> // 引入bool类型

/* ----------- 全局变量区域 ----------- */

uint8_t rx_buffer[10]; // 接收缓冲区（和原来一样）
volatile uint8_t g_data_received_flag = 0;       // 【新增】接收完成标志位 (volatile 关键字非常重要！)



/* --------------- 引用区 --------------- */

extern UART_HandleTypeDef g_uart1_handle; // 确保能引用串口句柄


/* --------------- 线程区 --------------- */
static void led_thread_entry(void *parameter) {
    
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // 根据LED电路选择
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); // 翻转PA5电平
        rt_thread_mdelay(100); // 延时500ms
        rt_kprintf("led on\n");
    }
}

static void led1_thread_entry(void *parameter) {
    
    __HAL_RCC_GPIOE_CLK_ENABLE();
    
    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // 根据LED电路选择
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    while (1) {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5); // 翻转PA5电平
        rt_thread_mdelay(500); // 延时500ms
    }
}

static void uart_thread_entry(void *parameter) {

    HAL_UART_Receive_IT(&g_uart1_handle, rx_buffer, 10);
    while (1) {
                    
//        // --- 新增测试代码 START ---
//        // 每隔 1秒 发送一次 "Alive\r\n"
//        // 这一步是为了验证：波特率对不对？线通不通？
//        uint8_t test_msg[] = "Alive\r\n";
//        HAL_UART_Transmit(&g_uart1_handle, test_msg, sizeof(test_msg)-1, 10);
//        // --- 新增测试代码 END ---
//        
            // 轮询标志位
        if (g_data_received_flag == 1) {
            
            // 1. 清除标志位 (防止重复处理)
            g_data_received_flag = 0;
            // 打印当前收到的所有字节，看看有没有 AA 开头，55 结尾
//            printf("RX: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", 
//                   rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3], rx_buffer[4],
//                   rx_buffer[5], rx_buffer[6], rx_buffer[7], rx_buffer[8], rx_buffer[9]);
            // 2. 处理数据

            Process_LoRa_Or_Serial_Data();
        }
        rt_thread_mdelay(5);
    }
}

int main(void) {
    usart_init(115200);
    PID_Init();

    
    // 创建LED线程
    rt_thread_t led_thread = rt_thread_create("led_thread", led_thread_entry, RT_NULL, 
                                       512, 15, 5);
    if (led_thread != RT_NULL) {
        rt_thread_startup(led_thread); // 启动线程
    }
//    
//    // 创建LED1线程
//    rt_thread_t led1_thread = rt_thread_create("led1_thread", led1_thread_entry, RT_NULL, 
//                                       512, 15, 5);
//    if (led1_thread != RT_NULL) {
//        rt_thread_startup(led1_thread); // 启动线程
//    }
        // 创建串口线程
    rt_thread_t uart_thread = rt_thread_create("uart_thread", uart_thread_entry, RT_NULL, 
                                       512, 10, 5);
    if (uart_thread != RT_NULL) {
        rt_thread_startup(uart_thread); // 启动线程
    }
    return 0;
}
