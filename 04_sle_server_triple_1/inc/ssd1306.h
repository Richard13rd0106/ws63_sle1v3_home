/*
 * Copyright (c) 2024 HiSilicon Technologies CO., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file ssd1306.h
 * @brief SSD1306 OLED显示屏驱动头文件
 * 
 * 本文件提供了SSD1306 OLED显示屏的完整驱动接口，支持：
 * - 基本显示控制（开关、对比度调节）
 * - 图形绘制（点、线、矩形、圆形）
 * - 文本显示（字符、字符串）
 * - 位图显示
 * - 屏幕缓冲区管理
 * 
 * 使用步骤：
 * 1. 调用 ssd1306_Init() 初始化显示屏
 * 2. 使用绘制函数在缓冲区中绘制内容
 * 3. 调用 ssd1306_UpdateScreen() 将缓冲区内容显示到屏幕
 */

#ifndef SSD1306_H
#define SSD1306_H

#include <stddef.h>
#include <stdint.h>
#include "ssd1306_fonts.h"

/* ========== 显示屏参数配置 ========== */
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH 128          /**< 显示屏宽度（像素） */
#endif

#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT 64          /**< 显示屏高度（像素） */
#endif

#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8)  /**< 显示缓冲区大小（字节） */

/* ========== 颜色枚举 ========== */
/**
 * @brief 像素颜色枚举
 * 
 * SSD1306是单色显示屏，只有黑白两种颜色
 */
typedef enum {
    Black = 0x00,   /**< 黑色 - 像素关闭，不发光 */
    White = 0x01    /**< 白色 - 像素开启，发光 */
} SSD1306_COLOR;

/* ========== 错误码枚举 ========== */
/**
 * @brief 操作结果枚举
 */
typedef enum {
    SSD1306_OK = 0x00,      /**< 操作成功 */
    SSD1306_ERR = 0x01      /**< 操作失败 */
} SSD1306_Error_t;

/* ========== 显示屏状态结构体 ========== */
/**
 * @brief SSD1306显示屏状态结构体
 * 
 * 保存显示屏的当前状态信息
 */
typedef struct {
    uint16_t CurrentX;      /**< 当前光标X坐标 */
    uint16_t CurrentY;      /**< 当前光标Y坐标 */
    uint8_t Inverted;       /**< 是否反色显示 (0=正常, 1=反色) */
    uint8_t Initialized;    /**< 是否已初始化 (0=未初始化, 1=已初始化) */
    uint8_t DisplayOn;      /**< 显示是否开启 (0=关闭, 1=开启) */
} SSD1306_t;

/* ========== 顶点结构体 ========== */
/**
 * @brief 顶点坐标结构体
 * 
 * 用于绘制多边形时定义顶点坐标
 */
typedef struct {
    uint8_t x;  /**< X坐标 */
    uint8_t y;  /**< Y坐标 */
} SSD1306_VERTEX;

/* ========== 基本控制函数 ========== */

/**
 * @brief 初始化SSD1306显示屏
 * 
 * 执行完整的初始化序列，包括：
 * - 硬件复位
 * - 发送初始化命令序列
 * - 清空屏幕
 * - 设置默认参数
 * 
 * @note 使用显示屏前必须先调用此函数
 * 
 * 使用示例：
 * @code
 * ssd1306_Init();  // 初始化显示屏
 * @endcode
 */
void ssd1306_Init(void);

/**
 * @brief 填充整个屏幕
 * 
 * 用指定颜色填充整个显示缓冲区
 * 
 * @param color 填充颜色 (Black=黑色, White=白色)
 * 
 * 使用示例：
 * @code
 * ssd1306_Fill(Black);         // 清空屏幕（全黑）
 * ssd1306_Fill(White);         // 填充屏幕（全白）
 * ssd1306_UpdateScreen();      // 更新显示
 * @endcode
 */
void ssd1306_Fill(SSD1306_COLOR color);

/**
 * @brief 设置光标位置
 * 
 * 设置下次绘制文本的起始位置
 * 
 * @param x X坐标 (0 ~ SSD1306_WIDTH-1)
 * @param y Y坐标 (0 ~ SSD1306_HEIGHT-1)
 * 
 * 使用示例：
 * @code
 * ssd1306_SetCursor(0, 0);     // 设置到左上角
 * ssd1306_SetCursor(64, 32);   // 设置到屏幕中央
 * @endcode
 */
void ssd1306_SetCursor(uint8_t x, uint8_t y);

/**
 * @brief 更新屏幕显示
 * 
 * 将显示缓冲区的内容传输到显示屏硬件，使绘制的内容可见
 * 
 * @note 所有绘制操作都是在缓冲区中进行，必须调用此函数才能在屏幕上看到效果
 * 
 * 使用示例：
 * @code
 * ssd1306_DrawString("Hello", Font_7x10, White);  // 在缓冲区绘制文本
 * ssd1306_UpdateScreen();                         // 显示到屏幕
 * @endcode
 */
void ssd1306_UpdateScreen(void);

/* ========== 文本绘制函数 ========== */

/**
 * @brief 绘制单个字符
 * 
 * 在当前光标位置绘制一个字符，绘制后光标自动右移
 * 
 * @param ch 要绘制的字符 (ASCII 32-126)
 * @param Font 字体 (Font_7x10, Font_11x18, Font_16x26等)
 * @param color 字符颜色 (Black=黑色, White=白色)
 * @return 成功返回绘制的字符，失败返回0
 * 
 * 使用示例：
 * @code
 * ssd1306_SetCursor(10, 10);
 * ssd1306_DrawChar('A', Font_7x10, White);
 * ssd1306_UpdateScreen();
 * @endcode
 */
char ssd1306_DrawChar(char ch, FontDef Font, SSD1306_COLOR color);

/**
 * @brief 绘制字符串
 * 
 * 在当前光标位置绘制字符串，绘制后光标移动到字符串末尾
 * 
 * @param str 要绘制的字符串
 * @param Font 字体 (Font_7x10, Font_11x18, Font_16x26等)
 * @param color 字符颜色 (Black=黑色, White=白色)
 * @return 成功返回0，失败返回无法绘制的字符
 * 
 * 使用示例：
 * @code
 * ssd1306_SetCursor(0, 0);
 * ssd1306_DrawString("Hello World!", Font_7x10, White);
 * ssd1306_UpdateScreen();
 * @endcode
 */
char ssd1306_DrawString(char *str, FontDef Font, SSD1306_COLOR color);

/* ========== 图形绘制函数 ========== */

/**
 * @brief 绘制像素点
 * 
 * 在指定位置绘制一个像素点
 * 
 * @param x X坐标 (0 ~ SSD1306_WIDTH-1)
 * @param y Y坐标 (0 ~ SSD1306_HEIGHT-1)
 * @param color 像素颜色 (Black=黑色, White=白色)
 * 
 * 使用示例：
 * @code
 * ssd1306_DrawPixel(64, 32, White);  // 在屏幕中央绘制白点
 * ssd1306_UpdateScreen();
 * @endcode
 */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);

/**
 * @brief 绘制直线
 * 
 * 使用Bresenham算法在两点间绘制直线
 * 
 * @param x1 起点X坐标
 * @param y1 起点Y坐标
 * @param x2 终点X坐标
 * @param y2 终点Y坐标
 * @param color 线条颜色 (Black=黑色, White=白色)
 * 
 * 使用示例：
 * @code
 * ssd1306_DrawLine(0, 0, 127, 63, White);    // 绘制对角线
 * ssd1306_DrawLine(0, 32, 127, 32, White);   // 绘制水平线
 * ssd1306_UpdateScreen();
 * @endcode
 */
void ssd1306_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);

/**
 * @brief 绘制多边形
 * 
 * 根据顶点数组绘制连接的线段
 * 
 * @param par_vertex 顶点数组指针
 * @param par_size 顶点数量
 * @param color 线条颜色 (Black=黑色, White=白色)
 * 
 * 使用示例：
 * @code
 * SSD1306_VERTEX triangle[] = {{64, 10}, {40, 50}, {88, 50}};
 * ssd1306_DrawPolyline(triangle, 3, White);  // 绘制三角形
 * ssd1306_UpdateScreen();
 * @endcode
 */
void ssd1306_DrawPolyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color);

/**
 * @brief 绘制矩形
 * 
 * 绘制矩形边框（不填充）
 * 
 * @param x1 左上角X坐标
 * @param y1 左上角Y坐标
 * @param x2 右下角X坐标
 * @param y2 右下角Y坐标
 * @param color 边框颜色 (Black=黑色, White=白色)
 * 
 * 使用示例：
 * @code
 * ssd1306_DrawRectangle(10, 10, 118, 54, White);  // 绘制矩形边框
 * ssd1306_UpdateScreen();
 * @endcode
 */
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);

/**
 * @brief 绘制圆形
 * 
 * 使用Bresenham算法绘制圆形边框（不填充）
 * 
 * @param par_x 圆心X坐标
 * @param par_y 圆心Y坐标
 * @param par_r 半径
 * @param par_color 边框颜色 (Black=黑色, White=白色)
 * 
 * 使用示例：
 * @code
 * ssd1306_DrawCircle(64, 32, 20, White);  // 在屏幕中央绘制圆形
 * ssd1306_UpdateScreen();
 * @endcode
 */
void ssd1306_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color);

/* ========== 位图绘制函数 ========== */

/**
 * @brief 绘制位图
 * 
 * 在屏幕上绘制位图数据
 * 
 * @param bitmap 位图数据指针
 * @param size 位图数据大小（字节）
 * 
 * 使用示例：
 * @code
 * const uint8_t logo[] = {0xFF, 0x81, 0x81, 0xFF};  // 简单的位图数据
 * ssd1306_DrawBitmap(logo, sizeof(logo));
 * ssd1306_UpdateScreen();
 * @endcode
 */
void ssd1306_DrawBitmap(const uint8_t *bitmap, uint32_t size);

/**
 * @brief 绘制区域位图
 * 
 * 在指定位置绘制指定大小的位图区域
 * 
 * @param x 起始X坐标
 * @param y 起始Y坐标
 * @param w 宽度
 * @param data 位图数据指针
 * @param size 数据大小（字节）
 * 
 * 使用示例：
 * @code
 * const uint8_t icon[] = {0xFF, 0x81, 0x81, 0xFF};
 * ssd1306_DrawRegion(10, 10, 16, icon, sizeof(icon));  // 在(10,10)绘制16x16图标
 * ssd1306_UpdateScreen();
 * @endcode
 */
void ssd1306_DrawRegion(uint8_t x, uint8_t y, uint8_t w, const uint8_t *data, uint32_t size);

/* ========== 显示控制函数 ========== */

/**
 * @brief 设置显示对比度
 * 
 * 调整显示屏的亮度对比度
 * 
 * @param value 对比度值 (0x00-0xFF, 0x00=最暗, 0xFF=最亮)
 * 
 * 使用示例：
 * @code
 * ssd1306_SetContrast(0x80);  // 设置中等亮度
 * ssd1306_SetContrast(0xFF);  // 设置最高亮度
 * @endcode
 */
void ssd1306_SetContrast(const uint8_t value);

/**
 * @brief 开启/关闭显示
 * 
 * 控制显示屏的开关状态
 * 
 * @param on 显示状态 (0=关闭显示, 1=开启显示)
 * 
 * 使用示例：
 * @code
 * ssd1306_SetDisplayOn(1);  // 开启显示
 * ssd1306_SetDisplayOn(0);  // 关闭显示（省电）
 * @endcode
 */
void ssd1306_SetDisplayOn(const uint8_t on);

/**
 * @brief 获取显示状态
 * 
 * 查询当前显示是否开启
 * 
 * @return 显示状态 (0=关闭, 1=开启)
 * 
 * 使用示例：
 * @code
 * if (ssd1306_GetDisplayOn()) {
 *     printf("显示已开启\n");
 * }
 * @endcode
 */
uint8_t ssd1306_GetDisplayOn(void);

/* ========== 底层控制函数 ========== */

/**
 * @brief 复位显示屏
 * 
 * 执行硬件复位操作
 * 
 * @note 通常在初始化时自动调用，一般不需要手动调用
 */
void ssd1306_Reset(void);

/**
 * @brief 发送命令
 * 
 * 向显示屏发送控制命令
 * 
 * @param byte 命令字节
 * 
 * @note 这是底层函数，一般不需要直接调用
 */
void ssd1306_WriteCommand(uint8_t byte);

/**
 * @brief 发送数据
 * 
 * 向显示屏发送数据
 * 
 * @param buffer 数据缓冲区
 * @param buff_size 数据大小
 * 
 * @note 这是底层函数，一般不需要直接调用
 */
void ssd1306_WriteData(uint8_t *buffer, size_t buff_size);

/**
 * @brief 填充显示缓冲区
 * 
 * 用外部数据填充显示缓冲区
 * 
 * @param buf 数据源
 * @param len 数据长度
 * @return 操作结果 (SSD1306_OK=成功, SSD1306_ERR=失败)
 */
SSD1306_Error_t ssd1306_FillBuffer(uint8_t *buf, uint32_t len);

/* ========== 便捷函数 ========== */

/**
 * @brief 清空OLED显示
 * 
 * 清空屏幕并重置光标位置到(0,0)
 * 
 * 使用示例：
 * @code
 * ssd1306_ClearOLED();  // 清空屏幕
 * @endcode
 */
void ssd1306_ClearOLED(void);

/**
 * @brief 格式化打印到OLED
 * 
 * 类似printf的功能，将格式化文本显示到OLED屏幕
 * 每次调用会自动换行
 * 
 * @param fmt 格式化字符串
 * @param ... 可变参数
 * 
 * 使用示例：
 * @code
 * ssd1306_ClearOLED();                    // 清空屏幕
 * ssd1306_printf("Hello World!");        // 第1行
 * ssd1306_printf("Count: %d", 123);      // 第2行
 * ssd1306_printf("Temp: %.1f°C", 25.6);  // 第3行
 * @endcode
 */
void ssd1306_printf(char *fmt, ...);

#endif /* SSD1306_H */