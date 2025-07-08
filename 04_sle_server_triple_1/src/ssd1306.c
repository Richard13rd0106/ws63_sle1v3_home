/**
 * @file ssd1306.c
 * @brief SSD1306 OLED显示屏驱动实现文件
 * 
 * 本文件实现了SSD1306 OLED显示屏的完整驱动功能，包括：
 * - I2C通信接口
 * - 显示屏初始化和控制
 * - 图形绘制算法
 * - 文本显示功能
 * - 屏幕缓冲区管理
 * 
 * 硬件连接：
 * - I2C总线：CONFIG_I2C_MASTER_BUS_ID (默认为1)
 * - 设备地址：0x3C
 * - 显示尺寸：128x64像素
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <securec.h>
#include "i2c.h"
#include "soc_osal.h"
#include "ssd1306.h"

/* ========== 硬件配置常量 ========== */
#define CONFIG_I2C_MASTER_BUS_ID 1      /**< I2C主机总线ID */
#define I2C_SLAVE2_ADDR 0x3C            /**< SSD1306设备I2C地址 */
#define SSD1306_CTRL_CMD 0x00           /**< 命令控制字节 */
#define SSD1306_CTRL_DATA 0x40          /**< 数据控制字节 */
#define SSD1306_MASK_CONT (0x1 << 7)    /**< 连续传输掩码 */
#define DOUBLE 2                        /**< 倍数常量 */

/* ========== 全局变量 ========== */

/**
 * @brief 显示缓冲区
 * 
 * 存储整个屏幕的像素数据，每个字节代表8个垂直像素
 * 缓冲区大小 = 宽度 × 高度 ÷ 8 = 128 × 64 ÷ 8 = 1024字节
 */
static uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE];

/**
 * @brief 显示屏状态对象
 * 
 * 保存当前显示屏的状态信息，包括光标位置、初始化状态等
 */
static SSD1306_t SSD1306;

/* ========== 底层硬件控制函数 ========== */

/**
 * @brief 复位显示屏
 * 
 * 执行硬件复位操作，等待显示屏启动完成
 * 复位后需要等待1ms让显示屏稳定
 */
void ssd1306_Reset(void)
{
    // 等待屏幕启动，1ms延时非常重要
    osal_mdelay(1);
}

/**
 * @brief 发送数据到I2C总线
 * 
 * 通过I2C接口向SSD1306发送数据
 * 
 * @param buffer 数据缓冲区指针
 * @param size 数据大小（字节）
 * @return 0=成功，其他值=失败
 */
static uint32_t ssd1306_SendData(uint8_t *buffer, uint32_t size)
{
    uint16_t dev_addr = I2C_SLAVE2_ADDR;
    i2c_data_t data = {0};
    data.send_buf = buffer;
    data.send_len = size;
    uint32_t retval = uapi_i2c_master_write(CONFIG_I2C_MASTER_BUS_ID, dev_addr, &data);
    if (retval != 0) {
        printf("I2cWrite(%02X) failed, %0X!\n", data.send_buf[1], retval);
        return retval;
    }
    return 0;
}

/**
 * @brief 写入单个字节
 * 
 * 向指定寄存器写入一个字节数据
 * 
 * @param regAddr 寄存器地址（命令/数据控制字节）
 * @param byte 要写入的数据字节
 * @return 0=成功，其他值=失败
 */
static uint32_t ssd1306_WiteByte(uint8_t regAddr, uint8_t byte)
{
    uint8_t buffer[] = {regAddr, byte};
    return ssd1306_SendData(buffer, sizeof(buffer));
}

/**
 * @brief 发送命令到显示屏
 * 
 * 向SSD1306的命令寄存器发送控制命令
 * 
 * @param byte 命令字节
 */
void ssd1306_WriteCommand(uint8_t byte)
{
    ssd1306_WiteByte(SSD1306_CTRL_CMD, byte);
}

/**
 * @brief 发送数据到显示屏
 * 
 * 向SSD1306发送显示数据，支持连续传输模式
 * 
 * @param buffer 数据缓冲区
 * @param buff_size 数据大小
 */
void ssd1306_WriteData(uint8_t *buffer, uint32_t buff_size)
{
    uint8_t data[SSD1306_WIDTH * DOUBLE] = {0};
    // 为每个数据字节添加控制字节
    for (uint32_t i = 0; i < buff_size; i++) {
        data[i * DOUBLE] = SSD1306_CTRL_DATA | SSD1306_MASK_CONT;  // 连续传输标志
        data[i * DOUBLE + 1] = buffer[i];                          // 实际数据
    }
    data[(buff_size - 1) * DOUBLE] = SSD1306_CTRL_DATA;  // 最后一个字节不设置连续标志
    ssd1306_SendData(data, sizeof(data));
}

/* ========== 缓冲区管理函数 ========== */

/**
 * @brief 用外部数据填充显示缓冲区
 * 
 * 将外部提供的数据复制到内部显示缓冲区
 * 
 * @param buf 源数据缓冲区
 * @param len 数据长度
 * @return SSD1306_OK=成功，SSD1306_ERR=失败
 */
SSD1306_Error_t ssd1306_FillBuffer(uint8_t *buf, uint32_t len)
{
    SSD1306_Error_t ret = SSD1306_ERR;
    if (len <= SSD1306_BUFFER_SIZE) {
        memcpy_s(SSD1306_Buffer, len + 1, buf, len);
        ret = SSD1306_OK;
    }
    return ret;
}

/* ========== 显示屏初始化函数 ========== */

/**
 * @brief 发送初始化命令序列
 * 
 * 发送SSD1306的基本配置命令
 */
void ssd1306_Init_CMD(void)
{
    ssd1306_WriteCommand(0xA4); // 0xa4=显示RAM内容；0xa5=忽略RAM内容

    ssd1306_WriteCommand(0xD3); // 设置显示偏移
    ssd1306_WriteCommand(0x00); // 无偏移

    ssd1306_WriteCommand(0xD5); // 设置显示时钟分频比/振荡器频率
    ssd1306_WriteCommand(0xF0); // 设置分频比

    ssd1306_WriteCommand(0xD9); // 设置预充电周期
    ssd1306_WriteCommand(0x11); // 默认为0x22

    ssd1306_WriteCommand(0xDA); // 设置COM引脚硬件配置
#if (SSD1306_HEIGHT == 32)
    ssd1306_WriteCommand(0x02);
#elif (SSD1306_HEIGHT == 64)
    ssd1306_WriteCommand(0x12);
#elif (SSD1306_HEIGHT == 128)
    ssd1306_WriteCommand(0x12);
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

    ssd1306_WriteCommand(0xDB); // 设置VCOMH电压
    ssd1306_WriteCommand(0x30); // 0x20=0.77xVcc, 0x30=0.83xVcc

    ssd1306_WriteCommand(0x8D); // 设置DC-DC使能
    ssd1306_WriteCommand(0x14); // 使能
    ssd1306_SetDisplayOn(1);    // 开启显示
}

/**
 * @brief 初始化SSD1306显示屏
 * 
 * 执行完整的初始化序列：
 * 1. 硬件复位
 * 2. 发送初始化命令
 * 3. 配置显示参数
 * 4. 清空屏幕
 * 5. 设置默认状态
 */
void ssd1306_Init(void)
{
    // 复位OLED
    ssd1306_Reset();
    
    // 初始化OLED
    ssd1306_SetDisplayOn(0); // 关闭显示

    ssd1306_WriteCommand(0x20); // 设置内存寻址模式
    ssd1306_WriteCommand(0x00); // 00b=水平寻址模式；01b=垂直寻址模式；
                                // 10b=页寻址模式（复位默认）；11b=无效

    ssd1306_WriteCommand(0xB0); // 设置页寻址模式的页起始地址，0-7

#ifdef SSD1306_MIRROR_VERT
    ssd1306_WriteCommand(0xC0); // 垂直镜像
#else
    ssd1306_WriteCommand(0xC8); // 设置COM输出扫描方向
#endif

    ssd1306_WriteCommand(0x00); // 设置低列地址
    ssd1306_WriteCommand(0x10); // 设置高列地址

    ssd1306_WriteCommand(0x40); // 设置起始行地址

    ssd1306_SetContrast(0xFF);  // 设置对比度为最大

#ifdef SSD1306_MIRROR_HORIZ
    ssd1306_WriteCommand(0xA0); // 水平镜像
#else
    ssd1306_WriteCommand(0xA1); // 设置段重映射 0到127
#endif

#ifdef SSD1306_INVERSE_COLOR
    ssd1306_WriteCommand(0xA7); // 设置反色显示
#else
    ssd1306_WriteCommand(0xA6); // 设置正常颜色
#endif

// 设置多路复用比
#if (SSD1306_HEIGHT == 128)
    // 在Luma Python库中为SH1106找到
    ssd1306_WriteCommand(0xFF);
#else
    ssd1306_WriteCommand(0xA8); // 设置多路复用比（1到64）
#endif

#if (SSD1306_HEIGHT == 32)
    ssd1306_WriteCommand(0x1F); // 32行
#elif (SSD1306_HEIGHT == 64)
    ssd1306_WriteCommand(0x3F); // 64行
#elif (SSD1306_HEIGHT == 128)
    ssd1306_WriteCommand(0x3F); // 对于128像素高的显示屏也适用
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif
    
    ssd1306_Init_CMD();
    
    // 清空屏幕
    ssd1306_Fill(Black);

    // 将缓冲区刷新到屏幕
    ssd1306_UpdateScreen();

    // 设置屏幕对象的默认值
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;
    SSD1306.Initialized = 1;
}

/* ========== 基本显示控制函数 ========== */

/**
 * @brief 用指定颜色填充整个屏幕
 * 
 * 将显示缓冲区的所有像素设置为指定颜色
 * 
 * @param color 填充颜色（Black=黑色，White=白色）
 */
void ssd1306_Fill(SSD1306_COLOR color)
{
    /* 设置内存 */
    uint32_t i;

    for (i = 0; i < sizeof(SSD1306_Buffer); i++) {
        SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

/**
 * @brief 更新屏幕显示
 * 
 * 将显示缓冲区的内容传输到显示屏硬件
 * 根据屏幕高度写入相应的页数：
 * - 32像素 == 4页
 * - 64像素 == 8页  
 * - 128像素 == 16页
 */
void ssd1306_UpdateScreen(void)
{
    // 设置列和页的地址范围
    uint8_t cmd[] = {
        0X21, // 设置列起始和结束地址
        0X00, // 列起始地址 0
        0X7F, // 列终止地址 127
        0X22, // 设置页起始和结束地址
        0X00, // 页起始地址 0
        0X07, // 页终止地址 7
    };
    uint32_t count = 0;
    uint8_t data[sizeof(cmd) * DOUBLE + SSD1306_BUFFER_SIZE + 1] = {};

    // 复制命令序列
    for (uint32_t i = 0; i < sizeof(cmd) / sizeof(cmd[0]); i++) {
        data[count++] = SSD1306_CTRL_CMD | SSD1306_MASK_CONT;  // 命令+连续标志
        data[count++] = cmd[i];                                // 命令数据
    }

    // 复制帧数据
    data[count++] = SSD1306_CTRL_DATA;  // 数据控制字节
    memcpy_s(&data[count], SSD1306_BUFFER_SIZE + 1, SSD1306_Buffer, SSD1306_BUFFER_SIZE);
    count += sizeof(SSD1306_Buffer);

    // 发送到I2C总线
    uint32_t retval = ssd1306_SendData(data, count);
    if (retval != 0) {
        printf("ssd1306_UpdateScreen send frame data filed: %d!\r\n", retval);
    }
}

/* ========== 像素级绘制函数 ========== */

/**
 * @brief 在屏幕缓冲区中绘制一个像素
 * 
 * 在指定坐标位置设置像素颜色
 * SSD1306的像素组织方式：每个字节代表8个垂直像素
 * 
 * @param x X坐标（0 ~ SSD1306_WIDTH-1）
 * @param y Y坐标（0 ~ SSD1306_HEIGHT-1）
 * @param color 像素颜色（Black=黑色，White=白色）
 */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        // 不要在缓冲区外写入
        return;
    }
    SSD1306_COLOR color1 = color;
    
    // 检查是否需要反色
    if (SSD1306.Inverted) {
        color1 = (SSD1306_COLOR)!color1;
    }

    // 用正确的颜色绘制
    uint32_t c = 8; // 每字节8位
    if (color == White) {
        // 设置对应位为1（白色）
        SSD1306_Buffer[x + (y / c) * SSD1306_WIDTH] |= 1 << (y % c);
    } else {
        // 设置对应位为0（黑色）
        SSD1306_Buffer[x + (y / c) * SSD1306_WIDTH] &= ~(1 << (y % c));
    }
}

/* ========== 文本绘制函数 ========== */

/**
 * @brief 在屏幕缓冲区中绘制一个字符
 * 
 * 在当前光标位置绘制字符，使用指定字体和颜色
 * 绘制完成后光标自动右移到下一个字符位置
 * 
 * @param ch 要绘制的字符（ASCII 32-126）
 * @param Font 字体定义
 * @param color 字符颜色
 * @return 成功返回绘制的字符，失败返回0
 */
char ssd1306_DrawChar(char ch, FontDef Font, SSD1306_COLOR color)
{
    uint32_t i, b, j;

    // 检查字符是否有效（可打印ASCII字符）
    uint32_t ch_min = 32;  // 空格
    uint32_t ch_max = 126; // 波浪号
    if ((uint32_t)ch < ch_min || (uint32_t)ch > ch_max) {
        return 0;
    }

    // 检查当前行是否有足够空间
    if (SSD1306_WIDTH < (SSD1306.CurrentX + Font.FontWidth) || 
        SSD1306_HEIGHT < (SSD1306.CurrentY + Font.FontHeight)) {
        // 当前行空间不足
        return 0;
    }

    // 使用字体数据绘制字符
    for (i = 0; i < Font.FontHeight; i++) {
        // 获取字符的第i行数据
        b = Font.data[(ch - ch_min) * Font.FontHeight + i];
        for (j = 0; j < Font.FontWidth; j++) {
            // 检查当前位是否需要绘制
            if ((b << j) & 0x8000) {
                // 绘制前景色（字符本身）
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)color);
            } else {
                // 绘制背景色
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }

    // 当前位置已被占用，光标右移
    SSD1306.CurrentX += Font.FontWidth;

    // 返回绘制的字符用于验证
    return ch;
}

/**
 * @brief 在屏幕缓冲区中绘制字符串
 * 
 * 在当前光标位置绘制整个字符串
 * 
 * @param str 要绘制的字符串
 * @param Font 字体定义
 * @param color 字符颜色
 * @return 成功返回0，失败返回无法绘制的字符
 */
char ssd1306_DrawString(char *str, FontDef Font, SSD1306_COLOR color)
{
    // 逐个绘制字符直到字符串结束
    char *str1 = str;
    while (*str1) {
        if (ssd1306_DrawChar(*str1, Font, color) != *str1) {
            // 字符无法绘制
            return *str1;
        }
        // 下一个字符
        str1++;
    }

    // 全部绘制成功
    return *str1;
}

/**
 * @brief 设置光标位置
 * 
 * 设置下次绘制文本的起始坐标
 * 
 * @param x X坐标
 * @param y Y坐标
 */
void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}

/* ========== 图形绘制函数 ========== */

/**
 * @brief 使用Bresenham算法绘制直线
 * 
 * 在两个点之间绘制直线，使用经典的Bresenham直线算法
 * 该算法只使用整数运算，效率高且精度好
 * 
 * @param x1 起点X坐标
 * @param y1 起点Y坐标
 * @param x2 终点X坐标
 * @param y2 终点Y坐标
 * @param color 线条颜色
 */
void ssd1306_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    uint8_t x = x1;
    uint8_t y = y1;
    int32_t deltaX = abs(x2 - x1);  // X方向距离
    int32_t deltaY = abs(y2 - y1);  // Y方向距离
    int32_t signX = ((x1 < x2) ? 1 : -1);  // X方向步进
    int32_t signY = ((y1 < y2) ? 1 : -1);  // Y方向步进
    int32_t error = deltaX - deltaY;       // 误差项
    int32_t error2;
    
    ssd1306_DrawPixel(x2, y2, color);  // 绘制终点
    
    while ((x1 != x2) || (y1 != y2)) {
        ssd1306_DrawPixel(x1, y1, color);  // 绘制当前点
        error2 = error * DOUBLE;
        
        if (error2 > -deltaY) {
            error -= deltaY;
            x += signX;
        } else {
            /* 无需操作 */
        }
        
        if (error2 < deltaX) {
            error += deltaX;
            y += signY;
        } else {
            /* 无需操作 */
        }
    }
}

/**
 * @brief 绘制多边形
 * 
 * 根据顶点数组绘制连接的线段形成多边形
 * 
 * @param par_vertex 顶点数组指针
 * @param par_size 顶点数量
 * @param color 线条颜色
 */
void ssd1306_DrawPolyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color)
{
    uint16_t i;
    if (par_vertex != 0) {
        // 连接相邻的顶点
        for (i = 1; i < par_size; i++) {
            ssd1306_DrawLine(par_vertex[i - 1].x, par_vertex[i - 1].y, 
                           par_vertex[i].x, par_vertex[i].y, color);
        }
    } else {
        /* 无需操作 */
    }
    return;
}

/**
 * @brief 使用Bresenham算法绘制圆形
 * 
 * 绘制圆形边框（不填充），使用Bresenham圆形算法
 * 该算法利用圆的8重对称性，只计算1/8圆弧，其余通过对称得到
 * 
 * @param par_x 圆心X坐标
 * @param par_y 圆心Y坐标
 * @param par_r 半径
 * @param par_color 边框颜色
 */
void ssd1306_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color)
{
    int32_t x = -par_r;
    int32_t y = 0;
    int32_t b = 2;
    int32_t err = b - b * par_r;  // 误差项
    int32_t e2;

    if (par_x >= SSD1306_WIDTH || par_y >= SSD1306_HEIGHT) {
        return;
    }

    do {
        // 利用圆的8重对称性绘制8个点
        ssd1306_DrawPixel(par_x - x, par_y + y, par_color);  // 第1象限
        ssd1306_DrawPixel(par_x + x, par_y + y, par_color);  // 第2象限
        ssd1306_DrawPixel(par_x + x, par_y - y, par_color);  // 第3象限
        ssd1306_DrawPixel(par_x - x, par_y - y, par_color);  // 第4象限
        
        e2 = err;
        if (e2 <= y) {
            y++;
            err = err + (y * b + 1);
            if (-x == y && e2 <= x) {
                e2 = 0;
            } else {
                /* 无需操作 */
            }
        } else {
            /* 无需操作 */
        }
        
        if (e2 > x) {
            x++;
            err = err + (x * b + 1);
        } else {
            /* 无需操作 */
        }
    } while (x <= 0);

    return;
}

/**
 * @brief 绘制矩形边框
 * 
 * 绘制矩形的四条边（不填充）
 * 
 * @param x1 左上角X坐标
 * @param y1 左上角Y坐标
 * @param x2 右下角X坐标
 * @param y2 右下角Y坐标
 * @param color 边框颜色
 */
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    ssd1306_DrawLine(x1, y1, x2, y1, color);  // 上边
    ssd1306_DrawLine(x2, y1, x2, y2, color);  // 右边
    ssd1306_DrawLine(x2, y2, x1, y2, color);  // 下边
    ssd1306_DrawLine(x1, y2, x1, y1, color);  // 左边
}

/* ========== 位图绘制函数 ========== */

/**
 * @brief 绘制位图
 * 
 * 在屏幕上绘制位图数据，从左上角开始
 * 位图数据格式：每个字节代表8个水平像素
 * 
 * @param bitmap 位图数据指针
 * @param size 位图数据大小（字节）
 */
void ssd1306_DrawBitmap(const uint8_t *bitmap, uint32_t size)
{
    unsigned int c = 8;  // 每字节8位
    uint8_t rows = size * c / SSD1306_WIDTH;  // 计算行数
    
    if (rows > SSD1306_HEIGHT) {
        rows = SSD1306_HEIGHT;  // 限制在屏幕高度内
    }
    
    for (uint8_t y = 0; y < rows; y++) {
        for (uint8_t x = 0; x < SSD1306_WIDTH; x++) {
            // 计算当前像素在位图数据中的位置
            uint8_t byte = bitmap[(y * SSD1306_WIDTH / c) + (x / c)];
            uint8_t bit = byte & (0x80 >> (x % c));  // 提取对应位
            ssd1306_DrawPixel(x, y, bit ? White : Black);
        }
    }
}

/**
 * @brief 在指定位置绘制区域位图
 * 
 * 在指定坐标位置绘制指定大小的位图区域
 * 
 * @param x 起始X坐标
 * @param y 起始Y坐标
 * @param w 宽度（像素）
 * @param data 位图数据指针
 * @param size 数据大小（字节）
 */
void ssd1306_DrawRegion(uint8_t x, uint8_t y, uint8_t w, const uint8_t *data, uint32_t size)
{
    uint32_t stride = w;
    uint8_t h = w; // 字体宽高一样（假设为正方形）
    uint8_t width = w;
    
    // 边界检查
    if (x + w > SSD1306_WIDTH || y + h > SSD1306_HEIGHT || w * h == 0) {
        printf("%dx%d @ %d,%d out of range or invalid!\r\n", w, h, x, y);
        return;
    }

    // 限制在屏幕范围内
    width = (width <= SSD1306_WIDTH ? width : SSD1306_WIDTH);
    h = (h <= SSD1306_HEIGHT ? h : SSD1306_HEIGHT);
    stride = (stride == 0 ? w : stride);
    unsigned int c = 8;  // 每字节8位

    uint8_t rows = size * c / stride;  // 计算实际行数
    for (uint8_t i = 0; i < rows; i++) {
        uint32_t base = i * stride / c;  // 当前行的字节偏移
        for (uint8_t j = 0; j < width; j++) {
            uint32_t idx = base + (j / c);
            uint8_t byte = idx < size ? data[idx] : 0;  // 防止越界
            uint8_t bit = byte & (0x80 >> (j % c));     // 提取对应位
            ssd1306_DrawPixel(x + j, y + i, bit ? White : Black);
        }
    }
}

/* ========== 显示控制函数 ========== */

/**
 * @brief 设置显示对比度
 * 
 * 调整显示屏的亮度/对比度
 * 
 * @param value 对比度值（0x00-0xFF，0x00=最暗，0xFF=最亮）
 */
void ssd1306_SetContrast(const uint8_t value)
{
    const uint8_t kSetContrastControlRegister = 0x81;
    ssd1306_WriteCommand(kSetContrastControlRegister);  // 对比度控制命令
    ssd1306_WriteCommand(value);                        // 对比度值
}

/**
 * @brief 开启/关闭显示
 * 
 * 控制显示屏的开关状态，关闭显示可以节省电力
 * 
 * @param on 显示状态（0=关闭显示，1=开启显示）
 */
void ssd1306_SetDisplayOn(const uint8_t on)
{
    uint8_t value;
    if (on) {
        value = 0xAF; // 显示开启命令
        SSD1306.DisplayOn = 1;
    } else {
        value = 0xAE; // 显示关闭命令
        SSD1306.DisplayOn = 0;
    }
    ssd1306_WriteCommand(value);
}

/**
 * @brief 获取显示状态
 * 
 * 查询当前显示是否开启
 * 
 * @return 显示状态（0=关闭，1=开启）
 */
uint8_t ssd1306_GetDisplayOn(void)
{
    return SSD1306.DisplayOn;
}

/* ========== 便捷函数 ========== */

/**
 * @brief 当前垂直位置（用于printf函数）
 * 
 * 全局变量，记录ssd1306_printf函数的当前打印位置
 */
int g_ssd1306_current_loc_v = 0;

/**
 * @brief 行间距（用于printf函数）
 */
#define SSD1306_INTERVAL_V (15)

/**
 * @brief 清空OLED显示
 * 
 * 清空屏幕内容并重置打印位置到顶部
 * 这是一个便捷函数，相当于 ssd1306_Fill(Black) + 重置光标
 */
void ssd1306_ClearOLED(void)
{
    ssd1306_Fill(Black);           // 用黑色填充屏幕
    g_ssd1306_current_loc_v = 0;   // 重置垂直打印位置
}

/**
 * @brief 格式化打印到OLED屏幕
 * 
 * 类似printf的功能，将格式化文本显示到OLED屏幕
 * 每次调用会自动换行，使用Font_7x10字体
 * 
 * @param fmt 格式化字符串
 * @param ... 可变参数列表
 * 
 * 使用示例：
 * @code
 * ssd1306_ClearOLED();                    // 清空屏幕
 * ssd1306_printf("Hello World!");        // 第1行
 * ssd1306_printf("Count: %d", 123);      // 第2行
 * ssd1306_printf("Temp: %.1f°C", 25.6);  // 第3行
 * @endcode
 */
void ssd1306_printf(char *fmt, ...)
{
    char buffer[20];  // 格式化缓冲区
    int ret = 0;
    
    if (fmt) {
        va_list argList;
        va_start(argList, fmt);
        // 格式化字符串到缓冲区
        ret = vsnprintf_s(buffer, sizeof(buffer), sizeof(buffer), fmt, argList);
        if (ret < 0) {
            printf("buffer is null\r\n");
        }
        va_end(argList);
        
        // 设置光标到当前行的开始位置
        ssd1306_SetCursor(0, g_ssd1306_current_loc_v);
        // 绘制文本
        ssd1306_DrawString(buffer, Font_7x10, White);
        // 立即更新显示
        ssd1306_UpdateScreen();
    }
    
    // 移动到下一行
    g_ssd1306_current_loc_v += SSD1306_INTERVAL_V;
}
