/**
 * @file ssd1306_fonts.h
 * @brief SSD1306 OLED显示屏字体定义文件
 * 
 * 本文件定义了用于SSD1306 OLED显示屏的各种字体，包括：
 * - 字体数据结构定义
 * - 多种尺寸的字体（6x8, 7x10, 11x18, 16x26）
 * - 字体数据数组
 * 
 * 支持的字符范围：ASCII 32-126（可打印字符）
 * 包括：空格、数字、字母、标点符号等
 */

#ifndef SSD1306_FONTS_H
#define SSD1306_FONTS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup LCD_FONTS FONTS: 支持的LCD字体
 * @{
 */

/* ========== 字体数据数组声明 ========== */

/**
 * @brief 6x8字体数据数组
 * 
 * 每个字符占用6个字节，高度8像素
 */
extern const unsigned char g_f6X8[][6];

/**
 * @brief 8x16字体数据数组
 * 
 * 用于8x16字体的原始数据
 */
extern const unsigned char g_f8X16[];

/* ========== 字体结构体定义 ========== */

/**
 * @brief 字体定义结构体
 * 
 * 定义了字体的基本属性和数据指针
 * 用于描述字体的尺寸和数据存储位置
 */
typedef struct {
    const unsigned char FontWidth;  /**< 字体宽度（像素）- 每个字符的宽度 */
    unsigned char FontHeight;       /**< 字体高度（像素）- 每个字符的高度 */
    const unsigned short *data;     /**< 字体数据指针 - 指向字体位图数据数组 */
} FontDef;

/* ========== 可用字体声明 ========== */

/**
 * @brief 7x10字体
 * 
 * 小号字体，适合显示较多文本内容
 * - 字符尺寸：7像素宽 x 10像素高
 * - 适用场景：状态信息、小号文本显示
 * - 每行可显示约18个字符（128像素宽度）
 * 
 * 使用示例：
 * @code
 * ssd1306_DrawString("Hello", Font_7x10, White);
 * @endcode
 */
extern FontDef Font_7x10;

/**
 * @brief 6x8字体
 * 
 * 最小字体，适合显示大量文本
 * - 字符尺寸：6像素宽 x 8像素高
 * - 适用场景：密集文本显示、调试信息
 * - 每行可显示约21个字符（128像素宽度）
 * 
 * 使用示例：
 * @code
 * ssd1306_DrawString("Status: OK", Font_6x8, White);
 * @endcode
 */
extern FontDef Font_6x8;

/**
 * @brief 11x18字体
 * 
 * 中号字体，平衡可读性和显示容量
 * - 字符尺寸：11像素宽 x 18像素高
 * - 适用场景：标题、重要信息显示
 * - 每行可显示约11个字符（128像素宽度）
 * 
 * 使用示例：
 * @code
 * ssd1306_DrawString("MENU", Font_11x18, White);
 * @endcode
 */
extern FontDef Font_11x18;

/**
 * @brief 16x26字体
 * 
 * 大号字体，最佳可读性
 * - 字符尺寸：16像素宽 x 26像素高
 * - 适用场景：主要显示内容、数值显示
 * - 每行可显示约8个字符（128像素宽度）
 * 
 * 使用示例：
 * @code
 * ssd1306_DrawString("25.6°C", Font_16x26, White);
 * @endcode
 */
extern FontDef Font_16x26;

/**
 * @}
 */

/* ========== 字体选择指南 ========== */

/**
 * @brief 字体选择建议
 * 
 * 根据不同的使用场景选择合适的字体：
 * 
 * 1. Font_6x8 - 最小字体
 *    - 优点：显示内容最多，节省空间
 *    - 缺点：可读性较差
 *    - 适用：调试信息、状态列表、密集文本
 * 
 * 2. Font_7x10 - 小号字体
 *    - 优点：较好的可读性，显示内容较多
 *    - 缺点：在小屏幕上可能略显拥挤
 *    - 适用：一般文本显示、菜单项、参数显示
 * 
 * 3. Font_11x18 - 中号字体
 *    - 优点：良好的可读性，适中的显示容量
 *    - 缺点：显示内容有限
 *    - 适用：标题、重要信息、用户界面
 * 
 * 4. Font_16x26 - 大号字体
 *    - 优点：最佳可读性，醒目显示
 *    - 缺点：显示内容最少
 *    - 适用：主要数值、温度显示、时间显示
 * 
 * 混合使用示例：
 * @code
 * // 标题使用大字体
 * ssd1306_SetCursor(20, 0);
 * ssd1306_DrawString("TEMP", Font_16x26, White);
 * 
 * // 数值使用大字体
 * ssd1306_SetCursor(30, 30);
 * ssd1306_DrawString("25.6", Font_16x26, White);
 * 
 * // 单位使用小字体
 * ssd1306_SetCursor(100, 40);
 * ssd1306_DrawString("C", Font_7x10, White);
 * 
 * ssd1306_UpdateScreen();
 * @endcode
 */

#ifdef __cplusplus
}
#endif

#endif // SSD1306_FONTS_H