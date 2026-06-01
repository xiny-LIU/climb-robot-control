// ps2_filter.c - 优化版本
#include "ps2_filter.h"
#include <string.h>
#include <stdlib.h>

#define ANOMALY_LY_CENTER       90
#define ANOMALY_LY_RANGE        12      // 扩大范围 78-102
#define ANOMALY_RX_MAX          12      // 0-12都认为是异常
#define MAX_JUMP                30      // 降低跳变阈值

typedef struct {
    uint8_t history[4];     // 增加到4个，更好滤波
    uint8_t idx;
    uint8_t last_output;
    uint8_t initialized;
    uint8_t fill_count;
} Filter_t;

static Filter_t f_ly = {0};
static Filter_t f_rx = {0};
static Filter_t f_lx = {0};
static Filter_t f_ry = {0};

// ========== 核心修改：LY异常检测 ==========

static uint8_t is_ly_anomaly(uint8_t val, uint8_t last_out)
{
    // 条件1：值是否在90危险区域（78-102）
    uint8_t in_danger_zone = (val >= (ANOMALY_LY_CENTER - ANOMALY_LY_RANGE) && 
                              val <= (ANOMALY_LY_CENTER + ANOMALY_LY_RANGE));
    
    if (!in_danger_zone) {
        return 0;  // 不在危险区域，肯定正常
    }
    
    // 条件2：突然跳到90附近（与上次输出差异大）
    // 人手滑动是连续的，不会瞬移
    int16_t jump = abs((int16_t)val - (int16_t)last_out);
    if (jump > MAX_JUMP) {
        return 1;  // 跳变太大，是干扰！
    }
    
    // 条件3：从非90区域突然进入90区域
    // 上次不在78-102，这次突然在 → 可疑
    uint8_t last_in_danger = (last_out >= (ANOMALY_LY_CENTER - ANOMALY_LY_RANGE) && 
                              last_out <= (ANOMALY_LY_CENTER + ANOMALY_LY_RANGE));
    if (!last_in_danger && in_danger_zone) {
        // 进一步确认：如果历史值都不在90附近，这次突然在 → 干扰
        return 1;
    }
    
    return 0;  // 可能是正常操作
}

// ========== RX异常检测（同理） ==========

static uint8_t is_rx_anomaly(uint8_t val, uint8_t last_out)
{
    // rx在0-12之间
    if (val > ANOMALY_RX_MAX) {
        return 0;
    }
    
    // 跳变太大（从正常值突然跳到0附近）
    int16_t jump = abs((int16_t)val - (int16_t)last_out);
    if (jump > MAX_JUMP) {
        return 1;
    }
    
    // 上次在右边（值大），这次突然到0
    if (last_out > 20 && val <= ANOMALY_RX_MAX) {
        return 1;
    }
    
    return 0;
}

// ========== 工具函数 ==========

static void swap(uint8_t *a, uint8_t *b)
{
    uint8_t t = *a; *a = *b; *b = t;
}

// 4值取中值（去掉最大最小，中间两个平均）
static uint8_t median4(uint8_t *arr)
{
    uint8_t temp[4] = {arr[0], arr[1], arr[2], arr[3]};
    
    // 冒泡排序
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3-i; j++) {
            if (temp[j] > temp[j+1]) swap(&temp[j], &temp[j+1]);
        }
    }
    
    // 去掉最小和最大，中间两个取平均
    return (temp[1] + temp[2]) / 2;
}

// 历史平均值
static uint8_t history_avg(Filter_t *f)
{
    uint16_t sum = 0;
    for (int i = 0; i < 4; i++) sum += f->history[i];
    return (uint8_t)(sum / 4);
}

// ========== 滤波更新 ==========

static uint8_t filter_update(Filter_t *f, uint8_t raw, 
                              uint8_t (*is_anomaly)(uint8_t, uint8_t))
{
    // 初始化阶段
    if (f->fill_count < 4) {
        f->history[f->fill_count] = raw;
        f->fill_count++;
        f->last_output = raw;
        return raw;
    }
    
    f->initialized = 1;
    
    // 异常检测和处理
    uint8_t use_val = raw;
    if (is_anomaly && is_anomaly(raw, f->last_output)) {
        // 用历史平均值代替（更平滑）
        use_val = history_avg(f);
    }
    
    // 更新环形缓冲区
    f->history[f->idx] = use_val;
    f->idx = (f->idx + 1) % 4;
    
    // 输出中值
    f->last_output = median4(f->history);
    return f->last_output;
}

// ========== 公共接口 ==========

void PS2_Filter_Init(void)
{
    memset(&f_ly, 0, sizeof(Filter_t));
    memset(&f_rx, 0, sizeof(Filter_t));
    memset(&f_lx, 0, sizeof(Filter_t));
    memset(&f_ry, 0, sizeof(Filter_t));
    
    for (int i = 0; i < 4; i++) {
        f_ly.history[i] = 128;
        f_rx.history[i] = 128;
        f_lx.history[i] = 128;
        f_ry.history[i] = 128;
    }
    
    f_ly.last_output = 128;
    f_rx.last_output = 128;
    f_ly.fill_count = 4;  // 预填充，立即生效
    f_rx.fill_count = 4;
    f_lx.fill_count = 4;
    f_ry.fill_count = 4;
    f_ly.initialized = 1;
    f_rx.initialized = 1;
}

uint8_t PS2_Filter_Get_LY(uint8_t raw_value)
{
    return filter_update(&f_ly, raw_value, is_ly_anomaly);
}

uint8_t PS2_Filter_Get_RX(uint8_t raw_value)
{
    return filter_update(&f_rx, raw_value, is_rx_anomaly);
}

uint8_t PS2_Filter_Get_LX(uint8_t raw_value)
{
    return filter_update(&f_lx, raw_value, NULL);
}

uint8_t PS2_Filter_Get_RY(uint8_t raw_value)
{
    return filter_update(&f_ry, raw_value, NULL);
}
