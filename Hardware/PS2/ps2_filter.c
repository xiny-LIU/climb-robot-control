#include "ps2_filter.h"

#include <stdlib.h>
#include <string.h>

/*
 * All four axes use the same policy. Values such as 0, 90 and 255 are valid
 * stick positions and must not be blacklisted. A large jump is accepted only
 * after three consecutive samples with the same trend.
 */
#define PS2_FILTER_HISTORY_SIZE        4U
#define PS2_FILTER_MAX_JUMP           30U
#define PS2_FILTER_CONFIRM_SAMPLES     3U
#define PS2_FILTER_CONFIRM_TOLERANCE  12U

typedef struct
{
    uint8_t history[PS2_FILTER_HISTORY_SIZE];
    uint8_t index;
    uint8_t last_output;
    uint8_t fill_count;
    uint8_t jump_candidate;
    uint8_t jump_confirm_count;
} PS2_FilterState_t;

static PS2_FilterState_t filter_ly;
static PS2_FilterState_t filter_rx;
static PS2_FilterState_t filter_lx;
static PS2_FilterState_t filter_ry;

static void swap_u8(uint8_t *a, uint8_t *b)
{
    uint8_t temp = *a;
    *a = *b;
    *b = temp;
}

static uint8_t median4(const uint8_t values[PS2_FILTER_HISTORY_SIZE])
{
    uint8_t sorted[PS2_FILTER_HISTORY_SIZE] = {
        values[0], values[1], values[2], values[3]
    };

    for (uint8_t i = 0U; i < (PS2_FILTER_HISTORY_SIZE - 1U); i++)
    {
        for (uint8_t j = 0U; j < (PS2_FILTER_HISTORY_SIZE - 1U - i); j++)
        {
            if (sorted[j] > sorted[j + 1U])
            {
                swap_u8(&sorted[j], &sorted[j + 1U]);
            }
        }
    }

    return (uint8_t)(((uint16_t)sorted[1] + (uint16_t)sorted[2]) / 2U);
}

static void fill_history(PS2_FilterState_t *filter, uint8_t value)
{
    for (uint8_t i = 0U; i < PS2_FILTER_HISTORY_SIZE; i++)
    {
        filter->history[i] = value;
    }
    filter->index = 0U;
}

static uint8_t filter_update(PS2_FilterState_t *filter, uint8_t raw)
{
    if (filter->fill_count < PS2_FILTER_HISTORY_SIZE)
    {
        filter->history[filter->fill_count] = raw;
        filter->fill_count++;
        filter->last_output = raw;
        return raw;
    }

    if (abs((int16_t)raw - (int16_t)filter->last_output) >
        (int16_t)PS2_FILTER_MAX_JUMP)
    {
        if ((filter->jump_confirm_count > 0U) &&
            (abs((int16_t)raw - (int16_t)filter->jump_candidate) <=
             (int16_t)PS2_FILTER_CONFIRM_TOLERANCE))
        {
            filter->jump_candidate = raw;
            filter->jump_confirm_count++;
        }
        else
        {
            filter->jump_candidate = raw;
            filter->jump_confirm_count = 1U;
        }

        if (filter->jump_confirm_count < PS2_FILTER_CONFIRM_SAMPLES)
        {
            return filter->last_output;
        }

        fill_history(filter, raw);
        filter->last_output = raw;
        filter->jump_confirm_count = 0U;
        return raw;
    }

    filter->jump_confirm_count = 0U;
    filter->history[filter->index] = raw;
    filter->index = (uint8_t)((filter->index + 1U) % PS2_FILTER_HISTORY_SIZE);
    filter->last_output = median4(filter->history);
    return filter->last_output;
}

void PS2_Filter_Init(void)
{
    memset(&filter_ly, 0, sizeof(filter_ly));
    memset(&filter_rx, 0, sizeof(filter_rx));
    memset(&filter_lx, 0, sizeof(filter_lx));
    memset(&filter_ry, 0, sizeof(filter_ry));

    fill_history(&filter_ly, 128U);
    fill_history(&filter_rx, 128U);
    fill_history(&filter_lx, 128U);
    fill_history(&filter_ry, 128U);

    filter_ly.last_output = 128U;
    filter_rx.last_output = 128U;
    filter_lx.last_output = 128U;
    filter_ry.last_output = 128U;

    filter_ly.fill_count = PS2_FILTER_HISTORY_SIZE;
    filter_rx.fill_count = PS2_FILTER_HISTORY_SIZE;
    filter_lx.fill_count = PS2_FILTER_HISTORY_SIZE;
    filter_ry.fill_count = PS2_FILTER_HISTORY_SIZE;
}

uint8_t PS2_Filter_Get_LY(uint8_t raw_value)
{
    return filter_update(&filter_ly, raw_value);
}

uint8_t PS2_Filter_Get_RX(uint8_t raw_value)
{
    return filter_update(&filter_rx, raw_value);
}

uint8_t PS2_Filter_Get_LX(uint8_t raw_value)
{
    return filter_update(&filter_lx, raw_value);
}

uint8_t PS2_Filter_Get_RY(uint8_t raw_value)
{
    return filter_update(&filter_ry, raw_value);
}
