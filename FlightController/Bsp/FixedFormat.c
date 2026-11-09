#include "FixedFormat.h"

#include <math.h>
#include <string.h>

/*
 * 把浮点值四舍五入到指定小数位。缩放值过大时分离整数和小数，避免 int32_t 溢出。
 * 该函数使用单精度运算；目标 F103 没有 FPU，但仍保持 float 语义。
 */
bool FixedFormat_Float(char *out, size_t out_size, float value, uint8_t decimals)
{
    static const float scale_table[] = {1.0f, 10.0f, 100.0f, 1000.0f};
    char digits[10];
    size_t digit_count = 0;
    size_t needed;
    size_t position = 0;
    uint32_t divisor;
    uint32_t fraction_divisor;
    uint32_t magnitude;
    uint32_t integer_part;
    uint32_t fraction_part;
    int32_t truncated;
    bool round_up;
    bool negative;
    float scale;
    float scaled_value;
    float fraction_toward_zero;
    float absolute_value;

    if (out == NULL || out_size == 0U || decimals > 3U || !isfinite(value)) {
        return false;
    }

    scale = scale_table[decimals];
    divisor = (uint32_t)scale;
    scaled_value = value * scale;
    negative = value < 0.0f;
    /* 缩放值达到 2^24 后，float 已无法逐个表示整数，应先分离小数。 */
    if (scaled_value >= 16777216.0f || scaled_value <= -16777216.0f) {
        /* 整数部分在 [0, 2^32) 时可安全转换；更大的值明确拒绝。 */
        absolute_value = fabsf(value);
        if (absolute_value >= 4294967296.0f) return false;
        integer_part = (uint32_t)absolute_value;
        fraction_toward_zero =
            (absolute_value - (float)integer_part) * scale;
        fraction_part = (uint32_t)fraction_toward_zero;
        if (fraction_toward_zero - (float)fraction_part >= 0.5f) {
            ++fraction_part;
        }
        if (fraction_part >= divisor) {
            if (integer_part == UINT32_MAX) return false;
            ++integer_part;
            fraction_part -= divisor;
        }
    } else {
        truncated = (int32_t)scaled_value;
        /* 按截尾后的小数余量判断半单位进位，避免先加减 0.5 改变单精度边界。 */
        fraction_toward_zero = fabsf(scaled_value - (float)truncated);
        round_up = fraction_toward_zero >= 0.5f;
        magnitude = (uint32_t)(negative ? -truncated : truncated);
        if (round_up) {
            ++magnitude;
        }
        /* 舍入到零后不输出 "-0.xx"。 */
        if (magnitude == 0U) {
            negative = false;
        }
        integer_part = magnitude / divisor;
        fraction_part = magnitude % divisor;
    }

    do {
        digits[digit_count++] = (char)('0' + integer_part % 10U);
        integer_part /= 10U;
    } while (integer_part != 0U && digit_count < sizeof digits);

    needed = (negative ? 1U : 0U) + digit_count;
    if (decimals != 0U) {
        needed += 1U + decimals;
    }
    if (needed + 1U > out_size) {
        return false;
    }

    if (negative) {
        out[position++] = '-';
    }
    while (digit_count != 0U) {
        out[position++] = digits[--digit_count];
    }

    if (decimals != 0U) {
        out[position++] = '.';
        fraction_divisor = divisor / 10U;
        while (fraction_divisor != 0U) {
            out[position++] = (char)('0' + fraction_part / fraction_divisor);
            fraction_part %= fraction_divisor;
            fraction_divisor /= 10U;
        }
    }
    out[position] = '\0';
    return true;
}

bool FixedFormat_Label(char *out, size_t out_size,
                       const char *label, float value, uint8_t decimals)
{
    size_t label_length;

    if (out == NULL || out_size == 0U || label == NULL) {
        return false;
    }

    label_length = strlen(label);
    if (label_length >= out_size) {
        return false;
    }
    memcpy(out, label, label_length);
    return FixedFormat_Float(out + label_length, out_size - label_length,
                             value, decimals);
}
