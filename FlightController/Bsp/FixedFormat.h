#ifndef FIXED_FORMAT_H
#define FIXED_FORMAT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * 有界定点浮点格式化接口。
 * 只支持 0~3 位小数，不依赖浮点 printf，不分配内存。
 * 绝对值须小于 2^32；输入非有限值、超过范围或缓冲区不足时返回 false。
 */
bool FixedFormat_Float(char *out, size_t out_size, float value, uint8_t decimals);

/* 将标签和格式化后的定点小数写入同一缓冲区。 */
bool FixedFormat_Label(char *out, size_t out_size,
                       const char *label, float value, uint8_t decimals);

#endif
