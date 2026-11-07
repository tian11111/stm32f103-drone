/* Run from FlightController with a host GCC:
 * gcc -std=c11 -Wall -Wextra -Werror -IBsp/BlueSerialTest -IBsp
 * Bsp/BlueSerialTest/test_blue_serial.c Bsp/BlueSerial.c Bsp/Variable.c
 * -o build/Debug/test_blue_serial.exe
 */
#include "usart.h"
#include "BlueSerial.h"
#include "Variable.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;
static uint8_t *receive_byte;
static unsigned receive_count, abort_count, clear_count;
static char output[512];
static size_t output_length;
static uint8_t *tx_data;
static uint16_t tx_length;
static unsigned tx_count;

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length)
{
    assert(uart == &huart1 && length == 1);
    receive_byte = data;
    receive_count++;
    uart->RxState = HAL_UART_STATE_BUSY_RX;
    uart->ErrorCode = HAL_UART_ERROR_NONE;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length, uint32_t timeout)
{
    assert(uart == &huart1 && timeout >= ((uint32_t)length * 10000U + 9599U) / 9600U);
    assert(output_length + length < sizeof output);
    memcpy(output + output_length, data, length);
    output_length += length;
    output[output_length] = '\0';
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length)
{
    assert(uart == &huart1 && uart->gState == HAL_UART_STATE_READY);
    tx_data = data; tx_length = length; tx_count++;
    uart->gState = HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    abort_count++;
    uart->RxState = HAL_UART_STATE_READY;
    return HAL_OK;
}

void test_uart_clear_error(UART_HandleTypeDef *uart)
{
    assert(uart == &huart1);
    clear_count++;
}

void Error_Handler(void) { assert(!"Unexpected initialization failure"); }

static void feed(const char *text)
{
    while (*text) {
        *receive_byte = (uint8_t)*text++;
        huart1.RxState = HAL_UART_STATE_READY;
        HAL_UART_RxCpltCallback(&huart1);
        assert(huart1.RxState == HAL_UART_STATE_BUSY_RX);
    }
}

static void test_pid_commands(void)
{
    struct mapping { const char *name; float *value; } mappings[] = {
        {"AngleKp", &pitch_angle_balance_Kp}, {"AngleKi", &pitch_angle_balance_Ki},
        {"AngleKd", &pitch_angle_balance_Kd}, {"SpeedKp", &pitch_gyro_balance_Kp},
        {"SpeedKi", &pitch_gyro_balance_Ki}, {"SpeedKd", &pitch_gyro_balance_Kd},
        {"TurnKp", &yaw_angle_balance_Kp}, {"TurnKi", &yaw_angle_balance_Ki},
        {"TurnKd", &yaw_angle_balance_Kd}, {"Offset", &fly_pitch_zero},
        {"RollAKp", &roll_angle_balance_Kp}, {"RollAKi", &roll_angle_balance_Ki},
        {"RollAKd", &roll_angle_balance_Kd}, {"RollGKp", &roll_gyro_balance_Kp},
        {"RollGKi", &roll_gyro_balance_Ki}, {"RollGKd", &roll_gyro_balance_Kd},
        {"YawGKp", &yaw_gyro_balance_Kp}, {"YawGKi", &yaw_gyro_balance_Ki},
        {"YawGKd", &yaw_gyro_balance_Kd}
    };
    const size_t count = sizeof mappings / sizeof mappings[0];
    char command[80];
    for (size_t i = 0; i < count; i++) *mappings[i].value = -100.0f;
    for (size_t i = 0; i < count; i++) {
        snprintf(command, sizeof command, "[slider,%s,%f]", mappings[i].name, (double)(i + 1));
        feed(command); BlueSerial_Proc();
        for (size_t j = 0; j < count; j++)
            assert(*mappings[j].value == (j <= i ? (float)(j + 1) : -100.0f));
    }
    feed("slider,Offset,-1.250000\n"); BlueSerial_Proc();
    assert(fly_pitch_zero == -1.25f);
    feed("[slider,AngleKi,0.000003]"); BlueSerial_Proc();
    assert(pitch_angle_balance_Ki == 0.000003f);
    feed("slider,AngleKi,3e-5\n"); BlueSerial_Proc();
    assert(pitch_angle_balance_Ki == 3e-5f);
    const char *invalid[] = {"nan", "inf", "-Infinity", "1e999", "garbage", "1xyz", "", "\\x31"};
    for (size_t i = 0; i < sizeof invalid / sizeof invalid[0]; i++) {
        snprintf(command, sizeof command, "[slider,AngleKp,%s]", invalid[i]);
        feed(command); BlueSerial_Proc();
        assert(pitch_angle_balance_Kp == 1.0f);
    }
    feed("[SETP:2.85,I:0.003,D:0.048]"); BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 2.85f && pitch_angle_balance_Ki == 0.003f && pitch_angle_balance_Kd == 0.048f);
    feed("SETI:0.125\n"); BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 2.85f && pitch_angle_balance_Ki == 0.125f && pitch_angle_balance_Kd == 0.048f);
    feed("SETP:nan,I:bad,D:0.4oops\n"); BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 2.85f && pitch_angle_balance_Ki == 0.125f && pitch_angle_balance_Kd == 0.048f);
    feed("SETP:bad,I:0.25,D:inf\n"); BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 2.85f && pitch_angle_balance_Ki == 0.25f && pitch_angle_balance_Kd == 0.048f);
}

int main(void)
{
    BlueSerial_Init();
    assert(receive_count == 1);
    test_pid_commands();
    feed("[slider,AngleKp,2.50]");
    assert(BlueSerial_RxFlag && strcmp(BlueSerial_RxPacket, "slider,AngleKp,2.50") == 0);
    BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 2.5f && !BlueSerial_RxFlag);

    feed("SETP:3.5,I:0.25,D:0.5\r\n");
    BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 3.5f);
    assert(pitch_angle_balance_Ki == 0.25f && pitch_angle_balance_Kd == 0.5f);
    feed("\r\n[]");
    assert(!BlueSerial_RxFlag);

    /* 99 bytes fit; 100 bytes must not publish a truncated SET command. */
    char packet[102];
    memset(packet, ' ', 99);
    memcpy(packet, "SETP:8", 6);
    packet[99] = '\n'; packet[100] = '\0';
    feed(packet);
    assert(BlueSerial_RxFlag && strlen(BlueSerial_RxPacket) == 99);
    BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 8.0f);
    packet[99] = ' '; packet[100] = '\n'; packet[101] = '\0';
    feed(packet);
    assert(!BlueSerial_RxFlag);
    feed("[slider,AngleKp,4]"); BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 4.0f);

    /* A pending packet and an interrupted following frame stay separate. */
    feed("[slider,AngleKp,5]");
    feed("slider,AngleKp,");
    BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 5.0f);
    feed("99\n");
    assert(!BlueSerial_RxFlag);
    feed("[slider,AngleKp,6]"); BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 6.0f);

    /* Match the F1 HAL ordering: Rx-complete can precede the error callback. */
    feed("[slider,AngleKp,99");
    unsigned before_error = receive_count;
    *receive_byte = ']';
    huart1.ErrorCode = 1;
    huart1.RxState = HAL_UART_STATE_READY;
    HAL_UART_RxCpltCallback(&huart1);
    assert(!BlueSerial_RxFlag && receive_count == before_error);
    HAL_UART_ErrorCallback(&huart1);
    assert(clear_count == 1 && receive_count == before_error + 1);
    feed("\n[slider,AngleKp,7]"); BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 7.0f);

    /* Non-blocking errors abort an active receive; a queued good packet survives. */
    feed("[slider,AngleKp,9]");
    huart1.ErrorCode = 2;
    HAL_UART_ErrorCallback(&huart1);
    assert(abort_count == 1 && BlueSerial_RxFlag);
    BlueSerial_Proc();
    assert(pitch_angle_balance_Kp == 9.0f);
    feed("\n");

    Lock = 1; Oil_Set = 123;
    feed("[key,unlock,1]"); BlueSerial_Proc();
    feed("[joystick,100,100]"); BlueSerial_Proc();
    assert(Lock == 1 && Oil_Set == 123);

    UART_HandleTypeDef other = {0};
    before_error = receive_count;
    HAL_UART_RxCpltCallback(&other); HAL_UART_ErrorCallback(&other);
    assert(receive_count == before_error);

    char long_text[200];
    memset(long_text, 'x', sizeof long_text - 1); long_text[199] = '\0';
    BlueSerial_Printf("%s", long_text);
    assert(output_length == 99);
    output_length = 0;
    fly_pitch_zero = 1; getpitch_SET = 2; Pitch = 3; Pitch_Angle_out = 4; Battery_average = 4.2f;
    BlueSerial_SendWave();
    assert(tx_count == 1 && tx_length == strlen("pid:50,3.00,3.00,4.00,4.20\r\n"));
    assert(memcmp(tx_data, "pid:50,3.00,3.00,4.00,4.20\r\n", tx_length) == 0);
    Pitch = 9;
    BlueSerial_SendWave();
    assert(tx_count == 1 && memcmp(tx_data, "pid:50,3.00,3.00,4.00,4.20\r\n", tx_length) == 0);
    huart1.gState = HAL_UART_STATE_READY;
    BlueSerial_SendWave();
    assert(tx_count == 2 && tx_length == strlen("pid:150,3.00,9.00,4.00,4.20\r\n"));
    assert(memcmp(tx_data, "pid:150,3.00,9.00,4.00,4.20\r\n", tx_length) == 0);
    huart1.gState = HAL_UART_STATE_READY;
    fly_pitch_zero = Pitch = Pitch_Angle_out = Battery_average = 1e38f;
    BlueSerial_SendWave();
    assert(tx_count == 2); /* Oversized CSV must not be transmitted. */
    puts("PASS: Bluetooth protocol boundaries, callbacks, PID mapping and output");
    return 0;
}
