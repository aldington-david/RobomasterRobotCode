/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       print_task.c/h
  * @brief      usb outputs the error message.usb输出错误信息
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "print_task.h"
#include "global_control_define.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "SEGGER_RTT.h"
#include "referee_task.h"
#include "remote_control.h"

#if PRINTF_MODE == RTT_MODE
#define LOG(format, args...)  SEGGER_RTT_printf(0, "[%s:%d] "format, __FILE__, __LINE__, ##args)
#define printf(format, args...)  SEGGER_RTT_printf(0, format, ##args)
#endif

#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"

static void usb_printf(const char *fmt, ...);

static void Print_RTT_ReadBuffer(void);

static void Use_RTT_SetConfig(void *const variable);

static uint8_t print_buf[256];
static uint8_t read_buf[256];
uint8_t test_var = 0;
uint8_t test_var1 = 0;
float test_var2 = 0.0;
float test_var3 = 0.0;
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_print_local;


void print_task(void const *argument) {
    if (PRINTF_MODE == USB_MODE) {
        MX_USB_DEVICE_Init();
        error_list_print_local = get_error_list_point();


        while (1) {
            osDelay(1000);
            usb_printf(
                    "******************************\r\n\
voltage percentage:%d%% \r\n\
DBUS:%s\r\n\
chassis motor1:%s\r\n\
chassis motor2:%s\r\n\
chassis motor3:%s\r\n\
chassis motor4:%s\r\n\
yaw motor:%s\r\n\
pitch motor:%s\r\n\
trigger motor:%s\r\n\
gyro sensor:%s\r\n\
accel sensor:%s\r\n\
mag sensor:%s\r\n\
referee usart:%s\r\n\
******************************\r\n",
                    get_battery_percentage(),
                    status[error_list_print_local[DBUS_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR1_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR2_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR3_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR4_TOE].error_exist],
                    status[error_list_print_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
                    status[error_list_print_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
                    status[error_list_print_local[TRIGGER_MOTOR_TOE].error_exist],
                    status[error_list_print_local[BOARD_GYRO_TOE].error_exist],
                    status[error_list_print_local[BOARD_ACCEL_TOE].error_exist],
                    status[error_list_print_local[BOARD_MAG_TOE].error_exist],
                    status[error_list_print_local[REFEREE_RX_TOE].error_exist]);

        }
    }
    if (PRINTF_MODE == RTT_MODE) {
        MX_USB_DEVICE_Init();
        error_list_print_local = get_error_list_point();


        while (1) {
//            SEGGER_RTT_printf(0, "test_var=%d \r\n",test_var);
//            SEGGER_RTT_printf(0, "test_var1=%d \r\n",test_var1);
//            SEGGER_RTT_printf(0, "test_var2=%f \r\n",test_var2);
//            SEGGER_RTT_printf(0, "test_var3=%f \r\n",test_var3);
//            SEGGER_RTT_printf(0, "RedText \r\n", RTT_CTRL_TEXT_BRIGHT_RED);
//            SEGGER_RTT_WriteString(0, "中文测试. \r\n ");
//            SEGGER_RTT_TerminalOut(1, "ERROR: Buffer overflow.\r\n");
            osDelay(1000);
            printf(
                    "******************************\r\n\
voltage percentage:%d%% \r\n\
DBUS:%s\r\n\
chassis motor1:%s\r\n\
chassis motor2:%s\r\n\
chassis motor3:%s\r\n\
chassis motor4:%s\r\n\
yaw motor:%s\r\n\
pitch motor:%s\r\n\
trigger motor:%s\r\n\
gyro sensor:%s\r\n\
accel sensor:%s\r\n\
mag sensor:%s\r\n\
referee rx usart:%s\r\n\
******************************\r\n\
*******Variable Start*********\r\n\
$param:Current=%d;\r\n\
$param:Power=%f;\r\n\
*******Variable End***********\r\n",
                    get_battery_percentage(),
                    status[error_list_print_local[DBUS_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR1_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR2_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR3_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR4_TOE].error_exist],
                    status[error_list_print_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
                    status[error_list_print_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
                    status[error_list_print_local[TRIGGER_MOTOR_TOE].error_exist],
                    status[error_list_print_local[BOARD_GYRO_TOE].error_exist],
                    status[error_list_print_local[BOARD_ACCEL_TOE].error_exist],
                    status[error_list_print_local[BOARD_MAG_TOE].error_exist],
                    status[error_list_print_local[REFEREE_RX_TOE].error_exist],
                    global_judge_info.PowerHeatData.chassis_current,
                    global_judge_info.PowerHeatData.chassis_power);
//            printf(
//                    "*******Variable Start*********\r\n\
//$param:Channal1=%d;\r\n\
//$param:Channal2=%d;\r\n\
//$param:Channal3=%d;\r\n\
//$param:Channal4=%d;\r\n\
//$param:SwitchRight=%d;\r\n\
//$param:SwitchLeft=%d;\r\n\
//$param:MouseX=%d;\r\n\
//$param:MouseY=%d;\r\n\
//$param:MouseZ=%d;\r\n\
//$param:MouseLeft=%d;\r\n\
//$param:MouseRight=%d;\r\n\
//$param:Key=%d;\r\n\
//*******Variable End***********\r\n",
//                    rc_ctrl.rc.ch[0],
//                    rc_ctrl.rc.ch[1],
//                    rc_ctrl.rc.ch[2],
//                    rc_ctrl.rc.ch[3],
//                    rc_ctrl.rc.s[0],
//                    rc_ctrl.rc.s[1],
//                    rc_ctrl.mouse.x,
//                    rc_ctrl.mouse.y,
//                    rc_ctrl.mouse.z,
//                    rc_ctrl.mouse.press_l,
//                    rc_ctrl.mouse.press_r,
//                    rc_ctrl.key.v);
        }
    }

}

static void usb_printf(const char *fmt, ...) {
    static va_list ap;
    static uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *) print_buf, fmt, ap);

    va_end(ap);


    CDC_Transmit_FS(print_buf, len);
}

static void Print_RTT_ReadBuffer(void) {
    unsigned NumBytes = sizeof(read_buf);
    NumBytes = SEGGER_RTT_Read(0, &read_buf[0], NumBytes);
    if (NumBytes) {
        int i;
        for (i = 0; i < NumBytes; i++)
            printf("%c", read_buf[i]);
        printf("\n");
    }
}

static void Use_RTT_SetConfig(void *const variable) {
    unsigned NumBytes = sizeof(read_buf);
    NumBytes = SEGGER_RTT_Read(0, &read_buf[0], NumBytes);
    if (NumBytes) {
        char *rest;
        int32_t var = strtol(&read_buf[0], &rest, 10);
        memcpy(variable, &var, sizeof(var));
    }
}