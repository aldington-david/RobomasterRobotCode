#include <memory.h>
#include "referee_task.h"
#include "CRC8_CRC16.h"
#include "usart.h"
#include "bsp_usart.h"
#include "detect_task.h"

drv_uart_t judge_sensor_driver = {
        .type = DRV_UART6,
        .tx_type = usart_SendData,
};

judge_info_t judge_sensor_info = {
        .offline_max_cnt = 200,
};

judge_sensor_t judge_sensor = {
        .info = &judge_sensor_info,
        .init = judge_init,

};

void judge_init(judge_sensor_t *judge) {
    judge->info->offline_cnt = (int16_t) (judge->info->offline_max_cnt + 1);
    judge->work_state = DEV_OFFLINE;

    if (judge->id == DEV_ID_VISION) {
        judge->errno = NONE_ERR;
    } else {
        judge->errno = DEV_ID_ERR;
    }
}

void judge_update(uint8_t *rxBuf) {
    uint8_t res = false;
    uint16_t frame_length;
    uint16_t cmd_id;
    judge_info_t *judge_info = judge_sensor.info;
    memcpy(&judge_info->FrameHeader, rxBuf, LEN_FRAME_HEAD);

    if (rxBuf[J_SOF] == 0xA5) {
        if (verify_CRC8_check_sum(rxBuf, LEN_FRAME_HEAD) == true) {
            frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + judge_info->FrameHeader.data_length + LEN_FRAME_TAIL;
            judge_info->frame_length = frame_length;

            if (verify_CRC16_check_sum(rxBuf, frame_length) == true) {
                res = true;
                judge_info->offline_cnt = 0;
                cmd_id = (rxBuf[CMD_ID + 1] << 8 | rxBuf[CMD_ID]);
                judge_info->cmd_id = cmd_id;

                switch (cmd_id) {
                    case ID_GAME_STATUS:
                        memcpy(&judge_info->GameStatus, (rxBuf + DATA_SEG), LEN_GAME_STATUS);
                        break;

                    case ID_GAME_RESULT:
                        memcpy(&judge_info->GameResult, (rxBuf + DATA_SEG), LEN_GAME_RESULT);
                        break;

                    case ID_GAME_ROBOT_HP:
                        memcpy(&judge_info->GameRobotHP, (rxBuf + DATA_SEG), LEN_GAME_ROBOT_HP);
                        break;

                    case ID_DART_STATUS:
                        memcpy(&judge_info->DartStatus, (rxBuf + DATA_SEG), LEN_DART_STATUS);
                        judge_info->dart_data_update = true;
                        break;

                    case ID_EVENT_DATA:
                        memcpy(&judge_info->EventData, (rxBuf + DATA_SEG), LEN_EVENT_DATA);
                        break;

                    case ID_SUPPLY_PROJECTILE_ACTION:
                        memcpy(&judge_info->SupplyProjectileAction, (rxBuf + DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
                        judge_info->supply_data_update = true;
                        break;

                    case ID_REFEREE_WARNING:
                        memcpy(&judge_info->RefereeWarning, (rxBuf + DATA_SEG), LEN_REFEREE_WARNING);
                        break;

                    case ID_DART_REMAINING_TIME:
                        memcpy(&judge_info->DartRemainingTime, (rxBuf + DATA_SEG), LEN_DART_REMAINING_TIME);
                        break;

                    case ID_GAME_ROBOT_STATUS:
                        memcpy(&judge_info->GameRobotStatus, (rxBuf + DATA_SEG), LEN_GAME_ROBOT_STATUS);
                        break;

                    case ID_POWER_HEAT_DATA:
                        memcpy(&judge_info->PowerHeatData, (rxBuf + DATA_SEG), LEN_POWER_HEAT_DATA);
                        judge_info->power_heat_update = true;
                        break;

                    case ID_GAME_ROBOT_POS:
                        memcpy(&judge_info->GameRobotPos, (rxBuf + DATA_SEG), LEN_GAME_ROBOT_POS);
                        break;

                    case ID_BUFF:
                        memcpy(&judge_info->Buff, (rxBuf + DATA_SEG), LEN_BUFF);
                        break;

                    case ID_AERIAL_ROBOT_ENERGY:
                        memcpy(&judge_info->AerialRobotEnergy, (rxBuf + DATA_SEG), LEN_AERIAL_ROBOT_ENERGY);
                        break;

                    case ID_ROBOT_HURT:
                        memcpy(&judge_info->RobotHurt, (rxBuf + DATA_SEG), LEN_ROBOT_HURT);
                        judge_info->hurt_data_update = true;
                        break;

                    case ID_SHOOT_DATA:
                        memcpy(&judge_info->ShootData, (rxBuf + DATA_SEG), LEN_SHOOT_DATA);
                        judge_info->shoot_update = true;
                        break;

                    case ID_BULLET_REMAINING:
                        memcpy(&judge_info->BulletRemaining, (rxBuf + DATA_SEG), LEN_BULLET_REMAINING);
                        break;

                    case ID_RFID_STATUS:
                        memcpy(&judge_info->RFIDStatus, (rxBuf + DATA_SEG), LEN_RFID_STATUS);
                        break;

                    case ID_COMMUNICATION:
                        memcpy(&judge_info->AerialData, (rxBuf + DATA_SEG), LEN_AERAL_DATA);
                        judge_info->communication_data_update = true;
                        break;

                    case ID_COMMAND:
                        memcpy(&judge_info->command, (rxBuf + DATA_SEG), 15);
                        judge_info->command_data_update = true;
                        break;
                    default:
                        break;
                }

                if (rxBuf[frame_length] == 0xA5) {
                    judge_update(&rxBuf[frame_length]);
                }
            }
        }
    }
    judge_info->data_valid = res;
    if (judge_info->data_valid != true) {
        judge_info->err_cnt++;
        judge_info->data_valid = false;
    } else {
        judge_info->data_valid = true;
    }
}

static void judge_check(judge_sensor_t *judge_sen) {
    judge_info_t *judge_info = judge_sen->info;
    if (judge_sen->work_state == DEV_OFFLINE) {
        judge_info->power_heat_update = false;
    }
}

static void judge_heart_beat(judge_sensor_t *judge_sen) {
    judge_info_t *judge_info = judge_sen->info;
    judge_info->offline_cnt++;
    if (judge_info->offline_cnt > judge_info->offline_max_cnt) {
        judge_info->offline_cnt = judge_info->offline_max_cnt;
        judge_sen->work_state = DEV_OFFLINE;
    } else {
        if (judge_sen->work_state == DEV_OFFLINE)
            judge_sen->work_state = DEV_ONLINE;
    }
}

extern void usart6_rxDataHandler(uint8_t *rxBuf) {
    judge_sensor.update(rxBuf);
    judge_sensor.check(&judge_sensor);
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer) {
    *power = judge_sensor.info->PowerHeatData.chassis_power;
    *buffer = judge_sensor.info->PowerHeatData.chassis_power_buffer;

}


uint8_t get_robot_id(void) {
    return judge_sensor.info->GameRobotStatus.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0) {
    *heat0_limit = judge_sensor.info->GameRobotStatus.shooter_heat0_cooling_limit;;
    *heat0 = judge_sensor.info->PowerHeatData.shooter_heat0;
}