#include <memory.h>
#include "referee_task.h"
#include "CRC8_CRC16.h"
#include "usart.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "fifo.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"
#include "global_control_define.h"
#include "math.h"
#include "task.h"
//#include "tim.h"//abundant


#define printf(format, args...)  SEGGER_RTT_printf(0, format, ##args)
judge_info_t global_judge_info;
uint8_t referee_fifo_rx_buf[REFEREE_FIFO_BUF_LENGTH];
uint8_t referee_fifo_tx_len_buf[REFEREE_FIFO_BUF_LENGTH];
uint8_t referee_fifo_tx_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
fifo_s_t referee_rx_fifo;
fifo_s_t referee_tx_len_fifo;
fifo_s_t referee_tx_fifo;
uint8_t usart6_rx_buf[2][USART6_RX_BUF_LENGHT];
uint8_t usart6_tx_buf[2][USART6_TX_BUF_LENGHT];
TaskHandle_t USART6TX_active_task_local_handler;
ext_client_custom_graphic_delete_t cleaning;
uint8_t data_pack[DRAWING_PACK * 7] = {0};
volatile uint8_t Referee_No_DMA_IRQHandler = 1;
volatile uint8_t referee_dma_send_data_len = 0;
volatile uint8_t Referee_IRQ_Return_Before = 0;
/* 发送数据包缓存区，最大128字节 */
uint8_t referee_transmit_pack[128] = {0};
/*****************裁判系统接收功能 Start**********************/
/**
  * @brief  裁判系统数据内存空间初始化
  * @param  None
  * @retval None
  */
void init_referee_struct_data(void) {
    memset(&global_judge_info, 0, sizeof(judge_info_t));
}
/**
  * @brief          referee rx task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统接收任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void referee_rx_task(void const *argument) {
    init_referee_struct_data();
    fifo_s_init(&referee_rx_fifo, referee_fifo_rx_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_rx_buf[0], usart6_rx_buf[1], USART6_RX_BUF_LENGHT, usart6_tx_buf[0], usart6_tx_buf[1],
                USART6_TX_BUF_LENGHT);
    TickType_t LoopStartTime;
    while (1) {
        LoopStartTime = xTaskGetTickCount();
        referee_unpack_fifo_data();
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(10));
    }
}

void judge_update(uint8_t *rxBuf) {
    uint8_t res = false;
    uint16_t frame_length = 0;
    uint16_t cmd_id = 0;
    judge_info_t *judge_info = &global_judge_info;
    memcpy(&judge_info->FrameHeader, rxBuf, LEN_FRAME_HEAD);
//    printf("%d",judge_info->FrameHeader.SEQ);

    if (rxBuf[J_SOF] == HEADER_SOF) {
        if (verify_CRC8_check_sum(rxBuf, LEN_FRAME_HEAD) == true) {
            frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + judge_info->FrameHeader.data_length + LEN_FRAME_TAIL;
            judge_info->frame_length = frame_length;

            if (verify_CRC16_check_sum(rxBuf, frame_length) == true) {
                res = true;
                cmd_id = (rxBuf[CMD_ID + 1] << 8 | rxBuf[CMD_ID]);
                judge_info->cmd_id = cmd_id;
//                printf("%d",cmd_id);
                switch (cmd_id) {
                    // 0x000x
                    case ID_GAME_STATUS://!< 0x0001 比赛状态数据
                        memcpy(&judge_info->GameStatus, (rxBuf + DATA_SEG), LEN_GAME_STATUS);
                        break;

                    case ID_GAME_RESULT://!< 0x0002 比赛结果数据
                        memcpy(&judge_info->GameResult, (rxBuf + DATA_SEG), LEN_GAME_RESULT);
                        break;

                    case ID_GAME_ROBOT_HP://!< 0x0003 比赛机器人血量数据
                        memcpy(&judge_info->GameRobotHP, (rxBuf + DATA_SEG), LEN_GAME_ROBOT_HP);
                        break;

                    case ID_ICRA_BUFF_DEBUFF_ZONE_STATUS://!< 0x0005 人工智能挑战赛(ICRA)加成与惩罚区状态
                        memcpy(&judge_info->ICRA_Buff_Debuff_zone_status, (rxBuf + DATA_SEG),
                               LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS);
                        judge_info->ICRA_buff_debuff_zone_status_update = true;
                        break;
                        // 0x010x
                    case ID_EVENT_DATA://!< 0x0101 场地事件数据
                        memcpy(&judge_info->EventData, (rxBuf + DATA_SEG), LEN_EVENT_DATA);
                        break;

                    case ID_SUPPLY_PROJECTILE_ACTION://!< 0x0102, 场地补给站动作标识数据
                        memcpy(&judge_info->SupplyProjectileAction, (rxBuf + DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
                        judge_info->supply_data_update = true;
                        break;

                    case ID_REFEREE_WARNING://!< 0x0104 裁判警告数据
                        memcpy(&judge_info->RefereeWarning, (rxBuf + DATA_SEG), LEN_REFEREE_WARNING);
                        break;

                    case ID_DART_REMAINING_TIME://!< 0x0105 比赛机器人血量数据
                        memcpy(&judge_info->DartRemainingTime, (rxBuf + DATA_SEG), LEN_DART_REMAINING_TIME);
                        break;
                        // 0x020x
                    case ID_GAME_ROBOT_STATUS://!< 0x0201 机器人状态数据
                        memcpy(&judge_info->GameRobotStatus, (rxBuf + DATA_SEG), LEN_GAME_ROBOT_STATUS);
                        judge_info->self_client_id = judge_info->GameRobotStatus.robot_id + 0x0100;
//                        printf("/r/n judge_client_id=%d", judge_info->self_client_id);
                        break;

                    case ID_POWER_HEAT_DATA://!< 0x0202 实时功率热量数据
                        memcpy(&judge_info->PowerHeatData, (rxBuf + DATA_SEG), LEN_POWER_HEAT_DATA);
                        judge_info->power_heat_update = true;
                        break;

                    case ID_GAME_ROBOT_POS://!< 0x0203 机器人位置数据
                        memcpy(&judge_info->GameRobotPos, (rxBuf + DATA_SEG), LEN_GAME_ROBOT_POS);
                        break;

                    case ID_BUFF://!< 0x0204 机器人增益数据
                        memcpy(&judge_info->Buff, (rxBuf + DATA_SEG), LEN_BUFF);
                        break;

                    case ID_AERIAL_ROBOT_ENERGY://!< 0x0205 空中机器人能量状态数据
                        memcpy(&judge_info->AerialRobotEnergy, (rxBuf + DATA_SEG), LEN_AERIAL_ROBOT_ENERGY);
                        break;

                    case ID_ROBOT_HURT://!< 0x0206 伤害状态数据
                        memcpy(&judge_info->RobotHurt, (rxBuf + DATA_SEG), LEN_ROBOT_HURT);
                        judge_info->hurt_data_update = true;
                        break;

                    case ID_SHOOT_DATA://!< 0x0207 实时射击数据
                        memcpy(&judge_info->ShootData, (rxBuf + DATA_SEG), LEN_SHOOT_DATA);
                        judge_info->shoot_update = true;
                        break;

                    case ID_BULLET_REMAINING://!< 0x0208 弹丸剩余发射数
                        memcpy(&judge_info->BulletRemaining, (rxBuf + DATA_SEG), LEN_BULLET_REMAINING);
                        break;

                    case ID_RFID_STATUS://!< 0x0209 机器人RFID状态
                        memcpy(&judge_info->RFIDStatus, (rxBuf + DATA_SEG), LEN_RFID_STATUS);
                        break;

                    case ID_DART_CLIENT_COMMAND://!< 0x020A 飞镖机器人客户端指令数据
                        memcpy(&judge_info->DartClientCMD, (rxBuf + DATA_SEG), LEN_DART_CLIENT_COMMAND);
                        judge_info->dart_data_update = true;
                        break;

                    case ID_COMMUNICATION://!< 0x0301 机器人间交互数据（裁判系统），10HZ，最大128字节，数据段113字节
                        memcpy(&judge_info->AerialData, (rxBuf + DATA_SEG), LEN_COMMUNICATION);
                        judge_info->communication_data_update = true;
                        break;

                    case ID_CUSTOMIZE_CONTROL_DATA://!< 0x0302 自定义控制器交互数据接口，30HZ
                        memcpy(&judge_info->RobotInteractiveData, (rxBuf + DATA_SEG), LEN_CUSTOMIZE_CONTROL_DATA);
                        judge_info->customize_control_data_update = true;
                        break;

                    case ID_MAP_INTERACTION_DATA://!< 0x0303 客户端小地图交互数据
                        memcpy(&judge_info->command, (rxBuf + DATA_SEG), LEN_MAP_INTERACTION_DATA);
                        judge_info->map_data_update = true;
                        break;
                    case ID_CONTROL_UART_DATA://!< 0x0304 图传串口发送键盘、鼠标信息
                        memcpy(&judge_info->ControlUart, (rxBuf + DATA_SEG), LEN_CONTROL_UART_DATA);
                        judge_info->map_data_update = true;
                        break;
                    case ID_MAP_DATA://!< 0x0305 客户端小地图接收信息
                        memcpy(&judge_info->ClientMapCommand, (rxBuf + DATA_SEG), LEN_MAP_DATA);
                        judge_info->client_map_data_update = true;
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

/**
  * @brief          single byte upacked
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void) {
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    unpack_data_t *p_obj = &referee_unpack_obj;

    while (fifo_s_used(&referee_rx_fifo)) {
        byte = fifo_s_get(&referee_rx_fifo);
        switch (p_obj->unpack_step) {
            case STEP_HEADER_SOF: {
                if (byte == sof) {
                    p_obj->unpack_step = STEP_LENGTH_LOW;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                } else {
                    p_obj->index = 0;
                }
            }
                break;

            case STEP_LENGTH_LOW: {
                p_obj->data_len = byte;
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_LENGTH_HIGH;
            }
                break;

            case STEP_LENGTH_HIGH: {
                p_obj->data_len |= (byte << 8);
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
                    p_obj->unpack_step = STEP_FRAME_SEQ;
                } else {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
                break;
            case STEP_FRAME_SEQ: {
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_HEADER_CRC8;
            }
                break;

            case STEP_HEADER_CRC8: {
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->index == REF_PROTOCOL_HEADER_SIZE) {
                    if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
                        p_obj->unpack_step = STEP_DATA_CRC16;
                    } else {
                        p_obj->unpack_step = STEP_HEADER_SOF;
                        p_obj->index = 0;
                    }
                }
            }
                break;

            case STEP_DATA_CRC16: {
                if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;

                    if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                        judge_update(p_obj->protocol_packet);
                    }
                }
            }
                break;

            default: {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
                break;
        }
    }
}

/**
  * @brief          裁判系统串口（USART6）回调函数
  * @param[in]      void
  * @retval         none
  */
void USART6_IRQHandler(void) {
//    SEGGER_RTT_WriteString(0,"TEST");
    static volatile uint8_t res;
    if (USART6->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
        static uint16_t this_time_rx_len = 0;
        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART6_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART6_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_rx_fifo, (char *) usart6_rx_buf[0], this_time_rx_len);
            detect_hook(REFEREE_RX_TOE);
        } else {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART6_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART6_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_rx_fifo, (char *) usart6_rx_buf[1], this_time_rx_len);
            detect_hook(REFEREE_RX_TOE);
        }
    }
}

/**
  * @brief          发送串口（USART6）DMA回调函数
  * @param[in]      void
  * @retval         none
  */
void DMA2_Stream6_IRQHandler(void) {
    if (__HAL_DMA_GET_FLAG(huart6.hdmatx, DMA_HISR_TCIF6) != RESET) {
        MY_USART_DMA_Stream6_TX_IRQHandler();
    }
}
/*****************裁判系统接收功能 END**********************/
/*****************裁判系统接收数据分析函数 START**********************/

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer) {
    *power = global_judge_info.PowerHeatData.chassis_power;
    *buffer = global_judge_info.PowerHeatData.chassis_power_buffer;

}

uint8_t get_robot_id(void) {
    return global_judge_info.GameRobotStatus.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0) {
    *heat0_limit = global_judge_info.GameRobotStatus.shooter_id1_17mm_cooling_limit;;
    *heat0 = global_judge_info.PowerHeatData.shooter_id1_17mm_cooling_heat;
}
/*****************裁判系统接收数据功能函数 END**********************/


/**
  * @brief          referee tx task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统发送任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void referee_tx_task(void const *argument) {
    fifo_s_init(&referee_tx_len_fifo, referee_fifo_tx_len_buf, REFEREE_FIFO_BUF_LENGTH);
    fifo_s_init(&referee_tx_fifo, referee_fifo_tx_buf, REFEREE_FIFO_BUF_LENGTH);
    static int draw_cnt = 0;//辅助命名图形
    static uint8_t name[3] = "13";
    //下坠UI标尺的水平刻度线长度、距离、颜色；垂直线总长度由为各水平刻度线距离之和
    uint16_t line_distance[6] = {10, 30, 30, 35/*哨兵*/, 30, 50};
    uint16_t line_length[6] = {120, 80, 70, 60, 20, 20};
    graphic_color_enum_t ruler_color[7] = {graphic_color_white, graphic_color_white, graphic_color_white,
                                           graphic_color_white, graphic_color_white, graphic_color_white,
                                           graphic_color_white};
    vTaskDelay(pdMS_TO_TICKS(5000));
    UI_clean_all();
    TickType_t LoopStartTime;
    while (1) {
        LoopStartTime = xTaskGetTickCount();
//        SEGGER_RTT_WriteString(0, "referee_task_loop_on");
//        draw_cnt++;
//        name[0] = draw_cnt / 255;
//        name[1] = draw_cnt % 255;
//        line_drawing(name,ADD_PICTURE,0,graphic_color_yellow, 5,200, 200, 400, 400  );
//        printf("%d", fifo_s_used(&referee_tx_fifo));
        Hero_UI_ruler(5, 961, 538, line_distance, line_length, ruler_color, ADD_PICTURE);
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief 打包机器人之间的交互数据，以及UI数据下发到底层发送
 * @param _data_cmd_id: 数据段ID
 * 		  _receiver_ID: 接收方ID，可以是机器人对应客户端、或者己方其他机器人
 * 		  _data: 数据段段首指针
 * 		  _data_len: 数据段长度
 * @retval None
 */
void pack_send_robotData(uint16_t _data_cmd_id, uint8_t *_data, uint16_t _data_len) {
    ext_student_interactive_header_data_t data_header;                                                                //定义数据段段首并设置
    memset(&data_header, 0, sizeof(data_header));
    data_header.data_CMD_ID = _data_cmd_id;
    data_header.sender_ID = 4;                                        //设置发送者ID
    data_header.receiver_ID = 0x0104;
//    printf("/r/nclient_id=%d", global_judge_info.self_client_id);
    uint8_t header_len = 6;
    memcpy((void *) (referee_transmit_pack + 7), &data_header, header_len);                        //将数据帧的数据段进行封装（封装段首）
    memcpy((void *) (referee_transmit_pack + 7 + header_len), _data, _data_len);                    //将数据帧的数据段进行封装（封装数据）
    send_toReferee(ID_COMMUNICATION, header_len + _data_len);
}

/**
 * @brief 底层发送函数。负责发送数据包，以及对发送速率做控制
 * @param _cmd_id，
 * @param _data，
 * @param data_len，
 * @param _receive_type，判断车间通信 or 雷达站 or UI，决定数据需不需要发多次（客户端数据经常丢包，所以需要发多次），以及发送频率
 */
void send_toReferee(uint16_t _cmd_id, uint16_t _data_len) {
    static uint8_t seq = 0;
    std_frame_header_t send_frame_header;                                                                                            //交互数据帧帧头设置
    memset(&send_frame_header, 0, sizeof(send_frame_header));
    send_frame_header.SOF = HEADER_SOF;
    send_frame_header.data_length = _data_len;
    send_frame_header.SEQ = seq++;
    if (seq == 255) {
        seq = 0;
    }
//    printf("/r/n seq=%d", send_frame_header.SEQ);
    append_CRC8_check_sum((uint8_t *) &send_frame_header, sizeof(send_frame_header));
//    send_frame_header.CRC8 = get_CRC8_check_sum((uint8_t *) &send_frame_header, 4, 0xff);
    uint16_t CmdID = _cmd_id;

    uint8_t header_len = 5;

    memcpy((void *) referee_transmit_pack, &send_frame_header, header_len);//将帧头装入缓存区
    memcpy((void *) (referee_transmit_pack + 5), &CmdID,
           2);                                                                    //将数据段转入缓存区
    append_CRC16_check_sum((uint8_t *) &referee_transmit_pack, header_len + _data_len + 4);
//    for (int i = 0; i <= header_len + _data_len + 4;i++) {
//        printf(" %02x",referee_transmit_pack[i]);
//    }
    fifo_s_put(&referee_tx_len_fifo, header_len + _data_len + 4);
    fifo_s_puts(&referee_tx_fifo, (char *) &referee_transmit_pack, header_len + _data_len + 4);
    memset(referee_transmit_pack, 0, sizeof(referee_transmit_pack));
    if (Referee_No_DMA_IRQHandler) {
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            xTaskNotifyGive(USART6TX_active_task_local_handler);
//            portYIELD();
        }
    }
}

////will_abandon
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Instance == htim2.Instance) {
////    printf("%d", fifo_s_used(&referee_tx_fifo));
//        if (Referee_No_DMA_IRQHandler) {
//            if (fifo_s_used(&referee_tx_fifo)) {
//                if (fifo_s_used(&referee_tx_len_fifo)) {
//                    if ((huart6.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
////                    SEGGER_RTT_WriteString(0, "ST0DMA_1");
//                        __HAL_DMA_DISABLE(huart6.hdmatx);
//                        referee_dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
//                        memset(&usart6_tx_buf[1], 0, USART6_TX_BUF_LENGHT);
//                        fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[1], referee_dma_send_data_len);
//                        huart6.hdmatx->Instance->CR |= DMA_SxCR_CT;
//                        __HAL_DMA_SET_COUNTER(huart6.hdmatx, referee_dma_send_data_len);
//                        __HAL_DMA_ENABLE(huart6.hdmatx);
//                        Referee_No_DMA_IRQHandler = 0;
//                        detect_hook(REFEREE_TX_TOE);
//                        if (fifo_s_used(&referee_tx_fifo)) {
//                            if (fifo_s_used(&referee_tx_len_fifo)) {
//                                referee_dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
//                                memset(&usart6_tx_buf[0], 0, USART6_TX_BUF_LENGHT);
//                                fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[0], referee_dma_send_data_len);
//                            }
//                        } else {
//                            referee_dma_send_data_len = 0;
//                            Referee_No_DMA_IRQHandler = 1;
//                            memset(&usart6_tx_buf[0], 0, USART6_TX_BUF_LENGHT);
//                        }
//                    } else {
////                    SEGGER_RTT_WriteString(0, "ST0DMA_0");
//                        __HAL_DMA_DISABLE(huart6.hdmatx);
//                        referee_dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
//                        memset(&usart6_tx_buf[0], 0, USART6_TX_BUF_LENGHT);
//                        fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[0], referee_dma_send_data_len);
//                        huart6.hdmatx->Instance->CR &= ~(DMA_SxCR_CT);
//                        __HAL_DMA_SET_COUNTER(huart6.hdmatx, referee_dma_send_data_len);
//                        __HAL_DMA_ENABLE(huart6.hdmatx);
//                        Referee_No_DMA_IRQHandler = 0;
//                        detect_hook(REFEREE_TX_TOE);
//                        if (fifo_s_used(&referee_tx_fifo)) {
//                            if (fifo_s_used(&referee_tx_len_fifo)) {
//                                referee_dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
//                                memset(&usart6_tx_buf[1], 0, USART6_TX_BUF_LENGHT);
//                                fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[1], referee_dma_send_data_len);
//
//                            }
//                        } else {
//                            referee_dma_send_data_len = 0;
//                            Referee_No_DMA_IRQHandler = 1;
//                            memset(&usart6_tx_buf[1], 0, USART6_TX_BUF_LENGHT);
//                        }
//                    }
//                }
//            }
//        }
//    }
//}

/**
  * @brief          usart6发送启动任务，由发送时任务通知激活
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void USART6TX_active_task(void const *pvParameters) {
    USART6TX_active_task_local_handler = xTaskGetCurrentTaskHandle();
    while (1) {
        while (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != pdPASS) {
        }
        vTaskSuspendAll();
        if (Referee_No_DMA_IRQHandler || Referee_IRQ_Return_Before) {
            if (fifo_s_used(&referee_tx_fifo)) {
                if (fifo_s_used(&referee_tx_len_fifo)) {
                    if ((huart6.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_1");
                        __HAL_DMA_DISABLE(huart6.hdmatx);
                        referee_dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                        if (referee_dma_send_data_len > 1) {
                            Referee_IRQ_Return_Before = 0;
                        }
                        memset(&usart6_tx_buf[1], 0, USART6_TX_BUF_LENGHT);
                        fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[1], referee_dma_send_data_len);
                        huart6.hdmatx->Instance->CR |= DMA_SxCR_CT;
                        __HAL_DMA_SET_COUNTER(huart6.hdmatx, referee_dma_send_data_len);
                        __HAL_DMA_ENABLE(huart6.hdmatx);
                        Referee_No_DMA_IRQHandler = 0;
                        referee_dma_send_data_len = 0;
                        detect_hook(REFEREE_TX_TOE);
                        taskENTER_CRITICAL();
                        if (!Referee_IRQ_Return_Before) {
                            if (fifo_s_used(&referee_tx_fifo)) {
                                if (fifo_s_used(&referee_tx_len_fifo)) {
                                    referee_dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                                    memset(&usart6_tx_buf[0], 0, USART6_TX_BUF_LENGHT);
                                    fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[0], referee_dma_send_data_len);
                                }
                            } else {
                                referee_dma_send_data_len = 0;
                                Referee_No_DMA_IRQHandler = 1;
                                memset(&usart6_tx_buf[0], 0, USART6_TX_BUF_LENGHT);
                            }
                        }
                        taskEXIT_CRITICAL();
                    } else {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_0");
                        __HAL_DMA_DISABLE(huart6.hdmatx);
                        referee_dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                        if (referee_dma_send_data_len > 1) {
                            Referee_IRQ_Return_Before = 0;
                        }
                        memset(&usart6_tx_buf[0], 0, USART6_TX_BUF_LENGHT);
                        fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[0], referee_dma_send_data_len);
                        huart6.hdmatx->Instance->CR &= ~(DMA_SxCR_CT);
                        __HAL_DMA_SET_COUNTER(huart6.hdmatx, referee_dma_send_data_len);
                        __HAL_DMA_ENABLE(huart6.hdmatx);
                        Referee_No_DMA_IRQHandler = 0;
                        referee_dma_send_data_len = 0;
                        detect_hook(REFEREE_TX_TOE);
                        taskENTER_CRITICAL();
                        if (!Referee_IRQ_Return_Before) {
                            if (fifo_s_used(&referee_tx_fifo)) {
                                if (fifo_s_used(&referee_tx_len_fifo)) {
                                    referee_dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                                    memset(&usart6_tx_buf[1], 0, USART6_TX_BUF_LENGHT);
                                    fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[1], referee_dma_send_data_len);
                                }
                            } else {
                                referee_dma_send_data_len = 0;
                                Referee_No_DMA_IRQHandler = 1;
                                memset(&usart6_tx_buf[1], 0, USART6_TX_BUF_LENGHT);
                            }
                        }
                        taskEXIT_CRITICAL();
                    }
                }
            }
        }
        xTaskResumeAll();
    }
}

void MY_USART_DMA_Stream6_TX_IRQHandler(void) {
    __HAL_DMA_DISABLE(huart6.hdmatx);
    while (huart6.hdmatx->Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(huart6.hdmatx);
    }
    __HAL_DMA_CLEAR_FLAG(huart6.hdmatx, DMA_HISR_TCIF6);
    __HAL_DMA_CLEAR_FLAG (huart6.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(huart6.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart6.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(huart6.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart6.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(huart6.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart6.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(huart6.hdmatx));
    if ((!fifo_s_used(&referee_tx_fifo)) || (!fifo_s_used(&referee_tx_len_fifo))) {
        if ((fifo_s_used(&referee_tx_fifo)) || (fifo_s_used(&referee_tx_len_fifo))) {
            //should not come in here
            fifo_s_flush(&referee_tx_fifo);
            fifo_s_flush(&referee_tx_len_fifo);
        }
    }
    if (referee_dma_send_data_len == 1) {
        Referee_IRQ_Return_Before = 1;
    }
    if (referee_dma_send_data_len) {
        if ((huart6.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart6.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart6.hdmatx, DMA_HISR_TCIF6);
            __HAL_DMA_SET_COUNTER(huart6.hdmatx, referee_dma_send_data_len);
//            SEGGER_RTT_WriteString(0, "ST1DMA_0");
            __HAL_DMA_ENABLE(huart6.hdmatx);
            referee_dma_send_data_len = 0;
            detect_hook(REFEREE_TX_TOE);
            if (!Referee_IRQ_Return_Before) {
                if (fifo_s_used(&referee_tx_fifo)) {
                    if (fifo_s_used(&referee_tx_len_fifo)) {
                        referee_dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                        memset(&usart6_tx_buf[1], 0, USART6_TX_BUF_LENGHT);
                        fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[1], referee_dma_send_data_len);
                    }
                } else {
                    referee_dma_send_data_len = 0;
                    Referee_No_DMA_IRQHandler = 1;
                    memset(&usart6_tx_buf[1], 0, USART6_TX_BUF_LENGHT);
                }
            }
        } else {
            __HAL_DMA_DISABLE(huart6.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart6.hdmatx, DMA_HISR_TCIF6);
            __HAL_DMA_SET_COUNTER(huart6.hdmatx, referee_dma_send_data_len);
//            SEGGER_RTT_WriteString(0, "ST1DMA_1");
            __HAL_DMA_ENABLE(huart6.hdmatx);
            referee_dma_send_data_len = 0;
            detect_hook(REFEREE_TX_TOE);
            if (!Referee_IRQ_Return_Before) {
                if (fifo_s_used(&referee_tx_fifo)) {
                    if (fifo_s_used(&referee_tx_len_fifo)) {
                        referee_dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                        memset(&usart6_tx_buf[0], 0, USART6_TX_BUF_LENGHT);
                        fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[0], referee_dma_send_data_len);
                    }
                } else {
                    referee_dma_send_data_len = 0;
                    Referee_No_DMA_IRQHandler = 1;
                    memset(&usart6_tx_buf[0], 0, USART6_TX_BUF_LENGHT);
                }
            }
        }
    } else {
        memset(&usart6_tx_buf[1], 0, USART6_TX_BUF_LENGHT);
        memset(&usart6_tx_buf[0], 0, USART6_TX_BUF_LENGHT);
        if (!Referee_No_DMA_IRQHandler) {
            Referee_IRQ_Return_Before = 1;
            if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
//                SEGGER_RTT_WriteString(0, "yield\r\n");
                static BaseType_t xHigherPriorityTaskWoken;
                vTaskNotifyGiveFromISR(USART6TX_active_task_local_handler, &xHigherPriorityTaskWoken);
            }
        }
    }
}

/************裁判系统UI绘图功能函数******************/
/**
 * @brief 清除所有UI绘制图形
 * @note Referee.clean_all();
 * @param
 * @retval
 */
void UI_clean_all(void)                                                                                    //清除所有自定义图案
{
    cleaning.operate_type = 2;
    pack_send_robotData(0x0100, (uint8_t *) &cleaning, sizeof(cleaning));
}

void Hero_UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t *line_distance, uint16_t *line_length,
                   graphic_color_enum_t *_color, drawOperate_e _operate_type) {
    uint16_t total_distance = 0;
    static uint8_t line_name[] = "he0";

    //绘制初始准星横线
    line_name[2] = '6';
    memcpy(&data_pack[DRAWING_PACK * 6],
           (uint8_t *) line_drawing(_layer, ADD_PICTURE, start_x - line_length[0] / 2, start_y,
                                    start_x + line_length[0] / 2, start_y, 3, _color[0], line_name), DRAWING_PACK);
    total_distance += line_distance[0];

    //封装好榴弹准星小横线，第2到6条
    for (uint8_t i = 1; i < 6; i++) {
        line_name[2] = '0' + i;
        memcpy(&data_pack[DRAWING_PACK * i],
               (uint8_t *) line_drawing(_layer, ADD_PICTURE, start_x - line_length[i] / 2, start_y - total_distance,
                                        start_x + line_length[i] / 2, start_y - total_distance, 3, _color[i],
                                        line_name), DRAWING_PACK);

        total_distance += line_distance[i];                //计算竖直线总长度
    }

    //封装好竖直线
    line_name[2] = '0';
    memcpy(data_pack,
           (uint8_t *) line_drawing(_layer, ADD_PICTURE, start_x, start_y, start_x, start_y - total_distance, 3,
                                    _color[6], line_name), DRAWING_PACK);

    pack_send_robotData(0x0104, (uint8_t *) data_pack, DRAWING_PACK * 7);
    static uint8_t data_pack[DRAWING_PACK * 7] = {0};
}

/**
 * @brief 直线绘制数据包
 * @param line_width 线宽
 * @retval
 */
graphic_data_struct_t *
line_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx, uint16_t starty, uint16_t endx,
             uint16_t endy, uint16_t line_width, graphic_color_enum_t vcolor, uint8_t *name) {
    static graphic_data_struct_t drawing;
//    SEGGER_RTT_WriteString(0,"line_drawing_on");
    memcpy(drawing.graphic_name, name, 3);    //图案名称，3位
    drawing.operate_type = _operate_type;
    drawing.graphic_type = 0U;
    drawing.layer = _layer;
    drawing.color = vcolor;
    drawing.width = line_width;
    drawing.start_x = startx;
    drawing.start_y = starty;
    drawing.end_x = endx;
    drawing.end_y = endy;
    return &drawing;
}

/*************************************************/
/*!
 * @brief 浮点数转字符串
 * @param value 想要打印的数据
 * @param decimal_digit 数字小数部分的位数
 * @param output_length 输出字符串的长度
 * @return
 */
unsigned char *out_float(double value, unsigned char decimal_digit, unsigned char *output_length) {
    unsigned char _output[20];
    unsigned long integer;
    unsigned long decimal;
    unsigned char _output_length = 0;
    unsigned char _length_buff = 0;
    static unsigned char *return_pointer;
    unsigned char signal_flag;
    if (value < 0)
        signal_flag = 1;
    else
        signal_flag = 0;
    value = fabs(value);
    integer = (unsigned long) value;
    decimal = (unsigned long) ((value - integer) * pow(10, decimal_digit));

    unsigned long integer_buff = integer;
    unsigned long decimal_buff = decimal;

    while (1) {
        if (integer / 10 != 0)
            _length_buff++;
        else {
            _length_buff++;
            break;
        }
        integer = integer / 10;
    }
    for (int i = 0; i < _length_buff; i++) {
        if (i == _length_buff - 1)
            _output[_output_length] = integer_buff % 10 + 0x30;
        else {
            //_output[_output_length] = integer_buff / 10 % 10 + 0x30;
            _output[_output_length] = integer_buff / (unsigned long) pow(10, _length_buff - i - 1) % 10 + 0x30;
            integer_buff = integer_buff % (unsigned long) pow(10, _length_buff - i - 1);
            //integer_buff = integer_buff % 10;
        }
        _output_length++;
    }
    _output[_output_length] = '.';
    _output_length++;
    _length_buff = 0;
    while (1) {
        if (decimal / 10 != 0)
            _length_buff++;
        else {
            _length_buff++;
            break;
        }
        decimal = decimal / 10;
    }
    for (int i = 0; i < _length_buff; i++) {
        if (i == _length_buff - 1)
            _output[_output_length] = decimal_buff % 10 + 0x30;
        else {
            _output[_output_length] = decimal_buff / (unsigned long) pow(10, _length_buff - i - 1) % 10 + 0x30;
            decimal_buff = decimal_buff % (unsigned long) pow(10, _length_buff - i - 1);
        }

        _output_length++;
    }
    _output[_output_length] = 0x00;
    _output_length++;
    return_pointer = (unsigned char *) realloc(return_pointer, _output_length);

    *output_length = _output_length - 1;
    if (return_pointer == 0)
        return 0;
    else {
        if (signal_flag == 1) {
            return_pointer[0] = '-';
            memcpy(return_pointer + 1, _output, _output_length);
        } else
            memcpy(return_pointer, _output, _output_length);
    }
    return return_pointer;
}