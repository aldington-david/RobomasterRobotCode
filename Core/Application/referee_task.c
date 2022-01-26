#include <memory.h>
#include "referee_task.h"
#include "CRC8_CRC16.h"
#include "usart.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "fifo.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"

#define printf(format, args...)  SEGGER_RTT_printf(0, format, ##args)
fifo_s_t referee_rx_fifo;
fifo_s_t referee_tx_len_fifo;
fifo_s_t referee_tx_fifo;
uint8_t referee_fifo_rx_buf[REFEREE_FIFO_BUF_LENGTH];
uint8_t referee_fifo_tx_len_buf[REFEREE_FIFO_BUF_LENGTH];
uint8_t referee_fifo_tx_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
judge_info_t global_judge_info;
uint8_t usart6_rx_buf[2][USART_RX_BUF_LENGHT];
uint8_t usart6_tx_buf[2][USART_TX_BUF_LENGHT];
/* 发送数据包缓存区，最大128字节 */
uint8_t transmit_pack[128];
static uint8_t dma_send_data_len;
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
    usart6_init(usart6_rx_buf[0], usart6_rx_buf[1], USART_RX_BUF_LENGHT, usart6_tx_buf[0], usart6_tx_buf[1],
                USART_TX_BUF_LENGHT);

    while (1) {

        referee_unpack_fifo_data();
        osDelay(10);
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
//                        printf("/r/n judge_client_id=%d",judge_info->self_client_id);
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
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_rx_fifo, (char *) usart6_rx_buf[0], this_time_rx_len);
            detect_hook(REFEREE_RX_TOE);
        } else {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_rx_fifo, (char *) usart6_rx_buf[1], this_time_rx_len);
            detect_hook(REFEREE_RX_TOE);
        }
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
    static uint8_t name[3] = "xx";
    osDelay(1000);
//    line_drawing(0, ADD_PICTURE, 200, 200, 400, 400, 100, graphic_color_yellow, name);
    while (1) {

//        SEGGER_RTT_WriteString(0, "referee_task_loop_on");
//        draw_cnt++;
//        name[0] = draw_cnt / 255;
//        name[1] = draw_cnt % 255;
        line_drawing(0, ADD_PICTURE, 200, 200, 400, 400, 100, graphic_color_yellow, name);
        line_drawing(0, MODIFY_PICTURE, 150, 200, 700, 400, 100, graphic_color_yellow, name);
        line_drawing(0, MODIFY_PICTURE, 150, 200, 700, 400, 100, graphic_color_yellow, name);
        osDelay(1000);
//        vTaskDelay(50000);
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
    data_header.data_CMD_ID = _data_cmd_id;
    data_header.sender_ID = global_judge_info.GameRobotStatus.robot_id;                                        //设置发送者ID
    data_header.receiver_ID = global_judge_info.self_client_id;
//    printf("/r/nclient_id=%d",global_judge_info.self_client_id);
    uint8_t header_len = sizeof(data_header);
    memcpy((void *) (transmit_pack + 7), &data_header, header_len);                        //将数据帧的数据段进行封装（封装段首）
    memcpy((void *) (transmit_pack + 7 + header_len), _data, _data_len);                    //将数据帧的数据段进行封装（封装数据）
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
    FrameHeader send_frame_header;                                                                                            //交互数据帧帧头设置
    send_frame_header.SOF = HEADER_SOF;
    send_frame_header.data_length = _data_len;
    if (seq == 255) {
        seq = 0;
    }
    send_frame_header.SEQ = seq++;
//    printf("/r/n seq=%d", send_frame_header.SEQ);
    send_frame_header.CRC8 = get_CRC8_check_sum((uint8_t *) &send_frame_header, 4, 0xff);
    send_frame_header.CmdID = _cmd_id;

    uint8_t header_len = sizeof(send_frame_header);

    memcpy((void *) transmit_pack, &send_frame_header,
           header_len);                                                            //将帧头装入缓存区																		//将数据段转入缓存区
    append_CRC16_check_sum((uint8_t *) &transmit_pack, header_len + _data_len + 2);
    fifo_s_put(&referee_tx_len_fifo, header_len + _data_len + 2);
    fifo_s_puts(&referee_tx_fifo, (char *) &transmit_pack, header_len + _data_len + 2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    printf("%d", fifo_s_used(&referee_tx_fifo));
    if (No_DMA_IRQHandler) {
        if (fifo_s_used(&referee_tx_fifo)) {
            if (fifo_s_used(&referee_tx_len_fifo)) {
                if ((huart6.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_1");
                    __HAL_DMA_DISABLE(huart6.hdmatx);
                    dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                    memset(&usart6_tx_buf[1], 0, USART_TX_BUF_LENGHT);
                    fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[1], dma_send_data_len);
                    huart6.hdmatx->Instance->CR |= DMA_SxCR_CT;
                    __HAL_DMA_SET_COUNTER(huart6.hdmatx, dma_send_data_len);
                    __HAL_DMA_ENABLE(huart6.hdmatx);
                    No_DMA_IRQHandler = 0;
                    detect_hook(REFEREE_TX_TOE);
                    if (fifo_s_used(&referee_tx_fifo)) {
                        if (fifo_s_used(&referee_tx_len_fifo)) {
                            dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                            memset(&usart6_tx_buf[0], 0, USART_TX_BUF_LENGHT);
                            fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[0], dma_send_data_len);
                        }
                    } else {
                        dma_send_data_len = 0;
                        No_DMA_IRQHandler = 1;
                        memset(&usart6_tx_buf[0], 0, USART_TX_BUF_LENGHT);
                    }
                } else {
//                    SEGGER_RTT_WriteString(0, "ST0DMA_0");
                    __HAL_DMA_DISABLE(huart6.hdmatx);
                    dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                    memset(&usart6_tx_buf[0], 0, USART_TX_BUF_LENGHT);
                    fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[0], dma_send_data_len);
                    huart6.hdmatx->Instance->CR &= ~(DMA_SxCR_CT);
                    __HAL_DMA_SET_COUNTER(huart6.hdmatx, dma_send_data_len);
                    __HAL_DMA_ENABLE(huart6.hdmatx);
                    No_DMA_IRQHandler = 0;
                    detect_hook(REFEREE_TX_TOE);
                    if (fifo_s_used(&referee_tx_fifo)) {
                        if (fifo_s_used(&referee_tx_len_fifo)) {
                            dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                            memset(&usart6_tx_buf[1], 0, USART_TX_BUF_LENGHT);
                            fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[1], dma_send_data_len);

                        }
                    } else {
                        dma_send_data_len = 0;
                        No_DMA_IRQHandler = 1;
                        memset(&usart6_tx_buf[1], 0, USART_TX_BUF_LENGHT);
                    }
                }
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
//void UI_clean_all(void)                                                                                    //清除所有自定义图案
//{
//    cleaning.operate_tpye = CLEAR_ALL;
//    pack_send_robotData(Drawing_Clean_ID, robot_client_ID.client, (uint8_t *) &cleaning, sizeof(cleaning));
//}
/**
 * @brief 直线绘制数据包
 * @param line_width 线宽
 * @retval
 */
void line_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx, uint16_t starty, uint16_t endx,
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
    pack_send_robotData(0x0101, (uint8_t *) &drawing, sizeof(drawing));
}

void MY_USART_DMA_Stream6_TX_IRQHandler(void) {
    __HAL_DMA_DISABLE(huart6.hdmatx);
    __HAL_DMA_CLEAR_FLAG(huart6.hdmatx, DMA_HISR_TCIF6);
    __HAL_DMA_CLEAR_FLAG (huart6.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(huart6.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart6.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(huart6.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart6.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(huart6.hdmatx));
    __HAL_DMA_CLEAR_FLAG (huart6.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(huart6.hdmatx));
    if (dma_send_data_len) {
        if ((huart6.hdmatx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart6.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart6.hdmatx, DMA_HISR_TCIF6);
            __HAL_DMA_SET_COUNTER(huart6.hdmatx, dma_send_data_len);
//            SEGGER_RTT_WriteString(0, "ST1DMA_0");
            __HAL_DMA_ENABLE(huart6.hdmatx);
            detect_hook(REFEREE_TX_TOE);
            if (fifo_s_used(&referee_tx_fifo)) {
                if (fifo_s_used(&referee_tx_len_fifo)) {
                    dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                    memset(&usart6_tx_buf[1], 0, USART_TX_BUF_LENGHT);
                    fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[1], dma_send_data_len);
                }
            } else {
                dma_send_data_len = 0;
                No_DMA_IRQHandler = 1;
                memset(&usart6_tx_buf[1], 0, USART_TX_BUF_LENGHT);
            }
        } else {
            __HAL_DMA_DISABLE(huart6.hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart6.hdmatx, DMA_HISR_TCIF6);
            __HAL_DMA_SET_COUNTER(huart6.hdmatx, dma_send_data_len);
//            SEGGER_RTT_WriteString(0, "ST1DMA_1");
            __HAL_DMA_ENABLE(huart6.hdmatx);
            detect_hook(REFEREE_TX_TOE);
            if (fifo_s_used(&referee_tx_fifo)) {
                if (fifo_s_used(&referee_tx_len_fifo)) {
                    dma_send_data_len = fifo_s_get(&referee_tx_len_fifo);
                    memset(&usart6_tx_buf[0], 0, USART_TX_BUF_LENGHT);
                    fifo_s_gets(&referee_tx_fifo, (char *) usart6_tx_buf[0], dma_send_data_len);
                }
            } else {
                dma_send_data_len = 0;
                No_DMA_IRQHandler = 1;
                memset(&usart6_tx_buf[0], 0, USART_TX_BUF_LENGHT);
            }
        }
    }
}