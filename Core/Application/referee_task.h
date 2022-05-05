#ifndef REFEREE_TASK_H
#define REFEREE_TASK_H

#include <stdbool.h>
#include <stdint-gcc.h>
#include "struct_typedef.h"
#include "fifo.h"
#include "cmsis_os.h"

#define USART6_RX_BUF_LENGHT     512
#define USART6_TX_BUF_LENGHT     128
#define REFEREE_FIFO_BUF_LENGTH 1024
#pragma pack(push, 1)
/**
 * @name std_frame_header_t
 * @brief 裁判系统自定义帧头
 */
typedef struct {
    uint8_t SOF;
    uint16_t data_length;
    uint8_t SEQ;
    uint8_t CRC8;
} std_frame_header_t;

/**
 * @name Judge_Frame_Offset_t
 * @brief 帧字节偏移
 */
typedef enum {
    FRAME_HEADER    = 0,
    CMD_ID          = 5,
    DATA_SEG        = 7
} Judge_Frame_Offset_t;

/**
 * @name Judge_Frame_Header_Offset_t
 * @brief 帧头字节偏移
 */
typedef enum {
    J_SOF           = 0,
    DATA_LENGTH     = 1,
    SEQ             = 3,
    J_CRC8          = 4
} Judge_Frame_Header_Offset_t;

/**
 * @name Judge_Cmd_ID_t
 * @brief 裁判系统命令码ID
 */
typedef enum {
    ID_GAME_STATUS = 0x0001, //!< 比赛状态数据
    ID_GAME_RESULT = 0x0002, //!< 比赛结果数据
    ID_GAME_ROBOT_HP = 0x0003, //!< 比赛机器人血量数据
    ID_ICRA_BUFF_DEBUFF_ZONE_STATUS = 0x0005, //!< 人工智能挑战赛(ICRA)加成与惩罚区状态

    ID_EVENT_DATA = 0x0101,                 //!< 场地事件数据
    ID_SUPPLY_PROJECTILE_ACTION = 0x0102,   //!< 场地补给站动作标识数据
//    ID_SUPPLY_PROJECTILE_BOOKING 	= 0x0103,	// 请求补给站补弹子弹(RM对抗赛尚未开放)
    ID_REFEREE_WARNING = 0x0104,                //!< 裁判警告数据
    ID_DART_REMAINING_TIME = 0x0105,         //!< 飞镖发射口倒计时

    ID_GAME_ROBOT_STATUS = 0x0201,       //!< 机器人状态数据
    ID_POWER_HEAT_DATA = 0x0202,        //!< 实时功率热量数据
    ID_GAME_ROBOT_POS = 0x0203,         //!< 机器人位置数据
    ID_BUFF = 0x0204,              //!< 机器人增益数据
    ID_AERIAL_ROBOT_ENERGY = 0x0205,    //!< 空中机器人能量状态数据
    ID_ROBOT_HURT = 0x0206,             //!< 伤害状态数据
    ID_SHOOT_DATA = 0x0207,             //!< 实时射击数据
    ID_BULLET_REMAINING = 0x0208,       //!< 弹丸剩余发射数
    ID_RFID_STATUS = 0x0209,            //!< 机器人RFID状态
    ID_DART_CLIENT_COMMAND = 0x020A,    //!< 飞镖机器人客户端指令数据

    ID_COMMUNICATION = 0x0301,      //!< 机器人间交互数据
    ID_CUSTOMIZE_CONTROL_DATA = 0x0302, //!<自定义控制器交互数据接口
    ID_MAP_INTERACTION_DATA = 0x0303,        //!< 客户端小地图交互数据
    ID_CONTROL_UART_DATA = 0x0304,      //!< 图传串口发送键盘、鼠标信息
    ID_MAP_DATA = 0X0305,            //!< 客户端小地图接收信息
} CmdID;

/**
 * @name Judge_Data_Length_t
 * @brief 裁判系统数据段长度
 */
typedef enum {
    LEN_FRAME_HEAD = 5,     //!< 帧头长度
    LEN_CMD_ID = 2,         //!< 命令码长度
    LEN_FRAME_TAIL = 2,     //!< 帧尾CRC16
    // 0x000x
    LEN_GAME_STATUS = 11,                      //!< 0x0001 比赛状态数据
    LEN_GAME_RESULT = 1,                      //!< 0x0002 比赛结果数据
    LEN_GAME_ROBOT_HP = 28,                   //!< 0x0003 比赛机器人血量数据
    LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS = 13,     //!< 0x0005 人工智能挑战赛加成与惩罚区状态
    // 0x010x
    LEN_EVENT_DATA = 4,                     //!< 0x0101 场地事件数据
    LEN_SUPPLY_PROJECTILE_ACTION = 3,       //!< 0x0102 场地补给站动作标识数据
//    LEN_SUPPLY_PROJECTILE_BOOKING = 2,       //!< 0x0103 请求补给站补弹数据(RM对抗赛尚未开放)
    LEN_REFEREE_WARNING = 2,                //!< 0x0104 裁判警告数据
    LEN_DART_REMAINING_TIME = 1,            //!< 0x0105 飞镖发射口倒计时
    // 0x020x
    LEN_GAME_ROBOT_STATUS = 15,      //!< 0x0201 机器人状态数据
    LEN_POWER_HEAT_DATA = 14,       //!< 0x0202 实时功率热量数据
    LEN_GAME_ROBOT_POS = 16,        //!< 0x0203 机器人位置数据
    LEN_BUFF = 1,              //!< 0x0204 机器人增益数据
    LEN_AERIAL_ROBOT_ENERGY = 3,    //!< 0x0205 空中机器人能量状态数据
    LEN_ROBOT_HURT = 1,             //!< 0x0206 伤害状态数据
    LEN_SHOOT_DATA = 6,             //!< 0x0207 实时射击数据
    LEN_BULLET_REMAINING = 2,       //!< 0x0208 弹丸剩余发射数
    LEN_RFID_STATUS = 4,            //!< 0x0209 机器人RFID状态
    LEN_DART_CLIENT_COMMAND = 12,          //!< 0x020A 飞镖机器人客户端指令书
    // 0x030x
    LEN_COMMUNICATION = 76,          //!< 0x0301 机器人间交互数据（裁判系统），10HZ，最大128字节，数据段113字节
    LEN_CUSTOMIZE_CONTROL_DATA = 30,          //!< 0x0302 自定义控制器交互数据接口，30HZ
    LEN_MAP_INTERACTION_DATA = 15,          //!< 0x0303 客户端小地图交互数据
    LEN_CONTROL_UART_DATA = 12,          //!< 0x0304 图传串口发送键鼠信息
    LEN_MAP_DATA = 10,          //!< 0x0305 客户端小地图接收信息
} Judge_Data_Length_t;


/**
 * @name ext_game_status_t
 * @brief 比赛状态数据：0x0001 发送频率：1Hz 发送范围：所有机器人
 * @param
 */
typedef struct {
    uint8_t game_type: 4;        //!< 比赛类型
    uint8_t game_progress: 4;    //!< 当前比赛阶段
    uint16_t stage_remain_time;   //!< 当前阶段剩余时间 单位s
    uint64_t SyncTimeStamp;       //!<机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效
} ext_game_status_t;

/**
 * @name ext_game_result_t
 * @brief 比赛结果数据：0x0002 发送频率：比赛结束后发送 发送范围：所有机器人
 */
typedef struct {
    uint8_t winner;     //!< 0 平局 1 红方胜利 2 蓝方胜利
} ext_game_result_t;

/**
 * @name ext_game_robot_HP_t
 * @brief 机器人血量数据：0x0003 发送频率：1Hz 发送范围：所有机器人
 */
typedef struct {
    uint16_t red_1_robot_HP;    //!< 红1英雄机器人血量 未上场以及罚下血量为0
    uint16_t red_2_robot_HP;    //!< 红2工程机器人血量
    uint16_t red_3_robot_HP;    //!< 红3步兵机器人血量
    uint16_t red_4_robot_HP;    //!< 红4步兵机器人血量
    uint16_t red_5_robot_HP;    //!< 红5步兵机器人血量
    uint16_t red_7_robot_HP;    //!< 红7哨兵机器人血量
    uint16_t red_outpost_HP;    //!< 红方前哨站血量
    uint16_t red_base_HP;       //!< 红方基地血量
    uint16_t blue_1_robot_HP;   //!< 蓝1英雄机器人血量
    uint16_t blue_2_robot_HP;   //!< 蓝2工程机器人血量
    uint16_t blue_3_robot_HP;   //!< 蓝3步兵机器人血量
    uint16_t blue_4_robot_HP;   //!< 蓝4步兵机器人血量
    uint16_t blue_5_robot_HP;   //!< 蓝5步兵机器人血量
    uint16_t blue_7_robot_HP;   //!< 蓝7哨兵机器人血量
    uint16_t blue_outpost_HP;   //!< 蓝方前哨站血量
    uint16_t blue_base_HP;      //!< 蓝方基地血量
} ext_game_robot_HP_t;

/**
 * @name ext_ICRA_buff_debuff_zone_status_t
 * @brief 人工智能挑战赛加成与惩罚区状态：0x0005 发送频率：1Hz周期发送 发送范围：所有机器人
 */
typedef struct {
    uint8_t F1_zone_status: 1;
    uint8_t F1_zone_buff_debuff_status: 3;
    uint8_t F2_zone_status: 1;
    uint8_t F2_zone_buff_debuff_status: 3;
    uint8_t F3_zone_status: 1;
    uint8_t F3_zone_buff_debuff_status: 3;
    uint8_t F4_zone_status: 1;
    uint8_t F4_zone_buff_debuff_status: 3;
    uint8_t F5_zone_status: 1;
    uint8_t F5_zone_buff_debuff_status: 3;
    uint8_t F6_zone_status: 1;
    uint8_t F6_zone_buff_debuff_status: 3;
    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;
    uint8_t lurk_mode;
    uint8_t res;
} ext_ICRA_buff_debuff_zone_status_t;

/**
 * @name ext_event_data_t
 * @brief 场地事件数据：0x0101 发送频率：1Hz周期发送 发送范围：己方机器人
 */
typedef struct {
    uint32_t event_type;     //!< bit 0-1:己方停机坪占领状态  bit 2-3:己方能量机关状态  bit 4:己方基地虚拟护盾状态  bit 5 -31:保留
} ext_event_data_t;

/**
 * @name ext_supply_projectile_action_t
 * @brief 补给站动作标识：0x0102 发送频率：动作触发后发送 发送范围：己方机器人
 */
typedef struct {
    uint8_t supply_projectile_id; //!< 补给站口ID
    uint8_t supply_robot_id; //!< 补弹机器人ID
    uint8_t supply_projectile_step; //!< 出弹口开闭状态
    uint8_t supply_projectile_num; //!< 补弹数量
} ext_supply_projectile_action_t;

/**
 * @name ext_referee_warning_t
 * @brief 裁判警告信息：0x0104 发送频率：警告发生后发送 发送范围：己方机器人
 */
typedef struct {
    uint8_t level; //!< 警告等级
    uint8_t foul_robot_id; //!< 犯规机器人ID
} ext_referee_warning_t;

/**
 * @name ext_dart_remaining_time_t
 * @brief 飞镖发射口倒计时：0x0105 发送频率：1Hz周期发送 发送范围：己方机器人
 */
typedef struct {
    uint8_t dart_remaining_time; //!< 15s 倒计时
} ext_dart_remaining_time_t;

/**
 * @name ext_game_robot_status_t
 * @brief 比赛机器人状态：0x0201 发送频率：10Hz 发送范围：单一机器人
 */
typedef struct {
    uint8_t robot_id;                           //!< 机器人ID
    uint8_t robot_level;                        //!< 机器人等级
    uint16_t remain_HP;                         //!< 机器人剩余血量
    uint16_t max_HP;                            //!< 机器人上限血量
    uint16_t shooter_id1_17mm_cooling_rate;        //!< 机器人1号17mm枪口每秒冷却值
    uint16_t shooter_id1_17mm_cooling_limit;      //!< 机器人1号17mm枪口热量上限
    uint16_t shooter_id1_17mm_speed_limit;        //!< 机器人1号17mm枪口上限速度 单位 m/s
    uint16_t shooter_id2_17mm_cooling_rate;        //!< 机器人2号17mm枪口每秒冷却值
    uint16_t shooter_id2_17mm_cooling_limit;        //!< 机器人2号17mm枪口热量上限
    uint16_t shooter_id2_17mm_speed_limit;        //!< 机器人2号17mm枪口上限速度 单位 m/s
    uint16_t shooter_id1_42mm_cooling_rate;        //!< 机器人42mm枪口每秒冷却值
    uint16_t shooter_id1_42mm_cooling_limit;        //!< 机器人42mm枪口热量上限
    uint16_t shooter_id1_42mm_speed_limit;        //!< 机器人42mm枪口口上限速度 单位 m/s
    uint16_t chassis_power_limit;                //!< 机器人底盘功率限制上限
    uint8_t mains_power_gimbal_output: 1;       //!< 主控电源 gimbal 口输出情况
    uint8_t mains_power_chassis_output: 1;      //!< 主控电源 chassis 口输出情况
    uint8_t mains_power_shooter_output: 1;      //!< 主控电源 shooter 口输出情况
} ext_game_robot_status_t;

/**
 * @name referee_robot_ID
 * @brief 裁判系统机器人ID定义
 */
typedef enum {
    hero_red = 1,                    //!< 红方英雄机器人
    engineer_red = 2,                    //!< 红方工程机器人
    infantry3_red = 3,                    //!< 红方步兵机器人
    infantry4_red = 4,                    //!< 红方步兵机器人
    infantry5_red = 5,                    //!< 红方步兵机器人
    plane_red = 6,                    //!< 红方空中机器人
    sentry_red = 7,                    //!< 红方哨兵机器人
    dart_red = 8,                    //!< 红方飞鏢机器人
    radar_red = 9,                    //!< 红方雷达站


    hero_blue = 101,                    //!< 蓝方英雄机器人
    engineer_blue = 102,                    //!< 蓝方工程机器人
    infantry3_blue = 103,                    //!< 蓝方步兵机器人
    infantry4_blue = 104,                    //!< 蓝方步兵机器人
    infantry5_blue = 105,                    //!< 蓝方步兵机器人
    plane_blue = 106,                    //!< 蓝方空中机器人
    sentry_blue = 107,                    //!< 蓝方哨兵机器人
    dart_blue = 108,                    //!< 蓝方飞鏢机器人
    radar_blue = 109,                    //!< 蓝方雷达站
} referee_robot_ID;

/**
 * @name ext_power_heat_data_t
 * @brief 实时功率热量数据 0x0202 发送频率：50Hz 发送范围：单一机器人
 */
typedef struct {
    uint16_t chassis_volt;        //!< 底盘输出电压，单位：mV
    uint16_t chassis_current;        //!< 底盘输出电流，单位：mA
    float chassis_power;            //!< 瞬时功率，单位：W
    uint16_t chassis_power_buffer;    //!< 底盘功率缓冲
    uint16_t shooter_id1_17mm_cooling_heat;        //!< 1号17mm 枪口热量
    uint16_t shooter_id2_17mm_cooling_heat;    //!< 二号机动17mm 枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;    //!<  42mm 枪口热量
} ext_power_heat_data_t;

/**
 * @name ext_game_robot_pos_t
 * @brief 机器人位置：0x0203 发送频率：10Hz 发送范围：单一机器人
 */
typedef struct {
    float target_x;         //!< 位置x坐标 单位m
    float target_y;         //!< 位置y坐标 单位m
    float target_z;         //!< 位置z坐标 单位m
    uint8_t commd_keyboard; //!< 发送指令时,云台手按下的键盘信息
    uint16_t target_ID;     //!< 要作用的目标机器人ID
} ext_game_robot_pos_t;

/**
 * @name ext_buff_t
 * @brief 机器人增益：0x0204 发送频率：1Hz周期发送 发送范围：单一机器人
 */
typedef struct {
    uint8_t power_rune_buff; //!< bit 0:机器人血量补血状态  bit 1:枪口热量冷却加速  bit 2:机器人防御加成  bit 3:机器人攻击加成
} ext_buff_t;

/**
 * @name ext_aerial_robot_energy_t
 * @brief 空中机器人能量状态：0x0205 发送频率：10Hz 发送范围：单一机器人
 */
typedef struct {
    uint8_t attack_time; //!< 可攻击时间 单位 s  30s递减至0
} ext_aerial_robot_energy_t;

/**
 * @name ext_robot_hurt_t
 * @brief 伤害状态：0x0206 发送频率：伤害发生后发送 发送范围：单一机器人
 */
typedef struct {
    uint8_t armor_id: 4;        //!< 当血量变化类型为装甲伤害，代表装甲ID，其中数值为0-4号代表机器人的五个装甲片，其他血量变化类型，该变量数值为0
    uint8_t hurt_type: 4;       //!< 0x0 装甲伤害扣血  0x1 模块掉线扣血  0x2 超射速扣血  0x3 超枪口热量扣血  0x4 超底盘功率扣血  0x5 装甲撞击扣血
} ext_robot_hurt_t;

/**
 * @name ext_shoot_data_t
 * @brief 实时射击信息：0x0207 发送频率：射击后发送 发送范围：单一机器人
 */
typedef struct {
    uint8_t bullet_type;     //!< 子弹类型： 1：17mm弹丸；2：42mm弹丸
    uint8_t shooter_id;      //!< 发射机构ID 1: 1 号 17mm 发射机构 2： 2 号 17mm 发射机构 3： 42mm 发射机构
    uint8_t bullet_freq;     //!< 子弹射频 单位 Hz
    float bullet_speed;      //!< 子弹射速 单位 m/s
} ext_shoot_data_t;

/**
 * @name ext_bullet_remaining_t
 * @brief 子弹剩余发射数：0x0208 发送频率：1Hz周期发送，空中机器人，哨兵机器人以及ICRA机器人主控发送 发送范围：单一机器人
 */
typedef struct {
    uint16_t bullet_remaining_num_17mm;     //!< 17mm子弹剩余发射数目
    uint16_t bullet_remaining_num_42mm;     //!< 42mm子弹剩余发射数目
    uint16_t coin_remaining_num;     //!< 剩余金市数量
} ext_bullet_remaining_t;

/**
 * @name ext_RFID_status_t
 * @brief 机器人RFID状态：0x0209 发送频率：1Hz 发送范围：单一机器人
 */
typedef struct {
    uint32_t RFID_status; //!< bit 0:基地增益点RFID状态  bit 1:高地增益点RFID状态  bit 2:能量机关激活点RFID状态  bit 3:飞坡增益点RFID状态  bit 4:前哨岗增益点RFID状态  bit 6:补血点增益点RFID状态  bit 7:工程机器人补血卡RFID状态  bit 8-25:保留  bit 26-31:人工智能挑战赛F1-F6 RFID状态
} ext_RFID_status_t;

/**
 * @name ext_dart_client_cmd_t
 * @brief 飞镖机器人客户端指令数据：0x020A 发送频率：10Hz 发送范围：单一机器人
 */
typedef struct {
    uint8_t dart_launch_opening_status;         //!< 当前飞镖发射口的状态 0：关闭  1：正在开启或者关闭中  2：已经开启
    uint8_t dart_attack_target;                 //!< 飞镖的打击目标，默认为前哨站  1：前哨站  2：基地
    uint16_t target_change_time;                 //!< 切换打击目标时的比赛剩余时间 单位秒 从未切换默认为0
    uint16_t operate_launch_cmd_time;           //!< 最近一次操作手确定发射指令时的比赛剩余时间 单位秒 初始值为0
} ext_dart_client_cmd_t;

/**
 * @name ext_student_interactive_header_data_t
 * @brief 机器人间通信 交互数据接收信息：0x0301
 */
typedef struct {
    uint16_t data_CMD_ID;         //!< 数据段的内容ID
    uint16_t sender_ID;           //!< 发送者的ID 需要校验发送者的ID正确性
    uint16_t receiver_ID;         //!< 接收者的ID 需要校验接收者的ID正确性
} ext_student_interactive_header_data_t;

/**
 * @name ext_aerial_data_t
 * @brief 云台手交互：0x0301
 */
typedef struct {
    uint16_t cmd_id;
    uint16_t send_id;
    uint16_t receive_id;

    enum cmd_t {
        stop_fire = 0x1,
        back_scan = 0x2,
        escape = 0x4,
        check_road = 0x8,
        control_aerial = 0x10,
    } cmd;
    int8_t control_dir; //!< -1和1
} ext_aerial_data_t;

/**
 * @name ext_aerial_data_t
 * @brief 交互数据接收信息：0x0302 发送频率：上限30Hz
 */
typedef struct {
    uint8_t data[30];
} robot_interactive_data_t;

/**
 * @name ext_robot_command_t
 * @brief 0x0303 客户端小地图交互数据
 */
typedef struct {
    float target_x;         //!< 目标ⅹ位置坐标,单位m,当发送目标机器人ID时,该项为0
    float target_y;         //!< 目标y位置坐标,单位m,当发送目标机器人ID时,该项为0
    float target_z;         //!< 目标z位置坐标,单位m,当发送目标机器人ID时,该项为0
    uint8_t command_keyboard;         //!< 发送指令时,云台手按下的键盘信息,无按键按下则为0
    uint16_t target_robot_ID;         //!< 要作用的目标机器人ID,当发送位置信息时,该项为0
} ext_robot_command_t;

/**
 * @name ext_control_uart_t
 * @brief 0x0304 图传遥控信息
 */
typedef struct {
    int16_t mouse_x;         //!< 鼠标X轴信息
    int16_t mouse_y;         //!< 鼠标Y轴信
    int16_t mouse_z;         //!< 鼠标滚轮信息
    uint8_t left_button_down;         //!< 鼠标左键是否按下
    uint8_t right_button_down;         //!< 鼠标右键是否按下
    uint16_t keyboard_value;         //!< 键盘信息
    uint16_t reserved;         //!< 保留位
} ext_control_uart_t;

/**
 * @name ext_client_map_command_t
 * @brief 0x0305 小地图接收信息
 */
typedef struct {
    uint16_t target_robot_ID;
    float target_position_x;
    float target_position_y;
    float reserved;
} ext_client_map_command_t;

/**
 * @name ext_client_custom_graphic_delete_t
 * @brief 客户端删除图形 机器人间通信：0x0301
 */
typedef struct {
    uint8_t operate_type;         //!< 图形操作 0: 空操作  1: 删除图层  2: 删除所有
    uint8_t layer;                //!< 图层数 : 0 - 9
} ext_client_custom_graphic_delete_t;

/**
 * @name graphic_data_struct_t
 * @brief 图形数据 机器人间通信 0x0301
 */
typedef struct {
    uint8_t graphic_name[3];         //!< 图形名 : 在删除，修改等操作中，作为客户端的索引
    uint32_t operate_type: 3;       //!< 图形操作
    uint32_t graphic_type: 3;       //!< 图形类型
    uint32_t layer: 4;              //!< 图层数 : 0 - 9
    uint32_t color: 4;              //!< 颜色
    uint32_t start_angle: 9;        //!< 起始角度 单位：° 范围[0,360]
    uint32_t end_angle: 9;          //!< 终止角度 单位：° 范围[0,360]
    uint32_t width: 10;             //!< 线宽
    uint32_t start_x: 11;           //!< 起点x坐标
    uint32_t start_y: 11;           //!< 起点y坐标
    uint32_t radius: 10;            //!< 半径
    uint32_t end_x: 11;             //!< 终点x坐标
    uint32_t end_y: 11;             //!< 终点y坐标
} graphic_data_struct_t;

/**
 * @name graphic_type_enum_t
 * @brief 图形配置 图形类型定义
 */
typedef enum {
    graphic_type_line       = 0, //!< 直线 @details start_angle : 空<br>end_angle : 空<br>width : 线条宽度<br>start_x : 起点x坐标<br>start_y : 起点y坐标<br>radius : 空<br>end_x : 终点x坐标<br>end_y : 终点y坐标
    graphic_type_rectangle  = 1, //!< 矩形 @details start_angle : 空<br>end_angle : 空<br>width : 线条宽度<br>start_x : 起点x坐标<br>start_y : 起点y坐标<br>radius : 空<br>end_x : 对角顶点x坐标<br>end_y : 对角顶点y坐标
    graphic_type_circle     = 2, //!< 正圆 @details start_angle : 空<br>end_angle : 空<br>width : 线条宽度<br>start_x : 圆心x坐标<br>start_y : 圆心y坐标<br>radius : 半径<br>end_x : 空<br>end_y : 空
    graphic_type_ellipse    = 3, //!< 椭圆 @details start_angle : 空<br>end_angle : 空<br>width : 线条宽度<br>start_x : 圆心x坐标<br>start_y : 圆心y坐标<br>radius : 空<br>end_x : x半轴长度<br>end_y : y半轴长度
    graphic_type_arc        = 4, //!< 圆弧 @details start_angle : 起始角度<br>end_angle : 终止角度<br>width : 线条宽度<br>start_x : 圆心x坐标<br>start_y : 圆心y坐标<br>radius : 空<br>end_x : x半轴长度<br>end_y : y半轴长度
    graphic_type_float      = 5, //!< 浮点数 @details start_angle : 字体大小<br>end_angle : 小数位有效个数<br>width : 线条宽度<br>start_x : 起点x坐标<br>start_y : 起点y坐标<br>radius, end_x, end_y : 32位浮点数，float
    graphic_type_int        = 6, //!< 浮点数 @details start_angle : 字体大小<br>end_angle : 空<br>width : 线条宽度<br>start_x : 起点x坐标<br>start_y : 起点y坐标<br>radius, end_x, end_y : 32位整型数，int32_t
    graphic_type_char       = 7  //!< 字符 @details start_angle : 字体大小<br>end_angle : 字符长度<br>width : 线条宽度start_x : 起点x坐标<br>start_y : 起点y坐标<br>radius, end_x, end_y : 空
} graphic_type_enum_t;

/**
 * @name graphic_color_enum_t
 * @brief 图形配置 图形颜色定义
 */
typedef enum {
    graphic_color_main      = 0, //!< 红蓝主色
    graphic_color_yellow    = 1, //!< 黄色
    graphic_color_green     = 2, //!< 绿色
    graphic_color_orange    = 3, //!< 橙色
    graphic_color_fuchsia   = 4, //!< 紫红色
    graphic_color_pink      = 5, //!< 粉色
    graphic_color_cyan      = 6, //!< 青色
    graphic_color_black     = 7, //!< 黑色
    graphic_color_white     = 8  //!< 白色
} graphic_color_enum_t;

/**
 * @name ext_client_custom_graphic_single_t
 * @brief 客户端绘制一个图形 机器人间通信：0x0301
 */
typedef struct {
    graphic_data_struct_t graphic_data_struct;
} ext_client_custom_graphic_single_t;

/**
 * @name ext_client_custom_graphic_double_t
 * @brief 客户端绘制二个图形 机器人间通信：0x0301
 */
typedef struct {
    graphic_data_struct_t graphic_data_struct[2];
} ext_client_custom_graphic_double_t;

/**
 * @name ext_client_custom_graphic_five_t
 * @brief 客户端绘制五个图形 机器人间通信：0x0301
 */
typedef struct {
    graphic_data_struct_t graphic_data_struct[5];
} ext_client_custom_graphic_five_t;

/**
 * @name ext_client_custom_graphic_seven_t
 * @brief 客户端绘制七个图形 机器人间通信：0x0301
 */
typedef struct {
    graphic_data_struct_t graphic_data_struct[7];
} ext_client_custom_graphic_seven_t;

/**
 * @name ext_client_custom_character_t
 * @brief 客户端绘制字符 机器人间通信：0x0301
 */
typedef struct {
    graphic_data_struct_t graphic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

typedef volatile struct {
    uint16_t frame_length;            //!< 帧长度(调试时使用)
    uint16_t cmd_id;                  //!< 命令码(调试时使用)
    uint16_t err_cnt;                 //!< 错帧数(调试时使用)
    bool data_valid;                  //!< 数据有效性
    uint16_t self_client_id;          //!< 机器人对应的客户端ID
    bool power_heat_update;           //!< 功率热量数据更新
    bool shoot_update;                //!< 射击数据更新
    bool hurt_data_update;            //!< 伤害数据更新
    bool ICRA_buff_debuff_zone_status_update; //!< 人工智能挑战赛加成与惩罚区状态更新
    bool supply_data_update;          //!< 补给站数据更新
    bool dart_data_update;            //!< 飞镖数据更新
    bool communication_data_update;   //!< 云台手数据更新
    bool customize_control_data_update;   //!< 自定义控制器交互数据更新
    bool map_data_update;         //!< 小地图数据更新
    bool control_uart_data_update;         //!< 图传串口发送键鼠信息
    bool client_map_data_update;         //!< 客户端小地图坐标数据更新
    //0x000x
    std_frame_header_t FrameHeader;       //!< 帧头信息
    ext_game_status_t GameStatus;         //!< 0x0001 比赛状态数据
    ext_game_result_t GameResult;         //!< 0x0002 比赛结果数据
    ext_game_robot_HP_t GameRobotHP;      //!< 0x0003 比赛机器人血量数据
    ext_ICRA_buff_debuff_zone_status_t ICRA_Buff_Debuff_zone_status; //!< 0x0005 人工智能挑战赛(ICRA)加成与惩罚区状态
    //0x010x
    ext_event_data_t EventData; //!< 0x0101 场地事件数据
    ext_supply_projectile_action_t SupplyProjectileAction; //!< 0x0102 场地补给站动作标识数据
    ext_referee_warning_t RefereeWarning; //!< 0x0104 裁判警告数据
    ext_dart_remaining_time_t DartRemainingTime; //!< 0x0105 飞镖发射口倒计时
    //0x020x
    ext_game_robot_status_t GameRobotStatus; //!< 0x0201 机器人状态数据
    ext_power_heat_data_t PowerHeatData; //!< 0x0202 实时功率热量数据
    ext_game_robot_pos_t GameRobotPos; //!< 0x0203 机器人位置数据
    ext_buff_t Buff; //!< 0x0204 机器人增益数据
    ext_aerial_robot_energy_t AerialRobotEnergy; //!< 0x0205 空中机器人能量状态数据
    ext_robot_hurt_t RobotHurt; //!< 0x0206 伤害状态数据
    ext_shoot_data_t ShootData; //!< 0x0207 实时射击数据
    ext_bullet_remaining_t BulletRemaining; //!< 0x0208 弹丸剩余发射数
    ext_RFID_status_t RFIDStatus; //!< 0x0209 机器人RFID状态
    ext_dart_client_cmd_t DartClientCMD; //!< 0x020A 飞镖机器人客户端指令数据
    //0x030x
    ext_aerial_data_t AerialData; //!< 0x0301 机器人间交互数据
    robot_interactive_data_t RobotInteractiveData;//!< 0x0302 自定义控制器交互数据接口数据
    ext_robot_command_t command; //!< 0x0303 客户端小地图交互数据
    ext_control_uart_t ControlUart; //!< 0x0304 图传串口发送键盘、鼠标信息
    ext_client_map_command_t ClientMapCommand; //!< 0x0305 客户端小地图接收信息
} judge_info_t;

/*************define for fifo start*********************/
#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            5
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + REF_PROTOCOL_CMD_SIZE)
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CMD_SIZE)


typedef enum {
    STEP_HEADER_SOF = 0,
    STEP_LENGTH_LOW = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16 = 5,
} unpack_step_e;

typedef struct {
    std_frame_header_t *p_header;
    uint16_t data_len;
    uint8_t protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e unpack_step;
    uint16_t index;
} unpack_data_t;

#pragma pack(pop)
/***************define for fifo end*******************/
/***************function and variable declare start*******************/
//RX Declare
extern judge_info_t global_judge_info;

void judge_update(uint8_t *rxBuf);

extern uint8_t get_robot_id(void);

static void referee_unpack_fifo_data(void);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);

extern void init_referee_struct_data(void);

extern void referee_rx_task(void const *argument);

extern void referee_tx_task(void const *argument);

#pragma pack(push, 1)
//TX Declare
//#define DRAWING_PACK	15
/* UI绘制数据包 */
//uint8_t data_pack[DRAWING_PACK*7] = {0};
/**
 * @name FrameHeader
 * @brief 裁判系统自定义发送帧头+cmdid
 */
typedef struct {
    uint8_t SOF;
    uint16_t data_length;
    uint8_t SEQ;
    uint8_t CRC8;
    uint16_t CmdID;
} FrameHeader;

typedef enum {
    NULL_OPERATION = 0U,
    ADD_PICTURE = 1U,
    MODIFY_PICTURE = 2U,
    CLEAR_ONE_PICTURE = 3U,
} drawOperate_e;
#pragma pack(pop)
#define DRAWING_PACK    15
extern volatile uint8_t Referee_No_DMA_IRQHandler;
extern volatile uint8_t referee_dma_send_data_len;
extern uint8_t data_pack[DRAWING_PACK * 7];
extern ext_client_custom_graphic_delete_t cleaning;
/******declare move frome referee_task.c********/
extern fifo_s_t referee_rx_fifo;
extern fifo_s_t referee_tx_len_fifo;
extern fifo_s_t referee_tx_fifo;
extern uint8_t usart6_rx_buf[2][USART6_RX_BUF_LENGHT];
extern uint8_t usart6_tx_buf[2][USART6_TX_BUF_LENGHT];

/******declare move frome referee_task.c********/
static void send_toReferee(uint16_t _cmd_id, uint16_t _data_len);

static void UI_clean_all(void);

extern TaskHandle_t USART6TX_active_task_local_handler;
void USART6_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
/**
  * @brief          usart6发送启动任务，由发送时任务通知激活
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void USART6TX_active_task(void const *pvParameters);

static void
Hero_UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t *line_distance, uint16_t *line_length,
              graphic_color_enum_t *_color, drawOperate_e _operate_type);

extern void MY_USART_DMA_Stream6_TX_IRQHandler(void);

static graphic_data_struct_t *
line_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx, uint16_t starty, uint16_t endx,
             uint16_t endy, uint16_t line_width, graphic_color_enum_t vcolor, uint8_t name[]);
//void line_drawing(uint8_t *name, drawOperate_e _operate_type, uint8_t _layer, graphic_color_enum_t vcolor, uint16_t line_width,uint16_t startx, uint16_t starty, uint16_t endx,
//                  uint16_t endy );
/***************function and variable declare end*******************/
unsigned char *out_float(double value, unsigned char decimal_digit, unsigned char *output_length);
#endif