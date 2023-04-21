#ifndef REFEREE_TASK_H
#define REFEREE_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "struct_typedef.h"
#include "fifo.h"
#include "cmsis_os.h"

#define USART6_RX_BUF_LENGHT     512
#define USART6_TX_BUF_LENGHT     128
#define REFEREE_FIFO_BUF_LENGTH 1024

//装甲板受击bit
#define ARMOR_0            ((uint8_t)1 << 0)
#define ARMOR_1            ((uint8_t)1 << 1)
#define ARMOR_2            ((uint8_t)1 << 2)
#define ARMOR_3            ((uint8_t)1 << 3)

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
    FRAME_HEADER = 0,
    CMD_ID = 5,
    DATA_SEG = 7
} Judge_Frame_Offset_t;

/**
 * @name Judge_Frame_Header_Offset_t
 * @brief 帧头字节偏移
 */
typedef enum {
    J_SOF = 0,
    DATA_LENGTH = 1,
    SEQ = 3,
    J_CRC8 = 4
} Judge_Frame_Header_Offset_t;

/**
 * @name Judge_Cmd_ID_t
 * @brief 裁判系统命令码ID
 */
typedef enum {
    //0x000x
    ID_GAME_STATUS = 0x0001, //!< 比赛状态数据
    ID_GAME_RESULT = 0x0002, //!< 比赛结果数据
    ID_GAME_ROBOT_HP = 0x0003, //!< 比赛机器人血量数据
    ID_ICRA_BUFF_DEBUFF_ZONE_STATUS = 0x0005, //!< 人工智能挑战赛(ICRA)加成与惩罚区状态
    //0x010x
    ID_EVENT_DATA = 0x0101,                 //!< 场地事件数据
    ID_SUPPLY_PROJECTILE_ACTION = 0x0102,   //!< 场地补给站动作标识数据
    ID_SUPPLY_PROJECTILE_BOOKING = 0x0103,    // 请求补给站补弹子弹(RM对抗赛尚未开放)
    ID_REFEREE_WARNING = 0x0104,                //!< 裁判警告数据
    ID_DART_REMAINING_TIME = 0x0105,         //!< 飞镖发射口倒计时
    //0x020x
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
    ID_GROUND_ROBOT_POSITION = 0x020B,    //!< 地面机器人位置数据
    ID_RADAR_MARK_DATA = 0x020C,    //!< 雷达标记进度数据
    //0x030x
    ID_COMMUNICATION = 0x0301,      //!< 机器人间交互数据
    ID_CUSTOMIZE_CONTROL_DATA = 0x0302, //!<自定义控制器交互数据接口
    ID_MAP_INTERACTION_DATA = 0x0303,        //!< 客户端小地图交互数据
    ID_CONTROL_UART_DATA = 0x0304,      //!< 图传串口发送键盘、鼠标信息
    ID_MAP_DATA = 0X0305,            //!< 客户端小地图接收信息
    ID_CUSTOM_CLIENT_DATA = 0X0306,     //!< 自定义控制器与选手端交互数据
    ID_MAP_SENTRY_DATA = 0X0307,            //!< 选手端小地图接收哨兵数据
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
    LEN_GAME_ROBOT_HP = 32,                   //!< 0x0003 比赛机器人血量数据
    LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS = 13,     //!< 0x0005 人工智能挑战赛加成与惩罚区状态
    // 0x010x
    LEN_EVENT_DATA = 4,                     //!< 0x0101 场地事件数据
    LEN_SUPPLY_PROJECTILE_ACTION = 4,       //!< 0x0102 场地补给站动作标识数据
    LEN_SUPPLY_PROJECTILE_BOOKING = 2,       //!< 0x0103 请求补给站补弹数据(RM对抗赛尚未开放)
    LEN_REFEREE_WARNING = 2,                //!< 0x0104 裁判警告数据
    LEN_DART_REMAINING_TIME = 1,            //!< 0x0105 飞镖发射口倒计时
    // 0x020x
    LEN_GAME_ROBOT_STATUS = 27,      //!< 0x0201 机器人状态数据
    LEN_POWER_HEAT_DATA = 16,       //!< 0x0202 实时功率热量数据
    LEN_GAME_ROBOT_POS = 16,        //!< 0x0203 机器人位置数据
    LEN_BUFF = 1,              //!< 0x0204 机器人增益数据
    LEN_AERIAL_ROBOT_ENERGY = 1,    //!< 0x0205 空中机器人能量状态数据
    LEN_ROBOT_HURT = 1,             //!< 0x0206 伤害状态数据
    LEN_SHOOT_DATA = 7,             //!< 0x0207 实时射击数据
    LEN_BULLET_REMAINING = 6,       //!< 0x0208 弹丸剩余发射数
    LEN_RFID_STATUS = 4,            //!< 0x0209 机器人RFID状态
    LEN_DART_CLIENT_COMMAND = 6,          //!< 0x020A 飞镖机器人客户端指令书
    LEN_GROUND_ROBOT_POSITION = 40,       //!< 0x020B 地面机器人位置数据
    LEN_RADAR_MARK_DATA = 6,             //!< 0x020C 雷达标记进度数据

    // 0x030x
    LEN_COMMUNICATION = 113 + 6,          //!< 0x0301 机器人间交互数据（裁判系统），10HZ，最大128字节，数据段113字节
    LEN_CUSTOMIZE_CONTROL_DATA = 30,          //!< 0x0302 自定义控制器交互数据接口，30HZ
    LEN_MAP_INTERACTION_DATA = 15,          //!< 0x0303 客户端小地图交互数据
    LEN_CONTROL_UART_DATA = 12,          //!< 0x0304 图传串口发送键鼠信息
    LEN_MAP_DATA = 10,          //!< 0x0305 客户端小地图接收信息
    LEN_CUSTOM_CLIENT_DATA = 8,          //!< 0x0306 自定义控制器与选手端交互数据
    LEN_MAP_SENTRY_DATA = 103,          //!< 0x0307 选手端小地图接收哨兵数据
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
} ext_ICRA_buff_debuff_zone_and_lurk_status_t;

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
    uint8_t offending_robot_id; //!< 犯规机器人ID
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

typedef enum {
    client_hero_red = 0x0101,                    //!< 红方英雄机器人
    client_engineer_red = 0x0102,                    //!< 红方工程机器人
    client_infantry3_red = 0x0103,                    //!< 红方步兵机器人
    client_infantry4_red = 0x0104,                    //!< 红方步兵机器人
    client_infantry5_red = 0x0105,                    //!< 红方步兵机器人
    client_plane_red = 0x0106,                    //!< 红方空中机器人


    client_hero_blue = 0x0165,                    //!< 蓝方英雄机器人
    client_engineer_blue = 0x0166,                    //!< 蓝方工程机器人
    client_infantry3_blue = 0x0167,                    //!< 蓝方步兵机器人
    client_infantry4_blue = 0x0168,                    //!< 蓝方步兵机器人
    client_infantry5_blue = 0x0169,                    //!< 蓝方步兵机器人
    client_plane_blue = 0x016A,                    //!< 蓝方空中机器人
} referee_client_ID;

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
    uint32_t RFID_status; //!< bit 0:基地增益点RFID状态  bit 1:高地增益点RFID状态  bit 2:能量机关激活点RFID状态  bit 3:飞坡增益点RFID状态  bit 4:前哨岗增益点RFID状态  bit 6:补血点增益点RFID状态  bit 7:工程机器人复活卡RFID状态  bit 8-25:保留  bit 26-31:人工智能挑战赛F1-F6 RFID状态
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
 * @name ground_robot_position_t
 * @brief 地面机器人位置数据：0x020B，对哨兵机器人发送，以1Hz 频率发送
 */
typedef struct {
    float hero_x;                               //!< 己方英雄机器人位置X 轴坐标，单位：m
    float hero_y;                               //!< 己方英雄机器人位置Y 轴坐标，单位：m
    float engineer_x;                           //!< 己方工程机器人位置X 轴坐标，单位：m
    float engineer_y;                           //!< 己方工程机器人位置Y 轴坐标，单位：m
    float standard_3_x;                         //!< 己方3 号步兵机器人位置X 轴坐标，单位：m
    float standard_3_y;                         //!< 己方3 号步兵机器人位置Y 轴坐标，单位：m
    float standard_4_x;                         //!< 己方4 号步兵机器人位置X 轴坐标，单位：m
    float standard_4_y;                         //!< 己方4 号步兵机器人位置Y 轴坐标，单位：m
    float standard_5_x;                         //!< 己方5 号步兵机器人位置X 轴坐标，单位：m
    float standard_5_y;                         //!< 己方5 号步兵机器人位置Y 轴坐标，单位：m
} ground_robot_position_t;

/**
 * @name radar_mark_data_t
 * @brief 雷达标记进度数据：0x020C 向雷达发送，以1Hz 频率发送
 */
typedef struct {
    uint8_t mark_hero_progress;                 //!< 对方英雄机器人被标记进度：0~120
    uint8_t mark_engineer_progress;             //!< 对方工程机器人被标记进度：0~120
    uint8_t mark_standard_3_progress;           //!< 对方3 号步兵机器人被标记进度：0~120
    uint8_t mark_standard_4_progress;           //!< 对方4 号步兵机器人被标记进度：0~120
    uint8_t mark_standard_5_progress;           //!< 对方5 号步兵机器人被标记进度：0~120
    uint8_t mark_sentry_progress;               //!< 对方哨兵机器人被标记进度：0~120
} radar_mark_data_t;


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
 * @name 自定义控制器与机器人交互数据
 * @brief 交互数据接收信息：0x0302 发送频率：上限30Hz
 */
typedef struct {
    uint8_t data[30];
} ext_custom_robot_data_t;

/**
 * @name ext_robot_command_t
 * @brief 0x0303 客户端小地图交互数据,两次发送间隔不得低于3秒
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
    uint16_t target_robot_ID;       //!< 目标机器人ID 当x,y 超出界限时则不显示。
    float target_position_x;        //!< 目标x 位置坐标，单位m 当x,y 超出界限时则不显示。
    float target_position_y;        //!< 目标y 位置坐标，单位m 当x,y 超出界限时则不显示。
    float reserved;
} ext_client_map_command_t;

/**
 * @name custom_client_data_t
 * @brief 0x0306 自定义控制器与选手端交互数据，发送方触发发送，频率上限为 30Hz
 */
typedef struct {
    uint16_t target_robot_ID;       //!< bit 0-7：按键1 键值，bit 8-15：按键2 键值（1.仅响应选手端开放的按键 2. 使用通用键值，支持2 键无冲，键值顺序 变更不会改变按下状态，若无新的按键信 息，将保持上一帧数据的按下状态）
    float target_position_x;        //!< bit 0-11：鼠标X 轴像素位置，bit 12-15：鼠标左键状态 （1.位置信息使用绝对像素点值（赛事客户端使用的分辨率为1920×1080，屏幕左上角为（0，0）） 2.鼠标按键状态1 为按下，其他值为未按 下，仅在出现鼠标图标后响应该信息，若 无新的鼠标信息，选手端将保持上一帧数 据的鼠标信息，当鼠标图标消失后该数据 不再保持）
    float target_position_y;        //!< bit 0-11：鼠标Y 轴像素位置，bit 12-15：鼠标右键状态 （1.位置信息使用绝对像素点值（赛事客户端使用的分辨率为1920×1080，屏幕左上角为（0，0）） 2.鼠标按键状态1 为按下，其他值为未按 下，仅在出现鼠标图标后响应该信息，若 无新的鼠标信息，选手端将保持上一帧数 据的鼠标信息，当鼠标图标消失后该数据 不再保持）
    float reserved;                 //!< 保留位
} custom_client_data_t;

/**
 * @name map_sentry_data_t
 * @brief 0x0307 选手端小地图接收哨兵数据，频率上限为1Hz
 */
typedef struct {
    uint8_t intention;              //!< 1：到目标点攻击2：到目标点防守3：移动到目标点
    uint16_t start_position_x;      //!< 路径起点X 轴坐标，单位：dm （小地图左下角为坐标原点，水平向右为X轴正方向，竖直向上为Y轴正方向。显示位置将按照场地尺寸与小地图尺寸等比缩放，超出边界的位置将在边界处显示）
    uint16_t start_position_y;      //!< 路径起点Y 轴坐标，单位：dm （小地图左下角为坐标原点，水平向右为X轴正方向，竖直向上为Y轴正方向。显示位置将按照场地尺寸与小地图尺寸等比缩放，超出边界的位置将在边界处显示）
    int8_t delta_x[49];             //!< 路径点X 轴增量数组，单位：dm （增量相较于上一个点位进行计算，共49 个新点位，X 与Y 轴增量对应组成点位）
    int8_t delta_y[49];             //!< 路径点Y 轴增量数组，单位：dm （增量相较于上一个点位进行计算，共49 个新点位，X 与Y 轴增量对应组成点位）
} map_sentry_data_t;

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
    graphic_type_line = 0, //!< 直线 @details start_angle : 空<br>end_angle : 空<br>width : 线条宽度<br>start_x : 起点x坐标<br>start_y : 起点y坐标<br>radius : 空<br>end_x : 终点x坐标<br>end_y : 终点y坐标
    graphic_type_rectangle = 1, //!< 矩形 @details start_angle : 空<br>end_angle : 空<br>width : 线条宽度<br>start_x : 起点x坐标<br>start_y : 起点y坐标<br>radius : 空<br>end_x : 对角顶点x坐标<br>end_y : 对角顶点y坐标
    graphic_type_circle = 2, //!< 正圆 @details start_angle : 空<br>end_angle : 空<br>width : 线条宽度<br>start_x : 圆心x坐标<br>start_y : 圆心y坐标<br>radius : 半径<br>end_x : 空<br>end_y : 空
    graphic_type_ellipse = 3, //!< 椭圆 @details start_angle : 空<br>end_angle : 空<br>width : 线条宽度<br>start_x : 圆心x坐标<br>start_y : 圆心y坐标<br>radius : 空<br>end_x : x半轴长度<br>end_y : y半轴长度
    graphic_type_arc = 4, //!< 圆弧 @details start_angle : 起始角度<br>end_angle : 终止角度<br>width : 线条宽度<br>start_x : 圆心x坐标<br>start_y : 圆心y坐标<br>radius : 空<br>end_x : x半轴长度<br>end_y : y半轴长度
    graphic_type_float = 5, //!< 浮点数 @details start_angle : 字体大小<br>end_angle : 小数位有效个数<br>width : 线条宽度<br>start_x : 起点x坐标<br>start_y : 起点y坐标<br>radius, end_x, end_y : 32位浮点数，float
    graphic_type_int = 6, //!< 浮点数 @details start_angle : 字体大小<br>end_angle : 空<br>width : 线条宽度<br>start_x : 起点x坐标<br>start_y : 起点y坐标<br>radius, end_x, end_y : 32位整型数，int32_t
    graphic_type_char = 7  //!< 字符 @details start_angle : 字体大小<br>end_angle : 字符长度<br>width : 线条宽度start_x : 起点x坐标<br>start_y : 起点y坐标<br>radius, end_x, end_y : 空
} graphic_type_enum_t;

/**
 * @name graphic_color_enum_t
 * @brief 图形配置 图形颜色定义
 */
typedef enum {
    graphic_color_main = 0, //!< 红蓝主色
    graphic_color_yellow = 1, //!< 黄色
    graphic_color_green = 2, //!< 绿色
    graphic_color_orange = 3, //!< 橙色
    graphic_color_fuchsia = 4, //!< 紫红色
    graphic_color_pink = 5, //!< 粉色
    graphic_color_cyan = 6, //!< 青色
    graphic_color_black = 7, //!< 黑色
    graphic_color_white = 8  //!< 白色
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
    bool ground_robot_position_update;//!< 地面机器人位置数据更新
    bool radar_mark_data_update;      //!< 雷达标记进度数据更新
    bool communication_data_update;   //!< 云台手数据更新
    bool customize_control_data_update;   //!< 自定义控制器交互数据更新
    bool map_data_update;         //!< 小地图数据更新
    bool control_uart_data_update;         //!< 图传串口发送键鼠信息
    bool client_map_data_update;         //!< 客户端小地图坐标数据更新
    bool custom_client_data_update;    //!< 自定义控制器与选手端交互数据更新
    bool map_sentry_data_update;    //!< 选手端小地图接收哨兵数据
    //0x000x
    std_frame_header_t FrameHeader;       //!< 帧头信息
    ext_game_status_t GameStatus;         //!< 0x0001 比赛状态数据
    ext_game_result_t GameResult;         //!< 0x0002 比赛结果数据
    ext_game_robot_HP_t GameRobotHP;      //!< 0x0003 比赛机器人血量数据
    ext_ICRA_buff_debuff_zone_and_lurk_status_t ICRA_Buff_Debuff_zone_status; //!< 0x0005 人工智能挑战赛(ICRA)加成与惩罚区状态
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
    ground_robot_position_t GroundRobotPosition; //!< 0x020B 地面机器人位置数据
    radar_mark_data_t RadarMarkData;  //!< 0x020C 雷达标记进度数据
    //0x030x
    ext_aerial_data_t AerialData; //!< 0x0301 机器人间交互数据
    ext_custom_robot_data_t RobotInteractiveData;//!< 0x0302 自定义控制器交互数据接口数据
    ext_robot_command_t RobotCommand; //!< 0x0303 客户端小地图交互数据
    ext_control_uart_t ControlUart; //!< 0x0304 图传串口发送键盘、鼠标信息
    ext_client_map_command_t ClientMapCommand; //!< 0x0305 客户端小地图接收信息
    custom_client_data_t CustomClientData; //!< 0x0306 自定义控制器与选手端交互数据
    map_sentry_data_t MapSentryData;  //!< 0x0307 选手端小地图接收哨兵数据
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

//TX Declare
#define DRAWING_PACK    15
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

/* 图形配置操作指令，参照官方手册P23 */
typedef enum {
    NULL_OPERATION = 0U,
    ADD_PICTURE = 1U,
    MODIFY_PICTURE = 2U,
    CLEAR_PICTURE = 3U,
} drawOperate_e;

/* 图形清除操作指令 */
typedef enum {
    CLEAR_ONE_LAYER = 1U,
    CLEAR_ALL = 2U,
} clearOperate_e;

/* 图形绘制类型 */
typedef enum {
    LINE = 0U,
    RECTANGLE = 1U,
    CIRCLE = 2U,
    OVAL = 3U,
    ARC = 4U,
    _FLOAT = 5U,
    _INT = 6U,
    _CHAR = 7U,
} graphic_tpye_e;

/* 图形色彩配置 */
typedef enum {
    RED = 0U,
    BLUE = 0U,
    YELLOW,
    GREEN,
    ORANGE,
    PURPLE,
    PINK,
    DARKGREEN,
    BLACK,
    WHITE
} colorType_e;

/* 交互数据帧的通信类型，机器人间通信 or 自定义UI or 小地图通信 */
typedef enum {
    CV_OtherRobot = 0U,
    UI_Client,
    MiniMap_Client
} receive_Type_e;

typedef enum {
    /* 机器人交互数据段的ID类型，注意这里是数据段内容ID！ */
    RobotComData_ID = 0x0233,        //车间交互，该ID由各个队伍自定义 0x0200~0x02FF
    CustomData_ID = 0xD180,        //自定义数据ID
    Drawing_Clean_ID = 0x0100,
    Drawing_1_ID = 0x0101,
    Drawing_2_ID = 0x0102,
    Drawing_5_ID = 0x0103,
    Drawing_7_ID = 0x0104,
    Drawing_Char_ID = 0x0110,
} data_id_e;


#pragma pack(pop)
/***************define for fifo end*******************/
/***************define for referee_tx start*******************/
typedef enum {
    VISION_OFF = 0U,
    VISION_ON = 1U,
} vision_state_e;

typedef enum {
    FOLLOW_GIMBAL = 0U,
    TO_GIMBAL = 1U,
    SPIN = 2U,
} mode_change_e;

typedef enum {
    NOT_USE_CAPACITANCE = 0U,
    USE_CAPACITANCE = 1U,
} capacitance_state_e;

typedef enum {
    SHOOT_HEAT_COOLING_10_MS = 10U,
    SHOOT_HEAT_COOLING_15_MS = 15U,
    SHOOT_HEAT_COOLING_25_MS = 25U,
    SHOOT_HEAT_COOLING_35_MS = 35U,
    SHOOT_HEAT_COOLING_40_MS = 40U,
    SHOOT_HEAT_COOLING_60_MS = 60U,
    SHOOT_HEAT_COOLING_80_MS = 80U,
} shoot_heat_cooling_rate_e;

typedef enum {
    MANUAL_CONTROL_OFF = 0U,
    MANUAL_CONTROL_ON = 1U,
} manual_control_state_e;

typedef enum {
    SHOOT_SPEED_15_MS = 15U,
    SHOOT_SPEED_18_MS = 18U,
    SHOOT_SPEED_30_MS = 30U,
} shoot_speed_e;

typedef enum {
    POWER_LIMIT_POWER_FIRST_60W= 60U,
    POWER_LIMIT_POWER_FIRST_80W = 80U,
    POWER_LIMIT_POWER_FIRST_100W = 100U,
} power_limit_power_first_e;

typedef enum {
    POWER_LIMIT_HP_FIRST_45W = 45U,
    POWER_LIMIT_HP_FIRST_50W = 50U,
    POWER_LIMIT_HP_FIRST_55W = 55U,
} power_limit_hp_first_e;

typedef enum {
    FRIC_OFF = 0u,
    FRIC_ON = 1U,
    FRIC_READY = 2U,
} shoot_state_e;

typedef struct {
    vision_state_e vision_state;
    mode_change_e mode;
    capacitance_state_e capacitance_state;
    manual_control_state_e manual_control_state;
    shoot_heat_cooling_rate_e shoot_heat_cooling_rate;
    shoot_speed_e shoot_speed;
    power_limit_hp_first_e power_limit_hp_first;
    power_limit_power_first_e power_limit_power_first;
} function_control_show_t;

/***************define for referee_tx end*******************/


/***************function and variable declare start*******************/
//RX Declare
extern judge_info_t global_judge_info;

void judge_update(uint8_t *rxBuf);

extern uint8_t get_robot_id(void);

static void referee_unpack_fifo_data(void);

extern void get_chassis_power_and_buffer(float32_t *power, float32_t *buffer);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);

extern void init_referee_struct_data(void);

extern void referee_rx_task(void const *argument);

extern void referee_tx_task(void const *argument);

/**
  * @brief          获取referee_tx_task栈大小
  * @param[in]      none
  * @retval         referee_tx_task_stack:任务堆栈大小
  */
extern uint32_t get_stack_of_referee_tx_task(void);

/**
  * @brief          获取referee_rx_task栈大小
  * @param[in]      none
  * @retval         referee_rx_task_stack:任务堆栈大小
  */
extern uint32_t get_stack_of_referee_rx_task(void);

/**
  * @brief          获取USART6TX_active_task栈大小
  * @param[in]      none
  * @retval         USART6TX_active_task_stack:任务堆栈大小
  */
extern uint32_t get_stack_of_USART6TX_active_task(void);

extern volatile uint8_t Referee_No_DMA_IRQHandler;
extern volatile uint8_t referee_dma_send_data_len;
extern volatile uint8_t Referee_IRQ_Return_Before;
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

extern void MY_USART_DMA_Stream6_TX_IRQHandler(void);


/**
 * @brief 直线绘制数据包
 * @param line_width 线宽
 * @retval
 */
static graphic_data_struct_t *
line_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx, uint16_t starty, uint16_t endx,
             uint16_t endy, uint16_t line_width, colorType_e vcolor, uint8_t name[]);

/***************function and variable declare end*******************/
static unsigned char *out_float(double value, unsigned char decimal_digit, unsigned char *output_length);

#endif