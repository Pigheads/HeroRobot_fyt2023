#ifndef _JUDGE_RX_H
#define _JUDGE_RX_H

#include "stdint.h"

typedef enum
{
    game_status_id = 0x0001,
    game_result_id = 0x0002,
    game_robot_HP_id = 0x0003,
    dart_status_id = 0x0004,
    event_data_id = 0x0101,
    supply_projectile_action_id = 0x0102,
    dart_remaining_time = 0x0105,
    game_robot_status = 0x0201,
    power_heat_data_id = 0x0202,
    game_game_robot_pos_id = 0x0203,
    buff_id = 0x0204,
    air_robot_energy = 0x0205,
    robot_hurt_id = 0x0206,
    shoot_data_id = 0x0207,
    bullet_remaining_id = 0x0208,
    dart_client_cmd_id = 0x020A,
} cmd_id_e;

typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;

typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_t;

typedef __packed struct
{
    uint8_t attack_time;
} air_robot_energy_t;

typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint8_t first_dart_speed;
    uint8_t second_dart_speed;
    uint8_t third_dart_speed;
    uint8_t fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launcch_cmd_time;
} ext_dart_client_cmd_t;

typedef struct
{
    ext_game_status_t ext_game_status;
    ext_game_result_t ext_game_result;
    ext_game_robot_HP_t ext_game_robot_HP;
    ext_dart_status_t ext_dart_status;
    ext_event_data_t ext_event_data;
    ext_supply_projectile_action_t ext_supply_projectile_action;
    ext_dart_remaining_time_t ext_dart_remaining_time;
    ext_game_robot_status_t ext_game_robot_status;
    ext_power_heat_data_t ext_power_heat_data;
    ext_game_robot_pos_t ext_game_robot_pos;
    ext_buff_t ext_buff;
    air_robot_energy_t air_robot_energy;
    ext_robot_hurt_t ext_robot_hurt;
    ext_shoot_data_t ext_shoot_data;
    ext_bullet_remaining_t ext_bullet_remaining;
    ext_dart_client_cmd_t ext_dart_client_cmd_t;
} judge_rxmsg_t;

void judge_rx_handler(uint8_t *p_frame);

/************************裁判系统辅助函数***********************/
float JUDGE_f32GetChassisPower(void);         /*< 读取底盘功率*/
uint16_t JUDGE_u16GetChassisPowerLimit(void); /*< 读取底盘功率上限*/
uint16_t JUDGE_u16GetRemainEnergy(void);      /*< 读取剩余焦耳能量*/
uint8_t JUDGE_u8GetRobotLevel(void);          /*< 读取机器人等级*/
uint16_t JUDGE_u16GetRemoteHeat42(void);      /*< 读取17mm枪口热量*/
float JUDGE_fGetSpeedHeat42(void);            /*< 读取17mm射速*/
uint16_t JUDGE_u16GetSpeedHeat42Limit(void);  /*< 读取17mm射速限制*/
uint16_t JUDGE_u16GetHeatLimit(void);         /*< 读取热量上限*/
uint16_t JUDGE_u16GetShootCold(void);         /*< 读取冷却秒速*/
uint8_t JUDGE_u8GetRobotId(void);             /*< 读取机器人ID*/
uint16_t JUDGE_u16GetHeatRate(void);          /*< 读取枪口热量冷却速率*/
uint16_t JUDGE_u16GetMaxHP(void);             /*< 读取机器人最大血量>*/
uint16_t JUDGE_u16GetRemainHP(void);          /*< 读取机器人当前血量>*/
#endif
