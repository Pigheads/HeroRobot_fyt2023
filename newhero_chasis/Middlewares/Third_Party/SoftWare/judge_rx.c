#include "judge_rx.h"
#include "dma_unpack.h"
#include "string.h"

judge_rxmsg_t judge_rxmsg;

void judge_rx_handler(uint8_t *p_frame)
{
    frame_header_t *p_header = (frame_header_t *)p_frame;

    uint16_t data_length = p_header->data_len;
    uint16_t cmd_id = *(uint16_t *)(p_frame + FRAME_HEADER_LEN);
    uint8_t *data_addr = p_frame + FRAME_HEADER_LEN + CMD_ID_LEN; /* 数据帧头地址 */

    memcpy(p_header, p_frame, FRAME_HEADER_LEN);

    switch (cmd_id)
    {
    case game_status_id:
        memcpy(&judge_rxmsg.ext_game_status, data_addr, data_length);
        break;

    case game_result_id:
        memcpy(&judge_rxmsg.ext_game_result, data_addr, data_length);
        break;

    case game_robot_HP_id:
        memcpy(&judge_rxmsg.ext_game_robot_HP, data_addr, data_length);
        break;

    case dart_status_id:
        memcpy(&judge_rxmsg.ext_dart_status, data_addr, data_length);
        break;

    case event_data_id:
        memcpy(&judge_rxmsg.ext_event_data, data_addr, data_length);
        break;

    case supply_projectile_action_id:
        memcpy(&judge_rxmsg.ext_supply_projectile_action, data_addr, data_length);
        break;

    case dart_remaining_time:
        memcpy(&judge_rxmsg.ext_dart_remaining_time, data_addr, data_length);
        break;

    case game_robot_status:
        memcpy(&judge_rxmsg.ext_game_robot_status, data_addr, data_length);
        break;

    case power_heat_data_id:
        memcpy(&judge_rxmsg.ext_power_heat_data, data_addr, data_length);
        break;

    case game_game_robot_pos_id:
        memcpy(&judge_rxmsg.ext_game_robot_pos, data_addr, data_length);
        break;

    case buff_id:
        memcpy(&judge_rxmsg.ext_buff, data_addr, data_length);
        break;

    case air_robot_energy:
        memcpy(&judge_rxmsg.air_robot_energy, data_addr, data_length);
        break;

    case robot_hurt_id:
        memcpy(&judge_rxmsg.ext_robot_hurt, data_addr, data_length);
        break;

    case shoot_data_id:
        memcpy(&judge_rxmsg.ext_shoot_data, data_addr, data_length);
        break;

    case bullet_remaining_id:
        memcpy(&judge_rxmsg.ext_bullet_remaining, data_addr, data_length);
        break;

    case dart_client_cmd_id:
        memcpy(&judge_rxmsg.ext_dart_client_cmd_t, data_addr, data_length);
        break;

    default:
        break;
    }
}

/********************裁判数据辅助判断函数***************************/

/**
  * @brief  读取瞬时功率
  * @param  void
  * @retval 实时功率值
  * @attention  
  */
float JUDGE_f32GetChassisPower(void)
{
    return (judge_rxmsg.ext_power_heat_data.chassis_power);
}

/**
  * @brief  读取功率限制
  * @param  void
  * @retval 功率限制
  * @attention  
  */
uint16_t JUDGE_u16GetChassisPowerLimit(void)
{
    return (judge_rxmsg.ext_game_robot_status.chassis_power_limit);
}

/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量(最大60)
  * @attention  
  */
uint16_t JUDGE_u16GetRemainEnergy(void)
{
    return (judge_rxmsg.ext_power_heat_data.chassis_power_buffer);
}

/**
  * @brief  读取当前等级
  * @param  void
  * @retval 当前等级
  * @attention  
  */
uint8_t JUDGE_u8GetRobotLevel(void)
{
    return (judge_rxmsg.ext_game_robot_status.robot_level);
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 42mm枪口热量
  * @attention  实时热量
  */
uint16_t JUDGE_u16GetRemoteHeat42(void)
{
    return (judge_rxmsg.ext_power_heat_data.shooter_id1_42mm_cooling_heat);
}

/**
  * @brief  读取射速
  * @param  void
  * @retval 42mm射速
  * @attention  实时射速
  */
float JUDGE_fGetSpeedHeat42(void)
{
    return judge_rxmsg.ext_shoot_data.bullet_speed;
}

/**
  * @brief  读取射速限制
  * @param  void
  * @retval 42mm射速限制
  * @attention 
  */
uint16_t JUDGE_u16GetSpeedHeat42Limit(void)
{
    return judge_rxmsg.ext_game_robot_status.shooter_id1_42mm_speed_limit;
}

/**
  * @brief  读取枪口热量上限
  * @param  void
  * @retval 当前等级42mm热量上限
  * @attention  
  */
uint16_t JUDGE_u16GetHeatLimit(void)
{
    return (judge_rxmsg.ext_game_robot_status.shooter_id1_42mm_cooling_limit);
}

/**
  * @brief  读取枪口热量冷却速率
  * @param  void
  * @retval 当前等级42mm热量冷却速率
  * @attention  
  */
uint16_t JUDGE_u16GetHeatRate(void)
{
    return (judge_rxmsg.ext_game_robot_status.shooter_id1_42mm_cooling_rate);
}
/**
  * @brief  获取机器人最大血量
  * @param  void
  * @retval 机器人最大血量
  * @attention 
  */
uint16_t JUDGE_u16GetMaxHP(void)
{
	   return (judge_rxmsg.ext_game_robot_status.max_HP);
}
/**
  * @brief  获取机器人当前血量
  * @param  void
  * @retval 机器人当前血量
  * @attention 
  */
uint16_t JUDGE_u16GetRemainHP(void)
{
	   return (judge_rxmsg.ext_game_robot_status.remain_HP);
}
/**
  * @brief  获取机器人ID
  * @param  void
  * @retval 机器人ID
  * @attention 
  */
uint8_t JUDGE_u8GetRobotId(void)
{
    return (judge_rxmsg.ext_game_robot_status.robot_id);
}
