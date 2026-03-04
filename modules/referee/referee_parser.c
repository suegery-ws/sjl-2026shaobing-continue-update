/**
 ******************************************************************************
 * @file    referee_parser.c
 * @brief   JudgeReadData 拷贝后的裁判系统协议解析辅助实现
 *
 * 解析策略：
 * 1) 每个解析函数都保持纯函数风格（只做输入->输出）。
 * 2) 原始值与语义值同时保留，便于调试和业务使用。
 * 3) 位段拆解严格对应 RoboMaster 2026 协议表。
 ******************************************************************************
 */

#include "referee_parser.h"
#include <limits.h>
#include <string.h>

/* 安全字符串拷贝：用于所有可读文本字段，确保不会越界。 */
static void CopyText(char *dst, uint32_t dst_len, const char *src)
{
    uint32_t i = 0u;
    if (dst == NULL || dst_len == 0u)
    {
        return;
    }

    if (src == NULL)
    {
        dst[0] = '\0';
        return;
    }

    while (src[i] != '\0' && (i + 1u) < dst_len)
    {
        dst[i] = src[i];
        ++i;
    }
    dst[i] = '\0';
}

/* 0x0001 比赛类型枚举 -> 可读文本。 */
static const char *GameTypeToString(uint8_t game_type)
{
    switch (game_type)
    {
    case 1:
        return "RoboMaster Super Duel";
    case 2:
        return "RoboMaster College Single";
    case 3:
        return "ICRA AI Challenge";
    case 4:
        return "RoboMaster RMUL 3V3";
    case 5:
        return "RoboMaster RMUL Infantry";
    default:
        return "Unknown";
    }
}

/* 0x0001 比赛阶段枚举 -> 可读文本。 */
static const char *GameProgressToString(uint8_t game_progress)
{
    switch (game_progress)
    {
    case 0:
        return "Not started";
    case 1:
        return "Prepare";
    case 2:
        return "Self-check 15s";
    case 3:
        return "Countdown 5s";
    case 4:
        return "In battle";
    case 5:
        return "Settlement";
    default:
        return "Unknown";
    }
}

/* 0x0002 胜负枚举 -> 可读文本。 */
static const char *WinnerToString(uint8_t winner)
{
    switch (winner)
    {
    case 0:
        return "Draw";
    case 1:
        return "Red wins";
    case 2:
        return "Blue wins";
    default:
        return "Unknown";
    }
}

/* 0x0104 裁判警告等级 -> 可读文本。 */
static const char *RefereeWarningLevelToString(uint8_t level)
{
    switch (level)
    {
    case 1:
        return "双方黄牌";
    case 2:
        return "黄牌";
    case 3:
        return "红牌";
    case 4:
        return "判负";
    default:
        return "未知警告等级";
    }
}

/* 0x0105 飞镖目标编号 -> 可读文本。 */
static const char *DartTargetToString(uint8_t target)
{
    switch (target)
    {
    case 0:
        return "未命中/默认";
    case 1:
        return "前哨站";
    case 2:
        return "基地固定目标";
    case 3:
        return "基地随机固定目标";
    case 4:
        return "基地随机移动目标";
    case 5:
        return "基地末端移动目标";
    default:
        return "未知目标";
    }
}

/* 0x0206 受伤类型枚举 -> 可读文本。 */
static const char *HurtTypeToString(uint8_t hurt_type)
{
    switch (hurt_type)
    {
    case 0:
        return "Projectile hit";
    case 1:
        return "Module offline";
    case 5:
        return "Collision hit";
    default:
        return "Reserved";
    }
}

/* 0x020A 飞镖发射站状态枚举 -> 可读文本。 */
static const char *DartLaunchStateToString(uint8_t state)
{
    switch (state)
    {
    case 0:
        return "Opened";
    case 1:
        return "Closed";
    case 2:
        return "Opening/Closing";
    default:
        return "Unknown";
    }
}

/* 0x020D 哨兵姿态枚举 -> 可读文本。 */
static const char *SentryPostureToString(uint8_t posture)
{
    switch (posture)
    {
    case 1:
        return "Attack";
    case 2:
        return "Defense";
    case 3:
        return "Move";
    default:
        return "Unknown";
    }
}

/* 0x0301 子内容 ID -> 可读类型名。 */
static const char *InteractionCmdToString(uint16_t data_cmd_id)
{
    if (data_cmd_id >= 0x0200u && data_cmd_id <= 0x02FFu)
    {
        return "Custom robot data";
    }

    switch (data_cmd_id)
    {
    case UI_Data_ID_Del:
        return "UI delete layer";
    case UI_Data_ID_Draw1:
        return "UI draw 1 graphic";
    case UI_Data_ID_Draw2:
        return "UI draw 2 graphics";
    case UI_Data_ID_Draw5:
        return "UI draw 5 graphics";
    case UI_Data_ID_Draw7:
        return "UI draw 7 graphics";
    case UI_Data_ID_DrawChar:
        return "UI draw character";
    case Sentinel_Autonomous_decision_making:
        return "Sentry decision";
    case Radar_autonomous_decision_making:
        return "Radar decision";
    default:
        return "Unknown";
    }
}

/* 按 0x0301 子内容 ID 计算理论载荷长度。
 * 说明：0x0200~0x02FF 是自定义区间，长度可变（最大 112），
 * 因此这里返回 112 作为安全上限。
 */
static uint16_t InteractionExpectedLen(uint16_t data_cmd_id)
{
    switch (data_cmd_id)
    {
    case UI_Data_ID_Del:
        return 2u;
    case UI_Data_ID_Draw1:
        return 15u;
    case UI_Data_ID_Draw2:
        return 30u;
    case UI_Data_ID_Draw5:
        return 75u;
    case UI_Data_ID_Draw7:
        return 105u;
    case UI_Data_ID_DrawChar:
        return 45u;
    case Sentinel_Autonomous_decision_making:
        return 4u;
    case Radar_autonomous_decision_making:
        return 1u;
    default:
        if (data_cmd_id >= 0x0200u && data_cmd_id <= 0x02FFu)
        {
            return 112u;
        }
        return 0u;
    }
}

/* 0x0301 图形操作类型 -> 可读文本。 */
static const char *UIGraphOperateToString(uint8_t op)
{
    switch (op)
    {
    case UI_Graph_ADD:
        return "增加";
    case UI_Graph_Change:
        return "修改";
    case UI_Graph_Del:
        return "删除";
    default:
        return "未知操作";
    }
}

/* 0x0301 图形类型 -> 可读文本。 */
static const char *UIGraphTypeToString(uint8_t type)
{
    switch (type)
    {
    case UI_Graph_Line:
        return "直线";
    case UI_Graph_Rectangle:
        return "矩形";
    case UI_Graph_Circle:
        return "正圆";
    case UI_Graph_Ellipse:
        return "椭圆";
    case UI_Graph_Arc:
        return "圆弧";
    case UI_Graph_Float:
        return "浮点数";
    case UI_Graph_Int:
        return "整型数";
    case UI_Graph_Char:
        return "字符";
    default:
        return "未知图形";
    }
}

/* 0x0301 图形颜色 -> 可读文本。 */
static const char *UIGraphColorToString(uint8_t color)
{
    switch (color)
    {
    case UI_Color_Main:
        return "红蓝主色";
    case UI_Color_Yellow:
        return "黄色";
    case UI_Color_Green:
        return "绿色";
    case UI_Color_Orange:
        return "橙色";
    case UI_Color_Purplish_red:
        return "紫红色";
    case UI_Color_Pink:
        return "粉色";
    case UI_Color_Cyan:
        return "青色";
    case UI_Color_Black:
        return "黑色";
    case UI_Color_White:
        return "白色";
    default:
        return "未知颜色";
    }
}

/* 将 interaction_figure_t 转换为易读字段。 */
static void ParseInteractionFigure(const interaction_figure_t *src, InteractionFigureInfo_t *dst)
{
    uint8_t i = 0u;
    if (src == NULL || dst == NULL)
    {
        return;
    }

    memset(dst, 0, sizeof(*dst));
    for (i = 0u; i < 3u; ++i)
    {
        dst->figure_name[i] = (char)src->figure_name[i];
    }
    dst->figure_name[3] = '\0';

    dst->operate_type = (uint8_t)src->operate_tpye;
    dst->figure_type = (uint8_t)src->figure_tpye;
    dst->layer = (uint8_t)src->layer;
    dst->color = (uint8_t)src->color;
    dst->details_a = (uint16_t)src->details_a;
    dst->details_b = (uint16_t)src->details_b;
    dst->width = (uint16_t)src->width;
    dst->start_x = (uint16_t)src->start_x;
    dst->start_y = (uint16_t)src->start_y;
    dst->details_c = (uint16_t)src->details_c;
    dst->details_d = (uint16_t)src->details_d;
    dst->details_e = (uint16_t)src->details_e;

    CopyText(dst->operate_type_str, sizeof(dst->operate_type_str), UIGraphOperateToString(dst->operate_type));
    CopyText(dst->figure_type_str, sizeof(dst->figure_type_str), UIGraphTypeToString(dst->figure_type));
    CopyText(dst->color_str, sizeof(dst->color_str), UIGraphColorToString(dst->color));
}

/* 0x0110 字符子内容解码。 */
static void ParseInteractionDrawChar(const uint8_t *payload, InteractionDrawCharInfo_t *dst)
{
    uint32_t i = 0u;
    interaction_figure_t graphic;
    if (payload == NULL || dst == NULL)
    {
        return;
    }

    memset(dst, 0, sizeof(*dst));
    dst->has_data = 1u;

    memset(&graphic, 0, sizeof(graphic));
    memcpy(&graphic, payload, sizeof(graphic));
    ParseInteractionFigure(&graphic, &dst->graphic);

    memcpy(dst->text_raw, payload + sizeof(graphic), REFEREE_DRAW_CHAR_RAW_LEN);
    for (i = 0u; i < REFEREE_DRAW_CHAR_RAW_LEN; ++i)
    {
        uint8_t ch = dst->text_raw[i];
        if (ch == 0u)
        {
            break;
        }

        if (ch >= 32u && ch <= 126u)
        {
            dst->text_ascii[i] = (char)ch;
        }
        else
        {
            dst->text_ascii[i] = '.';
            dst->text_non_printable_count++;
        }
    }
    dst->text_ascii[i] = '\0';
}

/* 0x0307 意图枚举 -> 可读文本。 */
static const char *MapIntentionToString(uint8_t intention)
{
    switch (intention)
    {
    case 1:
        return "Attack target";
    case 2:
        return "Defend target";
    case 3:
        return "Move to target";
    default:
        return "Unknown";
    }
}

/* 解析 cmd_id 0x0001：比赛状态。 */
void ParseGameState(const ext_game_state_t *game_state, GameStateInfo_t *info)
{
    if (game_state == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->game_type = game_state->game_type;
    info->game_progress = game_state->game_progress;
    info->stage_remain_time = game_state->stage_remain_time;
    info->sync_timestamp = game_state->SyncTimeStamp;
    info->is_game_started = (game_state->game_progress == 4u) ? 1u : 0u;

    CopyText(info->game_type_str, sizeof(info->game_type_str), GameTypeToString(game_state->game_type));
    CopyText(info->game_progress_str, sizeof(info->game_progress_str), GameProgressToString(game_state->game_progress));
}

/* 解析 cmd_id 0x0002：比赛结果。 */
void ParseGameResult(const ext_game_result_t *game_result, GameResultInfo_t *info)
{
    if (game_result == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->winner = game_result->winner;
    info->red_win = (game_result->winner == 1u) ? 1u : 0u;
    info->blue_win = (game_result->winner == 2u) ? 1u : 0u;
    info->draw = (game_result->winner == 0u) ? 1u : 0u;

    CopyText(info->winner_str, sizeof(info->winner_str), WinnerToString(game_result->winner));
}

/* 解析 cmd_id 0x0003：己方血量，并派生总血量和存活数量。 */
void ParseGameRobotHP(const ext_game_robot_HP_t *robot_hp, GameRobotHPInfo_t *info)
{
    uint32_t total_hp = 0u;
    if (robot_hp == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->hero_hp = robot_hp->ally_1_robot_HP;
    info->engineer_hp = robot_hp->ally_2_robot_HP;
    info->infantry3_hp = robot_hp->ally_3_robot_HP;
    info->infantry4_hp = robot_hp->ally_4_robot_HP;
    info->sentry_hp = robot_hp->ally_7_robot_HP;
    info->outpost_hp = robot_hp->ally_outpost_HP;
    info->base_hp = robot_hp->ally_base_HP;

    if (info->hero_hp > 0u)
    {
        ++info->alive_robot_count;
    }
    if (info->engineer_hp > 0u)
    {
        ++info->alive_robot_count;
    }
    if (info->infantry3_hp > 0u)
    {
        ++info->alive_robot_count;
    }
    if (info->infantry4_hp > 0u)
    {
        ++info->alive_robot_count;
    }
    if (info->sentry_hp > 0u)
    {
        ++info->alive_robot_count;
    }

    total_hp = (uint32_t)info->hero_hp + (uint32_t)info->engineer_hp + (uint32_t)info->infantry3_hp +
               (uint32_t)info->infantry4_hp + (uint32_t)info->sentry_hp + (uint32_t)info->outpost_hp +
               (uint32_t)info->base_hp;
    if (total_hp > UINT16_MAX)
    {
        total_hp = UINT16_MAX;
    }
    info->total_hp = (uint16_t)total_hp;
}

/* 解析 cmd_id 0x0101：32 位事件位图。
 * 位段拆解严格按协议表定义。
 */
void ParseEventData(const ext_event_data_t *event_data, EventDataInfo_t *info)
{
    uint32_t raw = 0u;
    if (event_data == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    raw = event_data->event_type;
    info->raw_event_data = raw;
    info->own_supply_area_unshared_occupied = (uint8_t)((raw >> 0) & 0x1u);
    info->own_supply_area_shared_occupied = (uint8_t)((raw >> 1) & 0x1u);
    info->own_supply_area_rmul_occupied = (uint8_t)((raw >> 2) & 0x1u);
    info->own_small_power_rune_state = (uint8_t)((raw >> 3) & 0x3u);
    info->own_large_power_rune_state = (uint8_t)((raw >> 5) & 0x3u);
    info->own_central_highland_state = (uint8_t)((raw >> 7) & 0x3u);
    info->own_trapezoid_highland_state = (uint8_t)((raw >> 9) & 0x3u);
    info->enemy_last_dart_hit_time = (uint16_t)((raw >> 11) & 0x1FFu);
    info->enemy_last_dart_hit_target = (uint8_t)((raw >> 20) & 0x7u);
    info->center_buff_state = (uint8_t)((raw >> 23) & 0x3u);
    info->own_fortress_buff_state = (uint8_t)((raw >> 25) & 0x3u);
    info->own_outpost_buff_state = (uint8_t)((raw >> 27) & 0x3u);
    info->own_base_buff_occupied = (uint8_t)((raw >> 29) & 0x1u);
}

/* 解析 cmd_id 0x0104：裁判警告。 */
void ParseRefereeWarning(const referee_warning_t *warning_data, RefereeWarningInfo_t *info)
{
    if (warning_data == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->level = warning_data->level;
    info->offending_robot_id = warning_data->offending_robot_id;
    info->count = warning_data->count;
    info->is_yellow_card = (warning_data->level == 1u || warning_data->level == 2u) ? 1u : 0u;
    info->is_red_card = (warning_data->level == 3u) ? 1u : 0u;
    info->is_foul_loss = (warning_data->level == 4u) ? 1u : 0u;

    CopyText(info->level_str, sizeof(info->level_str), RefereeWarningLevelToString(warning_data->level));
}

/* 解析 cmd_id 0x0105：飞镖发射相关数据。 */
void ParseDartInfo(const dart_info_t *dart_info, DartInfoParse_t *info)
{
    uint16_t raw = 0u;
    if (dart_info == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    raw = dart_info->dart_info;

    info->dart_remaining_time = dart_info->dart_remaining_time;
    info->raw_dart_info = raw;
    info->last_hit_target = (uint8_t)(raw & 0x7u);
    info->recent_hit_count = (uint8_t)((raw >> 3) & 0x7u);
    info->selected_target = (uint8_t)((raw >> 6) & 0x3u);
    info->reserved_bits = (uint8_t)((raw >> 8) & 0xFFu);

    CopyText(info->last_hit_target_str, sizeof(info->last_hit_target_str), DartTargetToString(info->last_hit_target));
    CopyText(info->selected_target_str, sizeof(info->selected_target_str), DartTargetToString(info->selected_target));
}

/* 解析 cmd_id 0x0201：机器人状态/电源输出。 */
void ParseRobotState(const ext_game_robot_state_t *robot_state, RobotStateInfo_t *info)
{
    if (robot_state == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->robot_id = robot_state->robot_id;
    info->robot_level = robot_state->robot_level;
    info->current_hp = robot_state->current_HP;
    info->maximum_hp = robot_state->maximum_HP;
    info->shooter_cooling_value = robot_state->shooter_barrel_cooling_value;
    info->shooter_heat_limit = robot_state->shooter_barrel_heat_limit;
    info->chassis_power_limit = robot_state->chassis_power_limit;
    info->gimbal_power_output = robot_state->power_management_gimbal_output ? 1u : 0u;
    info->chassis_power_output = robot_state->power_management_chassis_output ? 1u : 0u;
    info->shooter_power_output = robot_state->power_management_shooter_output ? 1u : 0u;
    info->active_power_output_count = info->gimbal_power_output + info->chassis_power_output + info->shooter_power_output;
}

/* 解析 cmd_id 0x0203：机器人位置。 */
void ParseRobotPos(const robot_pos_t *robot_pos, RobotPosInfo_t *info)
{
    if (robot_pos == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));
    info->x = robot_pos->x;
    info->y = robot_pos->y;
    info->angle = robot_pos->angle;
}

/* 解析 cmd_id 0x0204：增益数据与剩余能量阈值标志。 */
void ParseBuff(const buff_t *buff, BuffInfo_t *info)
{
    uint8_t mask = 0u;
    if (buff == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));
    /* bit7 为默认反馈标记（0x80），bit0~6 为阈值位。 */
    mask = (uint8_t)(buff->remaining_energy & 0x7Fu);

    info->recovery_buff_percent = buff->recovery_buff;
    info->cooling_buff_value = buff->cooling_buff;
    info->defence_buff_percent = buff->defence_buff;
    info->vulnerability_buff_percent = buff->vulnerability_buff;
    info->attack_buff_percent = buff->attack_buff;
    info->remaining_energy_raw = buff->remaining_energy;
    info->remaining_energy_mask = mask;
    info->remaining_energy_default_feedback = (buff->remaining_energy == 0x80u) ? 1u : 0u;

    info->remaining_energy_ge_125 = (uint8_t)((mask >> 0) & 0x1u);
    info->remaining_energy_ge_100 = (uint8_t)((mask >> 1) & 0x1u);
    info->remaining_energy_ge_50 = (uint8_t)((mask >> 2) & 0x1u);
    info->remaining_energy_ge_30 = (uint8_t)((mask >> 3) & 0x1u);
    info->remaining_energy_ge_15 = (uint8_t)((mask >> 4) & 0x1u);
    info->remaining_energy_ge_5 = (uint8_t)((mask >> 5) & 0x1u);
    info->remaining_energy_ge_1 = (uint8_t)((mask >> 6) & 0x1u);

    /* 协议说明：raw==0x80 时是默认反馈，
     * 语义上可理解为至少处于 >=50% 档位。
     */
    if (info->remaining_energy_default_feedback != 0u)
    {
        info->remaining_energy_ge_50 = 1u;
        info->remaining_energy_ge_30 = 1u;
        info->remaining_energy_ge_15 = 1u;
        info->remaining_energy_ge_5 = 1u;
        info->remaining_energy_ge_1 = 1u;
    }
}

/* 解析 cmd_id 0x0206：受伤来源和受伤类型。 */
void ParseRobotHurt(const ext_robot_hurt_t *robot_hurt, RobotHurtInfo_t *info)
{
    if (robot_hurt == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->armor_id = robot_hurt->armor_id;
    info->hurt_type = robot_hurt->hurt_type;
    CopyText(info->hurt_type_str, sizeof(info->hurt_type_str), HurtTypeToString(robot_hurt->hurt_type));
}

/* 解析 cmd_id 0x0207：射击数据并解码弹种标志。 */
void ParseShootData(const ext_shoot_data_t *shoot_data, ShootDataInfo_t *info)
{
    if (shoot_data == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->bullet_type_raw = shoot_data->bullet_type;
    info->shooter_id = shoot_data->shooter_id;
    info->bullet_frequency_hz = shoot_data->bullet_freq;
    info->bullet_speed_mps = shoot_data->bullet_speed;
    info->is_17mm = ((shoot_data->bullet_type & 0x01u) != 0u || shoot_data->bullet_type == 1u) ? 1u : 0u;
    info->is_42mm = ((shoot_data->bullet_type & 0x02u) != 0u || shoot_data->bullet_type == 2u) ? 1u : 0u;
}

/* 解析 cmd_id 0x0208：允许发弹量。 */
void ParseProjectileAllowance(const projectile_allowance_t *allowance, ProjectileAllowanceInfo_t *info)
{
    if (allowance == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->allowance_17mm = allowance->projectile_allowance_17mm;
    info->allowance_42mm = allowance->projectile_allowance_42mm;
    info->remaining_gold_coin = allowance->remaining_gold_coin;
    info->fortress_allowance_17mm = allowance->projectile_allowance_fortress;
}

/* 解析 cmd_id 0x0209：34 个 RFID 点位。
 * - 低 32 位来自 rfid_status
 * - 高 2 位来自 rfid_status_2[1:0]
 */
void ParseRFIDStatus(const rfid_status_t *rfid_status, RFIDStatusInfo_t *info)
{
    uint32_t i = 0u;
    if (rfid_status == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->rfid_status_low = rfid_status->rfid_status;
    info->rfid_status_high = rfid_status->rfid_status_2;

    for (i = 0u; i < 32u; ++i)
    {
        info->point_active[i] = (uint8_t)((rfid_status->rfid_status >> i) & 0x1u);
        info->active_count += info->point_active[i];
    }
    info->point_active[32] = (uint8_t)(rfid_status->rfid_status_2 & 0x1u);
    info->point_active[33] = (uint8_t)((rfid_status->rfid_status_2 >> 1) & 0x1u);
    info->active_count += info->point_active[32];
    info->active_count += info->point_active[33];
}

/* 解析 cmd_id 0x020A：飞镖发射站状态。 */
void ParseDartLaunchingState(const dart_client_cmd_t *dart_state, DartLaunchingStateInfo_t *info)
{
    if (dart_state == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->launch_opening_status = dart_state->dart_launch_opening_status;
    info->target_change_time = dart_state->target_change_time;
    info->latest_launch_cmd_time = dart_state->latest_launch_cmd_time;
    CopyText(info->launch_state_str, sizeof(info->launch_state_str), DartLaunchStateToString(dart_state->dart_launch_opening_status));
}

/* 解析 cmd_id 0x020B：己方地面机器人坐标。 */
void ParseGroundRobotPosition(const ground_robot_position_t *position, GroundRobotPositionInfo_t *info)
{
    if (position == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->hero_x = position->hero_x;
    info->hero_y = position->hero_y;
    info->engineer_x = position->engineer_x;
    info->engineer_y = position->engineer_y;
    info->infantry3_x = position->standard_3_x;
    info->infantry3_y = position->standard_3_y;
    info->infantry4_x = position->standard_4_x;
    info->infantry4_y = position->standard_4_y;
}

/* 解析 cmd_id 0x020C：雷达标记位图。 */
void ParseRadarMarkData(const radar_mark_data_t *mark_data, RadarMarkInfo_t *info)
{
    uint16_t raw = 0u;
    if (mark_data == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    raw = mark_data->mark_progress;
    info->raw_mark_progress = raw;
    info->enemy_hero_marked = (uint8_t)((raw >> 0) & 0x1u);
    info->enemy_engineer_marked = (uint8_t)((raw >> 1) & 0x1u);
    info->enemy_infantry3_marked = (uint8_t)((raw >> 2) & 0x1u);
    info->enemy_infantry4_marked = (uint8_t)((raw >> 3) & 0x1u);
    info->enemy_sentry_marked = (uint8_t)((raw >> 4) & 0x1u);
    info->ally_hero_special_marked = (uint8_t)((raw >> 5) & 0x1u);
    info->ally_engineer_special_marked = (uint8_t)((raw >> 6) & 0x1u);
    info->ally_infantry3_special_marked = (uint8_t)((raw >> 7) & 0x1u);
    info->ally_infantry4_special_marked = (uint8_t)((raw >> 8) & 0x1u);
    info->ally_sentry_special_marked = (uint8_t)((raw >> 9) & 0x1u);
}

/* 解析 cmd_id 0x020D：哨兵信息位段。 */
void ParseSentryInfo(const sentry_info_t *sentry_info, SentryInfoParse_t *info)
{
    uint16_t posture = 0u;
    if (sentry_info == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->raw_sentry_info = sentry_info->sentry_info;
    info->raw_sentry_info_2 = sentry_info->sentry_info_2;

    /* sentry_info 位段：
     * [0:10]  已兑换允许发弹量
     * [11:14] 远程兑换发弹量次数
     * [15:18] 远程兑换血量次数
     * [19]    是否可确认免费复活
     * [20]    是否可兑换立即复活
     * [21:30] 立即复活所需金币
     */
    info->exchanged_projectile_allowance = (uint16_t)(sentry_info->sentry_info & 0x7FFu);
    info->remote_exchange_projectile_count = (uint8_t)((sentry_info->sentry_info >> 11) & 0xFu);
    info->remote_exchange_hp_count = (uint8_t)((sentry_info->sentry_info >> 15) & 0xFu);
    info->can_confirm_free_revive = (uint8_t)((sentry_info->sentry_info >> 19) & 0x1u);
    info->can_exchange_instant_revive = (uint8_t)((sentry_info->sentry_info >> 20) & 0x1u);
    info->instant_revive_gold_cost = (uint16_t)((sentry_info->sentry_info >> 21) & 0x3FFu);

    /* sentry_info_2 位段：
     * [12:13] 哨兵姿态
     * [14]    己方能量机关是否可激活
     */
    posture = (uint16_t)((sentry_info->sentry_info_2 >> 12) & 0x3u);
    info->sentry_posture = (uint8_t)posture;
    info->own_energy_rune_can_activate = (uint8_t)((sentry_info->sentry_info_2 >> 14) & 0x1u);
    CopyText(info->sentry_posture_str, sizeof(info->sentry_posture_str), SentryPostureToString(info->sentry_posture));
}

/* 解析 cmd_id 0x020E：雷达状态位图。 */
void ParseRadarInfo(const radar_info_t *radar_info, RadarInfoParse_t *info)
{
    if (radar_info == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->raw_radar_info = radar_info->radar_info;
    info->double_vulnerability_chance = (uint8_t)(radar_info->radar_info & 0x3u);
    info->enemy_double_vulnerability_active = (uint8_t)((radar_info->radar_info >> 2) & 0x1u);
    info->own_encryption_level = (uint8_t)((radar_info->radar_info >> 3) & 0x3u);
    info->can_change_key = (uint8_t)((radar_info->radar_info >> 5) & 0x1u);
}

/* 解析 cmd_id 0x0301：交互通用头 + 部分子内容载荷。 */
void ParseRobotInteraction(const robot_interaction_data_t *interaction, RobotInteractionInfo_t *info)
{
    uint8_t i = 0u;
    if (interaction == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->data_cmd_id = interaction->data_cmd_id;
    info->sender_id = interaction->sender_id;
    info->receiver_id = interaction->receiver_id;
    info->expected_len = InteractionExpectedLen(interaction->data_cmd_id);
    /* 在缺少帧上下文时这里只能给理论值；
     * ParseRefereeInfo() 中会用真实帧长覆盖该值。
     */
    info->payload_len = (info->expected_len > 0u) ? info->expected_len : 112u;
    info->is_custom_data = (interaction->data_cmd_id >= 0x0200u && interaction->data_cmd_id <= 0x02FFu) ? 1u : 0u;
    info->is_ui_data = (interaction->data_cmd_id >= 0x0100u && interaction->data_cmd_id <= 0x0110u) ? 1u : 0u;
    info->has_sentry_cmd = (interaction->data_cmd_id == Sentinel_Autonomous_decision_making) ? 1u : 0u;
    info->has_radar_cmd = (interaction->data_cmd_id == Radar_autonomous_decision_making) ? 1u : 0u;
    CopyText(info->data_cmd_type_str, sizeof(info->data_cmd_type_str), InteractionCmdToString(interaction->data_cmd_id));
    memcpy(info->raw_payload, interaction->user_data, sizeof(info->raw_payload));

    switch (interaction->data_cmd_id)
    {
    case UI_Data_ID_Del:
    {
        info->has_delete_cmd = 1u;
        memcpy(&info->delete_cmd, interaction->user_data, sizeof(info->delete_cmd));
        info->delete_is_noop = (info->delete_cmd.delete_type == UI_Data_Del_NoOperate) ? 1u : 0u;
        info->delete_is_layer = (info->delete_cmd.delete_type == UI_Data_Del_Layer) ? 1u : 0u;
        info->delete_is_all = (info->delete_cmd.delete_type == UI_Data_Del_ALL) ? 1u : 0u;
        break;
    }
    case UI_Data_ID_Draw1:
    {
        interaction_figure_t figure;
        memset(&figure, 0, sizeof(figure));
        memcpy(&figure, interaction->user_data, sizeof(figure));
        info->draw1_valid = 1u;
        ParseInteractionFigure(&figure, &info->draw1);
        break;
    }
    case UI_Data_ID_Draw2:
    {
        interaction_figure_t figure;
        info->draw2_count = 2u;
        for (i = 0u; i < info->draw2_count; ++i)
        {
            memset(&figure, 0, sizeof(figure));
            memcpy(&figure, interaction->user_data + i * sizeof(interaction_figure_t), sizeof(figure));
            ParseInteractionFigure(&figure, &info->draw2[i]);
        }
        break;
    }
    case UI_Data_ID_Draw5:
    {
        interaction_figure_t figure;
        info->draw5_count = 5u;
        for (i = 0u; i < info->draw5_count; ++i)
        {
            memset(&figure, 0, sizeof(figure));
            memcpy(&figure, interaction->user_data + i * sizeof(interaction_figure_t), sizeof(figure));
            ParseInteractionFigure(&figure, &info->draw5[i]);
        }
        break;
    }
    case UI_Data_ID_Draw7:
    {
        interaction_figure_t figure;
        info->draw7_count = 7u;
        for (i = 0u; i < info->draw7_count; ++i)
        {
            memset(&figure, 0, sizeof(figure));
            memcpy(&figure, interaction->user_data + i * sizeof(interaction_figure_t), sizeof(figure));
            ParseInteractionFigure(&figure, &info->draw7[i]);
        }
        break;
    }
    case UI_Data_ID_DrawChar:
        ParseInteractionDrawChar(interaction->user_data, &info->draw_char);
        break;
    case Sentinel_Autonomous_decision_making:
        memcpy(&info->sentry_cmd, interaction->user_data, sizeof(info->sentry_cmd));
        break;
    case Radar_autonomous_decision_making:
        info->radar_cmd = interaction->user_data[0];
        break;
    default:
        if (info->is_custom_data != 0u)
        {
            info->custom_data_len = info->payload_len;
            if (info->custom_data_len > REFEREE_INTERACTION_PAYLOAD_MAX)
            {
                info->custom_data_len = REFEREE_INTERACTION_PAYLOAD_MAX;
            }
            memcpy(info->custom_data, interaction->user_data, info->custom_data_len);
        }
        break;
    }
}

/* 解析 cmd_id 0x0303：小地图指令。 */
void ParseMapCommand(const map_command_t *map_cmd, MapCommandInfo_t *info)
{
    if (map_cmd == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->target_position_x = map_cmd->target_position_x;
    info->target_position_y = map_cmd->target_position_y;
    info->cmd_keyboard = map_cmd->cmd_keyboard;
    info->target_robot_id = map_cmd->target_robot_id;
    info->cmd_source = map_cmd->cmd_source;
    info->is_target_robot_mode = (map_cmd->target_robot_id != 0u) ? 1u : 0u;
}

/* 解析 cmd_id 0x0305：小地图机器人坐标（cm -> m）。 */
void ParseMapRobotData(const map_robot_data_t *map_data, MapRobotDataInfo_t *info)
{
    if (map_data == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->hero_x_m = (float)map_data->hero_position_x / 100.0f;
    info->hero_y_m = (float)map_data->hero_position_y / 100.0f;
    info->engineer_x_m = (float)map_data->engineer_position_x / 100.0f;
    info->engineer_y_m = (float)map_data->engineer_position_y / 100.0f;
    info->infantry3_x_m = (float)map_data->infantry_3_position_x / 100.0f;
    info->infantry3_y_m = (float)map_data->infantry_3_position_y / 100.0f;
    info->infantry4_x_m = (float)map_data->infantry_4_position_x / 100.0f;
    info->infantry4_y_m = (float)map_data->infantry_4_position_y / 100.0f;
    info->infantry5_x_m = (float)map_data->infantry_5_position_x / 100.0f;
    info->infantry5_y_m = (float)map_data->infantry_5_position_y / 100.0f;
    info->sentry_x_m = (float)map_data->sentry_position_x / 100.0f;
    info->sentry_y_m = (float)map_data->sentry_position_y / 100.0f;
}

/* 解析 cmd_id 0x0307：路径数据，并通过累加增量得到终点。 */
void ParseMapPathData(const map_data_t *path_data, MapPathInfo_t *info)
{
    int32_t x = 0;
    int32_t y = 0;
    uint32_t i = 0u;
    if (path_data == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->intention = path_data->intention;
    info->start_x_dm = path_data->start_position_x;
    info->start_y_dm = path_data->start_position_y;
    info->sender_id = path_data->sender_id;
    CopyText(info->intention_str, sizeof(info->intention_str), MapIntentionToString(path_data->intention));

    x = (int32_t)path_data->start_position_x;
    y = (int32_t)path_data->start_position_y;
    for (i = 0u; i < 49u; ++i)
    {
        x += path_data->delta_x[i];
        y += path_data->delta_y[i];
    }

    if (x > INT16_MAX)
    {
        x = INT16_MAX;
    }
    if (x < INT16_MIN)
    {
        x = INT16_MIN;
    }
    if (y > INT16_MAX)
    {
        y = INT16_MAX;
    }
    if (y < INT16_MIN)
    {
        y = INT16_MIN;
    }

    info->end_x_dm = (int16_t)x;
    info->end_y_dm = (int16_t)y;
}

/* 解析 cmd_id 0x0308：客户端自定义消息。
 * 协议中 user_data 为 UTF-16(LE)；这里保留码元，并生成 ASCII 预览，
 * 方便快速日志查看。
 */
void ParseCustomInfo(const custom_info_t *custom_info, CustomInfoParse_t *info)
{
    uint32_t i = 0u;
    uint32_t ascii_index = 0u;
    if (custom_info == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->sender_id = custom_info->sender_id;
    info->receiver_id = custom_info->receiver_id;

    for (i = 0u; i < REFEREE_CUSTOM_INFO_UTF16_LEN; ++i)
    {
        uint16_t code_unit = (uint16_t)custom_info->user_data[i * 2u] |
                             ((uint16_t)custom_info->user_data[i * 2u + 1u] << 8);
        info->utf16_data[i] = code_unit;

        if (code_unit == 0u)
        {
            break;
        }

        if (ascii_index < REFEREE_CUSTOM_INFO_UTF16_LEN)
        {
            if (code_unit < 128u)
            {
                info->ascii_preview[ascii_index++] = (char)code_unit;
            }
            else
            {
                info->ascii_preview[ascii_index++] = '?';
                info->contains_non_ascii = 1u;
            }
        }
    }
    info->ascii_preview[ascii_index] = '\0';
}

/* 解析 cmd_id 0x0304：键鼠数据并拆分键盘 bitmask。 */
void ParseRemoteControl(const remote_control_t *remote, RemoteControlInfo_t *info)
{
    if (remote == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->mouse_x = remote->mouse_x;
    info->mouse_y = remote->mouse_y;
    info->mouse_z = remote->mouse_z;
    info->left_button_down = remote->left_button_down ? 1u : 0u;
    info->right_button_down = remote->right_button_down ? 1u : 0u;
    info->keyboard_value = remote->keyboard_value;

    info->key_w = (uint8_t)((remote->keyboard_value >> 0) & 0x1u);
    info->key_s = (uint8_t)((remote->keyboard_value >> 1) & 0x1u);
    info->key_a = (uint8_t)((remote->keyboard_value >> 2) & 0x1u);
    info->key_d = (uint8_t)((remote->keyboard_value >> 3) & 0x1u);
    info->key_shift = (uint8_t)((remote->keyboard_value >> 4) & 0x1u);
    info->key_ctrl = (uint8_t)((remote->keyboard_value >> 5) & 0x1u);
    info->key_q = (uint8_t)((remote->keyboard_value >> 6) & 0x1u);
    info->key_e = (uint8_t)((remote->keyboard_value >> 7) & 0x1u);
    info->key_r = (uint8_t)((remote->keyboard_value >> 8) & 0x1u);
    info->key_f = (uint8_t)((remote->keyboard_value >> 9) & 0x1u);
    info->key_g = (uint8_t)((remote->keyboard_value >> 10) & 0x1u);
    info->key_z = (uint8_t)((remote->keyboard_value >> 11) & 0x1u);
    info->key_x = (uint8_t)((remote->keyboard_value >> 12) & 0x1u);
    info->key_c = (uint8_t)((remote->keyboard_value >> 13) & 0x1u);
    info->key_v = (uint8_t)((remote->keyboard_value >> 14) & 0x1u);
    info->key_b = (uint8_t)((remote->keyboard_value >> 15) & 0x1u);
}

/* 聚合解析入口：
 * 解析 referee_info_t 的全部数据块，并保留最近命令字信息。
 */
void ParseRefereeInfo(const referee_info_t *referee, RefereeParsedInfo_t *info)
{
    if (referee == NULL || info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));

    info->cmd_id = referee->CmdID;
    info->frame_data_length = referee->FrameHeader.DataLength;

    ParseGameState(&referee->GameState, &info->game_state);
    ParseGameResult(&referee->GameResult, &info->game_result);
    ParseGameRobotHP(&referee->GameRobotHP, &info->game_robot_hp);
    ParseEventData(&referee->EventData, &info->event_data);
    ParseRefereeWarning(&referee->referee_warning, &info->referee_warning);
    ParseDartInfo(&referee->dart_info, &info->dart_info);
    ParseRobotState(&referee->GameRobotState, &info->robot_state);
    ParseRobotPos(&referee->robot_pos, &info->robot_pos);
    ParseBuff(&referee->buff, &info->buff);
    ParseRobotHurt(&referee->ext_robot_hurt, &info->robot_hurt);
    ParseShootData(&referee->ext_shoot_data, &info->shoot_data);
    ParseProjectileAllowance(&referee->projectile_allowance, &info->projectile_allowance);
    ParseRFIDStatus(&referee->rfid_status, &info->rfid_status);
    ParseDartLaunchingState(&referee->dart_client_cmd, &info->dart_launching_state);
    ParseGroundRobotPosition(&referee->ground_robot_position, &info->ground_robot_position);
    ParseRadarMarkData(&referee->radar_mark_data, &info->radar_mark);
    ParseSentryInfo(&referee->sentry_info, &info->sentry_info);
    ParseRadarInfo(&referee->radar_info, &info->radar_info);
    ParseRobotInteraction(&referee->robot_interaction_data, &info->robot_interaction);
    ParseMapCommand(&referee->map_command, &info->map_command);
    ParseMapRobotData(&referee->map_robot_data, &info->map_robot_data);
    ParseMapPathData(&referee->map_data, &info->map_path);
    ParseCustomInfo(&referee->custom_info, &info->custom_info);
    ParseRemoteControl(&referee->remote_control, &info->remote_control);

    /* 对于 0x0301，可从帧长计算真实 payload 长度：
     * DataLength = 6 字节交互头 + payload。
     */
    if (referee->CmdID == ID_ROBOT_INTERACTION &&
        referee->FrameHeader.DataLength >= Interactive_Data_LEN_Head)
    {
        uint16_t payload_len = (uint16_t)(referee->FrameHeader.DataLength - Interactive_Data_LEN_Head);
        if (payload_len > sizeof(referee->robot_interaction_data.user_data))
        {
            payload_len = sizeof(referee->robot_interaction_data.user_data);
        }
        info->robot_interaction.payload_len = payload_len;

        if (info->robot_interaction.is_custom_data != 0u)
        {
            info->robot_interaction.custom_data_len = payload_len;
            memcpy(info->robot_interaction.custom_data, referee->robot_interaction_data.user_data, payload_len);
        }
    }
}
