/**
 ******************************************************************************
 * @file    referee_parser.h
 * @brief   针对 JudgeReadData 接收数据的裁判系统解析辅助
 *
 * 本文件用于将协议原始结构体（由 rm_referee.c::JudgeReadData 拷贝）
 * 转换为更适合上层行为逻辑/UI 使用的语义化字段。
 *
 * 设计原则：
 * 1) 保留原始字段，便于回溯和调试。
 * 2) 同时提供解码字段，便于业务逻辑直接使用。
 * 3) 解析器保持无副作用，不修改全局状态。
 ******************************************************************************
 */

#ifndef REFEREE_PARSER_H
#define REFEREE_PARSER_H

#include "stdint.h"
#include "referee_protocol.h"
#include "rm_referee.h"

#define REFEREE_RFID_POINT_COUNT 34u      /* 0x0209 一共 34 个 RFID 状态位（32 + 2）。 */
#define REFEREE_CUSTOM_INFO_UTF16_LEN 15u /* 0x0308 的 30 字节 UTF-16 数据，共 15 个码元。 */
#define REFEREE_INTERACTION_PAYLOAD_MAX 112u /* 0x0301 子内容最大载荷。 */
#define REFEREE_DRAW_CHAR_RAW_LEN 30u        /* 0x0110 字符内容固定 30 字节。 */

/* cmd_id 0x0001（比赛状态）解析结果。 */
typedef struct
{
    /* 来自 ext_game_state_t 的原始字段。 */
    uint8_t game_type;
    uint8_t game_progress;
    uint16_t stage_remain_time;
    uint64_t sync_timestamp;

    /* 基于原始值的可读字段。 */
    char game_type_str[64];
    char game_progress_str[32];
    uint8_t is_game_started;
} GameStateInfo_t;

/* cmd_id 0x0002（比赛结果）解析结果。 */
typedef struct
{
    /* 原始 winner 值：0 平局，1 红方，2 蓝方。 */
    uint8_t winner;

    /* winner 的语义化映射。 */
    char winner_str[16];
    uint8_t red_win;
    uint8_t blue_win;
    uint8_t draw;
} GameResultInfo_t;

/* cmd_id 0x0003（己方血量汇总）解析结果。 */
typedef struct
{
    /* 己方机器人/建筑血量快照。 */
    uint16_t hero_hp;
    uint16_t engineer_hp;
    uint16_t infantry3_hp;
    uint16_t infantry4_hp;
    uint16_t sentry_hp;
    uint16_t outpost_hp;
    uint16_t base_hp;

    /* 供调度/策略逻辑直接使用的聚合字段。 */
    uint16_t total_hp;
    uint8_t alive_robot_count;
} GameRobotHPInfo_t;

/* cmd_id 0x0101（event_data 位图）解析结果。 */
typedef struct
{
    /* 接收的完整 32 位事件图。 */
    uint32_t raw_event_data;

    /* 按协议表拆出的位段语义。 */
    uint8_t own_supply_area_unshared_occupied;
    uint8_t own_supply_area_shared_occupied;
    uint8_t own_supply_area_rmul_occupied;
    uint8_t own_small_power_rune_state;
    uint8_t own_large_power_rune_state;
    uint8_t own_central_highland_state;
    uint8_t own_trapezoid_highland_state;
    uint16_t enemy_last_dart_hit_time;
    uint8_t enemy_last_dart_hit_target;
    uint8_t center_buff_state;
    uint8_t own_fortress_buff_state;
    uint8_t own_outpost_buff_state;
    uint8_t own_base_buff_occupied;
} EventDataInfo_t;

/* cmd_id 0x0104（裁判警告）解析结果。 */
typedef struct
{
    /* 原始字段。 */
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;

    /* 语义化字段。 */
    char level_str[24];
    uint8_t is_yellow_card;
    uint8_t is_red_card;
    uint8_t is_foul_loss;
} RefereeWarningInfo_t;

/* cmd_id 0x0105（飞镖发射相关）解析结果。 */
typedef struct
{
    /* 原始字段。 */
    uint8_t dart_remaining_time;
    uint16_t raw_dart_info;

    /* 位段解码（按协议中的 dart_info 位定义）。 */
    uint8_t last_hit_target;   /* bit0-2 */
    uint8_t recent_hit_count;  /* bit3-5 */
    uint8_t selected_target;   /* bit6-7 */
    uint8_t reserved_bits;     /* bit8-15 */

    /* 可读文本。 */
    char last_hit_target_str[32];
    char selected_target_str[32];
} DartInfoParse_t;

/* cmd_id 0x0201（机器人状态）解析结果。 */
typedef struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_cooling_value;
    uint16_t shooter_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t gimbal_power_output;
    uint8_t chassis_power_output;
    uint8_t shooter_power_output;
    uint8_t active_power_output_count;
} RobotStateInfo_t;

/* cmd_id 0x0203（机器人位置）解析结果。 */
typedef struct
{
    float x;
    float y;
    float angle;
} RobotPosInfo_t;

/* cmd_id 0x0204（增益 + 剩余能量掩码）解析结果。 */
typedef struct
{
    /* 协议中的增益数值。 */
    uint8_t recovery_buff_percent;
    uint16_t cooling_buff_value;
    uint8_t defence_buff_percent;
    uint8_t vulnerability_buff_percent;
    uint16_t attack_buff_percent;

    /* 剩余能量原始字节及阈值位解码。 */
    uint8_t remaining_energy_raw;
    uint8_t remaining_energy_mask;
    uint8_t remaining_energy_default_feedback;
    uint8_t remaining_energy_ge_125;
    uint8_t remaining_energy_ge_100;
    uint8_t remaining_energy_ge_50;
    uint8_t remaining_energy_ge_30;
    uint8_t remaining_energy_ge_15;
    uint8_t remaining_energy_ge_5;
    uint8_t remaining_energy_ge_1;
} BuffInfo_t;

/* cmd_id 0x0206（受伤事件）解析结果。 */
typedef struct
{
    uint8_t armor_id;
    uint8_t hurt_type;
    char hurt_type_str[32];
} RobotHurtInfo_t;

/* cmd_id 0x0207（射击事件）解析结果。 */
typedef struct
{
    /* 原始射击载荷。 */
    uint8_t bullet_type_raw;
    uint8_t shooter_id;
    uint8_t bullet_frequency_hz;
    float bullet_speed_mps;

    /* 弹种解码辅助字段。 */
    uint8_t is_17mm;
    uint8_t is_42mm;
} ShootDataInfo_t;

/* cmd_id 0x0208（允许发弹量）解析结果。 */
typedef struct
{
    uint16_t allowance_17mm;
    uint16_t allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t fortress_allowance_17mm;
} ProjectileAllowanceInfo_t;

/* cmd_id 0x0209（RFID 状态图）解析结果。 */
typedef struct
{
    /* 原始低位/高位比特集。 */
    uint32_t rfid_status_low;
    uint8_t rfid_status_high;

    /* 展平后的点位状态，便于按下标直接判断。 */
    uint8_t point_active[REFEREE_RFID_POINT_COUNT];
    uint8_t active_count;
} RFIDStatusInfo_t;

/* cmd_id 0x020A（飞镖发射站状态）解析结果。 */
typedef struct
{
    uint8_t launch_opening_status;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
    char launch_state_str[24];
} DartLaunchingStateInfo_t;

/* cmd_id 0x020B（地面机器人坐标）解析结果。 */
typedef struct
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float infantry3_x;
    float infantry3_y;
    float infantry4_x;
    float infantry4_y;
} GroundRobotPositionInfo_t;

/* cmd_id 0x020C（标记/特殊标识位图）解析结果。 */
typedef struct
{
    /* 原始 16 位位图，便于调试对照。 */
    uint16_t raw_mark_progress;

    /* 按协议定义拆出的机器人标志位。 */
    uint8_t enemy_hero_marked;
    uint8_t enemy_engineer_marked;
    uint8_t enemy_infantry3_marked;
    uint8_t enemy_infantry4_marked;
    uint8_t enemy_sentry_marked;
    uint8_t ally_hero_special_marked;
    uint8_t ally_engineer_special_marked;
    uint8_t ally_infantry3_special_marked;
    uint8_t ally_infantry4_special_marked;
    uint8_t ally_sentry_special_marked;
} RadarMarkInfo_t;

/* cmd_id 0x020D（哨兵信息）解析结果。 */
typedef struct
{
    /* 原始字段，保留以便后续协议升级时核对。 */
    uint32_t raw_sentry_info;
    uint16_t raw_sentry_info_2;

    /* 由 0x020D 位段解码得到的字段。 */
    uint16_t exchanged_projectile_allowance;
    uint8_t remote_exchange_projectile_count;
    uint8_t remote_exchange_hp_count;
    uint8_t can_confirm_free_revive;
    uint8_t can_exchange_instant_revive;
    uint16_t instant_revive_gold_cost;
    uint8_t sentry_posture;
    uint8_t own_energy_rune_can_activate;
    char sentry_posture_str[16];
} SentryInfoParse_t;

/* cmd_id 0x020E（雷达信息）解析结果。 */
typedef struct
{
    /* 协议原始位图。 */
    uint8_t raw_radar_info;

    /* 解码后的雷达状态。 */
    uint8_t double_vulnerability_chance;
    uint8_t enemy_double_vulnerability_active;
    uint8_t own_encryption_level;
    uint8_t can_change_key;
} RadarInfoParse_t;

/* 0x0301 图形子内容中的单图形语义化结果。 */
typedef struct
{
    char figure_name[4];
    uint8_t operate_type;
    uint8_t figure_type;
    uint8_t layer;
    uint8_t color;
    uint16_t details_a;
    uint16_t details_b;
    uint16_t width;
    uint16_t start_x;
    uint16_t start_y;
    uint16_t details_c;
    uint16_t details_d;
    uint16_t details_e;
    char operate_type_str[16];
    char figure_type_str[16];
    char color_str[16];
} InteractionFigureInfo_t;

/* 0x0110 字符子内容语义化结果。 */
typedef struct
{
    uint8_t has_data;
    InteractionFigureInfo_t graphic;
    uint8_t text_raw[REFEREE_DRAW_CHAR_RAW_LEN];
    char text_ascii[REFEREE_DRAW_CHAR_RAW_LEN + 1u];
    uint8_t text_non_printable_count;
} InteractionDrawCharInfo_t;

/* cmd_id 0x0301（机器人交互数据）解析结果。 */
typedef struct
{
    /* 通用交互头。 */
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;

    /* 载荷长度说明：
     * - expected_len：按 data_cmd_id 计算的理论长度。
     * - payload_len：真实长度（可由帧长推导时使用真实值）。
     */
    uint16_t payload_len;
    uint16_t expected_len;

    /* 分类标志，便于快速分发处理。 */
    uint8_t is_custom_data;
    uint8_t is_ui_data;
    uint8_t has_sentry_cmd;
    uint8_t has_radar_cmd;

    /* 0x0120/0x0121 的可选解码载荷。 */
    uint32_t sentry_cmd;
    uint8_t radar_cmd;
    char data_cmd_type_str[32];

    /* 原始 user_data 快照，便于调试。 */
    uint8_t raw_payload[REFEREE_INTERACTION_PAYLOAD_MAX];

    /* 0x0100 删除图层子内容。 */
    uint8_t has_delete_cmd;
    interaction_layer_delete_t delete_cmd;
    uint8_t delete_is_noop;
    uint8_t delete_is_layer;
    uint8_t delete_is_all;

    /* 0x0101/0x0102/0x0103/0x0104 图形子内容。 */
    uint8_t draw1_valid;
    InteractionFigureInfo_t draw1;
    uint8_t draw2_count;
    InteractionFigureInfo_t draw2[2];
    uint8_t draw5_count;
    InteractionFigureInfo_t draw5[5];
    uint8_t draw7_count;
    InteractionFigureInfo_t draw7[7];

    /* 0x0110 字符子内容。 */
    InteractionDrawCharInfo_t draw_char;

    /* 0x0200~0x02FF 自定义载荷。 */
    uint16_t custom_data_len;
    uint8_t custom_data[REFEREE_INTERACTION_PAYLOAD_MAX];
} RobotInteractionInfo_t;

/* cmd_id 0x0303（小地图指令）解析结果。 */
typedef struct
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
    uint8_t is_target_robot_mode;
} MapCommandInfo_t;

/* cmd_id 0x0305（小地图机器人坐标，cm->m）解析结果。 */
typedef struct
{
    float hero_x_m;
    float hero_y_m;
    float engineer_x_m;
    float engineer_y_m;
    float infantry3_x_m;
    float infantry3_y_m;
    float infantry4_x_m;
    float infantry4_y_m;
    float infantry5_x_m;
    float infantry5_y_m;
    float sentry_x_m;
    float sentry_y_m;
} MapRobotDataInfo_t;

/* cmd_id 0x0307（路径数据）解析结果。 */
typedef struct
{
    /* 路径头原始字段。 */
    uint8_t intention;
    uint16_t start_x_dm;
    uint16_t start_y_dm;

    /* 累加 49 组增量后的终点。 */
    int16_t end_x_dm;
    int16_t end_y_dm;

    uint16_t sender_id;
    char intention_str[24];
} MapPathInfo_t;

/* cmd_id 0x0308（向客户端发送自定义信息）解析结果。 */
typedef struct
{
    uint16_t sender_id;
    uint16_t receiver_id;

    /* UTF-16 原始码元（从 30 字节小端数据拆出）。 */
    uint16_t utf16_data[REFEREE_CUSTOM_INFO_UTF16_LEN];

    /* ASCII 预览：
     * - ASCII 码元直接拷贝。
     * - 非 ASCII 用 '?' 替代，并置标志位。
     */
    char ascii_preview[REFEREE_CUSTOM_INFO_UTF16_LEN + 1u];
    uint8_t contains_non_ascii;
} CustomInfoParse_t;

/* cmd_id 0x0304（键鼠遥控数据包）解析结果。 */
typedef struct
{
    /* 原始鼠标/键盘数据。 */
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t left_button_down;
    uint8_t right_button_down;
    uint16_t keyboard_value;
    uint8_t key_w;
    uint8_t key_s;
    uint8_t key_a;
    uint8_t key_d;
    uint8_t key_shift;
    uint8_t key_ctrl;
    uint8_t key_q;
    uint8_t key_e;
    uint8_t key_r;
    uint8_t key_f;
    uint8_t key_g;
    uint8_t key_z;
    uint8_t key_x;
    uint8_t key_c;
    uint8_t key_v;
    uint8_t key_b;
} RemoteControlInfo_t;

/* 聚合解析输出。与 referee_info_t 一一对应，保存 JudgeReadData
 * 当前已拷贝的所有协议块的解析结果。
 */
typedef struct
{
    /* 最近一帧元数据。 */
    uint16_t cmd_id;
    uint16_t frame_data_length;

    /* 按命令族组织的解析结果。 */
    GameStateInfo_t game_state;
    GameResultInfo_t game_result;
    GameRobotHPInfo_t game_robot_hp;
    EventDataInfo_t event_data;
    RefereeWarningInfo_t referee_warning;
    DartInfoParse_t dart_info;
    RobotStateInfo_t robot_state;
    RobotPosInfo_t robot_pos;
    BuffInfo_t buff;
    RobotHurtInfo_t robot_hurt;
    ShootDataInfo_t shoot_data;
    ProjectileAllowanceInfo_t projectile_allowance;
    RFIDStatusInfo_t rfid_status;
    DartLaunchingStateInfo_t dart_launching_state;
    GroundRobotPositionInfo_t ground_robot_position;
    RadarMarkInfo_t radar_mark;
    SentryInfoParse_t sentry_info;
    RadarInfoParse_t radar_info;
    RobotInteractionInfo_t robot_interaction;
    MapCommandInfo_t map_command;
    MapRobotDataInfo_t map_robot_data;
    MapPathInfo_t map_path;
    CustomInfoParse_t custom_info;
    RemoteControlInfo_t remote_control;
} RefereeParsedInfo_t;

/* 分命令解析接口。每个函数只读输入、只写输出。
 * 所有函数都做空指针保护（空指针直接返回）。
 */
void ParseGameState(const ext_game_state_t *game_state, GameStateInfo_t *info);
void ParseGameResult(const ext_game_result_t *game_result, GameResultInfo_t *info);
void ParseGameRobotHP(const ext_game_robot_HP_t *robot_hp, GameRobotHPInfo_t *info);
void ParseEventData(const ext_event_data_t *event_data, EventDataInfo_t *info);
void ParseRefereeWarning(const referee_warning_t *warning_data, RefereeWarningInfo_t *info);
void ParseDartInfo(const dart_info_t *dart_info, DartInfoParse_t *info);
void ParseRobotState(const ext_game_robot_state_t *robot_state, RobotStateInfo_t *info);
void ParseRobotPos(const robot_pos_t *robot_pos, RobotPosInfo_t *info);
void ParseBuff(const buff_t *buff, BuffInfo_t *info);
void ParseRobotHurt(const ext_robot_hurt_t *robot_hurt, RobotHurtInfo_t *info);
void ParseShootData(const ext_shoot_data_t *shoot_data, ShootDataInfo_t *info);
void ParseProjectileAllowance(const projectile_allowance_t *allowance, ProjectileAllowanceInfo_t *info);
void ParseRFIDStatus(const rfid_status_t *rfid_status, RFIDStatusInfo_t *info);
void ParseDartLaunchingState(const dart_client_cmd_t *dart_state, DartLaunchingStateInfo_t *info);
void ParseGroundRobotPosition(const ground_robot_position_t *position, GroundRobotPositionInfo_t *info);
void ParseRadarMarkData(const radar_mark_data_t *mark_data, RadarMarkInfo_t *info);
void ParseSentryInfo(const sentry_info_t *sentry_info, SentryInfoParse_t *info);
void ParseRadarInfo(const radar_info_t *radar_info, RadarInfoParse_t *info);
void ParseRobotInteraction(const robot_interaction_data_t *interaction, RobotInteractionInfo_t *info);
void ParseMapCommand(const map_command_t *map_cmd, MapCommandInfo_t *info);
void ParseMapRobotData(const map_robot_data_t *map_data, MapRobotDataInfo_t *info);
void ParseMapPathData(const map_data_t *path_data, MapPathInfo_t *info);
void ParseCustomInfo(const custom_info_t *custom_info, CustomInfoParse_t *info);
void ParseRemoteControl(const remote_control_t *remote, RemoteControlInfo_t *info);

/* 一次性总解析接口。上层如果希望直接从 referee_info_t 获取完整解析快照，
 * 可以调用此函数，无需逐个 ParseXxx 手动拼装。
 */
void ParseRefereeInfo(const referee_info_t *referee, RefereeParsedInfo_t *info);

#endif // REFEREE_PARSER_H
