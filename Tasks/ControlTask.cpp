/**
 *******************************************************************************
 * @file      :ControlTask.cpp
 * @brief     :GM6020 位置/速度双环控制任务 (考核作业)
 * @attention :1kHz 控制频率由 TIM6 提供 (84MHz / 84 / 1000 = 1kHz)
 *
 * 控制结构:
 *   位置给定(正弦) --> [位置环 PID] --> 转速给定 --> [速度环 PID] --> 控制量 --> CAN
 *
 * 调参说明: 所有 PID 增益都是 volatlie 全局变量, 上电即用其当前值,
 *           每拍都会重新 setParams, 所以可以在 Ozone 里在线改, 不用重编译。
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ControlTask.h"

#include "main.h"
#include "can.h"
#include "tim.h"
#include "iwdg.h"

#include "HW_can.hpp"
#include "system_user.hpp"

#include "Gm6020.h"
#include "Pid.h"

#include <math.h>

/* Private constants ---------------------------------------------------------*/
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

// ==================== 控制模式 ====================
//
// GM6020 有两种控制模式, 帧 ID 和量程都不同:
//   电压模式: 0x1FF (ID1~4) / 0x2FF (ID5~7), 量程 ±25000   <- 出厂默认
//   电流模式: 0x1FE (ID1~4) / 0x2FE (ID5~7), 量程 ±16384   <- 对应 ±3A
//
// 若电机指示灯"橙灯常亮"(手册定义: 电流控制模式下接收到电压指令),
// 说明那颗电机是电流模式, 把下面这行改成 true 即可。
constexpr bool kCurrentMode = false;

// 本模式下的满量程 (PID 输出限幅 / 最终指令限幅都用它)
constexpr float kMaxCmd = kCurrentMode ? 16384.0f : 25000.0f;
constexpr float kMinCmd = -kMaxCmd;

// ==================== 固定参数 ====================

// 电机拨码开关设置的 ID (1~7)
constexpr uint8_t kMotorId = 1;

// 发送帧标识符: 电流模式 0x1FE/0x2FE, 电压模式 0x1FF/0x2FF
constexpr uint32_t kTxFrameId = kCurrentMode
                                    ? ((kMotorId <= 4) ? 0x1FEu : 0x2FEu)
                                    : ((kMotorId <= 4) ? 0x1FFu : 0x2FFu);

// 位置环输出(即转速给定)限幅, rad/s
constexpr float kPosOutMax = 12.0f;
constexpr float kPosOutMin = -kPosOutMax;

// 转速给定限幅 (rad/s)。GM6020 空载最高约 320rpm ≈ 33.5rad/s
constexpr float kMaxSpeedRef = 30.0f;

// 转速给定斜坡(软启动/平滑): 每拍最多变化多少 rad/s
constexpr float kRefSlewPerTick = 0.05f;

// 反馈超时: 超过这么久没收到电机回帧就认为掉线
constexpr uint32_t kFeedbackTimeoutMs = 100u;

}  // namespace

/* ==================== 在线可调参数 (Ozone 里直接改) ==================== */

// 位置环开关: 1 = 位置正弦跟随 (默认), 0 = 纯速度环
volatile int g_pos_enable = 0;
// 位置给定幅值 (度)
volatile float g_pos_amp_deg = 60.0f;
// 位置给定频率 (Hz)。0.2Hz = 5 秒一个来回
volatile float g_pos_freq = 0.2f;

// ---------- 位置环增益 (输出: 转速给定 rad/s; 输入: 角度误差 rad) ----------
volatile float g_pos_kp = 18.0f;
volatile float g_pos_ki = 2.5f;
volatile float g_pos_kd = 0.0f;   // 微分项会放大编码器噪声, 建议 0

// 位置给定的速度前馈系数 (0~1)。
// 正弦给定所需的转速是已知的: d(A*sin(wt))/dt = A*w*cos(wt),
// 直接前馈给速度环, 位置环只修正残差 -> 大幅减小相位滞后,
// 而且不用把位置环增益堆很高(高增益会抖)。
volatile float g_pos_ff = 1.0f;

// ---------- 速度环增益 (输出: 控制量; 输入: 转速误差 rad/s) ----------
volatile float g_spd_kp = 1000.0f;
volatile float g_spd_ki = 150.0f;
volatile float g_spd_kd = 0.0f;

// 速度前馈 (模型逆前馈): 输出 ≈ g_spd_ff * 目标转速。
// 它负责提供"维持这个转速所需的稳态控制量", PID 只修残差。
// 电压模式下 GM6020 每 1rad/s 大约需要几百个控制量, 所以这个值比纯增益大得多。
// 太小 -> 幅值远远跟不上(表现为电机几乎不转); 太大 -> 冲过头/振荡。
volatile float g_spd_ff = 800.0f;

// 摩擦补偿 (单位同控制量, 满量程 25000)。
// 原理: 按给定转速的转向给一个固定的"推力", 抵消静摩擦/齿槽造成的
// 转向点粘滑(表现为峰值超前 + 下降段滞后 + 峰后小凹口)。
// 设 0 可关闭。太大表现为转向点发冲/抖。
volatile float g_fric = 300.0f;

// 加速度前馈系数 (单位: 控制量 / (rad/s^2))。
// 正弦给定所需的加速度是解析已知的: a = -w^2 * pos_ref。
// 乘上这个系数直接折算成控制量前馈出去, 等效于补偿转动惯量,
// 能让位置环几乎不用出力, 大幅减小动态跟随误差。
// 从 0 开始往大调, 直到峰值误差最小; 调过头会开始抖。
volatile float g_acc_ff = 50.0f;

// 转速反馈低通系数 (0~1)。
//   越小越平滑但相位滞后越大; 转速反馈量化(1rpm)噪声大时靠它压抖动。
//   0.12 @1kHz 约等于 19Hz 截止。
volatile float g_vel_lpf = 0.12f;

// 纯速度环模式下的转速给定 (rad/s); g_pos_enable = 0 时生效
volatile float g_speed_ref = 0.0f;
volatile int g_sine_enable = 1;
volatile float g_sine_amp = 2.0f;
volatile float g_sine_freq = 0.25f;

/* ==================== 调试镜像变量 (给 Ozone 曲线用) ====================
 * 在 Ozone 的 Data Sampling 窗口里 右键 -> Add Expression, 直接输变量名,
 * 然后在 Timeline 窗口看曲线。
 *
 *   g_dbg_ref      位置给定 (deg)   <- 位置跟随曲线的"给定"
 *   g_dbg_angle    实际角度 (deg)   <- 位置跟随曲线的"反馈"
 *   g_dbg_spd_ref  速度环给定 (rad/s)
 *   g_dbg_vel      实际转速 (rad/s, 已滤波, 环路实际用的就是这个)
 *   g_dbg_vel_raw  实际转速 (rad/s, 未滤波, 用来对比看噪声大小)
 *   g_dbg_rpm      实际转速 (rpm)
 *   g_dbg_out      控制量输出       <- 看有没有饱和/振荡
 */
volatile float g_dbg_ref = 0.0f;
volatile float g_dbg_angle = 0.0f;
volatile float g_dbg_spd_ref = 0.0f;
volatile float g_dbg_vel = 0.0f;
volatile float g_dbg_vel_raw = 0.0f;
volatile float g_dbg_rpm = 0.0f;
volatile float g_dbg_out = 0.0f;
// 位置误差 (deg) 及其上电以来的最大绝对值。判"跟得好不好"看这两个数就够,
// 比用眼睛比两条线靠谱。把 g_dbg_pos_err_max 手动写成 0 即可重新计最大值。
volatile float g_dbg_pos_err = 0.0f;
volatile float g_dbg_pos_err_max = 0.0f;

// 速度正弦参考曲线 (rad/s)。独立的正弦发生器, 与 g_dbg_vel 同单位, 可直接叠图。
//   - g_pos_enable=0 且 g_sine_enable=1 时, 它就是速度环真正跟随的给定,
//     此时它和 g_dbg_vel 应该贴合。
//   - 位置环模式下它只是一条独立的参考曲线(不参与控制), 用于对比;
//     位置环模式下实际速度是余弦(正弦位置给定的导数), 相位差 90°, 这是物理决定的。
// 幅值/频率沿用 g_sine_amp / g_sine_freq。
volatile float g_dbg_spd_sine = 0.0f;

// 位置给定, 单位恒为"度", 不随模式变化。
//   位置环模式: 正弦位置给定 (deg)
//   速度环模式: 恒为 0 (该模式没有位置给定)
// 这样 g_dbg_pos_ref 和 g_dbg_angle 永远可以放心叠在一起比。
volatile float g_dbg_pos_ref = 0.0f;

/* ==================== 迭代学习补偿 (ILC) ====================
 * 给定是周期信号(正弦), 那么跟随误差也是周期性的。用一张按"相位"索引的表,
 * 每个周期把同一相位上的误差累加进表里当补偿量, 若干周期后周期性误差会被
 * "学会"并抵消。齿槽转矩、摩擦这类周期性扰动最吃这一套。
 *
 *   g_ilc_enable = 1 开启, 0 关闭
 *   g_ilc_k      学习率(0.2 慢而稳, 1~2 快; 太大开始震荡)
 *   g_ilc_reset  写 1 清空学习表(改频率/幅值后必须清)
 *   g_dbg_ilc    当前相位点学到的补偿量, 可以画出来看收敛过程
 *
 * 需要等 10~20 个周期(0.2Hz 约 1 分钟)才能收敛到位。
 */
constexpr int kIlcSize = 1000;  // 一个周期分 1000 个相位点
volatile int g_ilc_enable = 1;
volatile float g_ilc_k = 1.0f;
volatile int g_ilc_reset = 0;
volatile float g_dbg_ilc = 0.0f;

static float s_ilc[kIlcSize];  // 4KB, 按相位存放学到的补偿量

/* Private variables ---------------------------------------------------------*/

static Gm6020 s_motor(kMotorId);
static Pid s_pos_pid(6.0f, 0.5f, 0.0f, kPosOutMax, kPosOutMin);
static Pid s_spd_pid(400.0f, 100.0f, 0.0f, kMaxCmd, kMinCmd);

static float s_ref_now = 0.0f;      // 斜坡处理后的转速给定 (rad/s)
static float s_vel_filt = 0.0f;     // 低通滤波后的转速反馈 (rad/s)
static uint32_t s_last_fb_ms = 0;   // 最后一次收到反馈的时刻
static bool s_has_fb = false;       // 是否已经收到过反馈
static float s_sine_phase = 0.0f;   // 给定相位, 0.0 ~ 1.0 对应 0 ~ 2pi
static float s_spd_sine_phase = 0.0f;  // 速度正弦参考的相位, 0.0 ~ 1.0

static bool s_zero_done = false;    // 是否已经记录零位
static float s_angle_zero = 0.0f;   // 上电时的角度, 作为位置零点 (rad)

/* Private functions ---------------------------------------------------------*/

// 组 8 字节控制帧并发出
static void sendControlFrame(void) {
  uint8_t data[8] = {0};  // 未使用的电机槽位发 0
  s_motor.encode(data);
  CAN_Send_Msg(&hcan1, data, kTxFrameId, 8);
}

// 清空迭代学习表
static void ilcReset(void) {
  for (int i = 0; i < kIlcSize; i++) {
    s_ilc[i] = 0.0f;
  }
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  控制任务初始化: 配 CAN 过滤器、启动 CAN 与 1kHz 定时器
 */
void ControlTaskInit(void) {
  HAL_IWDG_Refresh(&hiwdg);

  // CAN1: 过滤器接受全部报文(CanFilter_Init 内部已配好), 开 FIFO0 接收中断
  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // 清状态
  s_pos_pid.reset();
  s_spd_pid.reset();
  s_motor.setVoltage(0);
  s_ref_now = 0.0f;
  s_vel_filt = 0.0f;
  s_sine_phase = 0.0f;
  s_spd_sine_phase = 0.0f;
  ilcReset();
  s_has_fb = false;
  s_zero_done = false;
  s_angle_zero = 0.0f;
  s_last_fb_ms = HAL_GetTick();

  // 启动 1kHz 控制定时器 (TIM6)
  HAL_TIM_Base_Start_IT(&htim6);

  HAL_IWDG_Refresh(&hiwdg);
}

/**
 * @brief  1kHz 控制主任务, 由 TIM6 周期中断调用
 */
void MainTask(void) {
  // 1) 喂独立看门狗 (IWDG 超时约 0.8s, 必须每拍都喂)
  HAL_IWDG_Refresh(&hiwdg);

  // 2) 保留原工程的 tick 语义 (system_user.hpp 中 extern 的全局变量)
  tick++;

  // 3) 每拍同步一次在线增益, 让 Ozone 里改的值立刻生效
  s_pos_pid.setParams(g_pos_kp, g_pos_ki, g_pos_kd);
  s_spd_pid.setParams(g_spd_kp, g_spd_ki, g_spd_kd);

  // 4) 反馈超时判断
  if (s_has_fb && (HAL_GetTick() - s_last_fb_ms) > kFeedbackTimeoutMs) {
    s_has_fb = false;
  }

  // 5) 收到第一帧反馈时记录零位, 并清相位, 保证给定从 0 开始
  if (s_has_fb && !s_zero_done) {
    s_angle_zero = s_motor.angle();
    s_zero_done = true;
    s_pos_pid.reset();
    s_spd_pid.reset();
    s_ref_now = 0.0f;
    s_sine_phase = 0.0f;
    s_vel_filt = 0.0f;
    ilcReset();
  }

  // 6) 转速反馈低通滤波 (压掉 1rpm 量化噪声带来的高频抖动)
  float vel_raw = s_motor.vel();
  float a = g_vel_lpf;
  if (a < 0.0f) a = 0.0f;
  if (a > 1.0f) a = 1.0f;
  s_vel_filt += a * (vel_raw - s_vel_filt);

  // 7) 速度正弦参考曲线 (独立相位, 每拍都算, 供叠图对比)
  s_spd_sine_phase += g_sine_freq * 0.001f;
  if (s_spd_sine_phase >= 1.0f) s_spd_sine_phase -= 1.0f;
  if (s_spd_sine_phase < 0.0f) s_spd_sine_phase += 1.0f;
  g_dbg_spd_sine = g_sine_amp * sinf(s_spd_sine_phase * kTwoPi);

  // 8) 生成转速给定
  float spd_ref;          // 速度环给定 (rad/s)
  float out_ff = 0.0f;    // 加速度前馈 (直接加到控制量上)

  if (g_pos_enable) {
    // ---------------- 位置环: 位置正弦给定 -> 转速给定 ----------------
    s_sine_phase += g_pos_freq * 0.001f;  // 1kHz 一拍
    if (s_sine_phase >= 1.0f) s_sine_phase -= 1.0f;
    if (s_sine_phase < 0.0f) s_sine_phase += 1.0f;

    float theta = s_sine_phase * kTwoPi;      // 相位 (rad)
    float amp = g_pos_amp_deg * kDegToRad;    // 幅值 (rad)
    float omega = kTwoPi * g_pos_freq;        // 角频率 (rad/s)

    float pos_ref = amp * sinf(theta);                    // rad
    float spd_ff = g_pos_ff * amp * omega * cosf(theta);  // rad/s, 给定自身的导数
    float pos_now = s_motor.angle() - s_angle_zero;       // rad

    // 速度前馈 + 位置环残差修正
    spd_ref = spd_ff + s_pos_pid.calc(pos_ref, pos_now);

    // 加速度前馈: a = -w^2 * pos_ref, 折算成控制量
    out_ff = -g_acc_ff * omega * omega * pos_ref;

    // ---- 迭代学习: 把同一相位上的周期性误差累积成补偿量 ----
    int idx = static_cast<int>(s_sine_phase * kIlcSize);
    if (idx < 0) idx = 0;
    if (idx >= kIlcSize) idx = kIlcSize - 1;

    if (g_ilc_reset) {
      ilcReset();
      g_ilc_reset = 0;
    }

    if (g_ilc_enable) {
      float err_deg = (pos_ref - pos_now) * kRadToDeg;
      s_ilc[idx] += g_ilc_k * err_deg;
      if (s_ilc[idx] > 6000.0f) s_ilc[idx] = 6000.0f;
      if (s_ilc[idx] < -6000.0f) s_ilc[idx] = -6000.0f;
      out_ff += s_ilc[idx];
    }
    g_dbg_ilc = s_ilc[idx];

    g_dbg_ref = pos_ref * kRadToDeg;    // 位置给定 (deg)
    g_dbg_angle = pos_now * kRadToDeg;  // 实际角度 (deg)
    g_dbg_pos_ref = pos_ref * kRadToDeg;  // 单位恒定的位置给定 (deg)

    // 跟随误差指标
    g_dbg_pos_err = (pos_ref - pos_now) * kRadToDeg;
    if (fabsf(g_dbg_pos_err) > g_dbg_pos_err_max) {
      g_dbg_pos_err_max = fabsf(g_dbg_pos_err);
    }
  } else {
    // ---------------- 纯速度环 ----------------
    if (g_sine_enable) {
      spd_ref = g_dbg_spd_sine;  // 直接跟这条速度正弦参考
    } else {
      spd_ref = g_speed_ref;
    }
    float pos_now = s_motor.angle() - s_angle_zero;
    g_dbg_ref = spd_ref;                // 此模式下是转速给定 (rad/s)
    g_dbg_angle = pos_now * kRadToDeg;  // 实际角度 (deg)
    g_dbg_pos_ref = 0.0f;               // 速度模式无位置给定
  }

  // 8) 转速给定限幅 + 斜坡, 避免阶跃和冲击
  if (spd_ref > kMaxSpeedRef) spd_ref = kMaxSpeedRef;
  if (spd_ref < -kMaxSpeedRef) spd_ref = -kMaxSpeedRef;

  float step = spd_ref - s_ref_now;
  if (step > kRefSlewPerTick) step = kRefSlewPerTick;
  if (step < -kRefSlewPerTick) step = -kRefSlewPerTick;
  s_ref_now += step;

  // 9) 速度环: 反馈 = 滤波后的转速(rad/s), 输出 = 控制量
  if (!s_has_fb) {
    s_spd_pid.reset();  // 没反馈时清积分, 只靠比例+前馈把电机带动
  }
  float out = s_spd_pid.calc(s_ref_now, s_vel_filt) + g_spd_ff * s_ref_now + out_ff;

  // 摩擦补偿: 按转向给固定推力。±0.05rad/s 死区内不加, 避免零速附近来回抖。
  if (s_ref_now > 0.05f) {
    out += g_fric;
  } else if (s_ref_now < -0.05f) {
    out -= g_fric;
  }

  // 10) 限幅后下发
  if (out > kMaxCmd) out = kMaxCmd;
  if (out < kMinCmd) out = kMinCmd;

  s_motor.setVoltage(static_cast<int16_t>(out));

  // 11) 刷新调试镜像变量
  g_dbg_spd_ref = s_ref_now;
  g_dbg_vel = s_vel_filt;
  g_dbg_vel_raw = vel_raw;
  g_dbg_rpm = static_cast<float>(s_motor.velRpm());
  g_dbg_out = out;

  sendControlFrame();
}

/**
 * @brief  CAN 接收回调, 由 HAL_CAN_RxFifo0MsgPendingCallback 调用
 */
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
  if (data == NULL) {
    return;
  }
  // GM6020 反馈帧: 0x204 + ID
  if (std_id != s_motor.rxId()) {
    return;
  }

  s_motor.decode(data);
  s_has_fb = true;
  s_last_fb_ms = HAL_GetTick();
}
