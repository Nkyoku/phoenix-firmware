#pragma once

#include <math.h> // math.h内のfmaxf,fminfを後でカスタム命令版に置き換えるため最初にincludeする
#include <system.h>
#include "filter.hpp"
#include "data_holder.hpp"

/// 車輪速度を格納する構造体
struct WheelVelocity_t {
    union {
        struct {
            float V1, V2, V3, V4;
        };
        float V[4];
    };
};

/// 機体速度を格納する構造体
struct MachineVelocity_t {
    float Vx, Vy, Omega;
};

/**
 * 車輪制御を行う
 */
class WheelController {
public:
    /**
     * 初期化を行う
     */
    static void Initialize(void) {
        StopControl();
    }

    /**
     * モーター制御を開始する
     */
    static void StartControl(void);

    /**
     * モーター制御を終了し、モーターを脱力する
     */
    static void StopControl(void);

    /**
     * 指令値を更新する
     * @param new_parameters new_parameters 共有メモリーのParametersが更新されたときにtrueを指定する
     * @param force_brake ショートブレーキを使用するときに指定する
     */
    static void Update(bool new_parameters, bool force_brake);

    /**
     * 速度指令値を取得する
     * @return 速度指令値の配列へのポインタ (インデックスは0~3のみ有効)
     */
    static const float* GetWheelVelocityReference(void) {
        return _SpeedReference;
    }

    /**
     * 回生エネルギーを取得する
     * @return 回生エネルギーの配列へのポインタ (インデックスは0~3のみ有効)
     */
    static const float* GetWheelRenegerationEnergy(void) {
        return _CurrentLimit;
    }

    /**
     * 機体速度の推定値を取得する
     */
    static const MachineVelocity_t& GetEstimatedMachineVelocity(void) {
        return _MachineVelocityDummy;
    }

private:
    /**
     * 機体速度から車輪速度を求める
     * @param machine 機体速度
     * @return 車輪速度
     */
    static WheelVelocity_t VelocityVectorDeconmosition(const MachineVelocity_t &machine){
        float vx = machine.Vx * (WHEEL_POS_Y / sqrt(WHEEL_POS_R_2));
        float vy = machine.Vy * (WHEEL_POS_X / sqrt(WHEEL_POS_R_2));
        float vr = machine.Omega * sqrt(WHEEL_POS_R_2);
        float v1 = vr - vx + vy;
        float v2 = vr + vx + vy;
        float v3 = vr + vx - vy;
        float v4 = vr - vx - vy;
        return {v1, v2, v3, v4};
    }

    /**
     * 与えられた機体速度を実現した際にモーターに定常的に流れる電流の合計値を計算する(電源が流す電流ではない)
     * @param mv 機体速度
     * @return 電流の合計値 [A]
     */
    static float SteadyCurrent(const MachineVelocity_t &machine);

    /**
     * 与えられた機体速度を実現した際に車輪の最大速度を計算する
     * @param mv 機体速度
     * @return 車輪の最大速度 [m/s]
     */
    static float MaximumWheelVelocity(const MachineVelocity_t &machine);

    /**
     * 与えられた機体速度を実現可能な速度に制限する
     * @param mv 機体速度
     */
    static void LimitInputVelocity(MachineVelocity_t *machine);

    /**
     * 車輪速度とIMUの測定値から機体速度を推定する
     * 結果は_MachineVelocityに格納される
     * @param wheel 車輪速度
     * @param imu IMUの測定値
     */
    static void EstimateMachineVelocity(const WheelVelocity_t &wheel, const MotionData_t::Imu_t &imu);

    /**
     * 車輪速度と機体速度から車輪のスリップの度合いを計算する
     * 結果は_SlipIndicatorに格納される
     * @param wheel 車輪速度
     * @param reference_current リファレンス電流
     */
    static void DetectSlip(const WheelVelocity_t &wheel, const float *reference_current);

    /// 機体速度の指令値
    static MachineVelocity_t _MachineVelocityReference;

    /// 速度制御の指令値
    static float _SpeedReference[4];

    /// 速度制御の前回の誤差
    static float _LastSpeedError[4];

    /// 電流制御の指令値
    static float _CurrentReference[4];

    /// 電流制御の指令値(車輪)
    static float _CurrentReferenceOfWheel[4];

    /// 電流制御の指令値(機体)
    static float _CurrentReferenceOfMachine[4];

    /// モーターの発生させた回生エネルギー (負の値をとる)
    static float _RegenerationEnergy[4];

    static float _CurrentLimit[4];

    static float _LastCompensationForce[4];

    /// スリップの度合い
    static float _SlipIndicator[4];

    /// X, Y方向の並進速度を制御するかどうか
    //static bool _IgnoreVx, _IgnoreVy;

    /// 機体速度
    static MachineVelocity_t _MachineVelocity;

    static MachineVelocity_t _MachineVelocityDummy;

    /// 車輪の回転速度に適用するLPF
    static Lpf2ndOrder50 _LpfWheelVelocity[4];

    /// 電流制御の比例ゲイン
    static constexpr int CURRENT_CONTROL_GAIN_P = 3500;

    /// 電流制御の積分ゲイン
    static constexpr int CURRENT_CONTROL_GAIN_I = 500;

    /// 車体中心を原点とした車輪のX座標[m]
    static constexpr float WHEEL_POS_X = 0.063f;

    /// 車体中心を原点とした車輪のY座標[m]
    static constexpr float WHEEL_POS_Y = 0.042f;

    /// 車体中心から車輪までの距離の二乗[m]
    static constexpr float WHEEL_POS_R_2 = WHEEL_POS_X * WHEEL_POS_X + WHEEL_POS_Y * WHEEL_POS_Y;

    /// 並進速度指令値の最大値[m/s]
    static constexpr float MAX_TRANSLATION_REFERENCE = 10.0f;

    /// 回転速度指令の最大値[m/s]
    static constexpr float MAX_OMEGA_REFERENCE = 50.0f;

    /// 加速度の制限 [m/s^2]
    static constexpr float ACCELERATION_LIMIT_VX = 5.0f;

    /// 加速度の制限 [m/s^2]
    static constexpr float ACCELERATION_LIMIT_VY = 10.0f;

    /// 加速度の制限 [rad/s^2]
    static constexpr float ACCELERATION_LIMIT_OMEGA = 20.0f;

    /// 電流指令値の最小値[A]
    static constexpr float MIN_CURRENT_LIMIT_PER_MOTOR = 0.2f;

    /// 電流指令値の最大値[A]
    static constexpr float MAX_CURRENT_LIMIT_PER_MOTOR = 3.0f;

    /// 全てのモーターの電流指令値の絶対合計の最大値[A]
    static constexpr float TOTAL_CURRENT_LIMIT = 6.0f;

    /// 車輪速度の制限値 [m/s]
    static constexpr float WHEEL_VELOCITY_LIMIT = 6.0f;

    /// 定常的に流れる電流の合計の制限値 [A]
    static constexpr float STEADY_CURRENT_LIMIT = 4.0f;

    /// モーターの逆起電力定数(相間) [V/rps]
    static constexpr float MOTOR_SPEED_CONSTANT = 1.0f / 72.7f * 60.0f;

    /// モーターの巻線抵抗(相間) [Ω]
    static constexpr float MOTOR_RESISTANCE = 6.89f;

    /// モーターのトルク定数 [Nm/A]
    static constexpr float MOTOR_TORQUE_CONSTANT = 0.131f;

    /// モータードライバのベース消費電力 [W]
    static constexpr float BASE_POWER_CONSUMPTION_PER_MOTOR = 0.25f;

    /// ブレーキを有効にする回生エネルギーの閾値
    static constexpr float BRAKE_ENABLE_THRESHOLD = -0.01f; //-0.025f;

    /// ブレーキを無効にする回生エネルギーの閾値
    static constexpr float BRAKE_DISABLE_THRESHOLD = -0.005f; //-0.0125f;
};
