#include <math.h>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include "i2c.hpp"
#include "ads1015.hpp"

namespace phoenix {

/// I2Cのデバイスパス
static const char I2cDeviceName[] = "/dev/i2c-1";

/// ADS1015のI2Cアドレス
static constexpr uint8_t Ads1015Address = 0x48;

class AdcPublisherNode : public rclcpp::Node {
public:
    AdcPublisherNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("phoenix_battery"), _Adc3() {
        using namespace std::chrono_literals;
        (void)options;

        // I2Cデバイスを開く
        std::shared_ptr<I2c> i2c(new I2c);
        if (i2c->Open(phoenix::I2cDeviceName) == false) {
            fprintf(stderr, "Cannot open '%s'.\n", I2cDeviceName);
            throw;
        }

        // ADC3(ADS1015)を見つける
        _Adc3 = std::shared_ptr<ADS1015>(new ADS1015(i2c, phoenix::Ads1015Address));
        if (_Adc3->Initialize() == false) {
            fprintf(stderr, "ADS1015 is not found.\n");
            throw;
        }

        // publisherを作成する
        rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
        _BatteryPublisher = this->create_publisher<sensor_msgs::msg::BatteryState>("battery", qos);
        _Timer = this->create_wall_timer(10ms, std::bind(&AdcPublisherNode::TimerCallback, this));

        // _BatteryStateの不明な項目をNaNで初期化する
        _BatteryState.charge = std::nanf("");
        _BatteryState.capacity = std::nanf("");
        _BatteryState.design_capacity = std::nanf("");
        _BatteryState.percentage = std::nanf("");
    }

private:
    /**
     * 定期的に1チャンネルずつ変換を行う
     */
    void TimerCallback(void) {
        int16_t adc_result;
        static constexpr ADS1015::MUX_t mux_table[4] = {ADS1015::MUX_GND_to_AIN0, ADS1015::MUX_GND_to_AIN1, ADS1015::MUX_GND_to_AIN2, ADS1015::MUX_GND_to_AIN3};
        if (DoConversion(mux_table[_Channel], ADS1015::FSR_2048mV, ADS1015::DR_1600SPS, &adc_result) == false) {
            // 何度か失敗したらノードを終了するようにするといいかもしれない
            return;
        }
        if (_Channel == 0) {
            // AIN0 (VBAT_IN_MON)
            _BatteryState.voltage = adc_result * (2.048f / 32767 * 23);
        } else if (_Channel == 1) {
            // AIN1 (VSYS_MON)
            _VsysVoltage = adc_result * (2.048f / 32767 * 23);
        } else if (_Channel == 2) {
            // AIN2 (ESW_IS)
            _BatteryState.current = adc_result * (-2.048f / 32767 / 220 * 9700);  // 負荷は放電電流になるので負符号にする
        } else if (_Channel == 3) {
            // AIN3 (TEMP_MON)
            float ratio = 3.3f / (adc_result * (2.048f / 32767)) - 1.0f;
            _BatteryState.temperature = 1.0f / (-log(ratio) / 3380 + 1.0f / (273.15f + 25.0f)) - 273.15f;

            // 測定結果を配信する
            _BatteryState.present = (_VsysVoltage <= _BatteryState.voltage);  // VbatよりVsysが高かったらバッテリー非接続とする
            _BatteryState.power_supply_status = _BatteryState.present
                                                    ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
                                                    : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
            if (OverTemperatureWarningThreshold <= _BatteryState.temperature) {
                _BatteryState.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
            } else if (!_BatteryState.present) {
                _BatteryState.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
            } else if (_BatteryState.voltage < UnderVoltageWarningThreshold) {
                _BatteryState.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
            } else if (OverVoltageWarningThreshold < _BatteryState.voltage) {
                _BatteryState.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
            } else {
                _BatteryState.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
            }
            _BatteryPublisher->publish(_BatteryState);
        }
        _Channel = (_Channel + 1) % 4;
    }

    /**
     * ADCの変換を行って結果を取得する
     */
    bool DoConversion(ADS1015::MUX_t mux, ADS1015::FSR_t fsr, ADS1015::DR_t dr, int16_t *result) {
        using namespace std::chrono_literals;
        static constexpr int TIMEOUT = 100;
        if (_Adc3->StartConversion(mux, fsr, dr) == true) {
            int timeout = TIMEOUT;
            do {
                std::this_thread::sleep_for(1ms);
                bool complete = true;
                if (_Adc3->IsConversionCompleted(&complete) == false) {
                    break;
                }
                if (complete == true) {
                    if (_Adc3->GetConversionResult(result) == false) {
                        break;
                    }
                    return true;
                }
            } while (0 < --timeout);
        }
        return false;
    }

    /// ADC3
    std::shared_ptr<ADS1015> _Adc3;

    /// 測定中のチャンネル番号
    int _Channel = 0;

    /// VSYS電圧の測定値
    /// _BatteryStateの中には該当する項目が無いため
    float _VsysVoltage = 0.0f;

    /// バッテリー状態を格納するメッセージ
    sensor_msgs::msg::BatteryState _BatteryState;

    /// AD変換を行うタイミングを生成するタイマー
    rclcpp::TimerBase::SharedPtr _Timer;

    /// _BatteryStateを配信するpublisher
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr _BatteryPublisher;

    /// 過熱警報の閾値 [℃] (この値はSLG46826に設定した過熱シャットダウンの閾値より低い)
    static constexpr float OverTemperatureWarningThreshold = 60.0f;

    /// バッテリー過電圧警報の閾値 [V]
    static constexpr float OverVoltageWarningThreshold = 28.0f;

    /// バッテリー低電圧警報の閾値 [V] (この値はSLG46826に設定したUVLO電圧より高い)
    static constexpr float UnderVoltageWarningThreshold = 19.2f;
};

}  // namespace phoenix

int main(int argc, char *argv[]) {
    // ROS2ノードを起動する
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<phoenix::AdcPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
