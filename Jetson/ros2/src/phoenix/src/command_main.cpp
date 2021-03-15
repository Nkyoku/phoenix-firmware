#include "gpio.hpp"
#include "spi.hpp"
#include "avalon_st.hpp"
#include "avalon_mm.hpp"
#include <mutex>
#include <chrono>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <phoenix_msgs/srv/clear_error.hpp>
#include <phoenix_msgs/srv/set_speed.hpp>
#include <phoenix_msgs/srv/program_nios.hpp>
#include "../include/phoenix/shared_memory.hpp"

namespace phoenix {

/// SPIのデバイスパス
static constexpr char SpiDeviceName[] = "/dev/spidev1.0";

/// SPIのビットレート
static constexpr int SpiFrequency = 10000000;

/// SPIのモード
static constexpr int SpiMode = 1;

/// 速度制御の比例ゲインのパラメータ名
static const std::string PARAM_SPEED_KP("speed_kp");

/// 速度制御の積分ゲインのパラメータ名
static const std::string PARAM_SPEED_KI("speed_ki");

/// 姿勢補正制御の比例ゲインのパラメータ名
static const std::string PARAM_COMPENSATION_KP("compensation_kp");

/// 姿勢補正制御の積分ゲインのパラメータ名
static const std::string PARAM_COMPENSATION_KI("compensation_ki");

/// Nios IIの共有メモリーのベースアドレス
static constexpr uint32_t NIOS_SHARED_RAM_BASE = 0x0u;

/// Nios IIの命令メモリーのベースアドレス
static constexpr uint32_t NIOS_INSTRUCTION_RAM_BASE = 0xA5A50000u;

/// FPGA_MODEピン
static constexpr auto FPGA_MODE = Gpio::JetsonNanoModulePinGpio12;

/// FPGA_CONFIG_Nピン
static constexpr auto FPGA_CONFIG_N = Gpio::JetsonNanoModulePinGpio11;

/**
 * SPIで制御コマンドを送るノード
 */
class CommandServerNode : public rclcpp::Node {
public:
    /**
     * コンストラクタ
     */
    CommandServerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("phoenix_command") {
        using namespace std::placeholders;
        (void)options;

        // 共有メモリーを初期化する
        memset(&_SharedMemory, 0, sizeof(_SharedMemory));

        // SPIデバイスを開く
        auto spi = std::shared_ptr<Spi>(new Spi);
        if (spi->Open(phoenix::SpiDeviceName, phoenix::SpiFrequency) == false) {
            throw std::runtime_error("Failed to open SPI device");
        }
        spi->SetMode(phoenix::SpiMode);
        _AvalonMm = std::shared_ptr<AvalonMm>(new AvalonMm(spi));

        // サービスを登録する
        rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, QOS_DEPTH));
        _ClearErrorService = create_service<phoenix_msgs::srv::ClearError>("clear_error", std::bind(&CommandServerNode::ClearErrorCallback, this, _1, _2, _3));
        _SetSpeedService = create_service<phoenix_msgs::srv::SetSpeed>("set_speed", std::bind(&CommandServerNode::SetSpeedCallback, this, _1, _2, _3));
        _ProgramNiosService = create_service<phoenix_msgs::srv::ProgramNios>("program_nios", std::bind(&CommandServerNode::ProgramNiosCallback, this, _1, _2, _3));

        // パラメータを宣言する
        declare_parameter<double>(PARAM_SPEED_KP, 5.0);
        declare_parameter<double>(PARAM_SPEED_KI, 0.05);
        declare_parameter<double>(PARAM_COMPENSATION_KP, 2.0);
        declare_parameter<double>(PARAM_COMPENSATION_KI, 0.02);
    }

private:
    /*
     * ClearErrorサービスを処理する
     */
    void ClearErrorCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<phoenix_msgs::srv::ClearError::Request> request, const std::shared_ptr<phoenix_msgs::srv::ClearError::Response> response) {
        (void)request_header;
        (void)request;

        response->succeeded = false;
        response->any_errors = false;

        // 現在のErrorFlagsを確認する
        uint32_t error_flags;
        if (_AvalonMm->Read(static_cast<uint32_t>(offsetof(SharedMemory_t, ErrorFlags)), &error_flags) == false) {
            return;
        }
        if (error_flags != 0) {
            // 何らかのエラーが発生しているので消去を試みる
            // ErrorFlagsに0xFFFFFFFFを書き込むとエラーフラグの消去を要求できる
            if (_AvalonMm->Write(static_cast<uint32_t>(offsetof(SharedMemory_t, ErrorFlags)), 0xFFFFFFFFUL) == false) {
                return;
            }

            // ErrorFlagsをポーリングして変化したか確かめる
            int timeout = 3;
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                if (_AvalonMm->Read(static_cast<uint32_t>(offsetof(SharedMemory_t, ErrorFlags)), &error_flags) == false) {
                    return;
                }
                if (error_flags != 0xFFFFFFFFUL) {
                    break;
                }
                if (--timeout < 0) {
                    return;
                }
            } while (true);
            response->any_errors = (error_flags != 0);
        }
        response->succeeded = true;
    }

    /**
     * SetSpeedサービスを処理する
     */
    void SetSpeedCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<phoenix_msgs::srv::SetSpeed::Request> request, const std::shared_ptr<phoenix_msgs::srv::SetSpeed::Response> response) {
        (void)request_header;

        // パラメータをコピーする
        _SharedMemory.Parameters.FrameNumber++;
        _SharedMemory.Parameters.speed_x = request->speed_x;
        _SharedMemory.Parameters.speed_y = request->speed_y;
        _SharedMemory.Parameters.speed_omega = request->speed_omega;
        _SharedMemory.Parameters.dribble_power = request->dribble_power;
        _SharedMemory.Parameters.speed_gain_p = std::fmaxf(0.0f, GetFloatParameter(PARAM_SPEED_KP));
        _SharedMemory.Parameters.speed_gain_i = std::fmaxf(0.0f, GetFloatParameter(PARAM_SPEED_KI));
        _SharedMemory.Parameters.compensation_gain_p = std::fmaxf(0.0f, GetFloatParameter(PARAM_COMPENSATION_KP));
        _SharedMemory.Parameters.compensation_gain_i = std::fmaxf(0.0f, GetFloatParameter(PARAM_COMPENSATION_KI));

        // チェックサムを計算して格納する
        uint32_t checksum = _SharedMemory.Parameters.CalculateChecksum();
        _SharedMemory.HeadChecksum = checksum;
        _SharedMemory.TailChecksum = checksum;

        // パラメータと前後のチェックサムを書き込む
        response->succeeded = _AvalonMm->Write(
            NIOS_SHARED_RAM_BASE + static_cast<uint32_t>(offsetof(SharedMemory_t, HeadChecksum)),
            &_SharedMemory.HeadChecksum,
            sizeof(SharedMemory_t::Parameters_t) + sizeof(uint32_t) * 2);
    }

    /**
     * ProgramNiosサービスを処理する
     */
    void ProgramNiosCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<phoenix_msgs::srv::ProgramNios::Request> request, const std::shared_ptr<phoenix_msgs::srv::ProgramNios::Response> response) {
        (void)request_header;

        Gpio mode_pin(FPGA_MODE);
        Gpio config_pin(FPGA_CONFIG_N);
        if (!mode_pin.IsOpened() || !config_pin.IsOpened()) {
            response->succeeded = false;
            return;
        }

        // FPGA_MODEからLを出力してNios IIをリセットする
        mode_pin.SetOutputEnabled(true);
        mode_pin.SetOutputValue(false);

        // 一度の転送で送るデータのバイト数
        static constexpr size_t BYTES_PER_TRANSFER = 32;

        bool succeeded = true;

        // プログラムを転送する
        size_t written_bytes = 0;
        do {
            size_t length = std::min(request->program.size() - written_bytes, BYTES_PER_TRANSFER);
            if (!_AvalonMm->Write(NIOS_INSTRUCTION_RAM_BASE + written_bytes, request->program.data() + written_bytes, length)) {
                fprintf(stderr, "ProgramNios failed at _AvalonMm->Write(%zu, [%zu], %zu)\n", NIOS_INSTRUCTION_RAM_BASE + written_bytes, written_bytes, length);
                succeeded = false;
                break;
            }
            written_bytes += length;
        } while (written_bytes < request->program.size());

        if (succeeded) {
            // ベリファイする
            size_t read_bytes = 0;
            do {
                uint8_t buffer[BYTES_PER_TRANSFER];
                size_t length = std::min(request->program.size() - read_bytes, BYTES_PER_TRANSFER);
                if (!_AvalonMm->Read(NIOS_INSTRUCTION_RAM_BASE + read_bytes, buffer, length)) {
                    fprintf(stderr, "ProgramNios failed at _AvalonMm->Read(%zu, [0], %zu)\n", NIOS_INSTRUCTION_RAM_BASE + read_bytes, length);
                    succeeded = false;
                    break;
                }
                if (memcmp(buffer, request->program.data() + read_bytes, length) != 0) {
                    fprintf(stderr, "ProgramNios failed at memcmp([0], [%zu], %zu)\n", read_bytes, length);
                    succeeded = false;
                    break;
                }
                read_bytes += length;
            } while (read_bytes < request->program.size());
        }

        if (succeeded) {
            // 共有メモリーを消去する
            memset(&_SharedMemory, 0, sizeof(_SharedMemory));
            _AvalonMm->Write(NIOS_SHARED_RAM_BASE, &_SharedMemory, sizeof(SharedMemory_t));
        }

        if (succeeded) {
            // FPGA_MODEからHを出力してNios IIのリセットを解除する
            mode_pin.SetOutputValue(true);
        } else {
            // FPGA_CONFIG_NからLを出力してFPGAをリセットする
            config_pin.SetOutputEnabled(true);
            config_pin.SetOutputValue(false);

            // FPGA_MODEをHに戻す
            mode_pin.SetOutputValue(true);

            // FPGAをリコンフィギュレーションする
            config_pin.SetOutputValue(true);
        }

        response->succeeded = succeeded;
    }

    /**
     * パラメータをfloat型として取得する
     * @param parameter_name パラメータ名
     * @param default_value パラメータの型が不正だった場合に返すデフォルト値
     */
    float GetFloatParameter(const std::string &parameter_name, float default_value = 0.0f) {
        auto parameter = get_parameter(parameter_name);
        switch (parameter.get_type()) {
        case rclcpp::PARAMETER_DOUBLE:
            return parameter.as_double();
        case rclcpp::PARAMETER_INTEGER:
            return parameter.as_int();
        default:
            return default_value;
        }
    }

    /// Avalon-MMマスター
    std::shared_ptr<AvalonMm> _AvalonMm;

    /// ClearErrorサービス
    rclcpp::Service<phoenix_msgs::srv::ClearError>::SharedPtr _ClearErrorService;

    /// SetSpeedサービス
    rclcpp::Service<phoenix_msgs::srv::SetSpeed>::SharedPtr _SetSpeedService;

    /// SetSpeedサービス
    rclcpp::Service<phoenix_msgs::srv::ProgramNios>::SharedPtr _ProgramNiosService;

    /// 共有メモリー
    SharedMemory_t _SharedMemory;

    /// QoSのパラメータ
    static constexpr int QOS_DEPTH = 10;
};

}  // namespace phoenix

#ifdef PHOENIX_BUILD_AS_LIBRARY
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(phoenix::CommandServerNode)
#else
int main(int argc, char *argv[]) {
    // ROS2ノードを起動する
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<phoenix::CommandServerNode>());
    rclcpp::shutdown();
    return 0;
}
#endif
