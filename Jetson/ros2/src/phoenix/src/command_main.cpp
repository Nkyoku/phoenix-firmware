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
#include "../include/phoenix/shared_memory.hpp"

namespace phoenix {

/// SPIのデバイスパス
static constexpr char SpiDeviceName[] = "/dev/spidev1.0";

/// SPIのビットレート
static constexpr int SpiFrequency = 20000000;

/// SPIのモード
static constexpr int SpiMode = 1;

/// 速度制御の比例ゲインのパラメータ名
static const std::string PARAM_SPEED_KP("speed_kp");

/// 速度制御の積分ゲインのパラメータ名
static const std::string PARAM_SPEED_KI("speed_ki");

/**
 * SPIで制御コマンドを送るノード
 */
class CommandServerNode : public rclcpp::Node {
public:
    /**
     * コンストラクタ
     */
    CommandServerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("command_server") {
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

        // パラメータを宣言する
        declare_parameter<double>(PARAM_SPEED_KP, 0.0);
        declare_parameter<double>(PARAM_SPEED_KI, 0.0);
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
        for (int index = 0; index < 4; index++) {
            _SharedMemory.Parameters.wheel_speed[index] = request->wheel_speed[index];
        }
        _SharedMemory.Parameters.dribble_power = request->dribble_power;
        _SharedMemory.Parameters.speed_gain_p = std::fmaxf(0.0f, static_cast<float>(get_parameter(PARAM_SPEED_KP).as_double()));
        _SharedMemory.Parameters.speed_gain_i = std::fmaxf(0.0f, static_cast<float>(get_parameter(PARAM_SPEED_KI).as_double()));

        // チェックサムを計算して格納する
        uint32_t checksum = _SharedMemory.Parameters.CalculateChecksum();
        _SharedMemory.HeadChecksum = checksum;
        _SharedMemory.TailChecksum = checksum;

        // パラメータと前後のチェックサムを書き込む
        response->succeeded = _AvalonMm->Write(
            static_cast<uint32_t>(offsetof(SharedMemory_t, HeadChecksum)),
            &_SharedMemory.HeadChecksum,
            sizeof(SharedMemory_t::Parameters_t) + sizeof(uint32_t) * 2);
    }

    /// Avalon-MMマスター
    std::shared_ptr<AvalonMm> _AvalonMm;

    /// ClearErrorサービス
    rclcpp::Service<phoenix_msgs::srv::ClearError>::SharedPtr _ClearErrorService;

    /// SetSpeedサービス
    rclcpp::Service<phoenix_msgs::srv::SetSpeed>::SharedPtr _SetSpeedService;

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
