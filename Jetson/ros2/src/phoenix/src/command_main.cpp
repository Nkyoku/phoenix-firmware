#include "gpio.hpp"
#include "spi.hpp"
#include "avalon_st.hpp"
#include "avalon_mm.hpp"
#include "epcq.hpp"
#include <mutex>
#include <chrono>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <phoenix_msgs/srv/clear_error.hpp>
#include <phoenix_msgs/srv/set_speed.hpp>
#include <phoenix_msgs/srv/program_nios.hpp>
#include <phoenix_msgs/srv/program_fpga.hpp>
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

/// FPGAのアプリケーションビットストリームのベースアドレス
static constexpr uint32_t FPGA_APPLICATION_BASE = 0x100000u;

/// FPGAのアプリケーションビットストリームのサイズ
static constexpr uint32_t FPGA_APPLICATION_SIZE = 0x100000u;

/// FPGA_MODEピン
static constexpr auto FPGA_MODE = Gpio::JetsonNanoModulePinGpio12;

/// FPGA_CONFIG_Nピン
static constexpr auto FPGA_CONFIG_N = Gpio::JetsonNanoModulePinGpio11;

// FPGA_APPLICATION_BASEはセクターサイズの倍数である必要がある
static_assert((FPGA_APPLICATION_BASE % Epcq::SECTOR_SIZE) == 0, "FPGA_APPLICATION_BASE must be multiple of Epcq::SECTOR_SIZE");

// FPGA_APPLICATION_SIZEはセクターサイズの倍数である必要がある
static_assert((FPGA_APPLICATION_SIZE % Epcq::SECTOR_SIZE) == 0, "FPGA_APPLICATION_SIZE must be multiple of Epcq::SECTOR_SIZE");

// セクターサイズはページサイズの倍数である必要がある
static_assert((Epcq::SECTOR_SIZE % Epcq::PAGE_SIZE) == 0, "Epcq::SECTOR_SIZE must be multiple of Epcq::PAGE_SIZE");

/// ビット順序を反転するテーブル
static constexpr uint8_t BIT_REVERSAL_TABLE[256] = {0, 128, 64, 192, 32, 160, 96, 224, 16, 144, 80, 208, 48, 176, 112, 240, 8, 136, 72, 200, 40, 168, 104, 232, 24, 152, 88, 216, 56, 184, 120, 248, 4, 132, 68, 196, 36, 164, 100, 228, 20, 148, 84, 212, 52, 180, 116, 244, 12, 140, 76, 204, 44, 172, 108, 236, 28, 156, 92, 220, 60, 188, 124, 252, 2, 130, 66, 194, 34, 162, 98, 226, 18, 146, 82, 210, 50, 178, 114, 242, 10, 138, 74, 202, 42, 170, 106, 234, 26, 154, 90, 218, 58, 186, 122, 250, 6, 134, 70, 198, 38, 166, 102, 230, 22, 150, 86, 214, 54, 182, 118, 246, 14, 142, 78, 206, 46, 174, 110, 238, 30, 158, 94, 222, 62, 190, 126, 254, 1, 129, 65, 193, 33, 161, 97, 225, 17, 145, 81, 209, 49, 177, 113, 241, 9, 137, 73, 201, 41, 169, 105, 233, 25, 153, 89, 217, 57, 185, 121, 249, 5, 133, 69, 197, 37, 165, 101, 229, 21, 149, 85, 213, 53, 181, 117, 245, 13, 141, 77, 205, 45, 173, 109, 237, 29, 157, 93, 221, 61, 189, 125, 253, 3, 131, 67, 195, 35, 163, 99, 227, 19, 147, 83, 211, 51, 179, 115, 243, 11, 139, 75, 203, 43, 171, 107, 235, 27, 155, 91, 219, 59, 187, 123, 251, 7, 135, 71, 199, 39, 167, 103, 231, 23, 151, 87, 215, 55, 183, 119, 247, 15, 143, 79, 207, 47, 175, 111, 239, 31, 159, 95, 223, 63, 191, 127, 255};

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
        _Spi = std::shared_ptr<Spi>(new Spi);
        if (_Spi->Open(phoenix::SpiDeviceName, phoenix::SpiFrequency) == false) {
            throw std::runtime_error("Failed to open SPI device");
        }
        _Spi->SetMode(phoenix::SpiMode);
        _AvalonMm = std::shared_ptr<AvalonMm>(new AvalonMm(_Spi));

        // サービスを登録する
        rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, QOS_DEPTH));
        _ClearErrorService = create_service<phoenix_msgs::srv::ClearError>("clear_error", std::bind(&CommandServerNode::ClearErrorCallback, this, _1, _2, _3));
        _SetSpeedService = create_service<phoenix_msgs::srv::SetSpeed>("set_speed", std::bind(&CommandServerNode::SetSpeedCallback, this, _1, _2, _3));
        _ProgramNiosService = create_service<phoenix_msgs::srv::ProgramNios>("program_nios", std::bind(&CommandServerNode::ProgramNiosCallback, this, _1, _2, _3));
        _ProgramFpgaService = create_service<phoenix_msgs::srv::ProgramFpga>("program_fpga", std::bind(&CommandServerNode::ProgramFpgaCallback, this, _1, _2, _3));

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

        // GPIOを開く
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
                fprintf(stderr, "ProgramNios was failed at _AvalonMm->Write(%zu, [%zu], %zu)\n", NIOS_INSTRUCTION_RAM_BASE + written_bytes, written_bytes, length);
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
                    fprintf(stderr, "ProgramNios was failed at _AvalonMm->Read(%zu, [0], %zu)\n", NIOS_INSTRUCTION_RAM_BASE + read_bytes, length);
                    succeeded = false;
                    break;
                }
                if (memcmp(buffer, request->program.data() + read_bytes, length) != 0) {
                    fprintf(stderr, "ProgramNios was failed at memcmp([0], [%zu], %zu)\n", read_bytes, length);
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
            // FPGA_MODEを解放してNios IIのリセットを解除する
            mode_pin.SetOutputEnabled(false);
        } else {
            // FPGA_CONFIG_NからLを出力してFPGAをリセットする
            config_pin.SetOutputEnabled(true);
            config_pin.SetOutputValue(false);

            // FPGA_MODEを解放する
            mode_pin.SetOutputEnabled(false);

            // FPGAをリコンフィギュレーションする
            config_pin.SetOutputEnabled(false);
        }

        response->succeeded = succeeded;
    }

    /**
     * ProgramFpgaサービスを処理する
     */
    void ProgramFpgaCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<phoenix_msgs::srv::ProgramFpga::Request> request, const std::shared_ptr<phoenix_msgs::srv::ProgramFpga::Response> response) {
        (void)request_header;

        // GPIOを開く
        Gpio mode_pin(FPGA_MODE);
        Gpio config_pin(FPGA_CONFIG_N);
        if (!mode_pin.IsOpened() || !config_pin.IsOpened()) {
            response->succeeded = false;
            return;
        }

        // FPGA_CONFIG_NからLを出力してFPGAをリセットする
        config_pin.SetOutputEnabled(true);
        config_pin.SetOutputValue(false);

        // FPGA_MODEをLowにしておく
        // FPGAのリコンフィギュレーション後にファクトリーモードのまま維持できる
        mode_pin.SetOutputEnabled(true);
        mode_pin.SetOutputValue(false);

        // FPGA_CONFIG_Nを解放してFPGAをリコンフィギュレーションする
        config_pin.SetOutputEnabled(false);

        // 250ms待つ
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        // 書き込むデータのビット順序を反転する
        for (uint32_t address = FPGA_APPLICATION_BASE; address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE); address++) {
            request->bitstream[address] = BIT_REVERSAL_TABLE[request->bitstream[address]];
        }

        // フラッシュメモリーを操作する
        _Spi->SetMode(0);
        Epcq epcq(_Spi);
        bool succeeded = false;
        do {
            // シリコンIDを読み取る
            uint8_t sillicon_id;
            if (!epcq.ReadSilliconId(&sillicon_id)) {
                fprintf(stderr, "ProgramFpga was failed at epcq.ReadSilliconId()\n");
                break;
            }
            if ((sillicon_id == 0xFF) || (sillicon_id == 0x4A)) {
                fprintf(stderr, "ProgramFpga was failed because the sillicon ID is 0x%02X\n", sillicon_id);
                break;
            }
            fprintf(stderr, "Sillicon ID is 0x%02X\n", sillicon_id);

            // セクターを消去する
            uint32_t address;
            for (address = FPGA_APPLICATION_BASE; address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE); address += Epcq::SECTOR_SIZE) {
                if (!epcq.EraseSector(address)) {
                    fprintf(stderr, "ProgramFpga was failed at epcq.EraseSector(0x%06X)\n", address);
                    break;
                }
            }
            if (address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE)) {
                break;
            }

            // ページに書き込む
            for (address = FPGA_APPLICATION_BASE; address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE); address += Epcq::PAGE_SIZE) {
                const uint8_t *data = &request->bitstream[address];
                bool empty = true;
                for (uint32_t index = 0; index < Epcq::PAGE_SIZE; index++) {
                    if (data[index] != 0xFF) {
                        empty = false;
                        break;
                    }
                }
                if (!empty) {
                    if (!epcq.WritePage(address, data)) {
                        fprintf(stderr, "ProgramFpga was failed at epcq.WritePage(0x%06X)\n", address);
                        break;
                    }
                }
            }
            if (address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE)) {
                break;
            }

            // ベリファイする
            static constexpr uint32_t VERIFY_SIZE = 1024;
            static_assert((FPGA_APPLICATION_SIZE % VERIFY_SIZE) == 0, "VERIFY_SIZE must be divisor of FPGA_APPLICATION_SIZE");
            for (address = FPGA_APPLICATION_BASE; address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE); address += VERIFY_SIZE) {
                const uint8_t *written_data = &request->bitstream[address];
                bool empty = true;
                for (uint32_t index = 0; index < VERIFY_SIZE; index++) {
                    if (written_data[index] != 0xFF) {
                        empty = false;
                        break;
                    }
                }
                if (!empty) {
                    uint8_t read_data[VERIFY_SIZE];
                    if (!epcq.Read(address, read_data, VERIFY_SIZE)) {
                        fprintf(stderr, "ProgramFpga was failed at epcq.Read(0x%06X, %u)\n", address, VERIFY_SIZE);
                        break;
                    }
                    if (memcmp(written_data, read_data, VERIFY_SIZE) != 0) {
                        fprintf(stderr, "ProgramFpga was failed because verification error at 0x%06X\n", address);
                        break;
                    }
                }
            }
            if (address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE)) {
                break;
            }

            // 操作は正常に終了した
            succeeded = true;
        } while (false);
        _Spi->SetMode(phoenix::SpiMode);

        if (succeeded) {
            // FPGA_MODEを解放してアプリケーションを起動する
            mode_pin.SetOutputEnabled(false);
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
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

    /// SPI
    std::shared_ptr<Spi> _Spi;

    /// Avalon-MMマスター
    std::shared_ptr<AvalonMm> _AvalonMm;

    /// ClearErrorサービス
    rclcpp::Service<phoenix_msgs::srv::ClearError>::SharedPtr _ClearErrorService;

    /// SetSpeedサービス
    rclcpp::Service<phoenix_msgs::srv::SetSpeed>::SharedPtr _SetSpeedService;

    /// ProgramNiosサービス
    rclcpp::Service<phoenix_msgs::srv::ProgramNios>::SharedPtr _ProgramNiosService;

    /// ProgramFpgaサービス
    rclcpp::Service<phoenix_msgs::srv::ProgramFpga>::SharedPtr _ProgramFpgaService;

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
