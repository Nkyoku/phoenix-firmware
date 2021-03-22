#include "avalon_st.hpp"
#include "uart.hpp"
#include <memory>
#include <thread>
#include <exception>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <arm_fp16.h>
#include <sensor_msgs/msg/imu.hpp>
#include <phoenix_msgs/msg/stream_data_adc2.hpp>
#include <phoenix_msgs/msg/stream_data_status.hpp>
#include <phoenix_msgs/msg/stream_data_motion.hpp>
#include <phoenix_msgs/msg/stream_data_control.hpp>
#include "../include/phoenix/stream_data.hpp"
#include "../include/phoenix/status_flags.hpp"

namespace phoenix {

/// UARTのデバイスパス
static constexpr char UartDeviceName[] = "/dev/ttyTHS1";

/// UARTのボーレート
static constexpr int UartBaudrate = B4000000;

/**
 * UARTで受信したデータを整形して配信するノード
 */
class StreamPublisherNode : public rclcpp::Node {
public:
    /**
     * コンストラクタ
     */
    StreamPublisherNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("phoenix_stream"), _Uart(new Uart) {
        using namespace std::chrono_literals;
        (void)options;

        // UARTデバイスを開く
        if (_Uart->Open(UartDeviceName) == false) {
            fprintf(stderr, "Cannot open '%s'.\n", UartDeviceName);
            throw;
        }
        if (_Uart->SetBaudrate(UartBaudrate) == false) {
            fprintf(stderr, "Cannot configure desired baudrate '%d'.\n", UartBaudrate);
            throw;
        }

        // publisherを作成する
        rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, QOS_DEPTH));
        _StatusPublisher = this->create_publisher<phoenix_msgs::msg::StreamDataStatus>("phoenix_status", qos);
        _Adc2Publisher = this->create_publisher<phoenix_msgs::msg::StreamDataAdc2>("phoenix_adc2", qos);
        _MotionPublisher = this->create_publisher<phoenix_msgs::msg::StreamDataMotion>("phoenix_motion", qos);
        _ControlPublisher = this->create_publisher<phoenix_msgs::msg::StreamDataControl>("phoenix_control", qos);
        _ImuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("phoenix_imu", qos);

        // スレッドを起動する
        _ReceiveThread = new std::thread([this]() { ReceiveThread(); });
        _PublishThread = new std::thread([this]() { PublishThread(); });
    }

    virtual ~StreamPublisherNode() {
        _ThreadExitRequest = true;
        if (_ReceiveThread != nullptr) {
            _ReceiveThread->join();
            delete _ReceiveThread;
        }
        if (_PublishThread != nullptr) {
            _PublishThread->join();
            delete _PublishThread;
        }
    }

private:
    /**
     * 受信スレッドの本体
     */
    void ReceiveThread(void) {
        static constexpr size_t READ_BUFFER_SIZE = 4096;
        static constexpr size_t MAXIMUM_PAYLOAD_LENGTH = 1024;
        std::vector<uint8_t> payload;
        payload.reserve(MAXIMUM_PAYLOAD_LENGTH);
        int channel_number = -1;
        bool payload_overrun = false;
        std::array<uint8_t, READ_BUFFER_SIZE> buffer;
        AvalonStBytesToPacketsConverter converter;
        while (!_ThreadExitRequest) {
            // UARTからデータを受信する
            size_t read_length;
            if (_Uart->Read(buffer.data(), buffer.size(), &read_length) == false) {
                fprintf(stderr, "An error was occured while reading UART device.\n");
                return;
            }

            // バイトストリームをパースする
            converter.Parse(buffer.data(), read_length, [&](uint8_t data, int channel, bool sof, bool eof) {
                if ((channel_number != channel) || sof) {
                    if (!payload.empty() && !payload_overrun)
                        ProcessPacket(payload, channel_number);
                    channel_number = channel;
                    payload.clear();
                    payload_overrun = false;
                }
                if (payload.size() < MAXIMUM_PAYLOAD_LENGTH) {
                    payload.push_back(data);
                } else {
                    payload_overrun = true;
                }
                if (eof) {
                    if (!payload.empty() && !payload_overrun)
                        ProcessPacket(payload, channel_number);
                    payload.clear();
                    payload_overrun = false;
                }
                return true;
            });
        }
    }

    /**
     * 受信したパケットを処理する
     * @param payload パケットのペイロード
     * @param channel パケットのチャンネル番号
     */
    void ProcessPacket(const std::vector<uint8_t> &payload, int channel) {
        switch (channel) {
        case StreamIdStatus:
            if (payload.size() == sizeof(StreamDataStatus_t)) {
                _QueueMutex.lock();
                if (QUEUE_LENGTH <= _StatusQueue.size()) {
                    _StatusQueue.pop();
                }
                _StatusQueue.push(*reinterpret_cast<const StreamDataStatus_t *>(payload.data()));
                _QueueMutex.unlock();
                _ConditionVariable.notify_one();
            }
            break;

        case StreamIdAdc2:
            if (payload.size() == sizeof(StreamDataAdc2_t)) {
                _QueueMutex.lock();
                if (QUEUE_LENGTH <= _Adc2Queue.size()) {
                    _Adc2Queue.pop();
                }
                _Adc2Queue.push(*reinterpret_cast<const StreamDataAdc2_t *>(payload.data()));
                _QueueMutex.unlock();
                _ConditionVariable.notify_one();
            }
            break;

        case StreamIdMotion:
            if (payload.size() == sizeof(StreamDataMotion_t)) {
                _QueueMutex.lock();
                if (QUEUE_LENGTH <= _MotionQueue.size()) {
                    _MotionQueue.pop();
                }
                _MotionQueue.push(*reinterpret_cast<const StreamDataMotion_t *>(payload.data()));
                _QueueMutex.unlock();
                _ConditionVariable.notify_one();
            }
            break;

        case StreamIdControl:
            if (payload.size() == sizeof(StreamDataControl_t)) {
                _QueueMutex.lock();
                if (QUEUE_LENGTH <= _ControlQueue.size()) {
                    _ControlQueue.pop();
                }
                _ControlQueue.push(*reinterpret_cast<const StreamDataControl_t *>(payload.data()));
                _QueueMutex.unlock();
                _ConditionVariable.notify_one();
            }
            break;

        default:
            break;
        }
    }

    /**
     * 配信スレッドの本体
     */
    void PublishThread(void) {
        static constexpr auto TIMEOUT = std::chrono::milliseconds(100);
        while (!_ThreadExitRequest) {
            std::unique_lock<std::mutex> lock(_QueueMutex);
            _ConditionVariable.wait_for(lock, TIMEOUT);
            if (!_StatusQueue.empty()) {
                // StreamDataStatus_tをコピーする
                std::array<phoenix_msgs::msg::StreamDataStatus, QUEUE_LENGTH> buffer;
                int count = 0;
                do {
                    ConvertStatus(_StatusQueue.front(), &buffer[count]);
                    count++;
                    _StatusQueue.pop();
                } while (!_StatusQueue.empty());

                // StreamDataStatusを配信する
                lock.unlock();
                for (int index = 0; index < count; index++) {
                    _StatusPublisher->publish(buffer[index]);
                }
                lock.lock();
            }
            if (!_Adc2Queue.empty()) {
                // StreamDataAdc2_tをコピーする
                std::array<phoenix_msgs::msg::StreamDataAdc2, QUEUE_LENGTH> buffer;
                int count = 0;
                do {
                    ConvertAdc2(_Adc2Queue.front(), &buffer[count]);
                    count++;
                    _Adc2Queue.pop();
                } while (!_Adc2Queue.empty());

                // StreamDataAdc2を配信する
                lock.unlock();
                for (int index = 0; index < count; index++) {
                    _Adc2Publisher->publish(buffer[index]);
                }
                lock.lock();
            }
            if (!_MotionQueue.empty()) {
                // StreamDataMotion_tをコピーする
                std::array<phoenix_msgs::msg::StreamDataMotion, QUEUE_LENGTH> motion_buffer;
                std::array<sensor_msgs::msg::Imu, QUEUE_LENGTH> imu_buffer;
                int count = 0;
                do {
                    ConvertMotion(_MotionQueue.front(), &motion_buffer[count], &imu_buffer[count]);
                    count++;
                    _MotionQueue.pop();
                } while (!_MotionQueue.empty());

                // StreamDataMotionとImuを配信する
                lock.unlock();
                for (int index = 0; index < count; index++) {
                    _MotionPublisher->publish(motion_buffer[index]);
                    _ImuPublisher->publish(imu_buffer[index]);
                }
                lock.lock();
            }
            if (!_ControlQueue.empty()) {
                // StreamDataControl_tをコピーする
                std::array<phoenix_msgs::msg::StreamDataControl, QUEUE_LENGTH> buffer;
                int count = 0;
                do {
                    ConvertControl(_ControlQueue.front(), &buffer[count]);
                    count++;
                    _ControlQueue.pop();
                } while (!_ControlQueue.empty());

                // StreamDataControlを配信する
                lock.unlock();
                for (int index = 0; index < count; index++) {
                    _ControlPublisher->publish(buffer[index]);
                }
                lock.lock();
            }
        }
    }

    /**
     * StreamDataStatus_tをROS2メッセージに変換する
     */
    void ConvertStatus(const StreamDataStatus_t &data, phoenix_msgs::msg::StreamDataStatus *msg) {
        uint32_t error_flags = data.error_flags;
        uint32_t fault_flags = data.fault_flags;
        if (error_flags != 0) {
            msg->error_module_sleep = (error_flags & ErrorCauseModuleSleep);
            msg->error_fpga_stop = (error_flags & ErrorCauseFpgaStop);
            msg->error_dc48v_uv = (error_flags & ErrorCauseDc48vUnderVoltage);
            msg->error_dc48v_ov = (error_flags & ErrorCauseDc48vOverVoltage);
            msg->error_motor_oc[0] = (error_flags & ErrorCauseMotor1OverCurrent);
            msg->error_motor_oc[1] = (error_flags & ErrorCauseMotor2OverCurrent);
            msg->error_motor_oc[2] = (error_flags & ErrorCauseMotor3OverCurrent);
            msg->error_motor_oc[3] = (error_flags & ErrorCauseMotor4OverCurrent);
            msg->error_motor_oc[4] = (error_flags & ErrorCauseMotor5OverCurrent);
            msg->error_motor_hall_sensor[0] = (error_flags & ErrorCauseMotor1HallSensor);
            msg->error_motor_hall_sensor[1] = (error_flags & ErrorCauseMotor2HallSensor);
            msg->error_motor_hall_sensor[2] = (error_flags & ErrorCauseMotor3HallSensor);
            msg->error_motor_hall_sensor[3] = (error_flags & ErrorCauseMotor4HallSensor);
            msg->error_motor_hall_sensor[4] = (error_flags & ErrorCauseMotor5HallSensor);
        }
        if (fault_flags != 0) {
            msg->fault_adc2_timeout = (fault_flags & FaultCauseAdc2Timeout);
            msg->fault_imu_timeout = (fault_flags & FaultCauseImuTimeout);
            msg->fault_motor_ot[0] = (fault_flags & FaultCauseMotor1OverTemperature);
            msg->fault_motor_ot[1] = (fault_flags & FaultCauseMotor2OverTemperature);
            msg->fault_motor_ot[2] = (fault_flags & FaultCauseMotor3OverTemperature);
            msg->fault_motor_ot[3] = (fault_flags & FaultCauseMotor4OverTemperature);
            msg->fault_motor_ot[4] = (fault_flags & FaultCauseMotor5OverTemperature);
            msg->fault_motor_oc[0] = (fault_flags & FaultCauseMotor1OverCurrent);
            msg->fault_motor_oc[1] = (fault_flags & FaultCauseMotor2OverCurrent);
            msg->fault_motor_oc[2] = (fault_flags & FaultCauseMotor3OverCurrent);
            msg->fault_motor_oc[3] = (fault_flags & FaultCauseMotor4OverCurrent);
            msg->fault_motor_oc[4] = (fault_flags & FaultCauseMotor5OverCurrent);
            msg->fault_motor_load_switch[0] = (fault_flags & FaultCauseMotor1LoadSwitch);
            msg->fault_motor_load_switch[1] = (fault_flags & FaultCauseMotor2LoadSwitch);
            msg->fault_motor_load_switch[2] = (fault_flags & FaultCauseMotor3LoadSwitch);
            msg->fault_motor_load_switch[3] = (fault_flags & FaultCauseMotor4LoadSwitch);
            msg->fault_motor_load_switch[4] = (fault_flags & FaultCauseMotor5LoadSwitch);
        }
    }

    /**
     * StreamDataAdc2_tをROS2メッセージに変換する
     */
    void ConvertAdc2(const StreamDataAdc2_t &data, phoenix_msgs::msg::StreamDataAdc2 *msg) {
        msg->dc48v_voltage = data.dc48v_voltage;
        msg->dribble_voltage = data.dribble_voltage;
        msg->dribble_current = data.dribble_current;
    }

    /**
     * StreamDataMotion_tをROS2メッセージに変換する
     */
    void ConvertMotion(const StreamDataMotion_t &data, phoenix_msgs::msg::StreamDataMotion *motion, sensor_msgs::msg::Imu *imu) {
        // StreamDataMotionに変換する
        for (int axis = 0; axis < 3; axis++) {
            motion->accelerometer[axis] = data.accelerometer[axis];
            motion->gyroscope[axis] = data.gyroscope[axis];
        }
        for (int index = 0; index < 4; index++) {
            motion->wheel_velocity[index] = data.wheel_velocity[index];
            motion->wheel_current_d[index] = data.wheel_current_d[index];
            motion->wheel_current_q[index] = data.wheel_current_q[index];
        }

        // IMU測定値をImuメッセージに変換する
        imu->orientation_covariance[0] = -1.0;
        imu->linear_acceleration.x = data.accelerometer[0];
        imu->linear_acceleration.y = data.accelerometer[1];
        imu->linear_acceleration.z = data.accelerometer[2];
        imu->angular_velocity.x = data.gyroscope[0];
        imu->angular_velocity.y = data.gyroscope[1];
        imu->angular_velocity.z = data.gyroscope[2];
    }

    /**
     * StreamDataControl_tをROS2メッセージに変換する
     */
    void ConvertControl(const StreamDataControl_t &data, phoenix_msgs::msg::StreamDataControl *msg) {
        msg->performance_counter = data.performance_counter;
        for (int index = 0; index < 4; index++) {
            msg->wheel_velocity_ref[index] = data.wheel_velocity_ref[index];
            msg->wheel_current_ref[index] = data.wheel_current_ref[index];
            msg->wheel_current_limit[index] = data.wheel_current_limit[index];
        }
        for (int index = 0; index < 3; index++) {
            msg->machine_velocity[index] = data.machine_velocity[index];
        }
        msg->slip_flags = data.slip_flags;
    }

    /// UARTデバイス
    std::shared_ptr<Uart> _Uart;

    /// 受信スレッドへの終了リクエスト
    volatile bool _ThreadExitRequest = false;

    /// 受信スレッド
    std::thread *_ReceiveThread = nullptr;

    /// 配信スレッド
    std::thread *_PublishThread = nullptr;

    std::condition_variable _ConditionVariable;

    /// キュー操作を制御するミューテックス
    std::mutex _QueueMutex;

    /// StreamDataStatus_tを格納するキュー
    std::queue<StreamDataStatus_t> _StatusQueue;

    /// StreamDataAdc2_tを格納するキュー
    std::queue<StreamDataAdc2_t> _Adc2Queue;

    /// StreamDataMotion_tを格納するキュー
    std::queue<StreamDataMotion_t> _MotionQueue;

    /// StreamDataControl_tを格納するキュー
    std::queue<StreamDataControl_t> _ControlQueue;

    /// StreamDataStatus(FPGA内のNios IIのCentralizedMonitorのステータスフラグ)を配信するpublisher
    rclcpp::Publisher<phoenix_msgs::msg::StreamDataStatus>::SharedPtr _StatusPublisher;

    /// StreamDataAdc2(FPGAに繋がったADC2の測定値)を配信するpublisher
    rclcpp::Publisher<phoenix_msgs::msg::StreamDataAdc2>::SharedPtr _Adc2Publisher;

    /// StreamDataMotion(IMUやセンサーの情報)を配信するpublisher
    rclcpp::Publisher<phoenix_msgs::msg::StreamDataMotion>::SharedPtr _MotionPublisher;

    /// StreamDataControl(モーター制御の情報)を配信するpublisher
    rclcpp::Publisher<phoenix_msgs::msg::StreamDataControl>::SharedPtr _ControlPublisher;

    /// IMUの測定値を配信するpublisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _ImuPublisher;

    /// QoSのパラメータ
    static constexpr int QOS_DEPTH = 10;

    /// キューの長さ
    static constexpr size_t QUEUE_LENGTH = 10;
};

}  // namespace phoenix

#ifdef PHOENIX_BUILD_AS_LIBRARY
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(phoenix::StreamPublisherNode)
#else
int main(int argc, char *argv[]) {
    // ROS2ノードを起動する
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<phoenix::StreamPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
#endif
