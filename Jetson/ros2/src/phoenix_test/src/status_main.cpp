#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <sensor_msgs/msg/battery_state.hpp>
#include <phoenix_msgs/msg/stream_data_status.hpp>
#include <phoenix_msgs/msg/stream_data_adc2.hpp>
#include <phoenix_msgs/msg/stream_data_motion.hpp>
#include <ncurses.h>

// Command Template
// ros2 service call /set_speed phoenix_msgs/srv/SetSpeed "{wheel_speed: [0.0, 0.0, 0.0, 0.0], dribble_power: 0.0}"
// ros2 param set /command_server speed_kp 0.4
// ros2 param set /command_server speed_ki 0.008

class StatusTestNode : public rclcpp::Node {
public:
    StatusTestNode(void) : Node("status_test_subscriber") {
        using namespace std::placeholders;

        InitializeStatusWindow();
        InitializeAdc2Window();
        InitializeBatteryWindow();
        InitializeMotionWindow();
        _Initialized = true;

        _StatusSubscriber = create_subscription<phoenix_msgs::msg::StreamDataStatus>("stream_data_status", 10, [this](const std::shared_ptr<phoenix_msgs::msg::StreamDataStatus> msg) { this->UpdateStatus(msg.get()); });
        _Adc2Subscriber = create_subscription<phoenix_msgs::msg::StreamDataAdc2>("stream_data_adc2", 10, [this](const std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> msg) { this->_LastAdc2 = msg; });
        _MotionSubscriber = create_subscription<phoenix_msgs::msg::StreamDataMotion>("stream_data_motion", 10, [this](const std::shared_ptr<phoenix_msgs::msg::StreamDataMotion> msg) { this->_MotionMessage = msg; });
        _BatterySubscriber = create_subscription<sensor_msgs::msg::BatteryState>("battery", 10, [this](const std::shared_ptr<sensor_msgs::msg::BatteryState> msg) { this->_BatteryMessage = msg; });

        _Adc2Timer = create_wall_timer(std::chrono::milliseconds(200), [this](void) { UpdateAdc2(); });
        _BatteryTimer = create_wall_timer(std::chrono::milliseconds(200), [this](void) { UpdateBattery(); });
        _MotionTimer = create_wall_timer(std::chrono::milliseconds(100), [this](void) { UpdateMotion(); });
    }

    ~StatusTestNode() {
        delwin(_StatusWindow);
        delwin(_Adc2Window);
        delwin(_BatteryWindow);
        delwin(_MotionWindow);
    }

private:
    void InitializeStatusWindow(void) {
        _StatusWindow = newwin(17, 32, 1, 2);
        wborder(_StatusWindow, 0, 0, 0, 0, 0, 0, 0, 0);
        mvwaddstr(_StatusWindow, 0, 3, "  [Status Flags]");
        mvwaddstr(_StatusWindow, 2, 3, "=== Error Flags ======");
        mvwaddstr(_StatusWindow, 3, 3, " Module Stop [ ]");
        mvwaddstr(_StatusWindow, 4, 3, "   FPGA Stop [ ]");
        mvwaddstr(_StatusWindow, 5, 3, "       DC48V [  ] [  ]");
        mvwaddstr(_StatusWindow, 6, 3, "    Motor OC [     ]");
        mvwaddstr(_StatusWindow, 7, 3, " Hall Sensor [     ]");
        mvwaddstr(_StatusWindow, 9, 3, "=== Fault Flags ======");
        mvwaddstr(_StatusWindow, 10, 3, "ADC2 Timeout [ ]");
        mvwaddstr(_StatusWindow, 11, 3, " IMU Timeout [ ]");
        mvwaddstr(_StatusWindow, 12, 3, "    Motor OT [     ]");
        mvwaddstr(_StatusWindow, 13, 3, "    Motor OC [     ]");
        mvwaddstr(_StatusWindow, 14, 3, " Load Switch [     ]");
        _StatusLastUpdate = std::chrono::system_clock::now();
        UpdateStatus(&_LastStatus);
        wrefresh(_StatusWindow);
    }

    void UpdateStatus(const phoenix_msgs::msg::StreamDataStatus *msg) {
        bool update = false;
        static const char *ok = "_";
        static const char *ng = "X";
        update |= PrintFlag(_StatusWindow, 3, 17, ng, ok, msg->error_module_sleep, _LastStatus.error_module_sleep);
        update |= PrintFlag(_StatusWindow, 4, 17, ng, ok, msg->error_fpga_stop, _LastStatus.error_fpga_stop);
        update |= PrintFlag(_StatusWindow, 5, 17, "UV", "__", msg->error_dc48v_uv, _LastStatus.error_dc48v_uv);
        update |= PrintFlag(_StatusWindow, 5, 22, "OV", "__", msg->error_dc48v_ov, _LastStatus.error_dc48v_ov);
        update |= PrintFlag(_StatusWindow, 6, 17, "1", "_", msg->error_motor_oc[0], _LastStatus.error_motor_oc[0]);
        update |= PrintFlag(_StatusWindow, 6, 18, "2", "_", msg->error_motor_oc[1], _LastStatus.error_motor_oc[1]);
        update |= PrintFlag(_StatusWindow, 6, 19, "3", "_", msg->error_motor_oc[2], _LastStatus.error_motor_oc[2]);
        update |= PrintFlag(_StatusWindow, 6, 20, "4", "_", msg->error_motor_oc[3], _LastStatus.error_motor_oc[3]);
        update |= PrintFlag(_StatusWindow, 6, 21, "5", "_", msg->error_motor_oc[4], _LastStatus.error_motor_oc[4]);
        update |= PrintFlag(_StatusWindow, 7, 17, "1", "_", msg->error_motor_hall_sensor[0], _LastStatus.error_motor_hall_sensor[0]);
        update |= PrintFlag(_StatusWindow, 7, 18, "2", "_", msg->error_motor_hall_sensor[1], _LastStatus.error_motor_hall_sensor[1]);
        update |= PrintFlag(_StatusWindow, 7, 19, "3", "_", msg->error_motor_hall_sensor[2], _LastStatus.error_motor_hall_sensor[2]);
        update |= PrintFlag(_StatusWindow, 7, 20, "4", "_", msg->error_motor_hall_sensor[3], _LastStatus.error_motor_hall_sensor[3]);
        update |= PrintFlag(_StatusWindow, 7, 21, "5", "_", msg->error_motor_hall_sensor[4], _LastStatus.error_motor_hall_sensor[4]);
        update |= PrintFlag(_StatusWindow, 10, 17, ng, ok, msg->fault_adc2_timeout, _LastStatus.fault_adc2_timeout);
        update |= PrintFlag(_StatusWindow, 11, 17, ng, ok, msg->fault_imu_timeout, _LastStatus.fault_imu_timeout);
        update |= PrintFlag(_StatusWindow, 12, 17, "1", "_", msg->fault_motor_ot[0], _LastStatus.fault_motor_ot[0]);
        update |= PrintFlag(_StatusWindow, 12, 18, "2", "_", msg->fault_motor_ot[1], _LastStatus.fault_motor_ot[1]);
        update |= PrintFlag(_StatusWindow, 12, 19, "3", "_", msg->fault_motor_ot[2], _LastStatus.fault_motor_ot[2]);
        update |= PrintFlag(_StatusWindow, 12, 20, "4", "_", msg->fault_motor_ot[3], _LastStatus.fault_motor_ot[3]);
        update |= PrintFlag(_StatusWindow, 12, 21, "5", "_", msg->fault_motor_ot[4], _LastStatus.fault_motor_ot[4]);
        update |= PrintFlag(_StatusWindow, 13, 17, "1", "_", msg->fault_motor_oc[0], _LastStatus.fault_motor_oc[0]);
        update |= PrintFlag(_StatusWindow, 13, 18, "2", "_", msg->fault_motor_oc[1], _LastStatus.fault_motor_oc[1]);
        update |= PrintFlag(_StatusWindow, 13, 19, "3", "_", msg->fault_motor_oc[2], _LastStatus.fault_motor_oc[2]);
        update |= PrintFlag(_StatusWindow, 13, 20, "4", "_", msg->fault_motor_oc[3], _LastStatus.fault_motor_oc[3]);
        update |= PrintFlag(_StatusWindow, 13, 21, "5", "_", msg->fault_motor_oc[4], _LastStatus.fault_motor_oc[4]);
        update |= PrintFlag(_StatusWindow, 14, 17, "1", "_", msg->fault_motor_load_switch[0], _LastStatus.fault_motor_load_switch[0]);
        update |= PrintFlag(_StatusWindow, 14, 18, "2", "_", msg->fault_motor_load_switch[1], _LastStatus.fault_motor_load_switch[1]);
        update |= PrintFlag(_StatusWindow, 14, 19, "3", "_", msg->fault_motor_load_switch[2], _LastStatus.fault_motor_load_switch[2]);
        update |= PrintFlag(_StatusWindow, 14, 20, "4", "_", msg->fault_motor_load_switch[3], _LastStatus.fault_motor_load_switch[3]);
        update |= PrintFlag(_StatusWindow, 14, 21, "5", "_", msg->fault_motor_load_switch[4], _LastStatus.fault_motor_load_switch[4]);
        _LastStatus = *msg;
        _StatusCounter++;
        auto now = std::chrono::system_clock::now();
        if (100 < std::chrono::duration_cast<std::chrono::milliseconds>(now - _StatusLastUpdate).count()) {
            mvwprintw(_StatusWindow, 15, 3, "count=%d", _StatusCounter);
            _StatusLastUpdate = now;
            update = true;
        }
        if (update == true) {
            wrefresh(_StatusWindow);
        }
    }

    bool PrintFlag(WINDOW *window, int y, int x, const char *true_text, const char *false_text, bool current, bool previous) {
        if (_Initialized == false) {
            if (current && !previous) {
                mvwaddstr(window, y, x, true_text);
                return true;
            } else if (!current && previous) {
                mvwaddstr(window, y, x, false_text);
                return true;
            }
            return false;
        } else {
            mvwaddstr(window, y, x, current ? true_text : false_text);
            return true;
        }
    }

    void InitializeAdc2Window(void) {
        _Adc2Window = newwin(6, 32, 19, 2);
        wborder(_Adc2Window, 0, 0, 0, 0, 0, 0, 0, 0);
        mvwaddstr(_Adc2Window, 0, 3, "  [Adc2]");
        mvwaddstr(_Adc2Window, 2, 3, "DC48V   [       ] mV");
        mvwaddstr(_Adc2Window, 3, 3, "Dribble [       ] mV");
        mvwaddstr(_Adc2Window, 4, 3, "Dribble [       ] mA");
        wrefresh(_Adc2Window);
    }

    void UpdateAdc2(void) {
        if (_LastAdc2) {
            mvwprintw(_Adc2Window, 2, 13, "%5d", (int)(_LastAdc2->dc48v_voltage * 1000));
            mvwprintw(_Adc2Window, 3, 13, "%5d", (int)(_LastAdc2->dribble_voltage * 1000));
            mvwprintw(_Adc2Window, 4, 13, "%5d", (int)(_LastAdc2->dribble_current * 1000));
            wrefresh(_Adc2Window);
            _LastAdc2 = nullptr;
        } else {
            /*mvwaddstr(_Adc2Window, 2, 13, "_____");
            mvwaddstr(_Adc2Window, 3, 13, "_____");
            wrefresh(_Adc2Window);*/
        }
    }

    void InitializeMotionWindow(void) {
        _MotionWindow = newwin(32, 34, 1, 35);
        wborder(_MotionWindow, 0, 0, 0, 0, 0, 0, 0, 0);
        mvwaddstr(_MotionWindow, 0, 7, " [Motion]");
        mvwaddstr(_MotionWindow, 2, 3, "         Accel X [        ]");
        mvwaddstr(_MotionWindow, 3, 3, "         Accel Y [        ]");
        mvwaddstr(_MotionWindow, 4, 3, "         Accel Z [        ]");
        mvwaddstr(_MotionWindow, 5, 3, "          Gyro X [        ]");
        mvwaddstr(_MotionWindow, 6, 3, "          Gyro Y [        ]");
        mvwaddstr(_MotionWindow, 7, 3, "          Gyro Z [        ]");
        mvwaddstr(_MotionWindow, 8, 3, " Motor 1 Encoder [        ]");
        mvwaddstr(_MotionWindow, 9, 3, " Motor 2 Encoder [        ]");
        mvwaddstr(_MotionWindow, 10, 3, " Motor 3 Encoder [        ]");
        mvwaddstr(_MotionWindow, 11, 3, " Motor 4 Encoder [        ]");
        mvwaddstr(_MotionWindow, 12, 3, "  Motor 1 Meas D [        ]");
        mvwaddstr(_MotionWindow, 13, 3, "  Motor 1 Meas Q [        ]");
        mvwaddstr(_MotionWindow, 14, 3, "  Motor 2 Meas D [        ]");
        mvwaddstr(_MotionWindow, 15, 3, "  Motor 2 Meas Q [        ]");
        mvwaddstr(_MotionWindow, 16, 3, "  Motor 3 Meas D [        ]");
        mvwaddstr(_MotionWindow, 17, 3, "  Motor 3 Meas Q [        ]");
        mvwaddstr(_MotionWindow, 18, 3, "  Motor 4 Meas D [        ]");
        mvwaddstr(_MotionWindow, 19, 3, "  Motor 4 Meas Q [        ]");
        mvwaddstr(_MotionWindow, 20, 3, "   Motor 1 Ref Q [        ]");
        mvwaddstr(_MotionWindow, 21, 3, "   Motor 2 Ref Q [        ]");
        mvwaddstr(_MotionWindow, 22, 3, "   Motor 3 Ref Q [        ]");
        mvwaddstr(_MotionWindow, 23, 3, "   Motor 4 Ref Q [        ]");
        mvwaddstr(_MotionWindow, 26, 3, "    Perf Counter [        ]");
        wrefresh(_MotionWindow);
    }

    void UpdateMotion(void) {
        if (_MotionMessage) {
            mvwprintw(_MotionWindow, 2, 22, "%6f", _MotionMessage->accelerometer[0]);
            mvwprintw(_MotionWindow, 3, 22, "%6f", _MotionMessage->accelerometer[1]);
            mvwprintw(_MotionWindow, 4, 22, "%6f", _MotionMessage->accelerometer[2]);
            mvwprintw(_MotionWindow, 5, 22, "%6f", _MotionMessage->gyroscope[0]);
            mvwprintw(_MotionWindow, 6, 22, "%6f", _MotionMessage->gyroscope[1]);
            mvwprintw(_MotionWindow, 7, 22, "%6f", _MotionMessage->gyroscope[2]);
            mvwprintw(_MotionWindow, 8, 22, "%6f", _MotionMessage->wheel_velocity[0]);
            mvwprintw(_MotionWindow, 9, 22, "%6f", _MotionMessage->wheel_velocity[1]);
            mvwprintw(_MotionWindow, 10, 22, "%6f", _MotionMessage->wheel_velocity[2]);
            mvwprintw(_MotionWindow, 11, 22, "%6f", _MotionMessage->wheel_velocity[3]);
            mvwprintw(_MotionWindow, 12, 22, "%6f", _MotionMessage->wheel_current_meas_d[0]);
            mvwprintw(_MotionWindow, 13, 22, "%6f", _MotionMessage->wheel_current_meas_q[0]);
            mvwprintw(_MotionWindow, 14, 22, "%6f", _MotionMessage->wheel_current_meas_d[1]);
            mvwprintw(_MotionWindow, 15, 22, "%6f", _MotionMessage->wheel_current_meas_q[1]);
            mvwprintw(_MotionWindow, 16, 22, "%6f", _MotionMessage->wheel_current_meas_d[2]);
            mvwprintw(_MotionWindow, 17, 22, "%6f", _MotionMessage->wheel_current_meas_q[2]);
            mvwprintw(_MotionWindow, 18, 22, "%6f", _MotionMessage->wheel_current_meas_d[3]);
            mvwprintw(_MotionWindow, 19, 22, "%6f", _MotionMessage->wheel_current_meas_q[3]);
            mvwprintw(_MotionWindow, 20, 22, "%6f", _MotionMessage->wheel_current_ref_q[0]);
            mvwprintw(_MotionWindow, 21, 22, "%6f", _MotionMessage->wheel_current_ref_q[1]);
            mvwprintw(_MotionWindow, 22, 22, "%6f", _MotionMessage->wheel_current_ref_q[2]);
            mvwprintw(_MotionWindow, 23, 22, "%6f", _MotionMessage->wheel_current_ref_q[3]);
            mvwprintw(_MotionWindow, 26, 23, "%5d", _MotionMessage->performance_counter);
            wrefresh(_MotionWindow);
            _MotionMessage = nullptr;
        } else {
        }
    }

    void InitializeBatteryWindow(void) {
        _BatteryWindow = newwin(7, 32, 26, 2);
        wborder(_BatteryWindow, 0, 0, 0, 0, 0, 0, 0, 0);
        mvwaddstr(_BatteryWindow, 0, 3, "  [Adc3]");
        mvwaddstr(_BatteryWindow, 2, 3, "    Present [ ]");
        mvwaddstr(_BatteryWindow, 3, 3, "    Voltage [       ] mV");
        mvwaddstr(_BatteryWindow, 4, 3, "    Current [       ] mA");
        mvwaddstr(_BatteryWindow, 5, 3, "Temperature [       ] degC");
        wrefresh(_BatteryWindow);
    }

    void UpdateBattery(void) {
        if (_BatteryMessage) {
            mvwaddstr(_BatteryWindow, 2, 16, _BatteryMessage->present ? "X" : "_");
            mvwprintw(_BatteryWindow, 3, 16, "%6d", static_cast<int>(_BatteryMessage->voltage * 1000));
            mvwprintw(_BatteryWindow, 4, 16, "%6d", static_cast<int>(_BatteryMessage->current * -1000));
            mvwprintw(_BatteryWindow, 5, 16, "%6.2f", _BatteryMessage->temperature);
            wrefresh(_BatteryWindow);
            _BatteryMessage = nullptr;
        } else {
        }
    }

    bool _Initialized = false;

    WINDOW *_StatusWindow;
    WINDOW *_Adc2Window;
    WINDOW *_MotionWindow;
    WINDOW *_BatteryWindow;
    int _StatusCounter = 0;
    std::chrono::system_clock::time_point _StatusLastUpdate;

    phoenix_msgs::msg::StreamDataStatus _LastStatus;
    std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> _LastAdc2;

    rclcpp::TimerBase::SharedPtr _Adc2Timer;
    rclcpp::TimerBase::SharedPtr _BatteryTimer;
    rclcpp::TimerBase::SharedPtr _MotionTimer;

    std::shared_ptr<sensor_msgs::msg::BatteryState> _BatteryMessage;
    std::shared_ptr<phoenix_msgs::msg::StreamDataMotion> _MotionMessage;

    rclcpp::Subscription<phoenix_msgs::msg::StreamDataStatus>::SharedPtr _StatusSubscriber;
    rclcpp::Subscription<phoenix_msgs::msg::StreamDataAdc2>::SharedPtr _Adc2Subscriber;
    rclcpp::Subscription<phoenix_msgs::msg::StreamDataMotion>::SharedPtr _MotionSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr _BatterySubscriber;
};

int main(int argc, char *argv[]) {
    initscr();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusTestNode>());
    rclcpp::shutdown();
    endwin();
    return 0;
}
