#pragma once

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QTreeWidgetItem>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsEllipseItem>
#include <QtWidgets/QGraphicsLineItem>
#include <QtCore/QFile>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <phoenix_msgs/msg/stream_data_status.hpp>
#include <phoenix_msgs/msg/stream_data_adc2.hpp>
#include <phoenix_msgs/msg/stream_data_motion.hpp>
#include <phoenix_msgs/srv/clear_error.hpp>
#include <phoenix_msgs/srv/set_speed.hpp>
#include <phoenix_msgs/srv/program_nios.hpp>
#include <phoenix_msgs/srv/program_fpga.hpp>
#include "node_thread.hpp"
#include "image_viewer.hpp"
#include "gamepad_thread.hpp"

class Ui_MainWindow;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

private:
    /**
     *  設定ファイルから状態を復元する
     */
    void restoreSettings(void);

    /*
     * 設定ファイルに状態を保存する
     */
    void saveSettings(void) const;
    
    //void closeEvent(QCloseEvent *event) override;

    virtual bool eventFilter(QObject *obj, QEvent *event) override;
    
    Q_SLOT void connectToGamepad(int index);

    Q_SLOT void reloadNamespaceList(void);

    Q_SLOT void connectToNodes(const QString &namespace_name);

    Q_SLOT void updateTelemertyTreeItems(void);

    void generateTelemetryTreeItems(void);

    Q_SLOT void startLogging(void);

    Q_SLOT void stopLogging(void);

    Q_SLOT void quitNodeThread(void);

    Q_SIGNAL void updateRequest(void);

    Q_SLOT void sendCommand(void);

    Q_SLOT void programNios(void);

    Q_SLOT void programFpga(void);

    static std::shared_ptr<rclcpp::Node> createNode(void);

    void quitGamepadThread(void);

    /// Qt Designerで作成したUI要素
    Ui_MainWindow *_Ui;

    ImageViewerWidget *_ImageViewer;

    struct {
        QGraphicsScene *scene;
        QGraphicsLineItem *cross_h, *cross_v;
        double velocity_scale_x; // -1.0 ~ +1.0
        double velocity_scale_y; // -1.0 ~ +1.0
        double velocity_scale_omega; // degree
    } _Pad;

    

    /// telemetryTreeに表示する項目
    struct TreeItems_t {
        struct Error_t {
            QTreeWidgetItem *module_stop, *fpga_stop, *dc48v, *motor_oc, *hall_sensor;
        } error;
        struct Fault_t {
            QTreeWidgetItem *adc2_timeout, *imu_timeout, *motor_ot, *motor_oc, *load_switch;
        } fault;
        struct Battery_t {
            QTreeWidgetItem *present, *voltage, *current, *temperature;
        } battery;
        struct Adc2_t {
            QTreeWidgetItem *dc48v_voltage, *dribble_voltage, *dribble_current;
        } adc2;
        struct Motion_t {
            QTreeWidgetItem *accelerometer[3], *gyroscope[3];
            QTreeWidgetItem *wheel_velocity[4], *wheel_current_d[4], *wheel_current_q[4];
        } motion;
        struct Control_t {
            QTreeWidgetItem *perf_counter, *wheel_velocity_ref[4], *wheel_current_ref[4], *wheel_current_limit[4], *machine_velocity[3], *slip_flags;
        } control;
    } _TreeItems;

    /// ゲームパッド入力を処理するスレッド
    GamepadThread *_GamepadThread = nullptr;

    /// テレメトリのログを保存するファイル
    std::shared_ptr<QFile> _LogFile;

    /// テレメトリのログに含まれるフレーム番号
    uint32_t _LogFrameNumber = 0;

    /// ROS2のネットワークを監視するためのノード
    std::shared_ptr<rclcpp::Node> _NetworkAwarenessNode;

    /// ノードの処理を行うスレッド
    NodeThread *_NodeThread = nullptr;

    // 作成したSubscriptionを保持する
    struct Subscriptions_t {
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery;
        rclcpp::Subscription<phoenix_msgs::msg::StreamDataStatus>::SharedPtr status;
        rclcpp::Subscription<phoenix_msgs::msg::StreamDataAdc2>::SharedPtr adc2;
        rclcpp::Subscription<phoenix_msgs::msg::StreamDataMotion>::SharedPtr motion;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image;
    } _Subscribers;

    /// 受信した最後のトピックを保持する
    struct LastMessages_t {
        std::shared_ptr<sensor_msgs::msg::BatteryState> battery;
        std::shared_ptr<phoenix_msgs::msg::StreamDataStatus> status;
        std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> adc2;
        std::shared_ptr<phoenix_msgs::msg::StreamDataMotion> motion;
    } _LastMessages;

    struct Clients_t{
        rclcpp::Client<phoenix_msgs::srv::SetSpeed>::SharedPtr set_speed;
        std::shared_future<std::shared_ptr<phoenix_msgs::srv::SetSpeed_Response>> set_speed_future;
        rclcpp::Client<phoenix_msgs::srv::ProgramNios>::SharedPtr program_nios;
        rclcpp::Client<phoenix_msgs::srv::ProgramFpga>::SharedPtr program_fpga;
    } _Clients;

    /// GUIのノード名の頭に付ける文字列
    static constexpr char GUI_NODE_NAME_PREFIX[] = "phoenix_gui_";

    /// Phoenixのノード名の頭に付ける文字列
    static constexpr char PHOENIX_NODE_PREFIX[] = "phoenix_";

    /// 設定ファイルの組織名
    static constexpr char SETTINGS_ORGANIZATION[] = "KIKS";

    /// 設定ファイルのアプリケーション名
    static constexpr char SETTINGS_APPLICATION[] = "PhoenixGUI";
};
