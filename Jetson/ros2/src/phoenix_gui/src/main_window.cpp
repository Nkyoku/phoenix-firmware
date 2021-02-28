#include "main_window.hpp"
#include "ui_main_window.h"
#include <QtCore/QDebug>
#include <QtCore/QTimer>
#include <QtCore/QRandomGenerator>
#include <QtGui/QCloseEvent>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include <QtCore/QSettings>
#include <QtCore/QSysInfo>
#include <QtWidgets/QGestureEvent>
#include <chrono>
#include <algorithm>
#include "format_value.hpp"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    // UIを生成する
    _Ui = new Ui_MainWindow;
    _Ui->setupUi(this);
    _Ui->reloadButton->setIcon(QApplication::style()->standardPixmap(QStyle::SP_BrowserReload));
    _Ui->namespaceComboBox->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    _Ui->gamepadComboBox->addItem("None");
    generateTelemetryTreeItems();
    _ImageViewer = new ImageViewerWidget(_Ui->splitter_2);
    _Ui->splitter_2->insertWidget(0, _ImageViewer);
    _ImageViewer->convertBgrToRgb(true);
    _ImageViewer->setMirror(true, true);

    _Pad.scene = new QGraphicsScene(this);
    _Ui->controllerPadGraphics->setScene(_Pad.scene);
    _Ui->controllerPadGraphics->viewport()->grabGesture(Qt::PanGesture);
    _Ui->controllerPadGraphics->viewport()->grabGesture(Qt::PinchGesture);
    _Ui->controllerPadGraphics->viewport()->installEventFilter(this);
    _Pad.cross_h = _Pad.scene->addLine(0, 0, 1, 0, QPen(Qt::black));
    _Pad.cross_v = _Pad.scene->addLine(0, 0, 0, 1, QPen(Qt::black));

    // ウィンドウ位置とスプリッタの位置を戻す
    restoreSettings();

    // ゲームパッド入力スレッドを作成する
    _GamepadThread = new GamepadThread();
    connect(_GamepadThread, &GamepadThread::finished, _GamepadThread, &QObject::deleteLater);
    connect(
        _GamepadThread, &GamepadThread::gamepadConnected, this,
        [this](int deviceId) {
            _Ui->gamepadComboBox->addItem(QString("XInput %1").arg(deviceId), deviceId);
        },
        Qt::QueuedConnection);
    connect(
        _GamepadThread, &GamepadThread::gamepadDisconnected, this,
        [this](int deviceId) {
            int index = _Ui->gamepadComboBox->findData(deviceId);
            if (index == _Ui->gamepadComboBox->currentIndex()) {
                _Ui->gamepadComboBox->setCurrentIndex(0);
            }
            if (0 <= index) {
                _Ui->gamepadComboBox->removeItem(index);
            }
        },
        Qt::QueuedConnection);

    // ROS2のネットワークを監視するためのノードを生成する
    _NetworkAwarenessNode = createNode();

    // テレメトリ更新用のタイマーを生成する
    QTimer *timer = new QTimer(this);
    timer->start(100);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateTelemertyTreeItems);

    // コマンド送信用のタイマーを生成する
    QTimer *timer2 = new QTimer(this);
    timer2->start(100);
    connect(timer2, &QTimer::timeout, this, &MainWindow::sendCommand);

    // Qtのシグナルを接続する
    connect(_Ui->reloadButton, &QPushButton::clicked, this, &MainWindow::reloadNamespaceList);
    connect(_Ui->namespaceComboBox, &QComboBox::currentTextChanged, this, &MainWindow::connectToNodes);
    connect(this, &MainWindow::updateRequest, _ImageViewer, qOverload<>(&ImageViewerWidget::update), Qt::QueuedConnection);
    connect(_Ui->gamepadComboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::connectToGamepad);
    connect(_Ui->saveLogButton, &QPushButton::clicked, this, &MainWindow::startLogging);
    connect(_Ui->stopLogButton, &QPushButton::clicked, this, &MainWindow::stopLogging);

    // リストを更新する
    reloadNamespaceList();

    // ゲームパッド入力スレッドを開始する
    _GamepadThread->start();
}

MainWindow::~MainWindow() {
    quitNodeThread();
    quitGamepadThread();
    saveSettings();
}

void MainWindow::restoreSettings(void) {
    QSettings settings(QSettings::IniFormat, QSettings::UserScope, SETTINGS_ORGANIZATION, SETTINGS_APPLICATION);
    restoreGeometry(settings.value("WindowGeometry").toByteArray());
    restoreState(settings.value("WindowState").toByteArray());
    if (settings.value("WindowMaximized", false).toBool() == true) {
        setWindowState(Qt::WindowMaximized);
    }
    _Ui->splitter->restoreState(settings.value("Splitter1State").toByteArray());
    _Ui->splitter_2->restoreState(settings.value("Splitter2State").toByteArray());
    _Ui->tabWidget->setCurrentIndex(settings.value("TabIndex", _Ui->tabWidget->currentIndex()).toInt());
}

void MainWindow::saveSettings(void) const {
    QSettings settings(QSettings::IniFormat, QSettings::UserScope, SETTINGS_ORGANIZATION, SETTINGS_APPLICATION);
    settings.setValue("WindowGeometry", saveGeometry());
    settings.setValue("WindowState", saveState());
    settings.setValue("WindowMaximized", isMaximized());
    settings.setValue("Splitter1State", _Ui->splitter->saveState());
    settings.setValue("Splitter2State", _Ui->splitter_2->saveState());
    settings.setValue("TabIndex", _Ui->tabWidget->currentIndex());
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
    const QWidget *viewport = _Ui->controllerPadGraphics->viewport();
    if (obj == viewport) {
        if (event->type() == QEvent::Gesture) {
            QGestureEvent *gesture_event = static_cast<QGestureEvent *>(event);
            if (QGesture *gesture = gesture_event->gesture(Qt::PanGesture)) {
                QPanGesture *pan = static_cast<QPanGesture *>(gesture);
                QSize size = viewport->size();
                _Pad.velocity_scale_x = pan->offset().x() / (size.width() * 0.5);
                _Pad.velocity_scale_y = -pan->offset().y() / (size.height() * 0.5);
                if ((pan->state() == Qt::GestureFinished) || (pan->state() == Qt::GestureCanceled)) {
                    _Pad.velocity_scale_x = 0.0;
                    _Pad.velocity_scale_y = 0.0;
                }
            }
            if (QGesture *gesture = gesture_event->gesture(Qt::PinchGesture)) {
                QPinchGesture *pinch = static_cast<QPinchGesture *>(gesture);
                QPinchGesture::ChangeFlags change_flags = pinch->changeFlags();
                if (change_flags & QPinchGesture::RotationAngleChanged) {
                    double d = pinch->rotationAngle() - pinch->lastRotationAngle();
                    if (d <= -180.0) {
                        d += 360.0;
                    } else if (180.0 <= d) {
                        d -= 360.0;
                    }
                    _Pad.velocity_scale_omega -= d;
                }
                if ((pinch->state() == Qt::GestureFinished) || (pinch->state() == Qt::GestureCanceled)) {
                    _Pad.velocity_scale_omega = 0.0;
                }
            }
            return true;
        } else if (event->type() == QEvent::Resize) {
            int width = viewport->width();
            int height = viewport->height();
            _Pad.scene->setSceneRect(0, 0, width, height);
            _Pad.cross_h->setLine(0, height / 2, width, height / 2);
            _Pad.cross_v->setLine(width / 2, 0, width / 2, height);
        }
    }
    return false;
}

void MainWindow::connectToGamepad(int index) {
    qDebug() << "Selection changed " << index;
}

void MainWindow::reloadNamespaceList(void) {
    // TIMEOUTで指定した時間、spin_once()を実行しながら待つ
    // ここでGraphが更新されるのを待つ
    static constexpr auto TIMEOUT = std::chrono::milliseconds(100);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(_NetworkAwarenessNode);
    auto start_time = std::chrono::system_clock::now();
    do {
        exec.spin_once(std::chrono::milliseconds(1));
    } while ((std::chrono::system_clock::now() - start_time) < TIMEOUT);

    // ノード名と名前空間名のリストを取得し、PHOENIX_NODE_PREFIXから始まるノード名を含む名前空間のリストを抽出する
    // ただしGUI_NODE_NAME_PREFIXで始まる名前のノードは検索対象から除外する
    QStringList namespace_list;
    namespace_list.append("");
    auto graph = _NetworkAwarenessNode->get_node_graph_interface();
    auto node_and_namespace_list = graph->get_node_names_and_namespaces();
    for (auto &item : node_and_namespace_list) {
        if (namespace_list.contains(item.second.c_str())) {
            continue;
        }
        QString node_name(item.first.c_str());
        if (node_name.startsWith(PHOENIX_NODE_PREFIX) && !node_name.startsWith(GUI_NODE_NAME_PREFIX)) {
            namespace_list.append(item.second.c_str());
        }
    }
    std::sort(namespace_list.begin(), namespace_list.end());

    // コンボボックスの内容を更新する
    // 現在選択中の項目が新しいリストに存在しないときのみcurrentTextChanged()が発生するようにする
    QString current_text = _Ui->namespaceComboBox->currentText();
    if (namespace_list.contains(current_text)) {
        _Ui->namespaceComboBox->blockSignals(true);
        _Ui->namespaceComboBox->clear();
        _Ui->namespaceComboBox->addItems(namespace_list);
        _Ui->namespaceComboBox->setCurrentText(current_text);
        _Ui->namespaceComboBox->blockSignals(false);
    } else {
        _Ui->namespaceComboBox->clear();
        _Ui->namespaceComboBox->blockSignals(true);
        _Ui->namespaceComboBox->addItems(namespace_list);
        _Ui->namespaceComboBox->blockSignals(false);
    }
}

void MainWindow::connectToNodes(const QString &namespace_name) {
    // 既存のノードとスレッドを終了する
    quitNodeThread();

    if (!namespace_name.isEmpty()) {
        rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // ノードとスレッドを作成する
        _NodeThread = new NodeThread(this, createNode());
        connect(_NodeThread, &NodeThread::finished, _NodeThread, &QObject::deleteLater);

        // batteryトピックを受信するSubscriptionを作成する
        QString battery_topic_name = namespace_name + "/battery";
        _Subscribers.battery = _NodeThread->node()->create_subscription<sensor_msgs::msg::BatteryState>(
            battery_topic_name.toStdString(), qos, [this](const std::shared_ptr<sensor_msgs::msg::BatteryState> msg) { std::atomic_store(&_LastMessages.battery, msg); });

        // stream_data_statusトピックを受信するSubscriptionを作成する
        QString status_topic_name = namespace_name + "/phoenix_status";
        _Subscribers.status = _NodeThread->node()->create_subscription<phoenix_msgs::msg::StreamDataStatus>(
            status_topic_name.toStdString(), qos, [this](const std::shared_ptr<phoenix_msgs::msg::StreamDataStatus> msg) { std::atomic_store(&_LastMessages.status, msg); });

        // stream_data_adc2トピックを受信するSubscriptionを作成する
        QString adc2_topic_name = namespace_name + "/phoenix_adc2";
        _Subscribers.adc2 = _NodeThread->node()->create_subscription<phoenix_msgs::msg::StreamDataAdc2>(
            adc2_topic_name.toStdString(), qos, [this](const std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> msg) {
                if (!_LastMessages.adc2) {
                    std::atomic_store(&_LastMessages.adc2, msg);
                } else {
                    /*std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> last_msg = _LastMessages.adc2;
                    msg->dc48v_voltage = std::max(msg->dc48v_voltage, last_msg->dc48v_voltage);
                    msg->dribble_current = std::max(msg->dribble_current, last_msg->dribble_current);*/
                    std::atomic_store(&_LastMessages.adc2, msg);
                }
            });

        // stream_data_motionトピックを受信するSubscriptionを作成する
        QString motion_topic_name = namespace_name + "/phoenix_motion";
        _Subscribers.motion = _NodeThread->node()->create_subscription<phoenix_msgs::msg::StreamDataMotion>(
            motion_topic_name.toStdString(), qos, [this](const std::shared_ptr<phoenix_msgs::msg::StreamDataMotion> msg) {
                std::atomic_store(&_LastMessages.motion, msg);
                std::shared_ptr<QFile> file = _LogFile;
                if (file) {
                    QTextStream stream(file.get());
                    QChar sep(',');
                    std::shared_ptr<sensor_msgs::msg::BatteryState> battery_msg = _LastMessages.battery;
                    std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2> adc2_msg = _LastMessages.adc2;
                    std::shared_ptr<phoenix_msgs::msg::StreamDataControl> control_msg = _LastMessages.control;
                    float battery_voltage = battery_msg ? battery_msg->voltage : 0.0f;
                    float battery_current = battery_msg ? -battery_msg->current : 0.0f;
                    float dc48v_voltage = adc2_msg ? adc2_msg->dc48v_voltage : 0.0f;
                    stream << (0.001 * _LogFrameNumber) << sep;
                    stream << msg->wheel_velocity[0] << sep;
                    stream << msg->wheel_velocity[1] << sep;
                    stream << msg->wheel_velocity[2] << sep;
                    stream << msg->wheel_velocity[3] << sep;
                    stream << msg->wheel_current_q[0] << sep;
                    stream << msg->wheel_current_q[1] << sep;
                    stream << msg->wheel_current_q[2] << sep;
                    stream << msg->wheel_current_q[3] << sep;
                    stream << control_msg->wheel_velocity_ref[0] << sep;
                    stream << control_msg->wheel_velocity_ref[1] << sep;
                    stream << control_msg->wheel_velocity_ref[2] << sep;
                    stream << control_msg->wheel_velocity_ref[3] << sep;
                    stream << control_msg->wheel_current_ref[0] << sep;
                    stream << control_msg->wheel_current_ref[1] << sep;
                    stream << control_msg->wheel_current_ref[2] << sep;
                    stream << control_msg->wheel_current_ref[3] << sep;
                    stream << control_msg->wheel_energy[0] << sep;
                    stream << control_msg->wheel_energy[1] << sep;
                    stream << control_msg->wheel_energy[2] << sep;
                    stream << control_msg->wheel_energy[3] << sep;
                    stream << control_msg->machine_velocity[0] << sep;
                    stream << control_msg->machine_velocity[1] << sep;
                    stream << control_msg->machine_velocity[2] << sep;
                    stream << msg->accelerometer[0] << sep;
                    stream << msg->accelerometer[1] << sep;
                    stream << msg->accelerometer[2] << sep;
                    stream << msg->gyroscope[0] << sep;
                    stream << msg->gyroscope[1] << sep;
                    stream << msg->gyroscope[2] << sep;
                    stream << dc48v_voltage << sep;
                    stream << battery_voltage << sep;
                    stream << battery_current << Qt::endl;
                    _LogFrameNumber++;
                }
            });

        // stream_data_controlトピックを受信するSubscriptionを作成する
        QString control_topic_name = namespace_name + "/phoenix_control";
        _Subscribers.control = _NodeThread->node()->create_subscription<phoenix_msgs::msg::StreamDataControl>(
            control_topic_name.toStdString(), qos, [this](const std::shared_ptr<phoenix_msgs::msg::StreamDataControl> msg) {
                std::atomic_store(&_LastMessages.control, msg);
            });

        // imageトピックを受信するSubscriptionを作成する
        _Subscribers.image = _NodeThread->node()->create_subscription<sensor_msgs::msg::Image>(
            "/video_source/raw", qos, [this](const std::shared_ptr<sensor_msgs::msg::Image> msg) { _ImageViewer->setImage(msg); emit updateRequest(); });

        QString set_speed_service_name = namespace_name + "/set_speed";
        _Clients.set_speed = _NodeThread->node()->create_client<phoenix_msgs::srv::SetSpeed>(set_speed_service_name.toStdString());

        if (_Clients.set_speed->wait_for_service(std::chrono::milliseconds(1000)) == false) {
            _Clients.set_speed.reset();
            _Ui->enableControllerCheckBox->setChecked(false);
        }

        // スレッドを実行
        _NodeThread->start();
    } else {
        // 画面を初期化する
        _ImageViewer->setImage(nullptr);
        emit updateRequest();
    }
}

void MainWindow::updateTelemertyTreeItems(void) {
    static constexpr int COL = 1;
    if (_LastMessages.battery) {
        //auto msg = std::atomic_exchange(&_LastMessages.battery, std::shared_ptr<sensor_msgs::msg::BatteryState>());
        auto &msg = _LastMessages.battery;
        _TreeItems.battery.present->setText(COL, boolToString(msg->present));
        _TreeItems.battery.voltage->setText(COL, QString::number(msg->voltage, 'f', 3));
        _TreeItems.battery.current->setText(COL, QString::number(msg->current, 'f', 3));
        _TreeItems.battery.temperature->setText(COL, QString::number(msg->temperature, 'f', 3));
    }
    if (_LastMessages.status) {
        static const QStringList dc48v_text_list = {"UV", "OV"};
        static const QStringList motor_text_list = {"1", "2", "3", "4", "5"};
        auto msg = std::atomic_exchange(&_LastMessages.status, std::shared_ptr<phoenix_msgs::msg::StreamDataStatus>());
        _TreeItems.error.module_stop->setText(COL, boolToString(msg->error_module_sleep));
        _TreeItems.error.fpga_stop->setText(COL, boolToString(msg->error_fpga_stop));
        _TreeItems.error.dc48v->setText(COL, boolListToString(dc48v_text_list, msg->error_dc48v_uv, msg->error_dc48v_ov));
        _TreeItems.error.motor_oc->setText(COL, boolArrayToString(motor_text_list, msg->error_motor_oc));
        _TreeItems.error.hall_sensor->setText(COL, boolArrayToString(motor_text_list, msg->error_motor_hall_sensor));
        _TreeItems.fault.adc2_timeout->setText(COL, boolToString(msg->fault_adc2_timeout));
        _TreeItems.fault.imu_timeout->setText(COL, boolToString(msg->fault_imu_timeout));
        _TreeItems.fault.motor_oc->setText(COL, boolArrayToString(motor_text_list, msg->fault_motor_oc));
        _TreeItems.fault.motor_ot->setText(COL, boolArrayToString(motor_text_list, msg->fault_motor_ot));
        _TreeItems.fault.load_switch->setText(COL, boolArrayToString(motor_text_list, msg->fault_motor_load_switch));
    }
    if (_LastMessages.adc2) {
        //auto msg = std::atomic_exchange(&_LastMessages.adc2, std::shared_ptr<phoenix_msgs::msg::StreamDataAdc2>());
        auto &msg = _LastMessages.adc2;
        _TreeItems.adc2.dc48v_voltage->setText(COL, QString::number(msg->dc48v_voltage, 'f', 3));
        _TreeItems.adc2.dribble_voltage->setText(COL, QString::number(msg->dribble_voltage, 'f', 3));
        _TreeItems.adc2.dribble_current->setText(COL, QString::number(msg->dribble_current, 'f', 3));
    }
    if (_LastMessages.motion) {
        //auto msg = std::atomic_exchange(&_LastMessages.motion, std::shared_ptr<phoenix_msgs::msg::StreamDataMotion>());
        auto &msg = _LastMessages.motion;
        for (int index = 0; index < 3; index++) {
            _TreeItems.motion.accelerometer[index]->setText(COL, QString::number(msg->accelerometer[index], 'f', 3));
            _TreeItems.motion.gyroscope[index]->setText(COL, QString::number(msg->gyroscope[index], 'f', 3));
        }
        for (int index = 0; index < 4; index++) {
            _TreeItems.motion.wheel_velocity[index]->setText(COL, QString::number(msg->wheel_velocity[index], 'f', 3));
            _TreeItems.motion.wheel_current_d[index]->setText(COL, QString::number(msg->wheel_current_d[index], 'f', 3));
            _TreeItems.motion.wheel_current_q[index]->setText(COL, QString::number(msg->wheel_current_q[index], 'f', 3));
        }
    }
    if (_LastMessages.control) {
        //auto msg = std::atomic_exchange(&_LastMessages.control, std::shared_ptr<phoenix_msgs::msg::StreamDataControl>());
        auto &msg = _LastMessages.control;
        _TreeItems.control.perf_counter->setText(COL, QString::number(msg->performance_counter));
        for (int index = 0; index < 4; index++) {
            _TreeItems.control.wheel_velocity_ref[index]->setText(COL, QString::number(msg->wheel_velocity_ref[index], 'f', 3));
            _TreeItems.control.wheel_current_ref[index]->setText(COL, QString::number(msg->wheel_current_ref[index], 'f', 3));
            _TreeItems.control.wheel_energy[index]->setText(COL, QString::number(msg->wheel_energy[index], 'f', 4));
        }
        for (int index = 0; index < 3; index++) {
            _TreeItems.control.machine_velocity[index]->setText(COL, QString::number(msg->machine_velocity[index], 'f', 3));
        }
    }
}

void MainWindow::generateTelemetryTreeItems(void) {
    // stream_data_statusの内容を表示するアイテムを作成する
    auto status_top = new QTreeWidgetItem(_Ui->telemetryTree, {"Status"});
    auto error_flags = new QTreeWidgetItem(status_top, {"Error Flags"});
    _TreeItems.error.module_stop = new QTreeWidgetItem(error_flags, {"Module Stop"});
    _TreeItems.error.fpga_stop = new QTreeWidgetItem(error_flags, {"FPGA Stop"});
    _TreeItems.error.dc48v = new QTreeWidgetItem(error_flags, {"DC48V"});
    _TreeItems.error.motor_oc = new QTreeWidgetItem(error_flags, {"Motor OC"});
    _TreeItems.error.hall_sensor = new QTreeWidgetItem(error_flags, {"Hall Sensor"});
    auto fault_flags = new QTreeWidgetItem(status_top, {"Fault Flags"});
    _TreeItems.fault.adc2_timeout = new QTreeWidgetItem(fault_flags, {"ADC2 Timeout"});
    _TreeItems.fault.imu_timeout = new QTreeWidgetItem(fault_flags, {"IMU Timeout"});
    _TreeItems.fault.motor_oc = new QTreeWidgetItem(fault_flags, {"Motor OC"});
    _TreeItems.fault.motor_ot = new QTreeWidgetItem(fault_flags, {"Motor OT"});
    _TreeItems.fault.load_switch = new QTreeWidgetItem(fault_flags, {"Load Switch"});

    // batteryの内容を表示するアイテムを作成する
    auto battery_top = new QTreeWidgetItem(_Ui->telemetryTree, {"Battery"});
    _TreeItems.battery.present = new QTreeWidgetItem(battery_top, {"Battery Present"});
    _TreeItems.battery.voltage = new QTreeWidgetItem(battery_top, {"Battery Voltage", "", "V"});
    _TreeItems.battery.current = new QTreeWidgetItem(battery_top, {"Battery Current", "", "A"});
    _TreeItems.battery.temperature = new QTreeWidgetItem(battery_top, {"Board Temperature", "", u8"\u2103"});

    // stream_data_adc2の内容を表示するアイテムを作成する
    auto adc2_top = new QTreeWidgetItem(_Ui->telemetryTree, {"ADC2"});
    _TreeItems.adc2.dc48v_voltage = new QTreeWidgetItem(adc2_top, {"DC48V Voltage", "", "V"});
    _TreeItems.adc2.dribble_voltage = new QTreeWidgetItem(adc2_top, {"Dribble Voltage", "", "V"});
    _TreeItems.adc2.dribble_current = new QTreeWidgetItem(adc2_top, {"Dribble Current", "", "A"});

    // stream_data_motionの内容を表示するアイテムを作成する
    auto motion_top = new QTreeWidgetItem(_Ui->telemetryTree, {"Motion"});
    _TreeItems.motion.accelerometer[0] = new QTreeWidgetItem(motion_top, {"Accelerometer X", "", u8"m/s\u00B2"});
    _TreeItems.motion.accelerometer[1] = new QTreeWidgetItem(motion_top, {"Accelerometer Y", "", u8"m/s\u00B2"});
    _TreeItems.motion.accelerometer[2] = new QTreeWidgetItem(motion_top, {"Accelerometer Z", "", u8"m/s\u00B2"});
    _TreeItems.motion.gyroscope[0] = new QTreeWidgetItem(motion_top, {"Gyroscope X", "", "rad/s"});
    _TreeItems.motion.gyroscope[1] = new QTreeWidgetItem(motion_top, {"Gyroscope Y", "", "rad/s"});
    _TreeItems.motion.gyroscope[2] = new QTreeWidgetItem(motion_top, {"Gyroscope Z", "", "rad/s"});
    _TreeItems.motion.wheel_velocity[0] = new QTreeWidgetItem(motion_top, {"Wheel 1 Velocity", "", "m/s"});
    _TreeItems.motion.wheel_velocity[1] = new QTreeWidgetItem(motion_top, {"Wheel 2 Velocity", "", "m/s"});
    _TreeItems.motion.wheel_velocity[2] = new QTreeWidgetItem(motion_top, {"Wheel 3 Velocity", "", "m/s"});
    _TreeItems.motion.wheel_velocity[3] = new QTreeWidgetItem(motion_top, {"Wheel 4 Velocity", "", "m/s"});
    _TreeItems.motion.wheel_current_d[0] = new QTreeWidgetItem(motion_top, {"Wheel 1 Current D", "", "A"});
    _TreeItems.motion.wheel_current_q[0] = new QTreeWidgetItem(motion_top, {"Wheel 1 Current Q", "", "A"});
    _TreeItems.motion.wheel_current_d[1] = new QTreeWidgetItem(motion_top, {"Wheel 2 Current D", "", "A"});
    _TreeItems.motion.wheel_current_q[1] = new QTreeWidgetItem(motion_top, {"Wheel 2 Current Q", "", "A"});
    _TreeItems.motion.wheel_current_d[2] = new QTreeWidgetItem(motion_top, {"Wheel 3 Current D", "", "A"});
    _TreeItems.motion.wheel_current_q[2] = new QTreeWidgetItem(motion_top, {"Wheel 3 Current Q", "", "A"});
    _TreeItems.motion.wheel_current_d[3] = new QTreeWidgetItem(motion_top, {"Wheel 4 Current D", "", "A"});
    _TreeItems.motion.wheel_current_q[3] = new QTreeWidgetItem(motion_top, {"Wheel 4 Current Q", "", "A"});

    // stream_data_controlの内容を表示するアイテムを作成する
    auto control_top = new QTreeWidgetItem(_Ui->telemetryTree, {"Control"});
    _TreeItems.control.perf_counter = new QTreeWidgetItem(control_top, {"Performance Counter", "", "Cycles"});
    _TreeItems.control.wheel_velocity_ref[0] = new QTreeWidgetItem(control_top, {"Wheel 1 Velocity Ref", "", "m/s"});
    _TreeItems.control.wheel_velocity_ref[1] = new QTreeWidgetItem(control_top, {"Wheel 2 Velocity Ref", "", "m/s"});
    _TreeItems.control.wheel_velocity_ref[2] = new QTreeWidgetItem(control_top, {"Wheel 3 Velocity Ref", "", "m/s"});
    _TreeItems.control.wheel_velocity_ref[3] = new QTreeWidgetItem(control_top, {"Wheel 4 Velocity Ref", "", "m/s"});
    _TreeItems.control.wheel_current_ref[0] = new QTreeWidgetItem(control_top, {"Wheel 1 Current Ref", "", "A"});
    _TreeItems.control.wheel_current_ref[1] = new QTreeWidgetItem(control_top, {"Wheel 2 Current Ref", "", "A"});
    _TreeItems.control.wheel_current_ref[2] = new QTreeWidgetItem(control_top, {"Wheel 3 Current Ref", "", "A"});
    _TreeItems.control.wheel_current_ref[3] = new QTreeWidgetItem(control_top, {"Wheel 4 Current Ref", "", "A"});
    _TreeItems.control.wheel_energy[0] = new QTreeWidgetItem(control_top, {"Wheel 1 Energy", "", "J"});
    _TreeItems.control.wheel_energy[1] = new QTreeWidgetItem(control_top, {"Wheel 2 Energy", "", "J"});
    _TreeItems.control.wheel_energy[2] = new QTreeWidgetItem(control_top, {"Wheel 3 Energy", "", "J"});
    _TreeItems.control.wheel_energy[3] = new QTreeWidgetItem(control_top, {"Wheel 4 Energy", "", "J"});
    _TreeItems.control.machine_velocity[0] = new QTreeWidgetItem(control_top, {"Machine Velocity X", "", "m/s"});
    _TreeItems.control.machine_velocity[1] = new QTreeWidgetItem(control_top, {"Machine Velocity Y", "", "m/s"});
    _TreeItems.control.machine_velocity[2] = new QTreeWidgetItem(control_top, {u8"Machine Velocity \u03C9", "", "rad/s"});

    // カラム幅を文字に合わせてリサイズする
    _Ui->telemetryTree->expandAll();
    _Ui->telemetryTree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    _Ui->telemetryTree->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    _Ui->telemetryTree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
}

void MainWindow::startLogging(void) {
    QString path = QFileDialog::getSaveFileName(this, "Save Log File", "", "CSV (*.csv)");
    if (!path.isEmpty()) {
        auto file = std::make_shared<QFile>(path);
        if (file->open(QIODevice::WriteOnly | QIODevice::Text)) {
            _LogFrameNumber = 0;
            _LogFile = file;
            QTextStream stream(_LogFile.get());
            stream << "Time";
            for (int index = 1; index <= 4; index++)
                stream << ",Velocity " << index << " Meas";
            for (int index = 1; index <= 4; index++)
                stream << ",Current " << index << " Meas";
            for (int index = 1; index <= 4; index++)
                stream << ",Velocity " << index << " Ref";
            for (int index = 1; index <= 4; index++)
                stream << ",Current " << index << " Ref";
            for (int index = 1; index <= 4; index++)
                stream << ",Energy " << index;
            stream << ",Machine Vx,Machine Vy,Machine Omega";
            stream << ",Accel X,Accel Y,Accel Z";
            stream << ",Gyro X,Gyro Y,Gyro Z";
            stream << ",DC48V,Battery Voltage,Battery Current\n";
        }
        _Ui->saveLogButton->setEnabled(false);
        _Ui->stopLogButton->setEnabled(true);
    } else {
        QMessageBox::warning(this, "Telemtry Logging", "Failed to open file.");
    }
}

void MainWindow::stopLogging(void) {
    _LogFile.reset();
    _Ui->saveLogButton->setEnabled(true);
    _Ui->stopLogButton->setEnabled(false);
}

void MainWindow::quitNodeThread(void) {
    if (_NodeThread != nullptr) {
        // Subscriptionを破棄する
        _Subscribers.battery.reset();
        _Subscribers.status.reset();
        _Subscribers.adc2.reset();
        _Subscribers.motion.reset();
        _Subscribers.control.reset();
        _Subscribers.image.reset();

        // Clientsを破棄する
        _Clients.set_speed.reset();

        // スレッドを終了する
        // deleteはdeleteLater()スロットにより行われるのでここでする必要はない
        _NodeThread->requestInterruption();
        _NodeThread->quit();
        _NodeThread->wait(std::chrono::milliseconds(NodeThread::QUIT_TIMEOUT));
        _NodeThread = nullptr;
    }
}

void MainWindow::sendCommand(void) {
    if ((_Clients.set_speed) && _Ui->enableControllerCheckBox->isChecked()) {
        auto request = std::make_shared<phoenix_msgs::srv::SetSpeed::Request>();
        bool gamepad_selected;
        int gamepad_device_id = _Ui->gamepadComboBox->currentData().toInt(&gamepad_selected);
        if (gamepad_selected) {
            auto input_state = _GamepadThread->inputState(gamepad_device_id);
            if (input_state) {
                request->speed_x = 4.0f * input_state->leftStickX;
                request->speed_y = 4.0f * input_state->leftStickY;
                request->speed_omega = -10.0f * input_state->rightStickX;
                request->dribble_power = -input_state->rightTrigger;
            }
        } else {
            request->speed_x = _Pad.velocity_scale_x;
            request->speed_y = _Pad.velocity_scale_y;
            request->speed_omega = _Pad.velocity_scale_omega * 0.1;
        }
        auto result = _Clients.set_speed->async_send_request(request);
        result.wait();
    }
}

std::shared_ptr<rclcpp::Node> MainWindow::createNode(void) {
    // ノードを作成する
    // ノード名の被りを防止するため末尾に乱数を付与する
    QString node_name = QString("%1%2").arg(GUI_NODE_NAME_PREFIX).arg(QRandomGenerator::global()->generate(), 8, 16, QLatin1Char('0'));
    QString namespace_name = QSysInfo::machineHostName();
    return std::make_shared<rclcpp::Node>(node_name.toStdString(), namespace_name.toStdString());
}

void MainWindow::quitGamepadThread(void) {
    _GamepadThread->requestInterruption();
    _GamepadThread->quit();
    _GamepadThread->wait(std::chrono::milliseconds(NodeThread::QUIT_TIMEOUT));
    _GamepadThread = nullptr;
}
