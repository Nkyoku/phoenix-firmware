#include <QtCore/QCoreApplication>
#include <QtWidgets/QApplication>
#include <QtCore/QDebug>
#include <rclcpp/rclcpp.hpp>
#include "main_window.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    MainWindow win;
    win.show();
    int exit_code = app.exec();
    rclcpp::shutdown();
    return exit_code;
}
