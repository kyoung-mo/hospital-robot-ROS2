#include "mainwindow.h"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);

    MainWindow w;
    w.show();

    int r = a.exec();
    rclcpp::shutdown();
    return r;
}