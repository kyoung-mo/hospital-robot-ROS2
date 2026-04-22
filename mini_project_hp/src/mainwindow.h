#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QListWidget>
#include <QTimer>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QPointF>
#include <vector>
#include <QMouseEvent>
#include "ros_thread.h"

struct Point { double x; double y; };

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void mousePressEvent(QMouseEvent *event) override;

public slots:
    void updateCamera(int camId, QImage img);
    void handlePose(int id, double x, double y);
    void handleBattery(int id, float percentage);
    void onRoomButtonClicked();
    void onPatrolButtonClicked();
    void onTimerTick();
    void handleEvent(QString type, QString message);

private:
    QLabel *mapView, *d435View, *tb1View, *r1Status;
    QProgressBar *r1BattBar;
    QListWidget *logList;
    QPushButton *drawBtn, *clearBtn, *patrolBtn;

    QTimer *timer;
    RosThread *rosThread;
    QPixmap mapPixmap;

    // 시작 좌표: 스테이션 1 고정
    double r1x = 4.437;
    double r1y = -0.969;
    QString r1Target = "IDLE";

    std::vector<Point> r1PatrolPath;
    int r1PatrolIdx = 0;
    bool r1IsPatrolling = false;
    bool isDrawingMode = false;

    void drawMap();
    void addLog(QString msg, int robotId = 0);
    QString getRobotStatusText(QString target);
    QPointF mapToPixel(double wx, double wy, int sw, int sh);
};
#endif