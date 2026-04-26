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
    void handleBattery(int id, float p);
    void onRoomButtonClicked();
    void onPatrolButtonClicked();
    void onTimerTick();
    void handleEvent(QString type, QString message);

private:
    // UI 위젯 (로봇 2번용 tb2View, r2Status, r2BattBar 추가)
    QLabel *mapView, *d435View, *tb1View, *tb2View;
    QLabel *r1Status, *r2Status;
    QProgressBar *r1BattBar, *r2BattBar;
    QListWidget *logList;
    QPushButton *drawBtn, *clearBtn, *patrolBtn;

    QTimer *timer;
    RosThread *rosThread;
    QPixmap mapPixmap;

    // 제어 대상 로봇 선택 (기본값 1번 로봇)
    int selectedRobotId = 1;

    // 로봇 1번 데이터
    double r1x = 4.437;
    double r1y = -0.969;
    QString r1Target = "IDLE";
    std::vector<Point> r1PatrolPath;
    std::vector<Point> r1TrailPath;  // 실시간 이동 궤적
    int r1PatrolIdx = 0;
    bool r1IsPatrolling = false;

    // 로봇 2번 데이터 (추가됨)
    double r2x = 4.437;
    double r2y = -0.969;
    QString r2Target = "IDLE";
    std::vector<Point> r2PatrolPath;
    std::vector<Point> r2TrailPath;  // 실시간 이동 궤적
    int r2PatrolIdx = 0;
    bool r2IsPatrolling = false;

    static constexpr int MAX_TRAIL_SIZE = 300;

    bool isDrawingMode = false;

    void drawMap();
    void addLog(QString msg, int robotId = 0);
    QString getRobotStatusText(QString target);
    QPointF mapToPixel(double wx, double wy, int sw, int sh);
};
#endif
