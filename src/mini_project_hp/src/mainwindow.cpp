#include "mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPainter>
#include <QGroupBox>
#include <QTime>
#include <QMessageBox>
#include <QListWidgetItem>
#include <QSizePolicy>
#include <cmath>

// 좌표 정보 헬퍼 함수
Point getCoord(QString room) {
    if (room == "START") return {0.01, 0.01};
    if (room == "101")   return {1.686, -0.663};
    if (room == "102")   return {2.450, -0.546};
    if (room == "S1")    return {4.437, -0.969};
    if (room == "S2")    return {5.134, -0.996};
    if (room == "waste") return {5.379, 1.027};
    return {0, 0};
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("Smart Hospital Logistics Control System (Dual Robot Mode)");
    setMinimumSize(1200, 900);
    resize(1500, 1000);

    // 전체 스타일 설정
    this->setStyleSheet(
        "QMainWindow { background-color: #F8FAFC; }"
        "QGroupBox { font-family: 'Segoe UI', sans-serif; font-weight: bold; font-size: 15px; border: 2px solid #CBD5E1; border-radius: 12px; background-color: #FFFFFF; margin-top: 25px; padding-top: 35px; }"
        "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 15px; top: 5px; padding: 4px 12px; background-color: #0D9488; color: white; border-radius: 6px; }"
        "QPushButton { background-color: #0F766E; color: white; border-radius: 8px; font-weight: bold; padding: 8px; border: none; font-size: 13px; min-height: 35px; }"
        "QPushButton:hover { background-color: #14B8A6; }"
        "QProgressBar { border: 1px solid #E2E8F0; border-radius: 6px; text-align: center; background: #F1F5F9; font-weight: bold; height: 18px; }"
        "QProgressBar::chunk { background-color: #10B981; border-radius: 5px; }"
        );

    QWidget *central = new QWidget(this);
    QHBoxLayout *mainHorizontalLayout = new QHBoxLayout(central);

    // --- [왼쪽 영역] 카메라 및 지도 ---
    QVBoxLayout *leftLayout = new QVBoxLayout();

    // 카메라 그룹 (D435 / 로봇 1 / 로봇 2)
    QGroupBox *camGroup = new QGroupBox("🎥 실시간 모니터링 (D435 / 로봇 1 / 로봇 2)");
    QHBoxLayout *camLayout = new QHBoxLayout();

    d435View = new QLabel("D435 Waiting...");
    tb1View = new QLabel("로봇 1 Waiting...");
    tb2View = new QLabel("로봇 2 Waiting...");

    QString camStyle = "background: #0F172A; border-radius: 10px; color: #94A3B8; font-weight: bold; border: 1px solid #E2E8F0;";
    for(auto v : {d435View, tb1View, tb2View}) {
        // [밀림 방지] 사이즈 정책을 Ignored로 설정하여 영상 크기에 영향받지 않게 함
        v->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        v->setMinimumHeight(200);
        v->setStyleSheet(camStyle);
        v->setAlignment(Qt::AlignCenter);
        camLayout->addWidget(v);
    }
    camGroup->setLayout(camLayout);
    camGroup->setMaximumHeight(350); // 상단 영역 높이 제한

    mapView = new QLabel();
    mapView->setStyleSheet("background: #FFFFFF; border: 2px solid #CBD5E1; border-radius: 15px;");
    mapView->setAlignment(Qt::AlignCenter);
    mapView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    leftLayout->addWidget(camGroup, 1);
    leftLayout->addWidget(mapView, 2);

    // --- [오른쪽 영역] 제어판 (너비 고정) ---
    QWidget *rightWidget = new QWidget();
    rightWidget->setFixedWidth(350); // [밀림 방지] 제어판 너비를 350px로 고정
    QVBoxLayout *rightPanel = new QVBoxLayout(rightWidget);

    // 시스템 상태창
    QGroupBox *statGroup = new QGroupBox("📡 시스템 상태");
    QVBoxLayout *statLayout = new QVBoxLayout();

    QLabel *r1Title = new QLabel("🩺 로봇 1");
    r1Title->setStyleSheet("color: #0D9488; font-size: 14px; font-weight: bold;");
    r1Status = new QLabel("상태: 온라인");
    r1BattBar = new QProgressBar(); r1BattBar->setValue(100);

    QLabel *r2Title = new QLabel("🩺 로봇 2");
    r2Title->setStyleSheet("color: #2563EB; font-size: 14px; font-weight: bold; margin-top: 10px;");
    r2Status = new QLabel("상태: 온라인");
    r2BattBar = new QProgressBar(); r2BattBar->setValue(100);
    r2BattBar->setStyleSheet("QProgressBar::chunk { background-color: #3B82F6; }");

    statLayout->addWidget(r1Title); statLayout->addWidget(r1Status); statLayout->addWidget(r1BattBar);
    statLayout->addWidget(r2Title); statLayout->addWidget(r2Status); statLayout->addWidget(r2BattBar);
    statGroup->setLayout(statLayout);

    // 통합 제어창
    QGroupBox *cmdGroup = new QGroupBox("🚑 통합 제어");
    QVBoxLayout *cmdLayout = new QVBoxLayout();

    QHBoxLayout *selectRow = new QHBoxLayout();
    QPushButton *selR1 = new QPushButton("로봇 1 제어");
    QPushButton *selR2 = new QPushButton("로봇 2 제어");
    connect(selR1, &QPushButton::clicked, this, [=](){ selectedRobotId = 1; addLog("시스템: 로봇 1 제어 모드"); });
    connect(selR2, &QPushButton::clicked, this, [=](){ selectedRobotId = 2; addLog("시스템: 로봇 2 제어 모드"); });
    selectRow->addWidget(selR1); selectRow->addWidget(selR2);
    cmdLayout->addLayout(selectRow);

    drawBtn = new QPushButton("🖊️ 경로 편집");
    clearBtn = new QPushButton("🔄 초기화");
    patrolBtn = new QPushButton("🚀 순찰 시작");
    QHBoxLayout *pathRow = new QHBoxLayout();
    pathRow->addWidget(drawBtn); pathRow->addWidget(clearBtn);
    cmdLayout->addLayout(pathRow); cmdLayout->addWidget(patrolBtn);

    auto createBtn = [&](QString text, QString color) {
        QPushButton *b = new QPushButton(text);
        b->setStyleSheet(QString("background-color: %1;").arg(color));
        connect(b, &QPushButton::clicked, this, &MainWindow::onRoomButtonClicked);
        return b;
    };
    cmdLayout->addWidget(createBtn("🏥 101호 호출", "#0E7490"));
    cmdLayout->addWidget(createBtn("🏥 102호 호출", "#0E7490"));
    cmdLayout->addWidget(createBtn("💊 약 배송 요청", "#0284C7"));
    cmdLayout->addWidget(createBtn("🏠 S1 스테이션", "#334155"));
    cmdLayout->addWidget(createBtn("🗑️ 폐기물 수거", "#475569"));
    cmdGroup->setLayout(cmdLayout);

    logList = new QListWidget();
    rightPanel->addWidget(statGroup);
    rightPanel->addWidget(cmdGroup);
    rightPanel->addWidget(logList, 1);

    // 메인 레이아웃 배치
    mainHorizontalLayout->addLayout(leftLayout, 1);
    mainHorizontalLayout->addWidget(rightWidget);
    setCentralWidget(central);

    // 데이터 초기화 및 통신 시작
    mapPixmap.load(":/maps/newnew_map.png");
    rosThread = new RosThread(this);
    connect(rosThread, &RosThread::imageReceived, this, &MainWindow::updateCamera);
    connect(rosThread, &RosThread::poseReceived, this, &MainWindow::handlePose);
    connect(rosThread, &RosThread::batteryReceived, this, &MainWindow::handleBattery);
    connect(rosThread, &RosThread::eventReceived, this, &MainWindow::handleEvent);
    rosThread->start();

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::onTimerTick);
    timer->start(500);

    // UI 버튼 기능 연결
    connect(drawBtn, &QPushButton::clicked, this, [=](){
        isDrawingMode = !isDrawingMode;
        drawBtn->setText(isDrawingMode ? "✅ 편집 완료" : "🖊️ 경로 편집");
    });
    connect(clearBtn, &QPushButton::clicked, this, [=](){
        if(selectedRobotId == 1) { r1PatrolPath.clear(); r1TrailPath.clear(); }
        else { r2PatrolPath.clear(); r2TrailPath.clear(); }
        drawMap();
    });
    connect(patrolBtn, &QPushButton::clicked, this, &MainWindow::onPatrolButtonClicked);
}

MainWindow::~MainWindow() { rosThread->quit(); rosThread->wait(); }

// [함수] 마우스 클릭으로 경로 그리기
void MainWindow::mousePressEvent(QMouseEvent *event) {
    if (!isDrawingMode) return;
    QPoint pos = mapView->mapFrom(this, event->pos());
    if (mapView->rect().contains(pos)) {
        if (mapPixmap.isNull()) return;
        int curW = mapView->width(); int curH = mapView->height();
        QPixmap scaledMap = mapPixmap.scaled(curW, curH, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        int offsetX = (curW - scaledMap.width()) / 2;
        int offsetY = (curH - scaledMap.height()) / 2;
        int imgX = pos.x() - offsetX; int imgY = pos.y() - offsetY;
        if (imgX >= 0 && imgX <= scaledMap.width() && imgY >= 0 && imgY <= scaledMap.height()) {
            double res = 0.05, ox = -0.87, oy = -1.53;
            double scaleX = (double)scaledMap.width() / 548.0;
            double scaleY = (double)scaledMap.height() / 264.0;
            double wx = ((imgX / (4.0 * scaleX)) * res) + ox;
            double wy = (((264.0 - (imgY / scaleY)) / 4.0) * res) + oy;
            if (selectedRobotId == 1) r1PatrolPath.push_back({wx, wy});
            else r2PatrolPath.push_back({wx, wy});
            drawMap();
        }
    }
}

// [함수] 방 호출 버튼 클릭 시
void MainWindow::onRoomButtonClicked() {
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;
    QString roomText = btn->text();
    QString targetRoom = "";

    if (roomText.contains("101")) targetRoom = "101";
    else if (roomText.contains("102")) targetRoom = "102";
    else if (roomText.contains("S1")) targetRoom = "S1";
    else if (roomText.contains("S2")) targetRoom = "S2";
    else if (roomText.contains("START")) targetRoom = "START";
    else if (roomText.contains("waste")) targetRoom = "waste";

    if (selectedRobotId == 1) {
        r1Target = targetRoom;
        if (roomText.contains("약")) rosThread->publishMedicineRequest(targetRoom);
        else rosThread->publishCall(targetRoom);
        addLog(QString("로봇 1 -> %1 이동 명령").arg(targetRoom), 1);
    } else {
        r2Target = targetRoom;
        if (roomText.contains("약")) rosThread->publishMedicineRequest(targetRoom);
        else rosThread->publishCall(targetRoom);
        addLog(QString("로봇 2 -> %1 이동 명령").arg(targetRoom), 2);
    }
}

// [함수] 순찰 버튼 클릭 시
void MainWindow::onPatrolButtonClicked() {
    if (selectedRobotId == 1) {
        if (r1PatrolPath.empty()) { QMessageBox::warning(this, "알림", "로봇 1 경로를 먼저 그리세요."); return; }
        r1IsPatrolling = !r1IsPatrolling;
        if (r1IsPatrolling) { r1PatrolIdx = 0; r1Target = "PATROL"; rosThread->sendGoalPose(1, r1PatrolPath[0].x, r1PatrolPath[0].y); patrolBtn->setText("🛑 로봇 1 중단"); }
        else { r1Target = "IDLE"; patrolBtn->setText("🚀 순찰 시작"); }
    } else {
        if (r2PatrolPath.empty()) { QMessageBox::warning(this, "알림", "로봇 2 경로를 먼저 그리세요."); return; }
        r2IsPatrolling = !r2IsPatrolling;
        if (r2IsPatrolling) { r2PatrolIdx = 0; r2Target = "PATROL"; rosThread->sendGoalPose(2, r2PatrolPath[0].x, r2PatrolPath[0].y); patrolBtn->setText("🛑 로봇 2 중단"); }
        else { r2Target = "IDLE"; patrolBtn->setText("🚀 순찰 시작"); }
    }
}

// [함수] 로그 출력
void MainWindow::addLog(QString msg, int robotId) {
    QListWidgetItem *item = new QListWidgetItem(QString("[%1] %2").arg(QTime::currentTime().toString("HH:mm:ss")).arg(msg));
    if (robotId == 1) item->setForeground(QColor("#0D9488"));
    else if (robotId == 2) item->setForeground(QColor("#2563EB"));
    logList->insertItem(0, item);
}

// [함수] 카메라 영상 업데이트
void MainWindow::updateCamera(int id, QImage img) {
    QLabel *target = (id == 0 ? d435View : (id == 1 ? tb1View : tb2View));
    if (target && !img.isNull()) {
        target->setPixmap(QPixmap::fromImage(img).scaled(target->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
    }
}

// [함수] 지도 및 로봇 위치 그리기
void MainWindow::drawMap() {
    if (mapPixmap.isNull()) return;
    QPixmap scaledMap = mapPixmap.scaled(mapView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    QPainter painter(&scaledMap);
    painter.setRenderHint(QPainter::Antialiasing);

    auto drawPath = [&](std::vector<Point>& path, QColor color) {
        if (path.empty()) return;
        painter.setPen(QPen(color, 2, Qt::DashLine));
        for (size_t i = 0; i < path.size(); i++) {
            QPointF p = mapToPixel(path[i].x, path[i].y, scaledMap.width(), scaledMap.height());
            painter.setBrush(color); painter.drawEllipse(p, 4, 4);
            if (i > 0) {
                QPointF prev = mapToPixel(path[i-1].x, path[i-1].y, scaledMap.width(), scaledMap.height());
                painter.drawLine(prev, p);
            }
        }
    };

    drawPath(r1PatrolPath, QColor("#0D9488"));
    drawPath(r2PatrolPath, QColor("#2563EB"));

    // 실시간 이동 궤적 그리기
    auto drawTrail = [&](std::vector<Point>& trail, QColor color) {
        if (trail.size() < 2) return;
        int total = (int)trail.size();
        for (int i = 1; i < total; i++) {
            float alpha = 60.0f + 195.0f * ((float)i / total);
            QColor c = color;
            c.setAlpha((int)alpha);
            painter.setPen(QPen(c, 2.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            QPointF p0 = mapToPixel(trail[i-1].x, trail[i-1].y, scaledMap.width(), scaledMap.height());
            QPointF p1 = mapToPixel(trail[i].x, trail[i].y, scaledMap.width(), scaledMap.height());
            painter.drawLine(p0, p1);
        }
    };
    drawTrail(r1TrailPath, QColor("#0D9488"));
    drawTrail(r2TrailPath, QColor("#2563EB"));

    QPointF p1 = mapToPixel(r1x, r1y, scaledMap.width(), scaledMap.height());
    painter.setBrush(QColor("#0D9488")); painter.drawEllipse(p1, 8, 8);
    painter.drawText(p1 + QPointF(10, 0), "R1");

    QPointF p2 = mapToPixel(r2x, r2y, scaledMap.width(), scaledMap.height());
    painter.setBrush(QColor("#2563EB")); painter.drawEllipse(p2, 8, 8);
    painter.drawText(p2 + QPointF(10, 0), "R2");

    mapView->setPixmap(scaledMap);
}

// [함수] 로봇 좌표 수신 핸들러
void MainWindow::handlePose(int id, double x, double y) {
    if (id == 1) {
        r1x = x; r1y = y;
        if (r1TrailPath.empty() ||
            std::sqrt(std::pow(x - r1TrailPath.back().x, 2) + std::pow(y - r1TrailPath.back().y, 2)) > 0.05) {
            r1TrailPath.push_back({x, y});
            if ((int)r1TrailPath.size() > MAX_TRAIL_SIZE)
                r1TrailPath.erase(r1TrailPath.begin());
        }
        r1Status->setText(QString("상태: %1\n좌표: (%2, %3)").arg(getRobotStatusText(r1Target)).arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
        if (r1IsPatrolling && r1Target == "PATROL") {
            double tx = r1PatrolPath[r1PatrolIdx].x; double ty = r1PatrolPath[r1PatrolIdx].y;
            if (std::sqrt(std::pow(tx - x, 2) + std::pow(ty - y, 2)) < 0.3) {
                r1PatrolIdx = (r1PatrolIdx + 1) % (int)r1PatrolPath.size();
                rosThread->sendGoalPose(1, r1PatrolPath[r1PatrolIdx].x, r1PatrolPath[r1PatrolIdx].y);
            }
        }
    } else if (id == 2) {
        r2x = x; r2y = y;
        if (r2TrailPath.empty() ||
            std::sqrt(std::pow(x - r2TrailPath.back().x, 2) + std::pow(y - r2TrailPath.back().y, 2)) > 0.05) {
            r2TrailPath.push_back({x, y});
            if ((int)r2TrailPath.size() > MAX_TRAIL_SIZE)
                r2TrailPath.erase(r2TrailPath.begin());
        }
        r2Status->setText(QString("상태: %1\n좌표: (%2, %3)").arg(getRobotStatusText(r2Target)).arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
        if (r2IsPatrolling && r2Target == "PATROL") {
            double tx = r2PatrolPath[r2PatrolIdx].x; double ty = r2PatrolPath[r2PatrolIdx].y;
            if (std::sqrt(std::pow(tx - x, 2) + std::pow(ty - y, 2)) < 0.3) {
                r2PatrolIdx = (r2PatrolIdx + 1) % (int)r2PatrolPath.size();
                rosThread->sendGoalPose(2, r2PatrolPath[r2PatrolIdx].x, r2PatrolPath[r2PatrolIdx].y);
            }
        }
    }
}

void MainWindow::onTimerTick() { drawMap(); }
void MainWindow::handleBattery(int id, float p) { if(id == 1) r1BattBar->setValue((int)p); else if(id == 2) r2BattBar->setValue((int)p); }
void MainWindow::handleEvent(QString type, QString message) {
    QListWidgetItem *item = new QListWidgetItem(QString("[%1] %2: %3").arg(QTime::currentTime().toString("HH:mm:ss")).arg(type).arg(message));
    if (type.contains("긴급")) item->setForeground(Qt::red);
    logList->insertItem(0, item);
}
QPointF MainWindow::mapToPixel(double wx, double wy, int sw, int sh) {
    double res = 0.05, ox = -0.87, oy = -1.53;
    double scaleX = (double)sw / 548.0, scaleY = (double)sh / 264.0;
    return QPointF(((wx - ox) / res) * 4.0 * scaleX, (264.0 - ((wy - oy) / res) * 4.0) * scaleY);
}
QString MainWindow::getRobotStatusText(QString t) { return (t == "IDLE" ? "준비됨" : "임무 중"); }
