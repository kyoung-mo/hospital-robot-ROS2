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

Point getCoord(QString room) {
    if (room == "101호 약 배달" || room == "101호 호출") return {1.686, -0.663};
    if (room == "102호 약 배달" || room == "102호 호출") return {2.450, -0.546};
    if (room == "스테이션 1 복귀") return {4.437, -0.969};
    if (room == "폐기물 구역") return {5.379, 1.027};
    return {0, 0};
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("Smart Hospital Logistics Control System");
    setMinimumSize(1100, 850);
    resize(1400, 950);

    // [스타일] 제목 잘림 방지를 위해 마진과 패딩 최적화
    this->setStyleSheet(
        "QMainWindow { background-color: #F8FAFC; }"
        "QGroupBox { "
        "   font-family: 'Segoe UI', sans-serif; font-weight: bold; font-size: 15px; "
        "   border: 2px solid #CBD5E1; border-radius: 12px; background-color: #FFFFFF; "
        "   margin-top: 25px; padding-top: 35px; " // 상단 패딩 늘림
        "}"
        "QGroupBox::title { "
        "   subcontrol-origin: margin; subcontrol-position: top left; "
        "   left: 15px; top: 5px; padding: 4px 12px; " // 타이틀 위치 조정
        "   background-color: #0D9488; color: white; border-radius: 6px; "
        "}"
        "QPushButton { "
        "   background-color: #0F766E; color: white; border-radius: 8px; "
        "   font-weight: bold; padding: 8px; border: none; font-size: 13px; min-height: 35px; "
        "}"
        "QPushButton:hover { background-color: #14B8A6; }"
        "QProgressBar { border: 1px solid #E2E8F0; border-radius: 6px; text-align: center; background: #F1F5F9; font-weight: bold; height: 18px; }"
        "QProgressBar::chunk { background-color: #10B981; border-radius: 5px; }"
        );

    QWidget *central = new QWidget(this);
    QHBoxLayout *mainHorizontalLayout = new QHBoxLayout(central);
    mainHorizontalLayout->setContentsMargins(15, 15, 15, 15);
    mainHorizontalLayout->setSpacing(15);

    // --- [왼쪽 섹션] 카메라(위) + 지도(아래) ---
    QVBoxLayout *leftLayout = new QVBoxLayout();
    leftLayout->setSpacing(15);

    QGroupBox *camGroup = new QGroupBox("🎥 실시간 모니터링");
    QHBoxLayout *camLayout = new QHBoxLayout();
    camLayout->setContentsMargins(10, 15, 10, 10);
    camLayout->setSpacing(10);

    d435View = new QLabel("D435 Waiting...");
    tb1View = new QLabel("TB1 Waiting...");

    // [중요] Ignored 정책을 써야 이미지가 창을 강제로 키우지 않습니다.
    d435View->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
    tb1View->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
    d435View->setMinimumHeight(250);
    tb1View->setMinimumHeight(250);

    QString camStyle = "background: #0F172A; border-radius: 10px; color: #94A3B8; font-weight: bold; border: 1px solid #E2E8F0;";
    d435View->setStyleSheet(camStyle); d435View->setAlignment(Qt::AlignCenter);
    tb1View->setStyleSheet(camStyle); tb1View->setAlignment(Qt::AlignCenter);

    camLayout->addWidget(d435View);
    camLayout->addWidget(tb1View);
    camGroup->setLayout(camLayout);

    mapView = new QLabel();
    mapView->setStyleSheet("background: #FFFFFF; border: 2px solid #CBD5E1; border-radius: 15px;");
    mapView->setAlignment(Qt::AlignCenter);
    // 지도가 창 크기에 맞춰지도록 정책 설정
    mapView->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);

    leftLayout->addWidget(camGroup, 1); // 카메라 비중 1
    leftLayout->addWidget(mapView, 2);  // 지도 비중 2

    // --- [오른쪽 섹션] 사이드바 (상태 + 제어 + 로그) ---
    QVBoxLayout *rightPanel = new QVBoxLayout();
    rightPanel->setSpacing(12);

    QGroupBox *statGroup = new QGroupBox("📡 시스템 상태");
    QVBoxLayout *statLayout = new QVBoxLayout();
    QLabel *r1Title = new QLabel("🩺 [Robot 01] 의료 배송 유닛");
    r1Title->setStyleSheet("color: #0D9488; font-size: 15px;");
    r1Status = new QLabel(QString("상태: 온라인\n좌표: (%.2f, %.2f)").arg(r1x).arg(r1y));
    r1Status->setMinimumHeight(45);
    r1BattBar = new QProgressBar(); r1BattBar->setValue(100);
    statLayout->addWidget(r1Title); statLayout->addWidget(r1Status); statLayout->addWidget(r1BattBar);
    statGroup->setLayout(statLayout);

    QGroupBox *cmdGroup = new QGroupBox("🚑 자율 운행 제어");
    QVBoxLayout *cmdLayout = new QVBoxLayout();
    cmdLayout->setSpacing(8);
    drawBtn = new QPushButton("🖊️ 경로 편집");
    drawBtn->setStyleSheet("background-color: #64748B;");
    clearBtn = new QPushButton("🔄 초기화");
    patrolBtn = new QPushButton("🚀 순찰 시작");
    patrolBtn->setStyleSheet("background-color: #059669;");
    QHBoxLayout *pathRow = new QHBoxLayout();
    pathRow->addWidget(drawBtn); pathRow->addWidget(clearBtn);
    cmdLayout->addLayout(pathRow);
    cmdLayout->addWidget(patrolBtn);

    auto createBtn = [&](QString text, QString color) {
        QPushButton *b = new QPushButton(text);
        b->setStyleSheet(QString("background-color: %1;").arg(color));
        connect(b, &QPushButton::clicked, this, &MainWindow::onRoomButtonClicked);
        return b;
    };
    cmdLayout->addWidget(createBtn("🏥 101호 약 배송", "#0E7490"));
    cmdLayout->addWidget(createBtn("🏥 102호 약 배송", "#0E7490"));
    cmdLayout->addWidget(createBtn("🏠 스테이션 복귀", "#334155"));
    cmdGroup->setLayout(cmdLayout);

    QGroupBox *logGroup = new QGroupBox("📜 작업 기록");
    QVBoxLayout *logLayout = new QVBoxLayout();
    logList = new QListWidget();
    logLayout->addWidget(logList);
    logGroup->setLayout(logLayout);

    rightPanel->addWidget(statGroup);
    rightPanel->addWidget(cmdGroup);
    rightPanel->addWidget(logGroup, 1); // 로그 창이 남은 공간 차지

    mainHorizontalLayout->addLayout(leftLayout, 3);
    mainHorizontalLayout->addLayout(rightPanel, 1);

    setCentralWidget(central);

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

    connect(drawBtn, &QPushButton::clicked, this, [=](){
        isDrawingMode = !isDrawingMode;
        drawBtn->setText(isDrawingMode ? "✅ 편집 완료" : "🖊️ 경로 편집");
        drawBtn->setStyleSheet(isDrawingMode ? "background-color: #D97706;" : "background-color: #64748B;");
    });
    connect(clearBtn, &QPushButton::clicked, this, [=](){ r1PatrolPath.clear(); addLog("알림: 경로 초기화됨."); drawMap(); });
    connect(patrolBtn, &QPushButton::clicked, this, &MainWindow::onPatrolButtonClicked);
}

MainWindow::~MainWindow() { rosThread->quit(); rosThread->wait(); }

// [수정] 카메라 업데이트 시 라벨의 현재 크기에 이미지를 강제로 맞춤
void MainWindow::updateCamera(int id, QImage img) {
    QLabel *target = (id == 0 ? d435View : (id == 1 ? tb1View : nullptr));
    if (target && !img.isNull()) {
        // target->size()를 사용하여 현재 창 크기에 맞게 이미지 축소/확대
        target->setPixmap(QPixmap::fromImage(img).scaled(target->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void MainWindow::handleEvent(QString type, QString message) {
    addLog(QString("🔔 [%1] %2").arg(type).arg(message));
}

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
            r1PatrolPath.push_back({wx, wy});
            drawMap();
        }
    }
}

void MainWindow::onPatrolButtonClicked() {
    if (r1PatrolPath.empty()) { QMessageBox::warning(this, "알림", "경로를 먼저 그려주세요."); return; }
    r1IsPatrolling = !r1IsPatrolling;
    if (r1IsPatrolling) {
        if (isDrawingMode) drawBtn->click();
        r1PatrolIdx = 0; r1Target = "PATROL";
        rosThread->sendGoalPose(1, r1PatrolPath[0].x, r1PatrolPath[0].y);
        patrolBtn->setText("🛑 순찰 중단"); patrolBtn->setStyleSheet("background-color: #DC2626;");
    } else {
        r1Target = "IDLE";
        patrolBtn->setText("🚀 순찰 시작"); patrolBtn->setStyleSheet("background-color: #059669;");
    }
}

void MainWindow::handlePose(int id, double x, double y) {
    if (id == 1) {
        r1x = x; r1y = y;
        r1Status->setText(QString("상태: %1\n좌표: (%.2f, %.2f)").arg(getRobotStatusText(r1Target)).arg(x).arg(y));
        if (r1IsPatrolling && r1Target == "PATROL") {
            double tx = r1PatrolPath[r1PatrolIdx].x; double ty = r1PatrolPath[r1PatrolIdx].y;
            if (std::sqrt(std::pow(tx - x, 2) + std::pow(ty - y, 2)) < 0.3) {
                r1PatrolIdx = (r1PatrolIdx + 1) % (int)r1PatrolPath.size();
                rosThread->sendGoalPose(1, r1PatrolPath[r1PatrolIdx].x, r1PatrolPath[r1PatrolIdx].y);
            }
        }
    }
}

void MainWindow::onRoomButtonClicked() {
    QString roomName = qobject_cast<QPushButton*>(sender())->text().split(" ").last();
    if (r1IsPatrolling) { r1IsPatrolling = false; patrolBtn->setText("🚀 순찰 재개"); patrolBtn->setStyleSheet("background-color: #D97706;"); }
    r1Target = roomName;
    rosThread->sendGoalPose(1, getCoord(roomName).x, getCoord(roomName).y);
    addLog(QString("이동: %1(으)로 출동").arg(roomName));
}

void MainWindow::drawMap() {
    if (mapPixmap.isNull()) return;
    // [수정] 지도가 라벨 크기에 맞춰 그려지도록 함
    QPixmap scaledMap = mapPixmap.scaled(mapView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    QPainter painter(&scaledMap);
    painter.setRenderHint(QPainter::Antialiasing);
    if (!r1PatrolPath.empty()) {
        painter.setPen(QPen(QColor("#06B6D4"), 2, Qt::DashLine));
        for (size_t i = 0; i < r1PatrolPath.size(); i++) {
            QPointF p1 = mapToPixel(r1PatrolPath[i].x, r1PatrolPath[i].y, scaledMap.width(), scaledMap.height());
            painter.setBrush(QColor("#0891B2")); painter.drawEllipse(p1, 4, 4);
            if (i > 0) {
                QPointF p0 = mapToPixel(r1PatrolPath[i-1].x, r1PatrolPath[i-1].y, scaledMap.width(), scaledMap.height());
                painter.drawLine(p0, p1);
            }
        }
        if (r1PatrolPath.size() > 2) painter.drawLine(mapToPixel(r1PatrolPath.back().x, r1PatrolPath.back().y, scaledMap.width(), scaledMap.height()), mapToPixel(r1PatrolPath.front().x, r1PatrolPath.front().y, scaledMap.width(), scaledMap.height()));
    }
    QPointF robotPos = mapToPixel(r1x, r1y, scaledMap.width(), scaledMap.height());
    painter.setPen(QPen(Qt::white, 2)); painter.setBrush(QColor("#0D9488"));
    painter.drawEllipse(robotPos, 10, 10);
    mapView->setPixmap(scaledMap);
}

QPointF MainWindow::mapToPixel(double wx, double wy, int sw, int sh) {
    double res = 0.05, ox = -0.87, oy = -1.53;
    double scaleX = (double)sw / 548.0, scaleY = (double)sh / 264.0;
    return QPointF(((wx - ox) / res) * 4.0 * scaleX, (264.0 - ((wy - oy) / res) * 4.0) * scaleY);
}

QString MainWindow::getRobotStatusText(QString t) { return (t == "IDLE" ? "준비됨" : "임무 중"); }
void MainWindow::addLog(QString msg, int robotId) {
    QListWidgetItem *item = new QListWidgetItem(QString("[%1] %2").arg(QTime::currentTime().toString("HH:mm:ss")).arg(msg));
    if (robotId == 1) item->setForeground(QColor("#0891B2"));
    logList->insertItem(0, item);
}
void MainWindow::handleBattery(int id, float p) { if(id == 1) r1BattBar->setValue((int)p); }
void MainWindow::onTimerTick() { drawMap(); }