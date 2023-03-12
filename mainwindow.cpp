#include "mainwindow.h"
#include "ui_mainwindow.h"
QString save_map;         // 用于保存地图数据
QString point_msg_x;      // 用于保存点位x数据
QString point_msg_y;      // 用于保存点位y数据
QString point_msg_oz;     // 用于保存点位oz数据
QString point_msg_ow;     // 用于保存点位ow数据
double robot_positionx;   // 机器人位置信息的x
double robot_positiony;   // 机器人位置信息的y
bool isconnected = false; // 是否连接的标志位
// 多线程处理数据
MyThread laser_thread;
MyThread map_thread;
MyThread car_state_thread;
MyThread robot_pose_thread;
MyThread plan_thread;
MyThread movebase_result_thread;

struct Quaternion // 四元数结构体
{
    double w, x, y, z;
};

typedef struct { // 欧拉角结构体
    double yaw, pitch, roll;
} EulerAngle;
EulerAngle e;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // 设置窗口标题
    setWindowTitle("ROS Bridge控制端");

    // 初始化
    initUis();     // 场景的初始化
    combox_init(); // 选点导航初始化已有点位
    ip_init();     // 初始化上次连接IP和端口
    allconnect();

    // 在myWidget中加载摇杆，并连接摇杆信号
    joystick = new Joystick(ui->myWidget);
    connect(joystick, SIGNAL(getxy(int, int)), this, SLOT(receivexy(int, int))); // 连接摇杆的信号

    // 设置lineEdit（摇杆倍率）只能输入正数
    QRegExp exp("[0-9\\.]+$");
    QRegExpValidator *Validator = new QRegExpValidator(exp);
    ui->lineEdit->setValidator(Validator);

    // 设置iplineedit只能输入IP
    //  正在表达式限制输入
    ui->iplineedit->setValidator(new QRegExpValidator(QRegExp("\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b")));
    // 用于占位
    ui->iplineedit->setInputMask("000.000.000.000;");

    // 端口只能输入0-99999
    ui->portlineedit->setValidator(new QIntValidator(0, 65535));

    // 设置定时器间隔，启动定时器，用于显示更新时间
    timer.setInterval(10); // 单位ms，这个值越小时间越准
    timer.start();

    // 我发现QPlainTextEdit执行删除操作后才会显示我隐藏的提示文本,Qt版本5.14.1
    ui->sendmessagetextedit->clear();
}

MainWindow::~MainWindow()
{
    m_websocket.close();
    delete ui;
}

// 场景初始化
void MainWindow::initUis()
{
    ui->mapViz->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff); // 消除滚动条
    ui->mapViz->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    // 视图场景加载
    m_qgraphicsScene =
        new QGraphicsScene; // 要用QGraphicsView就必须要有QGraphicsScene搭配着用
    m_qgraphicsScene->clear();
    // 创建item
    m_roboItem = new Ui::roboItem();
    // 视图添加item
    m_qgraphicsScene->addItem(m_roboItem);
    // 设置item的坐标原点与视图的原点重合（默认为视图中心）
    //  widget添加视图
    ui->mapViz->setScene(m_qgraphicsScene);
}

// 加载上次连接的IP和Port
void MainWindow::ip_init()
{
    QSettings ip("./setting.ini", QSettings::IniFormat);
    ui->iplineedit->setText(ip.value("WS/IP").toString());
    ui->portlineedit->setText(ip.value("WS/Port").toString());
}

// 连接后加载
void MainWindow::init()
{
    QSettings checkbox_setting("./subsetting.ini", QSettings::IniFormat);
    QStringList keys = checkbox_setting.allKeys();
    if (keys.length() == 0) // 无配置文件将订阅所有话题，并生成配置文件
    {
        QStringList topics = {"/map", "/robot_pose", "/car_state", "/move_base/TebLocalPlannerROS/global_plan", "/scan_filtered","/move_base/result"};
        foreach (QString topic, topics) {
            checkbox_setting.setValue("checkbox_setting" + topic, "true");
        }
        readmap();        // 订阅地图话题map
        getrobot_pose();  // 订阅机器人位置，属自定义话题，话题robot_pose
        read_car_state(); // 订阅状态，属自定义消息，话题car_state
        GlobalPlan();     // 订阅全局路径move_base/TebLocalPlannerROS/global_plan
        read_laser();     // 订阅雷达消息，话题scan_filtered
        read_movebase_result();//订阅导航结果信息
    }
    foreach (QString key, keys) {
        if (checkbox_setting.value(key).toString() == "true") {
            key = key.remove("checkbox_setting");
            QString sub = QString("{\"op\":\"subscribe\",\"topic\":\"%1\"}").arg(key);
            m_websocket.sendTextMessage(sub);
        }
    }
}

// 启动时加载已保存的点位
void MainWindow::combox_init()
{
    QSettings settings("./pointslist.ini", QSettings::IniFormat);
    int size = settings.beginReadArray("pointslist");
    for (int i = 0; i < size; i++) {
        settings.setArrayIndex(i);
        QString pointnumber;
        QString getdata = settings.value("point " + pointnumber.setNum(i)).toString();
        ui->comboBox->addItem(getdata);
    }
    settings.endArray();
}

void MainWindow::allconnect()
{
    // connect(this, SIGNAL(updateMap(QImage)), m_roboItem,SLOT(paintMaps(QImage)));
    // connect(this, SIGNAL(plannerPath(QPolygonF)), m_roboItem,SLOT(paintPlannerPath(QPolygonF)));
    // connect(this, SIGNAL(updateRoboPose(double x,double y,double theta)), m_roboItem,SLOT(paintRoboPos(double x,double y,double theta)));
    connect(m_roboItem, SIGNAL(signalPub2DPose(QPointF, QPointF)), this, SLOT(pub2DPose(QPointF, QPointF)));
    connect(m_roboItem, SIGNAL(signalPub2DGoal(QPointF, QPointF)), this, SLOT(pub2DGoal(QPointF, QPointF)));
    // 设置2D Pose
    connect(ui->set_pos_btn, SIGNAL(clicked()), m_roboItem, SLOT(slot_set2DPos()));
    // 设置2D goal
    connect(ui->set_goal_btn, SIGNAL(clicked()), m_roboItem, SLOT(slot_set2DGoal()));
    // 数据的处理放在线程处理，以下是连接处理完得到的结果进行绘图和界面更新信号
    connect(this, SIGNAL(update_laser(QPolygonF)), m_roboItem, SLOT(paintLaserScan(QPolygonF)));
    connect(this, SIGNAL(update_map(QImage)), m_roboItem, SLOT(paintMaps(QImage)));
    connect(this, SIGNAL(update_robot_pose(double, double, double)), m_roboItem, SLOT(paintRoboPos(double, double, double)));
    connect(this, SIGNAL(update_plan(QPolygonF)), m_roboItem, SLOT(paintPlannerPath(QPolygonF)));
    connect(this, SIGNAL(update_car_state(QString, QString)), this, SLOT(update_car_state_ui(QString, QString)));
    connect(this, SIGNAL(update_movebase_result(QString)), this, SLOT(update_movebase_result_ui(QString)));
    // 连接Websocket的连接状态
    connect(&m_websocket, SIGNAL(textMessageReceived(QString)), this, SLOT(onTextMessageReceived(QString))); // 连接接收数据成功信号
    connect(&m_websocket, &QWebSocket::connected, this, [=]()                                                // 连接成功信号
            {
                isconnected = true;
                ui->pushButton->setText("断开");
                ui->statu_led->setStyleSheet("background-color:#8aff0c;"
                                             "border-radius: 10px;"
                                             "border:1px solid black;"); // 修改连接状态的label
                ui->statu_led->setText("已连接");
                ui->iplineedit->setEnabled(false);
                ui->portlineedit->setEnabled(false);
                // 保存IP和端口
                QSettings ip("./setting.ini", QSettings::IniFormat);
                ip.setValue("WS/IP", ui->iplineedit->text());
                ip.setValue("WS/Port", ui->portlineedit->text());
                init();
            });
    connect(&m_websocket, &QWebSocket::disconnected, this, [=]() // 连接断开信号
            {
                isconnected = false;
                ui->statu_led->setStyleSheet("background-color:rgb(255,0,0);"
                                             "border-radius: 10px;"
                                             "border:1px solid black;");
                ui->pushButton->setText("连接");
                ui->statu_led->setText("未连接");
                ui->iplineedit->setEnabled(true);
                ui->portlineedit->setEnabled(true);
            });
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        // 在槽函数中更新label的文本为当前时间
        ui->label_8->setText(QDateTime::currentDateTime().toString("HH:mm:ss"));
    });
}

// 有关设置的信号和槽连接
void MainWindow::about_setting_singal()
{
    connect(s, SIGNAL(point_add(QString)), this, SLOT(point_add(QString))); // 连接新增信号
    connect(s, SIGNAL(point_del(QString)), this, SLOT(point_del(QString))); // 连接删除信号
    connect(s, SIGNAL(point_del_all()), this, SLOT(point_del_all()));       // 连接重置信号
    connect(s, SIGNAL(checkbox_signal(QString, bool)), this, SLOT(setting_checkbox_slot(QString, bool)));
}

// 连接与断开
void MainWindow::on_pushButton_clicked()
{
    if (isconnected == false) {
        QString path = QString("ws://%1:%2").arg(ui->iplineedit->text()).arg(ui->portlineedit->text());
        QUrl url = QUrl(path);
        m_websocket.open(url);
    } else {
        m_websocket.close();
    }
}

// 接收到信息后来到这里进行判断，并启动相应的线程处理数据
void MainWindow::onTextMessageReceived(const QString &message)
{

    if (message.contains("/map")) // 如果是地图信息
    {
        map_thread.select = 1;
        map_thread.w = this;
        map_thread.message = message;
        map_thread.start();
        // map_data_paint(message);
    } else if (message.contains("/robot_pose")) {
        robot_pose_thread.select = 2;
        robot_pose_thread.w = this;
        robot_pose_thread.message = message;
        robot_pose_thread.start();
        // get_RobotPose(message);
    } else if (message.contains("/car_state")) {
        car_state_thread.select = 3;
        car_state_thread.w = this;
        car_state_thread.message = message;
        car_state_thread.start();
        // get_car_state(message);
    } else if (message.contains("/move_base/TebLocalPlannerROS/global_plan")) {
        plan_thread.select = 4;
        plan_thread.w = this;
        plan_thread.message = message;
        plan_thread.start();
        // get_plan_data(message);
    } else if (message.contains("/scan_filtered")) {
        laser_thread.select = 5;
        laser_thread.w = this;
        laser_thread.message = message;
        laser_thread.start();
        // laserScanCallback(message);
    } else if (message.contains("/move_base/result")) {
        movebase_result_thread.select = 6;
        movebase_result_thread.w = this;
        movebase_result_thread.message = message;
        movebase_result_thread.start();
    }
}

// 发送按钮
void MainWindow::on_pushButton_3_clicked()
{
    QString msg = ui->sendmessagetextedit->document()->toPlainText();
    m_websocket.sendTextMessage(msg);
}

// 进桩
void MainWindow::on_pushButton_14_clicked()
{
    QString data = "{\"msg\":{\"data\":\"ReCharge\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

// 关机
void MainWindow::on_pushButton_18_clicked()
{
    QString data = "{\"msg\":{\"data\":\"disReCharge\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

// 停止进桩
void MainWindow::on_pushButton_15_clicked()
{
    QString data = "{\"msg\":{\"data\":\"shut_down\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

// 读取地图（加载地图）
void MainWindow::on_pushButton_19_clicked()
// 注：这一部分未实现，水平不够，没研究出来怎么用websocket去启动地图服务
// 我在这段代码这里写的是你文本里的内容用websocket发送
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "",
                                                    tr("Map Files (*.map);;All Files (*)"));
    if (!fileName.isEmpty()) {
        // 根据选择的文件名打开文件并读取内容
        QFile file(fileName);
        if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QTextStream in(&file);
            QString mapdata = in.readAll();
            // 处理文件内容
            m_websocket.sendTextMessage(mapdata);
            file.close();
        }
    }
}

// 订阅地图信息
void MainWindow::readmap()
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/map\"}";
    m_websocket.sendTextMessage(data);
}

// 订阅机器人位置
void MainWindow::getrobot_pose()
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/robot_pose\"}";
    m_websocket.sendTextMessage(data);
}

// 订阅状态话题，自定义消息
void MainWindow::read_car_state()
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/car_state\"}";
    m_websocket.sendTextMessage(data);
}

// 订阅全局路线
void MainWindow::GlobalPlan()
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/move_base/TebLocalPlannerROS/global_plan\"}";
    m_websocket.sendTextMessage(data);
}

// 发送速度消息
void MainWindow::cmd_vel(QString x, QString z)
{
    QString data = QString("{\"msg\":{\"angular\":{\"x\":0,\"y\":0,\"z\":%1},\"linear\":{\"x\":%2,\"y\":0,\"z\":0}},\"op\":\"publish\",\"topic\":\"/cmd_vel\"}").arg(z).arg(x);
    m_websocket.sendTextMessage(data);
}

// 雷达订阅
void MainWindow::read_laser()
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/scan_filtered\"}";
    m_websocket.sendTextMessage(data);
}

//订阅导航结果
void MainWindow::read_movebase_result()
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/move_base/result\"}";
    m_websocket.sendTextMessage(data);
}

// 停止
void MainWindow::on_pushButton_4_clicked()
{
    cmd_vel("0", "0");
}

// 停止导航，即关闭move_base
void MainWindow::on_pushButton_12_clicked()
{
    QString data = "{\"msg\":{\"data\":\"move_base_stop\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

// 停止建图
void MainWindow::on_pushButton_10_clicked()
{
    QString data = "{\"msg\":{\"data\":\"gmapping_stop\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

// 启动建图
void MainWindow::on_pushButton_9_clicked() // 这个是我在ROS程序里写了通过control_cmd话题收到gmapping_start这个文本时，调用gmapping启动建图
{
    QString data = "{\"msg\":{\"data\":\"gmapping_start\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

// 启动导航，启动move_base
void MainWindow::on_pushButton_11_clicked()
{
    QString data = "{\"msg\":{\"data\":\"move_base_start\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

// 终止到点
void MainWindow::on_pushButton_17_clicked()
{
    QString data = "{\"op\":\"publish\",\"topic\":\"/move_base/cancel\",\"msg\":{\"stamp\":{\"secs\":0,\"nsecs\":0},\"id\":\"\"}}";
    m_websocket.sendTextMessage(data);
}

// QString转Json程序
QJsonObject MainWindow::QstringToJson(QString jsonString)
{
    QJsonDocument jsonDocument = QJsonDocument::fromJson(jsonString.toLocal8Bit().data());
    if (jsonDocument.isNull()) {
        // qDebug()<< "String NULL"<< jsonString.toLocal8Bit().data();
    }
    QJsonObject jsonObject = jsonDocument.object();
    return jsonObject;
}

// 对收到的地图数据进行处理
void MainWindow::map_data_handle(QString data)
{
    QJsonObject jsondata, msgdata, infodata, origin, position;
    jsondata = QstringToJson(data);
    msgdata = jsondata["msg"].toObject();
    infodata = msgdata["info"].toObject();
    QJsonValue width = infodata["width"];
    QJsonValue height = infodata["height"];
    QJsonValue resolution = infodata["resolution"];
    origin = infodata["origin"].toObject();
    position = origin["position"].toObject();
    QJsonValue positionx = position["x"];
    QJsonValue positiony = position["y"];
    QJsonValue map = msgdata["data"]; // 地图的所有数据
    // 使用map[num];访问其中的第num+1个元素元素
    mapCallback(width, height, map, resolution, positionx, positiony);
}

// 地图信息订阅回调函数
// 在此函数传入map_data_paint处理后的数据，使用roboItem定义好的paintMaps(map_image)，就可以在GraphiceView中画出地图，参考了古月居的程序
void MainWindow::mapCallback(QJsonValue width, QJsonValue height, QJsonValue map, QJsonValue resolution, QJsonValue positionx, QJsonValue positiony)
{
    int size;
    size = width.toInt() * height.toInt();
    m_mapResolution = resolution.toDouble(); // msg->info.resolution;
    double origin_x = positionx.toDouble();  // msg->info.origin.position.x;
    double origin_y = positiony.toDouble();  // msg->info.origin.position.y;
    QImage map_image(width.toInt(), height.toInt(), QImage::Format_RGB32);
    for (int i = 0; i < size; i++) { // 想办法获取map的长度
        int x = i % width.toInt();
        int y = (int)i / width.toInt();
        // 计算像素值
        QColor color;
        if (map[i] == 100) {
            color = Qt::black; // black
        } else if (map[i] == 0) {
            color = Qt::white; // white
        } else if (map[i] == -1) {
            color = Qt::gray; // gray
        }
        map_image.setPixel(x, y, qRgb(color.red(), color.green(), color.blue()));
    }
    // 延y翻转地图 因为解析到的栅格地图的坐标系原点为左下角
    // 但是图元坐标系为左上角度
    map_image = rotateMapWithY(map_image);
    emit update_map(map_image);
    // updateMap(map_image);
    // m_roboItem->paintMaps(map_image);
    // 计算翻转后的图元坐标系原点的世界坐标
    double origin_x_ = origin_x;
    double origin_y_ = origin_y + height.toInt() * m_mapResolution;
    // 世界坐标系原点在图元坐标系下的坐标
    m_wordOrigin.setX(fabs(origin_x_) / m_mapResolution);
    m_wordOrigin.setY(fabs(origin_y_) / m_mapResolution);
}

QImage MainWindow::rotateMapWithY(QImage map)
{ // 沿Y轴翻转
    QImage res = map;
    for (int x = 0; x < map.width(); x++) {
        for (int y = 0; y < map.height(); y++) {
            res.setPixelColor(x, map.height() - y - 1, map.pixel(x, y));
        }
    }
    return res;
}

// 四元数转欧拉角，传入四元数，返回欧拉角
EulerAngle quaternionToEuler(Quaternion q)
{
    // EulerAngle euler;
    //  计算yaw、pitch、roll
    e.yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    e.pitch = asin(2 * (q.w * q.y - q.z * q.x));
    e.roll = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    return e;
}
// 欧拉角转四元数，传入欧拉角，返回四元数
Quaternion ToQuaternion(double yaw, double pitch, double roll)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

// 设置初始位置
void MainWindow::pub2DPose(QPointF start_pose, QPointF end_pose)
{
    start_pose = transScenePoint2Word(start_pose);
    end_pose = transScenePoint2Word(end_pose);
    double angle = atan2(end_pose.y() - start_pose.y(), end_pose.x() - start_pose.x());
    long int secs = (QDateTime::currentMSecsSinceEpoch() / 1000); // 获取1970后的秒数
    QTime current_time = QTime::currentTime();
    int msec = current_time.msec(); // 当前的毫秒
    long int ns = msec * 1000000;   // 换算成ns
    QString data = QString("{\"msg\":{\"header\":{\"frame_id\":\"map\",\"seq\":0,\"stamp\":{\"secs\":%1,\"nsecs\":%2}},\"pose\":{\"covariance\":[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787],\"pose\":{\"position\":{\"x\":%3,\"y\":%4,\"z\":0},\"orientation\":{\"x\":0,\"y\":0,\"z\":%5,\"w\":%6}}}},\"op\":\"publish\",\"topic\":\"/initialpose\"}").arg(secs).arg(ns).arg(start_pose.x()).arg(start_pose.y()).arg(ToQuaternion(angle, 0, 0).z).arg(ToQuaternion(angle, 0, 0).w);
    ;
    m_websocket.sendTextMessage(data);
}

// 设置目标点
void MainWindow::pub2DGoal(QPointF start_pose, QPointF end_pose)
{
    start_pose = transScenePoint2Word(start_pose);
    end_pose = transScenePoint2Word(end_pose);
    double angle = atan2(end_pose.y() - start_pose.y(), end_pose.x() - start_pose.x());
    long int secs = (QDateTime::currentMSecsSinceEpoch() / 1000); // 获取1970后的秒数
    QTime current_time = QTime::currentTime();
    int msec = current_time.msec(); // 当前的毫秒
    long int ns = msec * 1000000;   // 换算成ns
    QString msg = QString("{\"msg\":{\"header\":{\"frame_id\":\"map\",\"seq\":0,\"stamp\":{\"secs\":%1,\"nsecs\":%2}},\"pose\":{\"position\":{\"x\":%3,\"y\":%4,\"z\":0},\"orientation\":{\"x\":0,\"y\":0,\"z\":%5,\"w\":%6}}},\"op\":\"publish\",\"topic\":\"/move_base_simple/goal\"}").arg(secs).arg(ns).arg(start_pose.x()).arg(start_pose.y()).arg(ToQuaternion(angle, 0, 0).z).arg(ToQuaternion(angle, 0, 0).w);
    m_websocket.sendTextMessage(msg);
}

QPointF MainWindow::transScenePoint2Word(QPointF pose)
{
    QPointF res;
    res.setX((pose.x() - m_wordOrigin.x()) * m_mapResolution);
    // y坐标系相反
    res.setY(-1 * (pose.y() - m_wordOrigin.y()) * m_mapResolution);
    return res;
}

QPointF MainWindow::transWordPoint2Scene(QPointF pose)
{
    QPointF res;
    res.setX(m_wordOrigin.x() + pose.x() / m_mapResolution);
    res.setY(m_wordOrigin.y() - (pose.y() / m_mapResolution));
    return res;
}

// 对机器人位置数据处理
void MainWindow::get_RobotPose(QString posedata) // 我自己根据move_base改出来的用于获取机器人位置的信息
{
    QJsonObject jsondata, msgdata, orientation, position;
    jsondata = QstringToJson(posedata);
    msgdata = jsondata["msg"].toObject(); // 提取msg内数据
    position = msgdata["position"].toObject();
    orientation = msgdata["orientation"].toObject();
    robot_positionx = position["x"].toDouble();
    point_msg_x = point_msg_x.setNum(robot_positionx);
    robot_positiony = position["y"].toDouble();
    point_msg_y = point_msg_y.setNum(robot_positiony);
    // 坐标转化为图元坐标系
    QPointF roboPos = transWordPoint2Scene(QPointF(robot_positionx, robot_positiony));
    double orientationw = orientation["w"].toDouble();
    point_msg_ow = point_msg_ow.setNum(orientationw);
    double orientationz = orientation["z"].toDouble();
    point_msg_oz = point_msg_oz.setNum(orientationz);
    double orientationx = orientation["x"].toDouble();
    double orientationy = orientation["y"].toDouble();
    Quaternion q;
    q.w = orientationw;
    q.x = orientationx;
    q.y = orientationy;
    q.z = orientationz;
    e = quaternionToEuler(q);
    emit update_robot_pose(roboPos.x(), roboPos.y(), e.yaw);
    // m_roboItem->paintRoboPos(roboPos.x(),roboPos.y(),e.yaw);
}

// 对路径消息的处理
void MainWindow::plannerPathCallback(float x[], float y[], int length)
{
    plannerPoints.clear();
    for (int i = 0; i < length; i++) {
        QPointF roboPos = transWordPoint2Scene(QPointF(x[i], y[i]));
        plannerPoints.append(roboPos);
    }
    emit update_plan(plannerPoints);
    // m_roboItem->paintPlannerPath(plannerPoints);
}

// 对car_state信息的处理
void MainWindow::get_car_state(QString car_state_data)
{
    QJsonDocument jsonDoc;
    QByteArray jsonByteArray = car_state_data.toUtf8();
    QJsonParseError jsonError;
    jsonDoc = QJsonDocument::fromJson(jsonByteArray, &jsonError);
    QJsonObject jsonObj = jsonDoc.object();
    QJsonValue msgvalue = jsonObj.value("msg");
    QJsonObject msgObj = msgvalue.toObject();
    QJsonValue Charge_State = msgObj.value("Charge_State");
    QJsonValue Battery_Voltage = msgObj.value("Battery_Voltage");
    QJsonValue Charge_Current = msgObj.value("Charge_Current");
    QString state, voltage;
    state = Charge_State.toString();
    voltage = Battery_Voltage.toString();
    emit update_car_state(state, voltage);
}

// 更新小车UI组件
void MainWindow::update_car_state_ui(QString Charge_State, QString Battery_Voltage)
{
    ui->label_7->setText(Charge_State);
    ui->label_6->setText(Battery_Voltage);
    int power = Battery_Voltage.toInt();
    ui->progressBar->setValue(power); // 一设置其最大值为29000，最小值为21500
    if (power >= 27125) {
        QString style;
        style = "QProgressBar {border: 2px solid gray;border-radius: 10px;background-color: white;height: 30px;}QProgressBar::chunk {background-color: #a0d468;border-radius:8px;}";
        ui->progressBar->setStyleSheet(style);
    } else if (power >= 25250 && power < 27125) {
        QString style;
        style = "QProgressBar {border: 2px solid gray;border-radius: 10px;background-color: white;height: 30px;}QProgressBar::chunk {background-color: #ffff00;border-radius:8px;}";
        ui->progressBar->setStyleSheet(style);
    } else if (power >= 23375 && power < 25250) {
        QString style;
        style = "QProgressBar {border: 2px solid gray;border-radius: 10px;background-color: white;height: 30px;}QProgressBar::chunk {background-color: #ffa008;border-radius:8px;}";
        ui->progressBar->setStyleSheet(style);
    } else {
        QString style;
        style = "QProgressBar {border: 2px solid gray;border-radius: 10px;background-color: white;height: 30px;}QProgressBar::chunk {background-color: #ff4b7b;border-radius:8px;}";
        ui->progressBar->setStyleSheet(style);
    }
}

// 对plan数据的处理
void MainWindow::get_plan_data(QString data)
{
    // json数据
    QByteArray jsonStr = data.toUtf8();
    ;
    // 将json数据解析为QJsonDocument
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonStr);
    // 获取msg对象
    QJsonObject msgObj = jsonDoc.object()["msg"].toObject();
    // 获取poses数组
    QJsonArray posesArr = msgObj["poses"].toArray();
    // 定义一个数组来存储position下的x
    // QVector<double> xArr,yArr;
    // 遍历poses数组
    int length = posesArr.size(); // 获取数组长度
    float xArr[length], yArr[length];
    for (int i = 0; i < posesArr.size(); i++) {
        // 获取pose对象
        QJsonObject poseObj = posesArr[i].toObject()["pose"].toObject();
        // 获取position对象
        QJsonObject positionObj = poseObj["position"].toObject();
        // 获取x
        double x = positionObj["x"].toDouble();
        double y = positionObj["y"].toDouble();
        // 将x添加到数组中
        xArr[i] = x;
        yArr[i] = y;
    }
    plannerPathCallback(xArr, yArr, length);
}

// 激光雷达点云信息处理
void MainWindow::laserScan_data_handle(QString laser_msg)
{
    // 将JSON字符串转换为JSON文档
    QJsonDocument doc = QJsonDocument::fromJson(laser_msg.toUtf8());
    // 提取JSON文档中的msg字段
    QJsonObject msgObj = doc["msg"].toObject();
    double angle_min = msgObj["angle_min"].toDouble();
    double angle_increment = msgObj["angle_increment"].toDouble();
    // 提取ranges数组
    QJsonArray rangesArr = msgObj["ranges"].toArray();
    QVector<double> rangesVec;
    for (int i = 0; i < rangesArr.size(); i++) {
        // 如果数组元素为null，则将其转换为0
        if (rangesArr[i].isNull()) {
            rangesVec.append(0);
        } else {
            rangesVec.append(rangesArr[i].toDouble());
        }
    }
    //  geometry_msgs::PointStamped laser_point;
    //  geometry_msgs::PointStamped map_point;
    //  laser_point.header.frame_id = laser_msg->header.frame_id;
    laserPoints.clear();
    // 转换到二维XY平面坐标系下;
    for (int i = 0; i < rangesArr.size(); i++) {
        // scan_laser坐标系下
        double angle = angle_min + i * angle_increment;
        double X = rangesVec[i] * cos(angle);
        double Y = rangesVec[i] * sin(angle);

        QTransform transform; // 需要进行tf变换，此处的tf变换不严谨，因为我也不会
        // 应该要变换到以base_footprint或者base_link的坐标系才是合适的，看看你定义的这两个哪个是小车旋转中心
        transform.translate(robot_positionx, robot_positiony); // 这个变换以小车位置为准，x y是小车位置
        // qDebug()<<e.yaw;
        transform.rotateRadians(e.yaw);           // 弧度
        QPointF point_A(X + 0.25, Y);             // 雷达在坐标系A的位置 +0.25是雷达在小车旋转中心的x+方向0.25m
        QPointF point_B = transform.map(point_A); // 转换后在坐标系B的位置
        double x_prime = point_B.x();
        double y_prime = point_B.y();
        // 转化为图元坐标系
        QPointF roboPos = transWordPoint2Scene(QPointF(x_prime, y_prime));
        //        transWordPoint2Scene(QPointF(map_point.point.x, map_point.point.y));
        laserPoints.append(roboPos);
    }
    emit update_laser(laserPoints);
    // m_roboItem->paintLaserScan(laserPoints);
}

void MainWindow::movebase_result_data_handle(QString data)
{
    // 假设jsonString是上面提供的QString格式的JSON数据
    QJsonDocument jsonDocument = QJsonDocument::fromJson(data.toUtf8());
    QJsonObject jsonObject = jsonDocument.object();
    // 获取status字段中的text值
    QString text = jsonObject.value("msg").toObject().value("status").toObject().value("text").toString();
    emit update_movebase_result(text);
}

void MainWindow::update_movebase_result_ui(QString data)
{
    if(data != "Goal reached.")
    {
        QMessageBox::warning(NULL, QStringLiteral("警告"), data, QMessageBox::Ok);// 添加提示
    }
    else//成功则清除路径
    {
        plannerPoints.clear();
        emit update_plan(plannerPoints);
    }
}
// 保存地图
void MainWindow::on_pushButton_13_clicked()
// 实力有限，并没实现，不会用rosbridge实现保存地图
// 这里写的是将地图话题map的信息保存成文本
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "",
                                                    tr("Map Files (*.map);;All Files (*)"));
    if (!fileName.isEmpty()) {
        QFile file(fileName);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            // 写入文件内容
            QTextStream out(&file);
            out << save_map;
            file.close();
        }
    }
}

// 设置遥感的倍率
void MainWindow::receivexy(int x, int y)
{
    float beilv = ui->lineEdit->text().toDouble(); // 摇杆倍率
    float lx, ly;
    lx = beilv * x / 50;
    ly = beilv * y / 50;
    QString lx_str, ly_str;
    cmd_vel(ly_str.setNum(ly), lx_str.setNum(-lx)); // 调转x,y方向
}

// 打开设置页
void MainWindow::on_pushButton_21_clicked()
{
    if (s != nullptr)
        delete s;
    s = new setting;
    s->show();              // 打开点位设置页
    about_setting_singal(); // 所有点位相关的信号连接
}

// 新增并保存点位槽
void MainWindow::point_add(QString name)
{
    ui->comboBox->addItem(name);
    // 创建QSettings对象并指定ini文件路径并将格式设置为ini
    QSettings savepoint("./pointdata.ini", QSettings::IniFormat);
    savepoint.setValue("pointdata/" + name + "x", point_msg_x);
    savepoint.setValue("pointdata/" + name + "y", point_msg_y);
    savepoint.setValue("pointdata/" + name + "oz", point_msg_oz);
    savepoint.setValue("pointdata/" + name + "ow", point_msg_ow);
}

// 删除点位槽
void MainWindow::point_del(QString name)
{
    //    int count = ui->comboBox->count();
    int flag = ui->comboBox->findText(name, Qt::MatchExactly);
    ui->comboBox->removeItem(flag);
    QSettings savepoint("./pointdata.ini", QSettings::IniFormat);
    savepoint.remove("pointdata/" + name + "x");
    savepoint.remove("pointdata/" + name + "y");
    savepoint.remove("pointdata/" + name + "oz");
    savepoint.remove("pointdata/" + name + "ow");
}

// 删除所有点位槽
void MainWindow::point_del_all()
{
    ui->comboBox->clear();
    QSettings savepoint("./pointdata.ini", QSettings::IniFormat);
    savepoint.clear();
    savepoint.sync();
}

// 导航到选择的点位
void MainWindow::on_pushButton_22_clicked()
{
    QString get_point_name = ui->comboBox->currentText(); // 当前选中的项
    if (get_point_name != "") {
        QSettings savepoint("./pointdata.ini", QSettings::IniFormat);
        QString point_data_x = savepoint.value("pointdata/" + get_point_name + "x").toString();
        QString point_data_y = savepoint.value("pointdata/" + get_point_name + "y").toString();
        QString point_data_oz = savepoint.value("pointdata/" + get_point_name + "oz").toString();
        QString point_data_ow = savepoint.value("pointdata/" + get_point_name + "ow").toString();
        long int secs = (QDateTime::currentMSecsSinceEpoch() / 1000); // 获取1970后的秒数
        QTime current_time = QTime::currentTime();
        int msec = current_time.msec(); // 当前的毫秒
        long int ns = msec * 1000000;   // 换算成ns
        QString data = QString("{\"msg\":{\"header\":{\"frame_id\":\"map\",\"seq\":0,\"stamp\":{\"secs\":%1,\"nsecs\":%2}},\"pose\":{\"position\":{\"x\":%3,\"y\":%4,\"z\":0},\"orientation\":{\"x\":0,\"y\":0,\"z\":%5,\"w\":%6}}},\"op\":\"publish\",\"topic\":\"/move_base_simple/goal\"}").arg(secs).arg(ns).arg(point_data_x).arg(point_data_y).arg(point_data_oz).arg(point_data_ow);
        m_websocket.sendTextMessage(data);
    }
}

// 取消订阅和重新订阅话题槽
void MainWindow::setting_checkbox_slot(QString name, bool ischecked)
{
    if (!ischecked) {
        QString unsub = QString("{\"op\":\"unsubscribe\",\"topic\":\"%1\"}").arg(name);
        m_websocket.sendTextMessage(unsub);
        QSettings checkbox_setting("./subsetting.ini", QSettings::IniFormat);
        checkbox_setting.setValue("checkbox_setting/" + name, ischecked);
    } else {
        QString sub = QString("{\"op\":\"subscribe\",\"topic\":\"%1\"}").arg(name);
        m_websocket.sendTextMessage(sub);
        QSettings checkbox_setting("./subsetting.ini", QSettings::IniFormat);
        checkbox_setting.setValue("checkbox_setting/" + name, ischecked);
    }
}
