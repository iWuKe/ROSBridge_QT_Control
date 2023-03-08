#include "mainwindow.h"
#include "ui_mainwindow.h"
QString save_map;//用于保存地图数据
QString point_msg_x;//用于保存点位x数据
QString point_msg_y;//用于保存点位y数据
QString point_msg_oz;//用于保存点位oz数据
QString point_msg_ow;//用于保存点位ow数据
double robot_positionx;//机器人位置信息的x
double robot_positiony;//机器人位置信息的y
bool isconnected = false;//是否连接的标志位
//MyThread mythrad;

struct Quaternion//四元数结构体
{
    double w, x, y, z;
};
typedef struct {
    double yaw, pitch, roll;
}EulerAngle;
EulerAngle e;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("荣师傅牛逼");
    //s = new setting;
    joystick = new Joystick(ui->myWidget);//摇杆
    //ui->myWidget->setFixedSize(300, 300);
    connect(joystick, SIGNAL(getxy(int,int)), this,SLOT(receivexy(int,int)));//连接摇杆的信号
    QRegExp exp("[0-9\\.]+$");
    QValidator *Validator = new QRegExpValidator(exp);//设置lineEdit只能输入数字和小数点
    ui->lineEdit->setValidator(Validator);

    timer.setInterval(1000); // 设置定时器间隔为1秒
    timer.start(); // 启动定时器
    ui->progressBar->setValue(0);//进度条，显示电量


    connect(&m_websocket, SIGNAL(textMessageReceived(QString)), this, SLOT(onTextMessageReceived(QString)));//连接接收数据成功信号
    connect(&m_websocket,&QWebSocket::connected,this,[=]()//连接成功信号
    {
        isconnected = true;
        ui->pushButton->setText("断开");
        ui->statu_led->setStyleSheet("background-color:#8aff0c;"
                                     "border-radius: 10px;"
                                     "border:1px solid black;");//修改连接状态的label
        ui->statu_led->setText("已连接");
        init();
        allconnect();
    });
    connect(&m_websocket,&QWebSocket::disconnected,this,[=]()//连接断开信号
    {
        isconnected = false;
        ui->statu_led->setStyleSheet("background-color:rgb(255,0,0);"
                                     "border-radius: 10px;"
                                     "border:1px solid black;");
        ui->pushButton->setText("连接");
        ui->statu_led->setText("未连接");
    });
    QObject::connect(&timer, &QTimer::timeout, [&](){
        // 在槽函数中更新label的文本为当前时间
        ui->label_8->setText(QDateTime::currentDateTime().toString("HH:mm:ss"));
    });
    initUis();//场景的初始化
    //about_point_singal();//所有点位相关的信号连接
    combox_init();//选点导航初始化已有点位
    ip_init();//初始化上次连接IP和端口
}

MainWindow::~MainWindow()
{
    delete ui;
    m_websocket.close();
}

void MainWindow::allconnect()
{
    //connect(this, SIGNAL(updateMap(QImage)), m_roboItem,SLOT(paintMaps(QImage)));
    //connect(this, SIGNAL(plannerPath(QPolygonF)), m_roboItem,SLOT(paintPlannerPath(QPolygonF)));
    //connect(this, SIGNAL(updateRoboPose(double x,double y,double theta)), m_roboItem,SLOT(paintRoboPos(double x,double y,double theta)));

    connect(m_roboItem,SIGNAL(signalPub2DPose(QPointF,QPointF)),this,SLOT(pub2DPose(QPointF,QPointF)));
    connect(m_roboItem,SIGNAL(signalPub2DGoal(QPointF,QPointF)),this,SLOT(pub2DGoal(QPointF,QPointF)));
    //设置2D Pose
    connect(ui->set_pos_btn, SIGNAL(clicked()), m_roboItem, SLOT(slot_set2DPos()));
    //设置2D goal
    connect(ui->set_goal_btn, SIGNAL(clicked()), m_roboItem, SLOT(slot_set2DGoal()));
}

void MainWindow::init()//连接后加载
{
    readmap();//订阅地图话题map，来自map_server
    getrobot_pose();//订阅机器人位置，属自定义话题，话题robot_pose
    car_state_init();//订阅状态，属自定义消息，话题car_state
    GlobalPlan();//订阅全局路径move_base/TebLocalPlannerROS/global_plan
    laser_init();//订阅雷达消息，话题scan_filtered
}

void MainWindow::connectToServer()
{
    QString path = QString("ws://%1:%2").arg(ui->iplineedit->text()).arg(ui->portspinbox->text());
    QUrl url = QUrl(path);
    m_websocket.open(url);
}

void MainWindow::on_pushButton_clicked()//连接与断开
{
    if(isconnected == false)
    {
        connectToServer();
        QSettings ip("./setting.ini", QSettings::IniFormat);
        ip.setValue("WS/IP", ui->iplineedit->text());
        ip.setValue("WS/Port", ui->portspinbox->text());
    }
    else
    {
        m_websocket.close();
    }
}


void MainWindow::onTextMessageReceived(const QString &message)
{

    if(message.contains("/map"))//如果是地图信息
    {
//        mythrad.select = 1;
//        save_map = message;
//        mythrad.message = message;
//        mythrad.start();
        map_data_paint(message);
    }
    else if(message.contains("/robot_pose"))
    {
//        mythrad.select = 2;
//        mythrad.message = message;
//        mythrad.start();
        get_RobotPose(message);
    }
    else if(message.contains("/car_state"))
    {
//        mythrad.select = 3;
//        mythrad.message = message;
//        mythrad.start();
        get_car_state(message);
    }
    else if(message.contains("/move_base/TebLocalPlannerROS/global_plan"))
    {
        get_plan_data(message);
    }
    else if(message.contains("/scan_filtered"))
    {
        laserScanCallback(message);
    }
}

void MainWindow::onSendButtonClicked()//发送
{
    QString msg = ui->sendmessagetextedit->document()->toPlainText();
    m_websocket.sendTextMessage(msg);
}
void MainWindow::on_pushButton_3_clicked()
{
    onSendButtonClicked();
}

void MainWindow::on_pushButton_14_clicked()//进桩
{
    QString data = "{\"msg\":{\"data\":\"ReCharge\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::on_pushButton_18_clicked()//关机
{
    QString data = "{\"msg\":{\"data\":\"disReCharge\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::on_pushButton_15_clicked()//停止进桩
{
    QString data = "{\"msg\":{\"data\":\"shut_down\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::on_pushButton_19_clicked()//读取地图（加载地图）,
//注：这一部分未实现，水平不够，没研究出来怎么用websocket去启动地图服务
//我在这段代码这里写的是你文本里的内容用websocket发送
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

void MainWindow::readmap()//获取地图信息
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/map\"}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::readlaser()//获取雷达信息
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/scan_filtered\"}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::getrobot_pose()//获取机器人位置
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/robot_pose\"}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::cmd_vel(QString x,QString z)//速度消息
{
    QString data = QString("{\"msg\":{\"angular\":{\"x\":0,\"y\":0,\"z\":%1},\"linear\":{\"x\":%2,\"y\":0,\"z\":0}},\"op\":\"publish\",\"topic\":\"/cmd_vel\"}").arg(z).arg(x);
    m_websocket.sendTextMessage(data);
}

void MainWindow::on_pushButton_4_clicked()//停止
{
    cmd_vel("0","0");
}

void MainWindow::on_pushButton_12_clicked()//停止导航，即关闭move_base
{
    QString data = "{\"msg\":{\"data\":\"move_base_stop\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::on_pushButton_10_clicked()//停止建图
{
    QString data = "{\"msg\":{\"data\":\"gmapping_stop\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::on_pushButton_9_clicked()//启动建图，这个是我在ROS程序里写了通过control_cmd话题收到gmapping_start这个文本时，调用gmapping启动建图
{
    QString data = "{\"msg\":{\"data\":\"gmapping_start\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::on_pushButton_11_clicked()//重启导航，启动move_base
{
    QString data = "{\"msg\":{\"data\":\"move_base_start\"},\"op\":\"publish\",\"topic\":\"/control_cmd\"}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::on_pushButton_17_clicked()//终止到点
{
    QString data = "{\"op\":\"publish\",\"topic\":\"/move_base/cancel\",\"msg\":{\"stamp\":{\"secs\":0,\"nsecs\":0},\"id\":\"\"}}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::map_data_paint(QString data)//对收到的地图数据进行处理
{
    QJsonObject jsondata,msgdata,infodata,origin,position;
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
    QJsonValue map = msgdata["data"];//地图的所有数据
    //使用map[num];访问其中的第num+1个元素元素
    mapCallback(width,height,map,resolution,positionx,positiony);
}

QJsonObject MainWindow::QstringToJson(QString jsonString)//QString转Json程序
{
    QJsonDocument jsonDocument = QJsonDocument::fromJson(jsonString.toLocal8Bit().data());
    if(jsonDocument.isNull())
    {
        //qDebug()<< "String NULL"<< jsonString.toLocal8Bit().data();
    }
    QJsonObject jsonObject = jsonDocument.object();
    return jsonObject;
}

//地图信息订阅回调函数，在此函数传入map_data_paint处理后的数据，使用roboItem定义好的paintMaps(map_image)，就可以在GraphiceView中画出地图，参考了古月居的程序
void MainWindow::mapCallback(QJsonValue width,QJsonValue height,QJsonValue map,QJsonValue resolution,QJsonValue positionx,QJsonValue positiony) {
//  int width = msg->info.width;
//  int height = msg->info.height;
  int size;
  size = width.toInt()*height.toInt();
  m_mapResolution = resolution.toDouble();//msg->info.resolution;
  double origin_x = positionx.toDouble();//msg->info.origin.position.x;
  double origin_y = positiony.toDouble();//msg->info.origin.position.y;
  QImage map_image(width.toInt(), height.toInt(), QImage::Format_RGB32);
  for (int i = 0; i < size; i++) {//想办法获取map的长度
    int x = i % width.toInt();
    int y = (int)i / width.toInt();
    //计算像素值
    QColor color;
    if (map[i] == 100) {
      color = Qt::black;  // black
    } else if (map[i] == 0) {
      color = Qt::white;  // white
    } else if (map[i] == -1) {
      color = Qt::gray;  // gray
    }
    map_image.setPixel(x, y, qRgb(color.red(), color.green(), color.blue()));
  }
  //延y翻转地图 因为解析到的栅格地图的坐标系原点为左下角
  //但是图元坐标系为左上角度
  map_image = rotateMapWithY(map_image);
  //emit updateMap(map_image);
  //updateMap(map_image);
  m_roboItem->paintMaps(map_image);
  //m_roboItem->paintMaps(map_image);
  //计算翻转后的图元坐标系原点的世界坐标
  double origin_x_ = origin_x;
  double origin_y_ = origin_y + height.toInt() * m_mapResolution;
  //世界坐标系原点在图元坐标系下的坐标
  m_wordOrigin.setX(fabs(origin_x_) / m_mapResolution);
  m_wordOrigin.setY(fabs(origin_y_) / m_mapResolution);
}

QImage MainWindow::rotateMapWithY(QImage map) {//沿Y轴翻转
  QImage res = map;
  for (int x = 0; x < map.width(); x++) {
    for (int y = 0; y < map.height(); y++) {
      res.setPixelColor(x, map.height() - y - 1, map.pixel(x, y));
    }
  }
  return res;
}

void MainWindow::initUis() {//
  ui->mapViz->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);//消除滚动条
  ui->mapViz->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  //视图场景加载
  m_qgraphicsScene =
      new QGraphicsScene;  //要用QGraphicsView就必须要有QGraphicsScene搭配着用
  m_qgraphicsScene->clear();
  //创建item
  m_roboItem = new Ui::roboItem();
  //视图添加item
  m_qgraphicsScene->addItem(m_roboItem);
  //设置item的坐标原点与视图的原点重合（默认为视图中心）
  // widget添加视图
  ui->mapViz->setScene(m_qgraphicsScene);
}

EulerAngle quaternionToEuler(Quaternion q) {//四元数转欧拉角
    //EulerAngle euler;
    // 计算yaw、pitch、roll
    e.yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    e.pitch = asin(2 * (q.w * q.y - q.z * q.x));
    e.roll = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    return e;
}
Quaternion ToQuaternion(double yaw, double pitch, double roll) // 欧拉角转四元数
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
void MainWindow::pub2DPose(QPointF start_pose,QPointF end_pose){//设置初始位置
    start_pose =transScenePoint2Word(start_pose);
    end_pose =transScenePoint2Word(end_pose);
    double angle = atan2(end_pose.y()-start_pose.y(),end_pose.x()-start_pose.x());
    long int secs = (QDateTime::currentMSecsSinceEpoch() / 1000);//获取1970后的秒数
    QTime current_time =QTime::currentTime();
    int msec = current_time.msec();//当前的毫秒
    long int ns = msec*1000000;//换算成ns
    QString data = QString("{\"msg\":{\"header\":{\"frame_id\":\"map\",\"seq\":0,\"stamp\":{\"secs\":%1,\"nsecs\":%2}},\"pose\":{\"covariance\":[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787],\"pose\":{\"position\":{\"x\":%3,\"y\":%4,\"z\":0},\"orientation\":{\"x\":0,\"y\":0,\"z\":%5,\"w\":%6}}}},\"op\":\"publish\",\"topic\":\"/initialpose\"}").arg(secs).arg(ns).arg(start_pose.x()).arg(start_pose.y()).arg(ToQuaternion(angle,0,0).z).arg(ToQuaternion(angle,0,0).w);;
    m_websocket.sendTextMessage(data);
}
void MainWindow::pub2DGoal(QPointF start_pose,QPointF end_pose){//设置目标点
    start_pose =transScenePoint2Word(start_pose);
    end_pose =transScenePoint2Word(end_pose);
    double angle = atan2(end_pose.y()-start_pose.y(),end_pose.x()-start_pose.x());
    long int secs = (QDateTime::currentMSecsSinceEpoch() / 1000);//获取1970后的秒数
    QTime current_time =QTime::currentTime();
    int msec = current_time.msec();//当前的毫秒
    long int ns = msec*1000000;//换算成ns
    QString msg = QString("{\"msg\":{\"header\":{\"frame_id\":\"map\",\"seq\":0,\"stamp\":{\"secs\":%1,\"nsecs\":%2}},\"pose\":{\"position\":{\"x\":%3,\"y\":%4,\"z\":0},\"orientation\":{\"x\":0,\"y\":0,\"z\":%5,\"w\":%6}}},\"op\":\"publish\",\"topic\":\"/move_base_simple/goal\"}").arg(secs).arg(ns).arg(start_pose.x()).arg(start_pose.y()).arg(ToQuaternion(angle,0,0).z).arg(ToQuaternion(angle,0,0).w);
    m_websocket.sendTextMessage(msg);
}

QPointF MainWindow::transScenePoint2Word(QPointF pose) {
  QPointF res;
  res.setX((pose.x() - m_wordOrigin.x()) * m_mapResolution);
  // y坐标系相反
  res.setY(-1 * (pose.y() - m_wordOrigin.y()) * m_mapResolution);
  return res;
}
QPointF MainWindow::transWordPoint2Scene(QPointF pose) {
  QPointF res;
  res.setX(m_wordOrigin.x() + pose.x() / m_mapResolution);
  res.setY(m_wordOrigin.y() - (pose.y() / m_mapResolution));
  return res;
}

void MainWindow::get_RobotPose(QString posedata)//对机器人位置数据处理，我自己根据move_base改出来的用于获取机器人位置的信息
{
    QJsonObject jsondata,msgdata,orientation,position;
    jsondata = QstringToJson(posedata);
    msgdata = jsondata["msg"].toObject();//提取msg内数据
    position = msgdata["position"].toObject();
    orientation = msgdata["orientation"].toObject();
    robot_positionx = position["x"].toDouble();
    point_msg_x = point_msg_x.setNum(robot_positionx);
    robot_positiony = position["y"].toDouble();
    point_msg_y = point_msg_y.setNum(robot_positiony);
    //坐标转化为图元坐标系
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
    m_roboItem->paintRoboPos(roboPos.x(),roboPos.y(),e.yaw);
}

void MainWindow::plannerPathCallback(float x[],float y[],int length) {//对路径消息的处理
  plannerPoints.clear();
  for (int i = 0; i < length; i++) {
    QPointF roboPos = transWordPoint2Scene(QPointF(x[i], y[i]));
    plannerPoints.append(roboPos);
  }
  m_roboItem->paintPlannerPath(plannerPoints);
}

void MainWindow::get_car_state(QString car_state_data)//对car_state信息的处理
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
    ui->label_7->setText(Charge_State.toString());
    ui->label_6->setText(Battery_Voltage.toString());
    int Battery_Voltage_Int = (Battery_Voltage.toString()).toInt();
    int power = (Battery_Voltage_Int-21500)*100/(29300-21500);//更新电量
    ui->progressBar->setValue(power);
    if(power>=75)
    {
        QString style;
        style = "QProgressBar {border: 2px solid gray;border-radius: 10px;background-color: white;height: 30px;}QProgressBar::chunk {background-color: #a0d468;border-radius:8px;}";
        ui->progressBar->setStyleSheet(style);
    }
    else if(power>=50&&power<75)
    {
        QString style;
        style = "QProgressBar {border: 2px solid gray;border-radius: 10px;background-color: white;height: 30px;}QProgressBar::chunk {background-color: #ffa008;border-radius:8px;}";
        ui->progressBar->setStyleSheet(style);
    }
    else if(power>=25&&power<50)
    {
        QString style;
       style = "QProgressBar {border: 2px solid gray;border-radius: 10px;background-color: white;height: 30px;}QProgressBar::chunk {background-color: #ffff00;border-radius:8px;}";
       ui->progressBar->setStyleSheet(style);
    }
    else
    {
        QString style;
       style = "QProgressBar {border: 2px solid gray;border-radius: 10px;background-color: white;height: 30px;}QProgressBar::chunk {background-color: #ff4b7b;border-radius:8px;}";
       ui->progressBar->setStyleSheet(style);
    }
}

void MainWindow::on_pushButton_13_clicked()//保存地图
//实力有限，并没实现，不会用rosbridge实现保存地图
//这里写的是将地图话题map的信息保存成文本
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

void MainWindow::receivexy(int x,int y)//设置遥感的倍率
{
    float beilv = ui->lineEdit->text().toDouble();//摇杆倍率
    float lx,ly;
    lx = beilv * x/50;
    ly = beilv * y/50;
    QString lx_str,ly_str;
    cmd_vel(ly_str.setNum(ly),lx_str.setNum(-lx));//调转x,y方向
}

void MainWindow::on_pushButton_21_clicked()
{
    if(s!=nullptr) delete s;
    s = new setting;
    s->show();//打开点位设置页
    about_point_singal();//所有点位相关的信号连接
}

void MainWindow::about_point_singal()//点位改动信号的连接
{
    connect(s, SIGNAL(point_add(QString)), this,SLOT(point_add(QString)));//连接新增信号
    connect(s, SIGNAL(point_del(QString)), this,SLOT(point_del(QString)));//连接删除信号
    connect(s, SIGNAL(point_del_all()), this,SLOT(point_del_all()));//连接重置信号
}

void MainWindow::point_add(QString name)//新增并保存点位
{
    ui->comboBox->addItem(name);
    //创建QSettings对象并指定ini文件路径并将格式设置为ini
    QSettings savepoint("./pointdata.ini", QSettings::IniFormat);
    savepoint.setValue("pointdata/"+name+"x", point_msg_x);
    savepoint.setValue("pointdata/"+name+"y", point_msg_y);
    savepoint.setValue("pointdata/"+name+"oz", point_msg_oz);
    savepoint.setValue("pointdata/"+name+"ow", point_msg_ow);
}
void MainWindow::point_del(QString name)//删除点位
{
//    int count = ui->comboBox->count();
    int flag = ui->comboBox->findText(name,Qt::MatchExactly);
    ui->comboBox->removeItem(flag);
    QSettings savepoint("./pointdata.ini", QSettings::IniFormat);
    savepoint.remove("pointdata/"+name+"x");
    savepoint.remove("pointdata/"+name+"y");
    savepoint.remove("pointdata/"+name+"oz");
    savepoint.remove("pointdata/"+name+"ow");
}
void MainWindow::point_del_all()//删除所有点位
{
    ui->comboBox->clear();
    QSettings savepoint("./pointdata.ini", QSettings::IniFormat);
    savepoint.clear();
    savepoint.sync();
}
void MainWindow::combox_init()//启动时加载已保存的点位
{
    QSettings settings("./pointslist.ini", QSettings::IniFormat);
    int size = settings.beginReadArray("pointslist");
    for(int i = 0;i < size;i++)
    {
        settings.setArrayIndex(i);
        QString pointnumber;
        QString getdata = settings.value("point "+pointnumber.setNum(i)).toString();
        ui->comboBox->addItem(getdata);
    }
    settings.endArray();
}

void MainWindow::on_pushButton_22_clicked()//导航到点位
{
    QString get_point_name = ui->comboBox->currentText();//当前选中的项
    if(get_point_name != "")
    {
        QSettings savepoint("./pointdata.ini", QSettings::IniFormat);
        QString point_data_x= savepoint.value("pointdata/"+get_point_name+"x").toString();
        QString point_data_y= savepoint.value("pointdata/"+get_point_name+"y").toString();
        QString point_data_oz= savepoint.value("pointdata/"+get_point_name+"oz").toString();
        QString point_data_ow= savepoint.value("pointdata/"+get_point_name+"ow").toString();
        long int secs = (QDateTime::currentMSecsSinceEpoch() / 1000);//获取1970后的秒数
        QTime current_time =QTime::currentTime();
        int msec = current_time.msec();//当前的毫秒
        long int ns = msec*1000000;//换算成ns
        QString data = QString("{\"msg\":{\"header\":{\"frame_id\":\"map\",\"seq\":0,\"stamp\":{\"secs\":%1,\"nsecs\":%2}},\"pose\":{\"position\":{\"x\":%3,\"y\":%4,\"z\":0},\"orientation\":{\"x\":0,\"y\":0,\"z\":%5,\"w\":%6}}},\"op\":\"publish\",\"topic\":\"/move_base_simple/goal\"}").arg(secs).arg(ns).arg(point_data_x).arg(point_data_y).arg(point_data_oz).arg(point_data_ow);
        m_websocket.sendTextMessage(data);
    }
}

void MainWindow::car_state_init()//订阅状态话题，自定义消息
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/car_state\"}";
    m_websocket.sendTextMessage(data);
}
void MainWindow::ip_init()//开启软件时装在上次连接的IP和Port
{
    QSettings ip("./setting.ini", QSettings::IniFormat);
    ip.setValue("TCP/IP", ui->iplineedit->text());
    ip.setValue("TCP/Port", ui->iplineedit->text());
    ui->iplineedit->setText(ip.value("WS/IP").toString());
    ui->portspinbox->setText(ip.value("WS/Port").toString());
}
void MainWindow::GlobalPlan()//订阅全局路线
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/move_base/TebLocalPlannerROS/global_plan\"}";
    m_websocket.sendTextMessage(data);
}

void MainWindow::get_plan_data(QString data)//对plan数据的处理
{
        // json数据
        QByteArray jsonStr = data.toUtf8();;
        // 将json数据解析为QJsonDocument
        QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonStr);
        // 获取msg对象
        QJsonObject msgObj = jsonDoc.object()["msg"].toObject();
        // 获取poses数组
        QJsonArray posesArr = msgObj["poses"].toArray();
        // 定义一个数组来存储position下的x
        //QVector<double> xArr,yArr;
        // 遍历poses数组
        int length = posesArr.size();//获取数组长度
        float xArr[length],yArr[length];
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
        plannerPathCallback(xArr,yArr,length);
}

void MainWindow::laser_init()//雷达订阅
{
    QString data = "{\"op\":\"subscribe\",\"topic\":\"/scan_filtered\"}";
    m_websocket.sendTextMessage(data);
}
//激光雷达点云话题回调
void MainWindow::laserScanCallback(QString laser_msg)
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
    for(int i = 0; i < rangesArr.size(); i++)
    {
        // 如果数组元素为null，则将其转换为0
        if(rangesArr[i].isNull())
        {
            rangesVec.append(0);
        }
        else
        {
            rangesVec.append(rangesArr[i].toDouble());
        }
    }
//  geometry_msgs::PointStamped laser_point;
//  geometry_msgs::PointStamped map_point;
//  laser_point.header.frame_id = laser_msg->header.frame_id;
  laserPoints.clear();
  //转换到二维XY平面坐标系下;
  for (int i = 0; i < rangesArr.size(); i++) {
    // scan_laser坐标系下
    double angle = angle_min + i * angle_increment;
    double X = rangesVec[i] * cos(angle);
    double Y = rangesVec[i] * sin(angle);

    QTransform transform;//需要进行tf变换，此处的tf变换不严谨，因为我也不会
    //应该要变换到以base_footprint或者base_link的坐标系才是合适的，看看你定义的这两个哪个是小车旋转中心
    transform.translate(robot_positionx+0.15,robot_positiony);//这个变换以小车位置为准（车中心），x y是小车位置，雷达在小车x+方向0.15米
    //qDebug()<<e.yaw;
    transform.rotateRadians(e.yaw);//弧度
    QPointF point_A(X, Y);
    QPointF point_B = transform.map(point_A);
    double x_prime = point_B.x();
    double y_prime = point_B.y();
    //转化为图元坐标系
    QPointF roboPos = transWordPoint2Scene(QPointF(x_prime, y_prime));
//        transWordPoint2Scene(QPointF(map_point.point.x, map_point.point.y));
    laserPoints.append(roboPos);
  }
  m_roboItem->paintLaserScan(laserPoints);
}

