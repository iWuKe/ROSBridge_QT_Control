#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "joystick.h"
#include "math.h"
#include "roboItem.h"
#include "setting.h"
#include <QDateTime>
#include <QFileDialog>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMainWindow>
#include <QPointF>
#include <QSettings>
#include <QString>
#include <QThread>
#include <QTimer>
#include <QWebSocket>
#include <QMessageBox>

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    // 初始加载保存的IP和Port
    void ip_init();
    // 场景初始化
    void initUis();
    // 连接Websocket后需初始化的内容
    void init();
    // 已保存点位的初始化
    void combox_init();

    // 订阅地图信息
    void readmap();
    // 订阅雷达信息
    void read_laser();
    // 控制速度
    void cmd_vel(QString x, QString z);
    // 订阅位置信息
    void getrobot_pose();
    // 订阅机器人位置信息
    void read_car_state();
    // 订阅全局路径
    void GlobalPlan();
    //订阅导航结果
    void read_movebase_result();

    // 地图信息处理函数
    void map_data_handle(QString data);
    // 雷达信息处理
    void laserScan_data_handle(QString laser_msg);
    // 位置信息处理
    void get_RobotPose(QString posedata);
    // 状态信息处理
    void get_car_state(QString car_state_data);
    // 路径信息处理
    void get_plan_data(QString data);
    //导航结果信息处理
    void movebase_result_data_handle(QString data);
    // String转json函数
    QJsonObject QstringToJson(QString jsonString);

    // 传入处理后的地图信息
    void mapCallback(QJsonValue width, QJsonValue height, QJsonValue map, QJsonValue resolution, QJsonValue positionx, QJsonValue positiony);
    // 传入处理后的路径信息
    void plannerPathCallback(float x[], float y[], int length);

    QPointF transScenePoint2Word(QPointF pose);
    QPointF transWordPoint2Scene(QPointF pose);

    // 连接信号与槽
    void allconnect();
    // 连接点位设置的信号与槽
    void about_setting_singal();

  public slots:
    void pub2DPose(QPointF start_pose, QPointF end_pose);
    void pub2DGoal(QPointF start_pose, QPointF end_pose);
    void receivexy(int x, int y); // 接收摇杆xy
    void point_add(QString name); // 点位信号连接槽
    void point_del(QString name);
    void point_del_all();
    void update_car_state_ui(QString Charge_State, QString Battery_Voltage);
    void update_movebase_result_ui(QString data);
    void setting_checkbox_slot(QString name, bool ischecked);

  Q_SIGNALS:
    // void updateMap(QImage map);
    // void updateRobotPose(double x,double y,double theta);
    // 触发画图的信号
    void update_laser(QPolygonF);
    void update_map(QImage);
    void update_robot_pose(double x, double y, double theta);
    void update_plan(QPolygonF);
    void update_car_state(QString, QString);
    void update_movebase_result(QString);

  private Q_SLOTS:
    // 收到message槽
    void onTextMessageReceived(const QString &message);

  private slots:
    void on_pushButton_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_14_clicked();

    void on_pushButton_18_clicked();

    void on_pushButton_15_clicked();

    void on_pushButton_19_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_12_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_9_clicked();

    void on_pushButton_11_clicked();

    void on_pushButton_17_clicked();

    void on_pushButton_13_clicked();

    void on_pushButton_21_clicked();

    void on_pushButton_22_clicked();

  private:
    Ui::MainWindow *ui;

    // Websocket
    QWebSocket m_websocket;
    // 定时器，用于显示时间
    QTimer timer;

    // 地图 0 0点坐标对应世界坐标系的坐标
    float m_mapOriginX;
    float m_mapOriginY;
    // 世界坐标系原点在图元坐标系坐标
    QPointF m_wordOrigin;
    // 地图一个像素对应真实世界的距离
    float m_mapResolution;
    // 地图是否被初始化
    bool m_bMapIsInit = false;
    // QImage Mat2QImage(cv::Mat const &src);
    // cv::Mat QImage2Mat(QImage &image);

    QImage rotateMapWithY(QImage map);
    // 全局路径
    QPolygonF plannerPoints;
    // 雷达
    QPolygonF laserPoints;
    // 场景
    QGraphicsScene *m_qgraphicsScene = NULL;
    // 绘画
    Ui::roboItem *m_roboItem = NULL;
    // 摇杆
    Joystick *joystick = NULL;
    // 点位设置
    setting *s = NULL;
};

// 线程类
class MyThread : public QThread
{

  public:
    MainWindow *w = NULL;
    // 接收的信息
    QString message = "";
    int select = 0;
    void run()
    {
        switch (select) {
        case 1:
            w->map_data_handle(message);
            break;
        case 2:
            w->get_RobotPose(message);
            break;
        case 3:
            w->get_car_state(message);
            break;
        case 4:
            w->get_plan_data(message);
            break;
        case 5:
            w->laserScan_data_handle(message);
            break;
        case 6:
            w->movebase_result_data_handle(message);
            break;
        }
    }
};

#endif // MAINWINDOW_H
