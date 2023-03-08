#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWebSocket>
#include <QLabel>
#include <QDateTime>
#include <QTimer>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>
#include <QPointF>
#include <QImage>
#include <QThread>
#include <QSettings>
#include <QJsonArray>
#include "math.h"
#include "roboItem.h"
#include "joystick.h"
#include "setting.h"
//#include "Mythread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void connectToServer();
    void closeConnection();
    void onSendButtonClicked();
    void setLED(QLabel* label, int color, int size);
    void readmap();//获取地图信息
    void readlaser();//获取雷达信息
    void init();
    void cmd_vel(QString x,QString z);
    void getrobot_pose();
    void map_data_paint(QString data);
    QJsonObject QstringToJson(QString jsonString);
    void mapCallback(QJsonValue width,QJsonValue height,QJsonValue map,QJsonValue resolution,QJsonValue positionx,QJsonValue positiony);
    void initUis();
    void allconnect();
    QPointF transScenePoint2Word(QPointF pose);
    QPointF transWordPoint2Scene(QPointF pose);
    void get_RobotPose(QString posedata);
    void get_car_state(QString car_state_data);
    void about_point_singal();
    void combox_init();
    void car_state_init();
    void ip_init();
    void GlobalPlan();
    void get_plan_data(QString data);
    void plannerPathCallback(float x[],float y[],int length);
    void laser_init();
    void laserScanCallback(QString laser_msg);

public slots:
    void pub2DPose(QPointF start_pose,QPointF end_pose);
    void pub2DGoal(QPointF start_pose,QPointF end_pose);
    void receivexy(int x,int y);//接收摇杆xy
    void point_add(QString name);//点位信号连接槽
    void point_del(QString name);
    void point_del_all();

Q_SIGNALS:
    //void updateMap(QImage map);
    //void updateRobotPose(double x,double y,double theta);

    private Q_SLOTS:
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

    QUrl m_url;
    QWebSocket m_websocket;
    QTimer timer;

    //地图 0 0点坐标对应世界坐标系的坐标
    float m_mapOriginX;
    float m_mapOriginY;
    //世界坐标系原点在图元坐标系坐标
    QPointF m_wordOrigin;
    //地图一个像素对应真实世界的距离
    float m_mapResolution;
    //地图是否被初始化
    bool m_bMapIsInit = false;
    //QImage Mat2QImage(cv::Mat const &src);
    //cv::Mat QImage2Mat(QImage &image);
    QImage rotateMapWithY(QImage map);
    QPolygonF plannerPoints;
    QPolygonF laserPoints;
    QGraphicsScene *m_qgraphicsScene = NULL;
    Ui::roboItem *m_roboItem = NULL;
    Joystick *joystick = NULL;
    setting *s = NULL;
};

class MyThread:public QThread
{
public:
    QString message="";
    void run(){
    }
};

#endif // MAINWINDOW_H
