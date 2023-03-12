#include "setting.h"
#include "ui_setting.h"
#include <QDebug>
#include <QDesktopServices>
#include <QSettings>
#include <QUrl>
setting::setting(QWidget *parent) : QWidget(parent),
                                    ui(new Ui::setting)
{
    ui->setupUi(this);
    // 设置窗口标题
    setWindowTitle("设置");
    // 初始化checkbox
    checkbox_init();
}

setting::~setting()
{
    delete ui;
}

void setting::on_pushButton_clicked()
{
    QString new_point_name = ui->lineEdit->text();
    QSettings settings("./pointslist.ini", QSettings::IniFormat);
    int size = settings.beginReadArray("pointslist"); // 读取已存在的点位个数
    for (int i = 0; i < size; i++) {
        settings.setArrayIndex(i);
        QString pointnumber;
        QString getdata = settings.value("point " + pointnumber.setNum(i)).toString();
        if (getdata == new_point_name) // 检查是否有重名
        {
            new_point_name = new_point_name + "_";
        }
    }
    settings.endArray();
    settings.beginWriteArray("pointslist");
    settings.setArrayIndex(size);
    QString pointnumber;
    settings.setValue("point " + pointnumber.setNum(size), new_point_name);
    settings.endArray();
    emit point_add(new_point_name); // 增加了新点位，发射信号
}

void setting::on_pushButton_2_clicked()
{
    QString point_name = ui->lineEdit->text();
    QSettings settings("./pointslist.ini", QSettings::IniFormat);
    int size = settings.beginReadArray("pointslist"); // 读取已存在的点位个数
    for (int i = 0; i < size; i++) {
        settings.setArrayIndex(i);
        QString pointnumber;
        QString getdata = settings.value("point " + pointnumber.setNum(i)).toString();
        if (getdata == ui->lineEdit->text()) {
            settings.remove("point " + pointnumber.setNum(i)); // 删除后编号不减
            emit point_del(point_name);                        // 删除了点位，发射信号
        }
    }
    settings.endArray();
}

void setting::on_pushButton_3_clicked() // 清除其中所有内容
{

    QSettings settings("./pointslist.ini", QSettings::IniFormat);
    int size = settings.beginReadArray("pointslist"); // 读取已存在的点位个数
    QString pointnumber;
    for (int i = 0; i < size; i++) {
        settings.clear();
        settings.remove("point " + pointnumber.setNum(i));
        settings.sync();
    }
    settings.endArray();
    emit point_del_all();
}

void setting::on_pushButton_4_clicked()
{
    QUrl url("https://github.com/stars0628");
    QDesktopServices::openUrl(url);
}

void setting::checkbox_init()
{
    QSettings checkbox_setting("./subsetting.ini", QSettings::IniFormat);
    QStringList keys = checkbox_setting.allKeys();
    foreach (QString key, keys) {
        // 处理每一个key
        if (checkbox_setting.value(key).toString() == "false") {
            if (key == "checkbox_setting/scan_filtered")
                ui->checkBox->setChecked(false);
            if (key == "checkbox_setting/car_state")
                ui->checkBox_4->setChecked(false);
            if (key == "checkbox_setting/map")
                ui->checkBox_5->setChecked(false);
            if (key == "checkbox_setting/move_base/TebLocalPlannerROS/global_plan")
                ui->checkBox_2->setChecked(false);
            if (key == "checkbox_setting/robot_pose")
                ui->checkBox_3->setChecked(false);
            if (key == "checkbox_setting/move_base/result")
                ui->checkBox_6->setChecked(false);
        }
    }
}

// 是否勾选雷达
void setting::on_checkBox_toggled(bool checked)
{
    if (checked) {
        emit checkbox_signal("/scan_filtered", true);
    } else {
        emit checkbox_signal("/scan_filtered", false);
    }
}

// 是否勾选路径
void setting::on_checkBox_2_toggled(bool checked)
{
    if (checked) {
        emit checkbox_signal("/move_base/TebLocalPlannerROS/global_plan", true);
    } else {
        emit checkbox_signal("/move_base/TebLocalPlannerROS/global_plan", false);
    }
}

// 是否勾选机器人位置
void setting::on_checkBox_3_toggled(bool checked)
{
    if (checked) {
        emit checkbox_signal("/robot_pose", true);
    } else {
        emit checkbox_signal("/robot_pose", false);
    }
}

// 是否勾选状态
void setting::on_checkBox_4_toggled(bool checked)
{
    if (checked) {
        emit checkbox_signal("/car_state", true);
    } else {
        emit checkbox_signal("/car_state", false);
    }
}

// 是否勾选地图
void setting::on_checkBox_5_toggled(bool checked)
{
    if (checked) {
        emit checkbox_signal("/map", true);
    } else {
        emit checkbox_signal("/map", false);
    }
}

// 是否勾选导航结果
void setting::on_checkBox_6_toggled(bool checked)
{
    if (checked) {
        emit checkbox_signal("/move_base/result", true);
    } else {
        emit checkbox_signal("/move_base/result", false);
    }
}
