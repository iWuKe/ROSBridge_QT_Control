#include "setting.h"
#include "ui_setting.h"
#include <QSettings>

setting::setting(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::setting)
{
    ui->setupUi(this);
}

setting::~setting()
{
    delete ui;
}

void setting::on_pushButton_clicked()
{
    QString new_point_name = ui->lineEdit->text();
    QSettings settings("./pointslist.ini", QSettings::IniFormat);
    int size = settings.beginReadArray("pointslist");//读取已存在的点位个数
    for(int i = 0;i < size;i++)
    {
        settings.setArrayIndex(i);
        QString pointnumber;
        QString getdata = settings.value("point "+pointnumber.setNum(i)).toString();
        if(getdata == new_point_name)//检查是否有重名
        {
            new_point_name = new_point_name+"_";
        }
    }
    settings.endArray();
    settings.beginWriteArray("pointslist");
    settings.setArrayIndex(size);
    QString pointnumber;
    settings.setValue("point "+pointnumber.setNum(size), new_point_name);
    settings.endArray();
    emit point_add(new_point_name);//增加了新点位，发射信号
}

void setting::on_pushButton_2_clicked()
{
    QString point_name = ui->lineEdit->text();
    QSettings settings("./pointslist.ini", QSettings::IniFormat);
    int size = settings.beginReadArray("pointslist");//读取已存在的点位个数
    for(int i = 0;i < size;i++)
    {
        settings.setArrayIndex(i);
        QString pointnumber;
        QString getdata = settings.value("point "+pointnumber.setNum(i)).toString();
        if(getdata == ui->lineEdit->text())
        {
            settings.remove("point "+pointnumber.setNum(i));//删除后编号不减
            emit point_del(point_name);//删除了点位，发射信号
        }
    }
    settings.endArray();
}


void setting::on_pushButton_3_clicked()//清除其中所有内容
{

    QSettings settings("./pointslist.ini", QSettings::IniFormat);
    int size = settings.beginReadArray("pointslist");//读取已存在的点位个数
    QString pointnumber;
    for(int i = 0;i < size;i++)
    {
        settings.clear();
        settings.remove("point "+pointnumber.setNum(i));
        settings.sync();
    }
    settings.endArray();
    emit point_del_all();
}
