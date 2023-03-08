#ifndef SETTING_H
#define SETTING_H

#include <QWidget>

namespace Ui {
class setting;
}

class setting : public QWidget
{
    Q_OBJECT

public:
    explicit setting(QWidget *parent = nullptr);
    ~setting();

signals:
    void point_add(QString point_name);//添加点位信号
    void point_del(QString point_name);//删除点位信号
    void point_del_all();//清除所有点位信号

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

private:
    Ui::setting *ui;
};

#endif // SETTING_H
