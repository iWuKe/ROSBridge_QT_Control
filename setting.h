#ifndef SETTING_H
#define SETTING_H

#include <QWidget>

namespace Ui
{
class setting;
}

class setting : public QWidget
{
    Q_OBJECT

  public:
    explicit setting(QWidget *parent = nullptr);
    ~setting();
    void checkbox_init(); // 打开设置时初始化checkbox;

  signals:
    void point_add(QString point_name); // 添加点位信号
    void point_del(QString point_name); // 删除点位信号
    void point_del_all();               // 清除所有点位信号
    void checkbox_signal(QString name, bool ischecked);

  private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_checkBox_toggled(bool checked);

    void on_checkBox_2_toggled(bool checked);

    void on_checkBox_3_toggled(bool checked);

    void on_checkBox_4_toggled(bool checked);

    void on_checkBox_5_toggled(bool checked);

    void on_checkBox_6_toggled(bool checked);

private:
    Ui::setting *ui;
};

#endif // SETTING_H
