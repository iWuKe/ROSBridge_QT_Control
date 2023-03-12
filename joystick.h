#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <QWidget>

namespace Ui
{
class Joystick;
}

class Joystick : public QWidget
{
    Q_OBJECT
  public:
    explicit Joystick(QWidget *parent = nullptr);
    int getX() const;
    int getY() const;
    void setKeepInBounds(bool enable);

  signals:
    void getxy(int x, int y);

  protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

  private:
    QPoint centerPoint;
    QPoint currentPoint;
    bool isPressed = false;
    bool keepInBounds; // 新增成员变量
};

#endif // JOYSTICK_H
