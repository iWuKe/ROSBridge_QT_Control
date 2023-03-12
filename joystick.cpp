#include "joystick.h"
#include <QLine>
#include <QtWidgets>

Joystick::Joystick(QWidget *parent)
    : QWidget(parent)
{
    setFixedSize(300, 300);
    centerPoint = rect().center();
    currentPoint = centerPoint; // 将 currentPoint 初始化为 centerPoint
    isPressed = false;
    keepInBounds = true; // 初始化为 true
}

void Joystick::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 绘制摇杆外圈
    painter.setBrush(Qt::darkGray);
    painter.drawEllipse(rect().center(), 110, 110); // 110是大小

    // 绘制摇杆内圈
    painter.setBrush(QColor(0x00, 0xBF, 0xFF));
    painter.drawEllipse(currentPoint, 30, 30);
}

void Joystick::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && rect().contains(event->pos())) {
        QPoint newPos = event->pos();
        currentPoint = newPos;
        int x = currentPoint.x() - centerPoint.x();
        int y = -1 * (currentPoint.y() - centerPoint.y());
        emit getxy(x, y); // y是前进速度，x是转向速度
        currentPoint = event->pos();
        isPressed = true;
        update();
    }
}

void Joystick::mouseMoveEvent(QMouseEvent *event)
{
    if (isPressed) {
        QPoint newPos = event->pos();

        // 计算距离和方向
        int distance = QLineF(centerPoint, newPos).length();
        // QPointF direction = QPointF(newPos - centerPoint) / distance;

        // 如果超出了范围且启用了保持在边界，则强制限制坐标
        if (distance > 110 && keepInBounds) {
            // newPos = centerPoint + direction.toPoint() * 110;//强制限制坐标
            // 以下可以绕着边界移动
            qreal radius = 110.0;
            QPointF centerToPoint = newPos - centerPoint;
            centerToPoint *= radius / distance;
            newPos = centerPoint + centerToPoint.toPoint();
            currentPoint = newPos;
        }

        // 否则，如果超出了范围但没有启用保持在边界，则将当前点设置为新位置
        else if (distance > 110) {
            currentPoint = newPos;
        }

        // 如果没有超出范围，则将当前点设置为新位置
        else {
            currentPoint = newPos;
        }
        int x = currentPoint.x() - centerPoint.x();
        int y = -1 * (currentPoint.y() - centerPoint.y());
        emit getxy(x, y);
        update();
    }
}

void Joystick::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        currentPoint = centerPoint;
        isPressed = false;
        emit getxy(0, 0);
        update();
    }
}
// 要在外部访问当前摇杆的X和Y坐标，请在Joystick类中添加以下两个公共函数：
int Joystick::getX() const
{
    return currentPoint.x() - centerPoint.x();
}

int Joystick::getY() const
{
    return -1 * (currentPoint.y() - centerPoint.y());
}

void Joystick::setKeepInBounds(bool enable)
{
    keepInBounds = enable;
}

// ui->joystick->setKeepInBounds(true);//需要启用保持在边界
