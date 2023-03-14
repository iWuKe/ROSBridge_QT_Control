#include "roboItem.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QVector2D>
#include <QDebug>

namespace Ui
{
roboItem::roboItem()
{
    setAcceptHoverEvents(true);
    setAcceptedMouseButtons(Qt::AllButtons);
    setAcceptDrops(true);
    setFlag(ItemAcceptsInputMethod, true);
    moveBy(0, 0);
    m_moveCursor = new QCursor(QPixmap("://images/cursor_move"), 0, 0);
    set2DPoseCursor = new QCursor(QPixmap("://images/cursor_pos.png"), 0, 0);
    set2DGoalCursor = new QCursor(QPixmap("://images/cursor_pos.png"), 0, 0);
    setRobotColor(eRobotColor::red);
    setDefault();
}
void roboItem::setRobotColor(eRobotColor color)
{
    switch (color) {
    case eRobotColor::blue: {
        robotImg.load("://images/robot_blue.png");
    } break;
    case eRobotColor::red: {
        robotImg.load("://images/robot_red.png");
    } break;
    case eRobotColor::yellow: {
        robotImg.load("://images/robot_yellow.png");
    } break;
    case eRobotColor::rongshifu: {
        robotImg.load("://images/rsf.png");
    } break;
        //      case eRobotColor::car: {
        //      robotImg.load("://images/car.png");
        //      } break;
    }
    QMatrix matrix;
    matrix.rotate(90);
    robotImg = robotImg.transformed(matrix, Qt::SmoothTransformation);
}
void roboItem::setRobotSize(QSize size) { robotImg = robotImg.scaled(size); }
int roboItem::QColorToInt(const QColor &color)
{
    // 将Color 从QColor 转换成 int
    return (int)(((unsigned int)color.blue() << 16) |
                 (unsigned short)(((unsigned short)color.green() << 8) |
                                  color.red()));
}
void roboItem::paintImage(int id, QImage image) { m_image = image; }
void roboItem::paintLaserScan(QPolygonF points)
{
    laserPoints = points;
    update();
}
// palnner规划path绘制
void roboItem::paintPlannerPath(QPolygonF path)
{
    plannerPath = path;
    update();
}
void roboItem::paintMaps(QImage map)
{
    m_imageMap = map;
    update();
}
void roboItem::paintRoboPos(double x, double y, double theta)
{
    //  qDebug()<<"pos:"<<pos.x<<" "<<pos.y<<" "<<pos.theta;
    RoboPostion = QPointF(x, y);
    m_roboYaw = theta;
    update();
}
void roboItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                     QWidget *widget)
{
    drawMap(painter);
    drawRoboPos(painter);
    drawPlannerPath(painter);
    drawLaserScan(painter);
    drawTools(painter);
}
void roboItem::drawTools(QPainter *painter)
{
    if (m_currCursor == set2DPoseCursor) {
        // 绘制箭头
        if (m_startPose.x() != 0 && m_startPose.y() != 0 &&
            m_endPose.x() != 0 && m_endPose.y() != 0) {
            QPen pen(QColor(255, 85, 255, 255), 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
            QBrush brush(QColor(255, 85, 255, 255), Qt::SolidPattern);

            painter->setPen(pen);
            painter->setBrush(brush);
            // 计算线弧度
            double theta = atan((m_endPose.y() - m_startPose.y()) /
                                (m_endPose.x() - m_startPose.x()));
            // 绘制直线
            QPointF startPoint, endPoint;
            startPoint = m_startPose;
            endPoint = m_endPose;
            QLineF line(startPoint, endPoint);
            painter->drawLine(line);
            float angle = atan2(endPoint.y() - startPoint.y(), endPoint.x() - startPoint.x()) + 3.1415926; //
            // 绘制三角形
            QPolygonF points;
            points.push_back(endPoint);
            QPointF point1, point2;
            point1.setX(endPoint.x() + 10 * cos(angle - 0.5)); // 求得箭头点1坐标
            point1.setY(endPoint.y() + 10 * sin(angle - 0.5));
            point2.setX(endPoint.x() + 10 * cos(angle + 0.5)); // 求得箭头点2坐标
            point2.setY(endPoint.y() + 10 * sin(angle + 0.5));
            points.push_back(point1);
            points.push_back(point2);
            painter->drawPolygon(points);
        }
    } else if (m_currCursor == set2DGoalCursor) {
        // 绘制箭头
        if (m_startPose.x() != 0 && m_startPose.y() != 0 &&
            m_endPose.x() != 0 && m_endPose.y() != 0) {
            QPen pen(QColor(50, 205, 50, 255), 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
            QBrush brush(QColor(50, 205, 50, 255), Qt::SolidPattern);

            painter->setPen(pen);
            painter->setBrush(brush);
            // 计算线弧度
            double theta = atan((m_endPose.y() - m_startPose.y()) /
                                (m_endPose.x() - m_startPose.x()));
            // 绘制直线
            QPointF startPoint, endPoint;
            startPoint = m_startPose;
            endPoint = m_endPose;
            QLineF line(startPoint, endPoint);
            painter->drawLine(line);
            float angle = atan2(endPoint.y() - startPoint.y(), endPoint.x() - startPoint.x()) + 3.1415926; //
            // 绘制三角形
            QPolygonF points;
            points.push_back(endPoint);
            QPointF point1, point2;
            point1.setX(endPoint.x() + 10 * cos(angle - 0.5)); // 求得箭头点1坐标
            point1.setY(endPoint.y() + 10 * sin(angle - 0.5));
            point2.setX(endPoint.x() + 10 * cos(angle + 0.5)); // 求得箭头点2坐标
            point2.setY(endPoint.y() + 10 * sin(angle + 0.5));
            points.push_back(point1);
            points.push_back(point2);
            painter->drawPolygon(points);
        }
    }
}
void roboItem::drawMap(QPainter *painter)
{
    painter->drawImage(0, 0, m_imageMap);
}
void roboItem::drawRoboPos(QPainter *painter)
{
    painter->setPen(QPen(QColor(255, 0, 0, 255), 1, Qt::SolidLine, Qt::RoundCap,
                         Qt::RoundJoin));
    painter->save();
    painter->translate(RoboPostion.x(), RoboPostion.y());
    painter->rotate(180 / 3.1415926 * (-m_roboYaw));
    painter->drawPoint(QPoint(0, 0));
    painter->drawPixmap(QPoint(-robotImg.width() / 2, -robotImg.height() / 2),
                        robotImg);
    painter->restore();
}
void roboItem::drawLaserScan(QPainter *painter)
{
    // 绘制laser
    painter->setPen(QPen(QColor(255, 0, 0, 255), 1));
    painter->drawPoints(laserPoints);
}
void roboItem::drawPlannerPath(QPainter *painter)
{
    // 绘制planner Path
    painter->setPen(QPen(QColor(85, 170, 255, 255), 1));
    painter->drawPoints(plannerPath);
}
void roboItem::setMax()
{
    m_scaleValue *= 1.1; // 每次放大10%
    setScale(m_scaleValue);
}
void roboItem::setMin()
{
    m_scaleValue *= 0.9; // 每次缩小10%
    setScale(m_scaleValue);
}
void roboItem::setDefault()
{
    this->setScale(defaultScale);
    this->moveBy(0, 0);
    m_scaleValue = defaultScale;
}
QRectF roboItem::boundingRect() const
{
    // 设置当前item绘制区域 (x,y,width,height)
    return QRectF(0, 0, m_imageMap.width(), m_imageMap.height());
}

void roboItem::move(double x, double y) { this->moveBy(x, y); }
// mouse event
void roboItem::wheelEvent(QGraphicsSceneWheelEvent *event)
{
    this->setCursor(Qt::CrossCursor);
    if ((event->delta() > 0) && (m_scaleValue >= 50)) // 最大放大到原始图像的50倍
    {
        return;
    } else if ((event->delta() < 0) &&
               (m_scaleValue <=
                m_scaleDafault)) // 图像缩小到自适应大小之后就不继续缩小
    {
        // ResetItemPos();//重置图片大小和位置，使之自适应控件窗口大小
    } else {
        qreal qrealOriginScale = m_scaleValue;
        if (event->delta() > 0) // 鼠标滚轮向前滚动
        {
            m_scaleValue *= 1.1; // 每次放大10%
        } else {
            m_scaleValue *= 0.9; // 每次缩小10%
        }
        setScale(m_scaleValue);
        // 为解决在超过+-90°后缩放异常，将鼠标所在点的坐标从视图坐标系转换为场景坐标系，试过了，并没卵用
        //QPointF mousePos = mapToScene(event->pos());//要使用的话要修改if里面（不包含条件）的event为mousePos
        if (event->delta() > 0) {
            moveBy(-event->pos().x() * qrealOriginScale * 0.1,
                   -event->pos().y() * qrealOriginScale * 0.1); // 使图片缩放的效果看起来像是以鼠标所在点为中心进行缩放的
        } else {
            moveBy(event->pos().x() * qrealOriginScale * 0.1,
                   event->pos().y() * qrealOriginScale * 0.1); // 使图片缩放的效果看起来像是以鼠标所在点为中心进行缩放的
        }
    }
}

void roboItem::slot_set2DPos()
{
    this->setCursor(*set2DPoseCursor); // 设置自定义的鼠标样式
    m_currCursor = set2DPoseCursor;
}
void roboItem::slot_set2DGoal()
{
    this->setCursor(*set2DGoalCursor); // 设置自定义的鼠标样式
    m_currCursor = set2DGoalCursor;
}
void roboItem::slot_setMoveCamera()
{
    this->setCursor(*m_moveCursor); // 设置自定义的鼠标样式
    m_currCursor = m_moveCursor;
}
void roboItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        if (m_currCursor != m_moveCursor) {
            m_pressedPoint = event->pos();
        }
        m_startPose = event->pos(); // 鼠标左击时，获取当前鼠标在图片中的坐标，
        m_isMousePress = true;      // 标记鼠标左键被按下
        m_isRightMousePress = false;
    } else if (event->button() == Qt::RightButton) {
        // ResetItemPos();//右击鼠标重置大小
        m_lastPos = event->pos();
        m_isRightMousePress = true;
    }
    update();
}

void roboItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
        // 左键按下
        m_pressingPoint = mapToScene(event->pos());
        // 设置鼠标样式为移动
        if (m_currCursor == nullptr) {
            this->setCursor(*m_moveCursor);
            m_currCursor = m_moveCursor;
        }
        // 移动图层
        if (m_isMousePress && m_currCursor == m_moveCursor) {
            // 计算鼠标移动距离
            QVector2D point = QVector2D(mapToScene(event->pos()) - mapToScene(m_startPose)) ;//* m_scaleValue;
            // 平移图层
            moveBy(point.x(), point.y());
        }
        // 保存当前鼠标位置
        m_endPose = event->pos();
    }

    if (event->buttons() & Qt::RightButton) {
        // 右键按下
        // 记录右键是否按下的标志
        m_isRightMousePress = true;
        // 获取当前变换矩阵
        QTransform transform = this->transform();
        // 获取中心点坐标
        QPointF center = boundingRect().center();
        // 旋转操作
        QPointF delta = mapToScene(event->pos()) - mapToScene(m_lastPos);
        transform.translate(center.x(), center.y());
        transform.rotate(delta.x() / 10 / m_scaleValue);
        transform.translate(-center.x(), -center.y());
        setTransform(transform);
        // 更新上一次鼠标位置
        m_lastPos = event->pos();
    }
    // 更新视图
    update();
}

void roboItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
    emit cursorPos(event->pos());
}

void roboItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    if (m_isRightMousePress == false) // 不触发松开右键事件
    {
        m_isMousePress = false; // 标记鼠标左键已经抬起
        // 如果是选择点位模式 重置
        if (m_currCursor == set2DPoseCursor) {
            emit signalPub2DPose(m_startPose, m_endPose);
            m_currCursor = m_moveCursor;
            this->setCursor(*m_currCursor);
        } else if (m_currCursor == set2DGoalCursor) {
            emit signalPub2DGoal(m_startPose, m_endPose);
            m_currCursor = m_moveCursor;
            this->setCursor(*m_currCursor);
        }
        m_startPose = QPointF();
        m_endPose = QPointF();
        m_pressedPoint = QPointF();
        m_pressingPoint = QPointF();
    }
}

} // namespace Ui
