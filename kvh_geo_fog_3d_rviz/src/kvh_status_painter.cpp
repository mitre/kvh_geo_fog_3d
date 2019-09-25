#include "kvh_status_painter.hpp"

namespace kvh
{
    StatusPainter::StatusPainter(QWidget* parent)
    : QWidget(parent)
    {
    }
    
    void StatusPainter::paintEvent(QPaintEvent* event)
    {
        QPainter painter(this);
        QColor enabled;

        if (isEnabled())
        {
            enabled = Qt::green;
        }
        else
        {
            enabled = Qt::red;
        }
        

        painter.setBrush(enabled);

        int w = width();
        int h = height();

        painter.drawEllipse(0, h/2-3, 10, 10);
    }
}