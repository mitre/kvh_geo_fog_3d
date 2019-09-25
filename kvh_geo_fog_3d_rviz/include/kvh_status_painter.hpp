#pragma once

#include <QWidget>
#include <QPainter>

namespace kvh
{
class StatusPainter : public QWidget
{
Q_OBJECT;
        
public:
    StatusPainter(QWidget* parent = 0);
    virtual void paintEvent(QPaintEvent* event);

};

}