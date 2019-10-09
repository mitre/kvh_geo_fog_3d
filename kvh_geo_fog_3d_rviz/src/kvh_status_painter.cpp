/*********************************************************************
 * Software License Agreement (Apache 2.0)
 * 
 *  Copyright (c) 2019, The MITRE Corporation.
 *  All rights reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************/

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
