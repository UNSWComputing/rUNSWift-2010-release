/*
Copyright 2010 The University of New South Wales (UNSW).

This file is part of the 2010 team rUNSWift RoboCup entry. You may
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version as
modified below. As the original licensors, we add the following
conditions to that license:

In paragraph 2.b), the phrase "distribute or publish" should be
interpreted to include entry into a competition, and hence the source
of any derived work entered into a competition must be made available
to all parties involved in that competition under the terms of this
license.

In addition, if the authors of a derived work publish any conference
proceedings, journal articles or other academic papers describing that
derived work, then appropriate academic citations to the original work
must be included in that publication.

This rUNSWift source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with this source code; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include "tabs/plots.hpp"
#include <qwt-qt4/qwt_painter.h>
#include <qwt-qt4/qwt_plot_canvas.h>
#include <qwt-qt4/qwt_plot_marker.h>
#include <qwt-qt4/qwt_plot_curve.h>
#include <qwt-qt4/qwt_scale_widget.h>
#include <qwt-qt4/qwt_legend.h>
#include <qwt-qt4/qwt_scale_draw.h>
#include "utils/angles.hpp"

void DataPlot::alignScales() {
   // The code below shows how to align the scales to
   // the canvas frame, but is also a good example demonstrating
   // why the spreaded API needs polishing.

   canvas()->setFrameStyle(QFrame::Box | QFrame::Plain);
   canvas()->setLineWidth(1);

   for (int i = 0; i < QwtPlot::axisCnt; i++) {
      QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);
      if (scaleWidget)
         scaleWidget->setMargin(0);

      QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);
      if (scaleDraw)
         scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
   }
}

void DataPlot::push(std::vector<SensorValues> sensors, bool left) {
   if (left)
      for (std::vector<SensorValues>::reverse_iterator it = sensors.rbegin();
           it != sensors.rend(); it++)
         push(*it, left);
   else
      for (std::vector<SensorValues>::iterator it = sensors.begin();
           it != sensors.end(); it++)
         push(*it, left);
}

DataPlot::DataPlot(QWidget *parent):
   QwtPlot(parent) {
   // Disable polygon clipping
   QwtPainter::setDeviceClipping(false);

   // We don't need the cache here
   canvas()->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
   canvas()->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);

#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
   /*
      Qt::WA_PaintOnScreen is only supported for X11, but leads
      to substantial bugs with Qt 4.2.x/Windows
   */
   canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif

   alignScales();

   for (int i = 0; i < PLOT_SIZE; ++i) {
      t[i] = i;
   }
}

AccPlot::AccPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data_x[i] = data_y[i] = data_z[i] = 0.0;
   setTitle("Accelerometers");

   QwtPlotCurve *curveX = new QwtPlotCurve("AccX");
   curveX->attach(this);
   curveX->setRawData(t, data_x, PLOT_SIZE);
   curveX->setPen(QPen(Qt::red));

   QwtPlotCurve *curveY = new QwtPlotCurve("AccY");
   curveY->attach(this);
   curveY->setRawData(t, data_y, PLOT_SIZE);
   curveY->setPen(QPen(Qt::green));

   QwtPlotCurve *curveZ = new QwtPlotCurve("AccZ");
   curveZ->attach(this);
   curveZ->setRawData(t, data_z, PLOT_SIZE);
   curveZ->setPen(QPen(Qt::blue));

   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -64, 64);
}

void AccPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE; i > 0; --i) {
         data_x[i] = data_x[i-1];
         data_y[i] = data_y[i-1];
         data_z[i] = data_z[i-1];
      }
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i) {
         data_x[i] = data_x[i+1];
         data_y[i] = data_y[i+1];
         data_z[i] = data_z[i+1];
      }
   data_x[left ? 0 : PLOT_SIZE - 1] = sensors.sensors[Sensors::InertialSensor_AccX];
   data_y[left ? 0 : PLOT_SIZE - 1] = sensors.sensors[Sensors::InertialSensor_AccY];
   data_z[left ? 0 : PLOT_SIZE - 1] = sensors.sensors[Sensors::InertialSensor_AccZ];
   replot();
}

FsrPlot::FsrPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data_l[i] = data_r[i] = data_t[i] = 0.0;
   setTitle("FSRs");

   QwtPlotCurve *curveL = new QwtPlotCurve("Left");
   curveL->attach(this);
   curveL->setRawData(t, data_l, PLOT_SIZE);
   curveL->setPen(QPen(Qt::red));

   QwtPlotCurve *curveR = new QwtPlotCurve("Right");
   curveR->attach(this);
   curveR->setRawData(t, data_r, PLOT_SIZE);
   curveR->setPen(QPen(Qt::blue));

   QwtPlotCurve *curveT = new QwtPlotCurve("Total");
   curveT->attach(this);
   curveT->setRawData(t, data_t, PLOT_SIZE);

   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, 0, 4);
}

void FsrPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE; i > 0; --i) {
         data_l[i] = data_l[i-1];
         data_r[i] = data_r[i-1];
         data_t[i] = data_t[i-1];
      }
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i) {
         data_l[i] = data_l[i+1];
         data_r[i] = data_r[i+1];
         data_t[i] = data_t[i+1];
      }
   data_l[left ? 0 : PLOT_SIZE - 1] =
      sensors.sensors[Sensors::LFoot_FSR_FrontLeft] +
      sensors.sensors[Sensors::LFoot_FSR_FrontRight] +
      sensors.sensors[Sensors::LFoot_FSR_RearLeft] +
      sensors.sensors[Sensors::LFoot_FSR_RearRight];
   data_r[left ? 0 : PLOT_SIZE - 1] =
      sensors.sensors[Sensors::RFoot_FSR_FrontLeft] +
      sensors.sensors[Sensors::RFoot_FSR_FrontRight] +
      sensors.sensors[Sensors::RFoot_FSR_RearLeft] +
      sensors.sensors[Sensors::RFoot_FSR_RearRight];
   data_t[left ? 0 : PLOT_SIZE - 1] =
      data_l[left ? 0 : PLOT_SIZE - 1] + data_r[left ? 0 : PLOT_SIZE - 1];
   replot();
}

TiltPlot::TiltPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data_x[i] = data_y[i] = 0.0;
   setTitle("Torso Tilts");

   QwtPlotCurve *curveX = new QwtPlotCurve("AngleX");
   curveX->attach(this);
   curveX->setRawData(t, data_x, PLOT_SIZE);
   curveX->setPen(QPen(Qt::red));

   QwtPlotCurve *curveY = new QwtPlotCurve("AngleY");
   curveY->attach(this);
   curveY->setRawData(t, data_y, PLOT_SIZE);
   curveY->setPen(QPen(Qt::blue));

   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -90, 90);
}

void TiltPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE; i > 0; --i) {
         data_x[i] = data_x[i-1];
         data_y[i] = data_y[i-1];
      }
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i) {
         data_x[i] = data_x[i+1];
         data_y[i] = data_y[i+1];
      }
   data_x[left ? 0 : PLOT_SIZE - 1] =
      RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleX]);
   data_y[left ? 0 : PLOT_SIZE - 1] =
      RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleY]);
   replot();
}

SonarPlot::SonarPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      for (int j = 0; j < 10; ++j)
         data[j][i] = 0.0;
   setTitle("Sonar");

   QwtPlotCurve *curve = new QwtPlotCurve("Sonar");
   curve->attach(this);
   curve->setRawData(t, data[0], PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, 0, 2.55);
}

void SonarPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE; i > 0; --i)
         for (int j = 0; j < 10; ++j)
            data[j][i] = data[j][i-1];
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i)
         for (int j = 0; j < 10; ++j)
            data[j][i] = data[j][i+1];
   for (int j = 0; j < 10; ++j)
      data[j][left ? 0 : PLOT_SIZE - 1] = sensors.sonar[j];
   replot();
}

ChargePlot::ChargePlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data[i] = 0.0;
   setTitle("Battery Charge");

   QwtPlotCurve *curve = new QwtPlotCurve("Charge");
   curve->attach(this);
   curve->setRawData(t, data, PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, 0, 1);
}

void ChargePlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE; i > 0; --i)
         data[i] = data[i-1];
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i)
         data[i] = data[i+1];
   data[left ? 0 : PLOT_SIZE - 1] = sensors.sensors[Sensors::Battery_Charge];
   replot();
}

CurrentPlot::CurrentPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data[i] = 0.0;
   setTitle("Battery Current");

   QwtPlotCurve *curve = new QwtPlotCurve("Current");
   curve->attach(this);
   curve->setRawData(t, data, PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -2, 2);
}

void CurrentPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE; i > 0; --i)
         data[i] = data[i-1];
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i)
         data[i] = data[i+1];
   data[left ? 0 : PLOT_SIZE - 1] = sensors.sensors[Sensors::Battery_Current];
   replot();
}

CoronalZMPPlot::CoronalZMPPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i) {
      data_l[i] = 0.0;
      data_r[i] = 0.0;
      data_t[i] = 0.0;
   }
   setTitle("Coronal Plane");

   QwtPlotCurve *curveL = new QwtPlotCurve("Left");
   curveL->attach(this);
   curveL->setRawData(t, data_l, PLOT_SIZE);
   curveL->setPen(QPen(Qt::red));

   QwtPlotCurve *curveR = new QwtPlotCurve("Right");
   curveR->attach(this);
   curveR->setRawData(t, data_r, PLOT_SIZE);
   curveR->setPen(QPen(Qt::blue));

   QwtPlotCurve *curveT = new QwtPlotCurve("Total");
   curveT->attach(this);
   curveT->setRawData(t, data_t, PLOT_SIZE);

   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -100, 100);
}

void CoronalZMPPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE; i > 0; --i) {
         data_l[i] = data_l[i-1];
         data_r[i] = data_r[i-1];
         data_t[i] = data_t[i-1];
      }
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i) {
         data_l[i] = data_l[i+1];
         data_r[i] = data_r[i+1];
         data_t[i] = data_t[i+1];
      }
   float total_weight = (sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::LFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_RearRight]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::RFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_RearRight]);
   if (total_weight != 0)
      data_t[left ? 0 : PLOT_SIZE - 1] = 
         (80.0f*sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
        + 30.0f*sensors.sensors[Sensors::LFoot_FSR_FrontRight]
        + 80.0f*sensors.sensors[Sensors::LFoot_FSR_RearLeft]
        + 30.0f*sensors.sensors[Sensors::LFoot_FSR_RearRight]
        - 30.0f*sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
        - 80.0f*sensors.sensors[Sensors::RFoot_FSR_FrontRight]
        - 30.0f*sensors.sensors[Sensors::RFoot_FSR_RearLeft]
        - 80.0f*sensors.sensors[Sensors::RFoot_FSR_RearRight]) / total_weight;
   else
      data_t[left ? 0 : PLOT_SIZE - 1] = 0.0f;
   data_l[left ? 0 : PLOT_SIZE - 1] = 1000 *
      sensors.sensors[Sensors::LFoot_FSR_CenterOfPressure_Y];
   data_r[left ? 0 : PLOT_SIZE - 1] = 1000 *
      sensors.sensors[Sensors::RFoot_FSR_CenterOfPressure_Y];
   replot();
}

SagittalZMPPlot::SagittalZMPPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data[i] = 0.0;
   setTitle("Sagittal Plane");

   QwtPlotCurve *curve = new QwtPlotCurve("SagittalZMP");
   curve->attach(this);
   curve->setRawData(t, data, PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -100, 100);
}

void SagittalZMPPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE; i > 0; --i)
         data[i] = data[i-1];
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i)
         data[i] = data[i+1];
   float total_weight = (sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::LFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_RearRight]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::RFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_RearRight]);
   if (total_weight != 0)
      data[left ? 0 : PLOT_SIZE - 1] = 
         (50.0f*sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
        + 50.0f*sensors.sensors[Sensors::LFoot_FSR_FrontRight]
        - 50.0f*sensors.sensors[Sensors::LFoot_FSR_RearLeft]
        - 50.0f*sensors.sensors[Sensors::LFoot_FSR_RearRight]
        + 50.0f*sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
        + 50.0f*sensors.sensors[Sensors::RFoot_FSR_FrontRight]
        - 50.0f*sensors.sensors[Sensors::RFoot_FSR_RearLeft]
        - 50.0f*sensors.sensors[Sensors::RFoot_FSR_RearRight]) / total_weight;
   else
      data[left ? 0 : PLOT_SIZE - 1] = 0.0f;
   replot();
}
