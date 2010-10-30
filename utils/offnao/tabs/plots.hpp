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

#pragma once

#include <qwt-qt4/qwt_plot.h>
#include <vector>
#include "utils/SensorValues.hpp"

// PLOT_SIZE ~= 30s at 30fps or 9s at 100fps
#define PLOT_SIZE 301

class DataPlot : public QwtPlot {
   Q_OBJECT

   public:
      DataPlot(QWidget* = NULL);
      // put one frame in front of 
      virtual void push(SensorValues sensors, bool left=false) = 0;
      void push(std::vector<SensorValues> sensors, bool left=false);

   protected:
      void alignScales();
      double t[PLOT_SIZE];
};

class AccPlot : public DataPlot {
   Q_OBJECT

   public:
      AccPlot(QWidget* = NULL);
      virtual void push(SensorValues sensors, bool left=false);
   private:
      double data_x[PLOT_SIZE];
      double data_y[PLOT_SIZE];
      double data_z[PLOT_SIZE];
};
class FsrPlot : public DataPlot {
   Q_OBJECT

   public:
      FsrPlot(QWidget* = NULL);
      virtual void push(SensorValues sensors, bool left=false);
   private:
      double data_l[PLOT_SIZE];
      double data_r[PLOT_SIZE];
      double data_t[PLOT_SIZE];
};
class TiltPlot : public DataPlot {
   Q_OBJECT

   public:
      TiltPlot(QWidget* = NULL);
      virtual void push(SensorValues sensors, bool left=false);
   private:
      double data_x[PLOT_SIZE];
      double data_y[PLOT_SIZE];
};
class SonarPlot : public DataPlot {
   Q_OBJECT

   public:
      SonarPlot(QWidget* = NULL);
      virtual void push(SensorValues sensors, bool left=false);
   private:
      double data[10][PLOT_SIZE];
};
class ChargePlot : public DataPlot {
   Q_OBJECT

   public:
      ChargePlot(QWidget* = NULL);
      virtual void push(SensorValues sensors, bool left=false);
   private:
      double data[PLOT_SIZE];
};
class CurrentPlot : public DataPlot {
   Q_OBJECT

   public:
      CurrentPlot(QWidget* = NULL);
      virtual void push(SensorValues sensors, bool left=false);
   private:
      double data[PLOT_SIZE];
};
class CoronalZMPPlot : public DataPlot {
   Q_OBJECT

   public:
      CoronalZMPPlot(QWidget* = NULL);
      virtual void push(SensorValues sensors, bool left=false);
   private:
      double data_l[PLOT_SIZE];
      double data_r[PLOT_SIZE];
      double data_t[PLOT_SIZE];
};
class SagittalZMPPlot : public DataPlot {
   Q_OBJECT

   public:
      SagittalZMPPlot(QWidget* = NULL);
      virtual void push(SensorValues sensors, bool left=false);
   private:
      double data[PLOT_SIZE];
};
