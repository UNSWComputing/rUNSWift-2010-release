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
#include <stdint.h>
#include <iostream>
#include "utils/log.hpp"

/* Line structure, given in terms of parameters
 * t1, t2 and t3, that satisfy the equation:
 * t1x + t2y + t3 = 0, each of quich are integral.
 **/
struct Line {
   Line(int x1, int y1, int x2, int y2) {
      x1_ = x1;
      x2_ = x2;
      y1_ = y1;
      y2_ = y2;
      t1 = y2 - y1;
      t2 = x1 - x2;
      t3 = y1*(x2 - x1) - x1*(y2 - y1);
   }
   Line() : t1(0), t2(0), t3(0), var(0) {}
   int32_t t1;
   int32_t t2;
   int32_t t3;
   /* For debuggin */
   int x1_;
   int x2_;
   int y1_;
   int y2_;
   /* Variance */
   float var;
};
