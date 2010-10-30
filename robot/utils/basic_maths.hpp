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

#include <cmath>

#ifndef MAX
template <class T>
inline static T MAX(const T &x, const T &y) {
      return (x > y ? x : y);
}
inline static float MAX(const float x, const int y) {
      return (x > y ? x : y);
}
inline static float MAX(const int x, const float y) {
      return (x > y ? x : y);
}
#endif

#ifndef MIN
template <class T>
inline static T MIN(const T &x, const T &y) {
      return (x > y ? y : x);
}
inline static float MIN(const float x, const int y) {
      return (x > y ? y : x);
}
inline static float MIN(const int x, const float y) {
      return (x > y ? y : x);
}
#endif

template <class T>
inline static T ABS(const T &x) {
      return (x > 0 ? x : -x);
}

template <class T>
inline static T SQUARE(const T &x) {
      return x * x;
}

template <class T>
inline static T DISTANCE_SQR(const T &xA,
      const T &yA,
      const T &xB,
      const T &yB) {
   return SQUARE(xA - xB) + SQUARE(yA - yB);
}

// This is slow. The sqrt probably sucks.
inline static float DISTANCE(const float xA,
      const float yA,
      const float xB,
      const float yB) {
   return sqrt(DISTANCE_SQR(xA, yA, xB, yB));
}

template <class T>
inline static int SIGN(const T &x) {
   return (x < 0 ? -1 : (x > 0) ? 1 : 0);
}
