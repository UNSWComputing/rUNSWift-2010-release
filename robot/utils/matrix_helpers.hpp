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

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>

/* Creates a matrix that Rotate vector about the z axis 
 * Currently needed for our robot relative coordinate system
 */
boost::numeric::ublas::matrix<float> rotateZMatrix(float theta);

/* Creates a translation matrix
 */
boost::numeric::ublas::matrix<float> translateMatrix(float x, float y, float z);

/* Creates a projection matrix
 */
boost::numeric::ublas::matrix<float> projectionMatrix(float ex, float ey,
                                                      float ez);


/* Creates a DH matrix used in Kinematics
 */
boost::numeric::ublas::matrix<float> createDHMatrix(float a, float alpha,
                                              float d, float theta);

/* Function to invert matrix
 */
bool invertMatrix(const boost::numeric::ublas::matrix<float>& input,
      boost::numeric::ublas::matrix<float>& inverse);

/* Creates a 4,1 vector */
inline boost::numeric::ublas::matrix<float>
            vec4(float a, float b, float c, float d) {
   boost::numeric::ublas::matrix<float> m(4, 1);
   m(0, 0) = a;
   m(1, 0) = b;
   m(2, 0) = c;
   m(3, 0) = d;
   return m;
}

