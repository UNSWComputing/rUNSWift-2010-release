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

#include <utils/matrix_helpers.hpp>

using namespace boost::numeric::ublas;

matrix<float> translateMatrix(float x, float y, float z) {
   matrix<float> r(4, 4);
   r(0, 0) = 1;
   r(0, 1) = 0;
   r(0, 2) = 0;
   r(0, 3) = x;

   r(1, 0) = 0;
   r(1, 1) = 1;
   r(1, 2) = 0;
   r(1, 3) = y;

   r(2, 0) = 0;
   r(2, 1) = 0;
   r(2, 2) = 1;
   r(2, 3) = z;

   r(3, 0) = 0;
   r(3, 1) = 0;
   r(3, 2) = 0;
   r(3, 3) = 1;
   return r;
}


matrix<float> rotateZMatrix(float theta) {
   matrix<float> r(4, 4);
   r(0, 0) = cos(theta);
   r(0, 1) = -sin(theta);
   r(0, 2) = 0;
   r(0, 3) = 0;

   r(1, 0) = sin(theta);
   r(1, 1) = cos(theta);
   r(1, 2) = 0;
   r(1, 3) = 0;

   r(2, 0) = 0;
   r(2, 1) = 0;
   r(2, 2) = 1;
   r(2, 3) = 0;

   r(3, 0) = 0;
   r(3, 1) = 0;
   r(3, 2) = 0;
   r(3, 3) = 1;
   return r;
}
matrix<float> createDHMatrix(float a, float alpha,
      float d, float theta) {
   matrix<float> m(4, 4);
   m(0, 0) = cos(theta);
   m(0, 1) = -sin(theta);
   m(0, 2) = 0;
   m(0, 3) = a;

   m(1, 0) = sin(theta) * cos(alpha);
   m(1, 1) = cos(theta) * cos(alpha);
   m(1, 2) = -sin(alpha);
   m(1, 3) = -sin(alpha) * d;

   m(2, 0) = sin(theta) * sin(alpha);
   m(2, 1) = cos(theta) * sin(alpha);
   m(2, 2) = cos(alpha);
   m(2, 3) = cos(alpha) * d;

   m(3, 0) = 0;
   m(3, 1) = 0;
   m(3, 2) = 0;
   m(3, 3) = 1;
   return m;
}



bool invertMatrix(const matrix<float>& input,
                  matrix<float>& inverse) {
   typedef permutation_matrix<std::size_t> pmatrix;
   // create a working copy of the input
   matrix<float> A(input);

   // create a permutation matrix for the LU-factorization
   pmatrix pm(A.size1());

   // perform LU-factorization
   int res = lu_factorize(A, pm);
   if (res != 0)
      return false;

   // create identity matrix of "inverse"
   inverse.assign(identity_matrix<float> (A.size1()));

   // backsubstitute to get the inverse
   lu_substitute(A, pm, inverse);

   return true;
}

matrix<float> projectionMatrix(float ex, float ey, float ez) {
   matrix<float> projection(4, 4);
   projection(0, 0) = 1;
   projection(0, 1) = 0;
   projection(0, 2) = 0;
   projection(0, 3) = -ex;

   projection(1, 0) = 0;
   projection(1, 1) = 1;
   projection(1, 2) = 0;
   projection(1, 3) = -ey;

   projection(2, 0) = 0;
   projection(2, 1) = 0;
   projection(2, 2) = 1;
   projection(2, 3) = 0;

   projection(3, 0) = 0;
   projection(3, 1) = 0;
   projection(3, 2) = 1.0/ez;
   projection(3, 3) = 0;
   return projection;
}

