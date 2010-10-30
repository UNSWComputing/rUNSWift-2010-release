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
#include "utils/angles.hpp"

#define IMAGE_ROWS 480
#define IMAGE_COLS 640

#define MAXY 128
#define MAXY_POW 7
#define MAXU 128
#define MAXU_POW 7
#define MAXV 128
#define MAXV_POW 7
#define MAYBE_BIT 0x10

#define MAX_POSTS 2
#define MAX_BALLS 2
#define MAX_FIELD_EDGES 2
#define MAX_FIELD_LINE_POINTS 40
#define MAX_NUM_ROBOTS 5

/* Scan every SALIENCY_DENSITYth pixel */
// #define SALIENCY_DENSITY 4
// #define SALIENCY_DENSITY_POW 2
#define SALIENCY_DENSITY 4
#define SALIENCY_DENSITY_POW 2

#define MIN_ANGLE_BETWEEN_EDGE_LINES DEG2RAD(7)

/* Values of the various colours that are present on the field.
 * Anything not categorized as a 'colour' is named unclassified */
typedef enum {
   cBALL = 0,
   cGOAL_BLUE = 1,
   cGOAL_YELLOW = 2,
   cROBOT_BLUE = 1,
   cROBOT_RED = 4,
   cFIELD_GREEN = 5,
   cWHITE = 6,
   cBLACK = 7,
   cBACKGROUND = 8,
   cUNCLASSIFIED = 9,
   cNUM_COLOURS = 10
}


__attribute__((packed)) Colour;

/* Which posts are seen by vision */
typedef enum {
   pNONE = 0,
   pBLUE_LEFT = 1,
   pBLUE_RIGHT = 2,
   pBLUE_BOTH = 3,
   pBLUE_EITHER = 4,
   pYELLOW_LEFT = 5,
   pYELLOW_RIGHT = 6,
   pYELLOW_BOTH = 7,
   pYELLOW_EITHER = 8
}
__attribute__((packed)) WhichPosts;
/* The number that has to be added to one of the blue post options to
   get the equivalent yellow post option. MAKE sure this is updated
   if the WhichPosts enum changes, otherwise the goal detection will
   break */
#define NUMBER_ADD_TO_BLUE 4

/* Components of a pixel */
typedef enum {
   ycomponent,
   ucomponent,
   vcomponent,
   NUM_COMPONENTS
}
__attribute__((packed)) Component;

typedef struct pixelValues PixelValues;
struct pixelValues {
   uint8_t y;
   uint8_t u;
   uint8_t v;
};

typedef struct twoParamLine TwoParamLine;
struct twoParamLine {
   float m;
   float b;
};

/* Name of the various colours that are present on the field.
 * Anything not categorized as a 'colour' is named unclassified */

extern const char* ColourNames[];
extern const char* CPLANE_COLOURS[];

/**
 * The histogram types, in case the SALIENCY_DENSITY is 1 or 2
 */
#if IMAGE_ROWS/SALIENCY_DENSITY < 256
      typedef uint8_t  XHistogram;
#else
      typedef uint16_t XHistogram;
#endif
#if IMAGE_COLS/SALIENCY_DENSITY < 256
      typedef uint8_t  YHistogram;
#else
      typedef uint16_t YHistogram;
#endif
