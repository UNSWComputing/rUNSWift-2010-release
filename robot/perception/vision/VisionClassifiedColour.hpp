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

#include "perception/vision/Vision.hpp"
#include "perception/vision/VisionDefs.hpp"

inline Colour Vision::getColour(uint16_t row, uint16_t col) {
   uint8_t const* pixelpair = currentFrame +
      ((row * IMAGE_COLS + ((col >> 1) << 1)) << 1);
   return classify(pixelpair[(col & 1) << 1], pixelpair[1], pixelpair[3]);
}

inline Colour Vision::classify(uint8_t y, uint8_t u, uint8_t v) {
   return (Colour) (nnmc.get()[
         ((y >> 1) << MAXU_POW << MAXV_POW) +
         ((u >> 1) << MAXV_POW) +
          (v >> 1)] & ~MAYBE_BIT);  // discard maybe bit
}

inline const Colour Vision::classify(const uint8_t *const pixel,
                                     const uint8_t *const nnmc) {
   if ((size_t)pixel & 0x2)
      return classify_UYV(pixel, nnmc);
   else
      return classifyYU_V(pixel, nnmc);
}

// TODO(jayen): write big endian version
inline const Colour Vision::classifyYU_V(const uint8_t *const pixel,
                                         const uint8_t *const nnmc) {
   const uint32_t V_UY = *(const uint32_t* const)pixel;
   const uint32_t YUV =
         ((V_UY & 0x000000FE) << (MAXY_POW + MAXU_POW + MAXV_POW - 8)) |
         ((V_UY & 0x0000FE00) >> (16 - MAXU_POW - MAXV_POW)) |
         ((V_UY)              >> (32 - MAXV_POW));
   return (Colour) (nnmc[YUV] & ~MAYBE_BIT);  // discard maybe bit
}

// TODO(jayen): write big endian version
inline const Colour Vision::classify_UYV(const uint8_t *const pixel,
                                         const uint8_t *const nnmc) {
   const uint32_t VYU_ = *(const uint32_t* const)pixel;
   const uint32_t YUV =
         ((VYU_ & 0x00FE0000) >> (24 - MAXY_POW - MAXU_POW - MAXV_POW)) |
         ((VYU_ & 0x0000FE00) >> (16 - MAXU_POW - MAXV_POW)) |
         ((VYU_)              >> (32 - MAXV_POW));
   return (Colour) (nnmc[YUV] & ~MAYBE_BIT);  // discard maybe bit
}

// TODO(jayen): write big endian version
inline const Colour Vision::classify_UYV(const uint8_t *const pixel,
                                         const Colour *const nnmc) {
   const uint32_t VYU_ = *(const uint32_t* const)pixel;
   return nnmc[VYU_ >> 8];
}

inline PixelValues Vision::getPixelValues(uint16_t row, uint16_t col) {
   uint8_t const* pixelPair = currentFrame +
      ((row * IMAGE_COLS + ((col >> 1) << 1)) << 1);
   PixelValues values;
   values.y = pixelPair[(col & 1) << 1];
   values.u = pixelPair[1];
   values.v = pixelPair[3];
   return values;
}
