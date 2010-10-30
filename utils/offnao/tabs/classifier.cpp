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

#include <math.h>
#include <string.h>

#include "classifier.hpp"

using namespace std;

Classifier::Classifier(void) {
}
Classifier::~Classifier(void) {
}

bool Classifier::loadClassificationFile(std::string filename) {
   FILE* classificationFile = fopen(filename.c_str(), "r");

   for (int y = 0; y < YMAX; y++) {
      for (int u = 0; u < UMAX; u++) {
         for (int v = 0; v < VMAX; v++) {
            if (fread(&(weight[y][u][v].w[0]), sizeof(float), CMAX,
                classificationFile) != (int) CMAX) {
               return false;
            }
         }
      }
   }

   fclose(classificationFile);

   // TODO(Dave): Check file size
   /*if (size != YMAX * UMAX * VMAX * CMAX * sizeof(float)) {
     cerr << "selectClassificationFile(string): 
     Classification file is the wrong size." << endl;
     closeFile();
     return false;
     }*/


   // Work out maximums for each YUV
   makeNnmc();


   // Clear the undo buffer and file cache
   undoBuffer.clear();
   fileOpened = true;
   return true;
}

void Classifier::saveClassificationFile(string filename) {
   FILE* classificationFile = fopen(filename.c_str(), "w");

   for (int y = 0; y < YMAX; y++) {
      for (int u = 0; u < UMAX; u++) {
         for (int v = 0; v < VMAX; v++) {
            fwrite(&(weight[y][u][v].w[0]), sizeof(float), CMAX,
                  classificationFile);
         }
      }
   }

   fclose(classificationFile);
}

void Classifier::newClassificationFile(void) {
   for (int y = 0; y < YMAX; y++) {
      for (int u = 0; u < UMAX; u++) {
         for (int v = 0; v < VMAX; v++) {
            for (int c = 0; c < CMAX; c++) {
               weight[y][u][v].w[c] = 0.0;
            }
         }
      }
   }


   undoBuffer.clear();
   makeNnmc();
   fileOpened = true;
}

bool Classifier::classificationFileOpened(void) {
   return fileOpened;
}

void Classifier::beginAction(void) {
   if (not currentAction.gaussians.empty()) {
      cerr << "beginAction(): Began action without ending previous one" << endl;
      return;
   }
}
void Classifier::endAction(void) {
   undoBuffer.push_back(currentAction);

   applyAction(currentAction, false);

   currentAction.clear();
}

// This will actually do an action
void Classifier::applyAction(Action &a, bool undo) {
   for (unsigned int i = 0; i < a.gaussians.size(); i++) {
      applyGaussian(a.gaussians[i], undo);
   }
}

void Classifier::getUndoStatus(int& levels, Colour& lastColour) {
   levels = undoBuffer.size();
   if (not undoBuffer.empty()) {
      if (undoBuffer.back().gaussians.empty()) {
         lastColour = cUNCLASSIFIED;
      } else {
         lastColour = undoBuffer.back().gaussians[0].classified;
      }
   }
}

void Classifier::applyGaussian(YuvGaussian &g, bool undo) {
   // This adds a gaussian to the colour space for the given
   // classified colour.
   for (int dy = -g.yRadius; dy <= g.yRadius; dy++) {
      for (int du = -g.uRadius; du <= g.uRadius; du++) {
         for (int dv = -g.vRadius; dv <= g.vRadius; dv++) {
            YuvTriple yuv = g.yuv;
            yuv.y += dy;
            yuv.u += du;
            yuv.v += dv;
            addWeight(yuv, g.weight * rawKernelVal(dy, du, dv, g.yRadius,
                     g.uRadius, g.vRadius, g.hacks), g.classified, undo);
         }
      }
   }
}

// This will look at all the colours adjacent to (y, u, v) (in colourspace)
// and check if they are all equal to c - if one of them isn't, then we return
// false.
bool Classifier::isMostlyClassified(int y, int u, int v, Colour c) {
   // Magic number 2 is because we drop least significant bit
   return (getClassifiedColour(y, u, v) == c and
         getClassifiedColour(y - 2, u, v) == c and
         getClassifiedColour(y + 2, u, v) == c and
         getClassifiedColour(y, u - 2, v) == c and
         getClassifiedColour(y, u + 2, v) == c and
         getClassifiedColour(y, u, v - 2) == c and
         getClassifiedColour(y, u, v + 2) == c);
}

float Classifier::getColourMargin(int y, int u, int v) {
   // This returns the difference in weight between the top colour
   // and the second top
   float w[CMAX];

   colourInfo(y, u, v, w);

   Colour c = getClassifiedColour(y, u, v);

   float secondMaxWeight = w[0];
   if (c == 0) {
      secondMaxWeight = w[1];
   }
   for (int i = 0; i < CMAX; i++) {
      if (w[i] > secondMaxWeight) {
         secondMaxWeight = w[i];
      }
   }

   return w[c] - secondMaxWeight;
}

void Classifier::colourInfo(int y, int u, int v, float weights[CMAX]) {
   // This is a public wrapper around getWeights
   YuvTriple yuv;
   yuv.y = y/2;
   yuv.u = u/2;
   yuv.v = v/2;

   Weights w;

   getWeights(yuv, w);

   for (int i = 0; i < CMAX; i++) {
      weights[i] = w.w[i];
   }
}

void Classifier::addWeight(YuvTriple yuv, float amount, Colour c, bool undo) {
   Weights weights;
   float oldWeight;

   if (yuv.y < YMAX and yuv.u < UMAX and yuv.v < VMAX) {
      // Read the old values
      getWeights(yuv, weights);

      // Find new value
      oldWeight = weights.w[c];
      if (undo) {
         weights.w[c] -= amount;
      } else {
         weights.w[c] += amount;
      }

      // Write the result
      setWeights(yuv, weights);

      // Keep our nnmc correct
      nnmc[yuv.y][yuv.u][yuv.v] = maxWeightIndex(weights);
      if (useLiveNnmc) {
         liveNnmc[&(nnmc[yuv.y][yuv.u][yuv.v]) -
         &(nnmc[0][0][0])] = nnmc[yuv.y][yuv.u][yuv.v];
      }
   }
}

void Classifier::setWeights(YuvTriple yuv, Weights &weights) {
   weight[yuv.y][yuv.u][yuv.v] = weights;
}

int64_t Classifier::yuvToFilePos(YuvTriple yuv) {
   // This tells us where in the file we
   return  ((yuv.y * UMAX  * VMAX  * CMAX +
            yuv.u * VMAX  * CMAX +
            yuv.v * CMAX) * sizeof(float));
}

float Classifier::rawKernelVal(int dY, int dU, int dV, int yRadius,
                               int uRadius, int vRadius, bool hacks) {
   double uvVar = 10;
   if (not hacks) {
      uvVar = 1;
   }

   float result = 0;

   double dColSqr = ((double)dU * dU) / ((double)(uRadius * uRadius)) +
      ((double)dV * dV) / ((double)(vRadius * vRadius)) +
      ((double)dY * dY) / ((double)(yRadius * yRadius));

   if (hacks) {
      dColSqr = (double)dU*dU       * INV_U_KERNEL_RADIUS_SQUARED +
         (double)dV*dV       * INV_V_KERNEL_RADIUS_SQUARED +
         (double)dY*dY*dY*dY * INV_Y_KERNEL_RADIUS_POW_4;
         // ^4 to make it more sensitive to Y channel
   }

   if (dColSqr > 1.0) {
      return 0.0;
   }
   result += exp(-dColSqr*uvVar);
   if (hacks) {
      result -= exp(-uvVar);
   }

   if (hacks and dY == 0 && dU == 0 && dV == 0) {
      result *= 1.1;
   }


   return result;
}

void Classifier::classify(int y, int u, int v, double weight, Colour c,
                          int yRadius, int uRadius, int vRadius, bool hacks) {
   // We divide by 2 to drop the lsb.
   YuvTriple yuv;

   cout << "Classifying YUV(" << y << ", " << u << ", " << v
        << ") as " << c << ". Radius = (" << yRadius
        << ", " << uRadius << ", " << vRadius << ")" << endl;
   yuv.y = static_cast<unsigned char>(y / 2);
   yuv.u = static_cast<unsigned char>(u / 2);
   yuv.v = static_cast<unsigned char>(v / 2);

   YuvGaussian thisGaussian(yuv, weight, c, yRadius, uRadius, vRadius, hacks);
   currentAction.gaussians.push_back(thisGaussian);
}

bool Classifier::canUndo(void) {
   return not undoBuffer.empty();
}
void Classifier::undo(void) {
   if (not canUndo()) {
      cerr << "No actions to undo" << endl;
      return;
   }
   Action actionToUndo = undoBuffer.back();
   undoBuffer.pop_back();
   applyAction(actionToUndo, true);
}

void Classifier::saveNnmc(string filename) {
   FILE* nnmcFile = fopen(filename.c_str(), "w");

   fwrite(nnmc, YMAX * UMAX * VMAX, sizeof(nnmc[0][0][0]), nnmcFile);

   fclose(nnmcFile);
}

void Classifier::getWeights(YuvTriple yuv, Weights &weights) {
   weights = weight[yuv.y][yuv.u][yuv.v];
   return;
}

void Classifier::makeNnmc(void) {
   Weights weights;

   for (unsigned char y = 0; y < YMAX; y++) {
      for (unsigned char u = 0; u < UMAX; u++) {
         for (unsigned char v = 0; v < VMAX; v++) {
            YuvTriple yuv;
            yuv.y = y;
            yuv.u = u;
            yuv.v = v;
            getWeights(yuv, weights);
            nnmc[y][u][v] = maxWeightIndex(weights);
         }
      }
   }
}

Colour Classifier::maxWeightIndex(Weights &weights) {
   float maxWeight = INITIAL_NONE_VALUE;
   int maxIndex = cUNCLASSIFIED;

   for (int i = 0; i < CMAX; i++) {
      if (weights.w[i] > maxWeight) {
         maxIndex = i;
         maxWeight = weights.w[i];
      }
   }

   return static_cast<Colour>(maxIndex);
}

Colour Classifier::getClassifiedColour(int y, int u, int v) {
   // Returns a classified colour
   // We divide by 2 to drop the lsb.
   if (y < 0 or y/2 >= YMAX or
         u < 0 or u/2 >= UMAX or
         v < 0 or v/2 >= VMAX) {
      return cUNCLASSIFIED;
   }
   return static_cast<Colour>(nnmc[y/2][u/2][v/2]);
}

// The next few things are for doing dodgy hacks, but they are quite convenient.
unsigned char* Classifier::getNnmcPointer(void) {
   return &(nnmc[0][0][0]);
}

void Classifier::setNnmcPointer(unsigned char* nnmc) {
   liveNnmc = nnmc;
}

void Classifier::setUpdateLiveNnmc(bool useLive) {
   useLiveNnmc = useLive;

   if (useLive) {
      if (liveNnmc != NULL) {
         // Copy across our current nnmc to make sure it is up to date.
         memcpy(liveNnmc, getNnmcPointer(), YMAX * UMAX * VMAX);
      } else {
         cerr << "setUpdateLiveNnmc(bool): No live nnmc pointer given" << endl;
         useLive = false;
      }
   } else {
      liveNnmc = NULL;
   }
}

/* Converts a YUV pixel spec to RGB */
QRgb Classifier::yuv2rgb(int y, int u, int v) {
   y -= 16;
   u -= 128;
   v -= 128;
   int r = static_cast<int>((298.082 * y + 0       * u + 408.583 * v) / 256);
   int g = static_cast<int>((298.082 * y - 100.291 * u - 208.120 * v) / 256);
   int b = static_cast<int>((298.082 * y + 516.411 * u + 0       * v) / 256);
   if (r < 0) r = 0;
   if (g < 0) g = 0;
   if (b < 0) b = 0;
   if (r > 255) r = 255;
   if (g > 255) g = 255;
   if (b > 255) b = 255;
   return qRgb(r, g, b);
}
