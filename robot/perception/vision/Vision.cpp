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

#include "Vision.hpp"
#include <algorithm>
#include <ctime>
#include "utils/log.hpp"
#include "utils/Timer.hpp"
#include "perception/vision/VisionClassifiedColour.hpp"

using namespace std;
extern bool offNao;

Vision::Vision(bool _dumpframes, int _dumprate,
      string _dumpfile, bool _visionEnabled)
   : isColourCalibrationLoaded(false),
     dumpframes(_dumpframes), dumprate(_dumprate), dumpfile(_dumpfile),
     visionEnabled(_visionEnabled) {
   if (offNao) visionEnabled = false;
   if (visionEnabled) {
      loadColourCalibration();
      if (camera == NULL) {
         camera = boost::shared_ptr<RobotCamera>(new NaoCamera());
      }

      currentFrame = NULL;
      if (dumpframes) {
         camera->startRecording(dumpfile.c_str(), dumprate);
      }
   }

   // srand needs a seed, we could make this constant for repeatability
   seed = time(NULL);
   // seed = 42;
   convRR.setCamera(camera.get());
}

Vision::~Vision() {
   llog(INFO) << "Vision Destroyed" << endl;
   camera->stopRecording();
}

void Vision::getFrame() {
   camera->setCamera(whichCamera);
   currentFrame = camera->get();
}

void Vision::processFrame() {
   /* call each stage of the vision pipeline in order
    * once done there should be sane results in the various
    * fields of this Vision class
   **/
   convRR.isRobotMoving();
   timer.restart();
   if (offNao) {
      fieldEdgeDetection.fieldEdgePoints(saliency, pair<int, int>(0, 0),
            convRR.endScanCoords);
   } else {
      fieldEdgeDetection.fieldEdgePoints(saliency, convRR.pose.getHorizon(),
            convRR.endScanCoords);
   }
   llog(VERBOSE) << "fieldEdgePoints() took " << timer.elapsed_us()
      << " us" << endl;
   timer.restart();
   fieldEdgeDetection.fieldEdgeLines(seed, &convRR);
   llog(VERBOSE) << "fieldEdgeLines() took " << timer.elapsed_us()
      << " us" << endl;
   fieldEdgeDetection.findStartScanCoords(saliency);
   timer.restart();
   regionBuilder.buildRegions(saliency, fieldEdgeDetection.startScanCoords,
         convRR.endScanCoords);
   llog(VERBOSE) << "buildRegions() took " << timer.elapsed_us()
      << " us" << endl;
   timer.restart();
   regionBuilder.analyseRegions(fieldEdgeDetection.startScanCoords);
   llog(VERBOSE) << "analyseRegions() took " << timer.elapsed_us()
      << " us" << endl;
   timer.restart();
   robotDetection.findRobots(regionBuilder.robotRegions,
         regionBuilder.numRobotRegions, fieldEdgeDetection.startScanCoords,
         saliency);
   llog(VERBOSE) << "findRobots() took " << timer.elapsed_us()
      << " us" << endl;
   timer.restart();
   if (offNao) {
      ballDetection.findBalls(regionBuilder.ballRegions,
         regionBuilder.numPossibleBallRegions, &convRR, this, &seed,
         &robotDetection, fieldEdgeDetection.startScanCoords,
         pair<int, int>(0, 0));
   } else {
      ballDetection.findBalls(regionBuilder.ballRegions,
         regionBuilder.numPossibleBallRegions, &convRR, this, &seed,
         &robotDetection, fieldEdgeDetection.startScanCoords,
         convRR.pose.getHorizon());
   }
   llog(VERBOSE) << "findBalls() took " << timer.elapsed_us()
      << " us" << endl;
   timer.restart();
   fieldLineDetection.processFieldLines(regionBuilder.lineRegions,
         regionBuilder.numLineRegions, &convRR);
   llog(VERBOSE) << "processFieldLines() took " << timer.elapsed_us()
      << " us" << endl;
   timer.restart();
   if (offNao) {
      goalDetection.findGoals(xhistogram, yhistogram, &convRR,
         fieldEdgeDetection.startScanCoords, this, convRR.endScanCoords,
         robotDetection.numRobotRegions, robotDetection.robotRegions,
         pair<int, int>(0, 0));
   } else {
      goalDetection.findGoals(xhistogram, yhistogram, &convRR,
         fieldEdgeDetection.startScanCoords, this, convRR.endScanCoords,
         robotDetection.numRobotRegions, robotDetection.robotRegions,
         convRR.pose.getHorizon());
   }
   llog(VERBOSE) << "findGoals() took " << timer.elapsed_us()
      << " ms" << endl;
   timer.restart();
   robotDetection.sanityCheckRobots(goalDetection.numPosts,
         goalDetection.postCoords, &convRR, ballDetection.bestBallRegion,
         ballDetection.radius, ballDetection.ballCentre);
   llog(VERBOSE) << "sanityCheckRobots() took " << timer.elapsed_us()
      << " us" << endl;
}

// j is the number of columns
// i is the number of rows
void Vision::saliencyScan() {
   // Set up some pointers for pointer arithmetic use
   const uint8_t* currentFramePixel = &currentFrame[0];
   Colour* saliencyPixel = &saliency[0][0];
   const int* stop = convRR.endScanCoords;
   const Colour* const saliencyEnd = &saliency[IMAGE_COLS/SALIENCY_DENSITY][0];
   // zero the histograms
   bzero(xhistogram, sizeof(xhistogram));
   bzero(yhistogram, sizeof(yhistogram));
   XHistogram *xHistogram = &xhistogram[0][0];
   // if the saliency density is even, then optimise and use the 16MB table
   #if SALIENCY_DENSITY % 2
   const uint8_t *const nnmc = this->nnmc.get();
   #else
   const Colour *const nnmcVYU = this->nnmcVYU.get();
   #endif
   do {
      // More pointers for pointer arithmetic
      YHistogram *yHistogram = &yhistogram[0][0];
      const int bodyRow = max(*stop, 0);
      const Colour* const saliencyRowBodyStart = saliencyPixel + bodyRow;
      // TODO(jayen): fill/skip above the horizon
      while (saliencyPixel < saliencyRowBodyStart) {
         // if the saliency density is even, then optimise and use the 16MB
         // table
         #if SALIENCY_DENSITY % 2
         // special classify function which reads a pixel pair
         const Colour c = classify(currentFramePixel, nnmc);
         #else
         // special classify function which reads a pixel pair
         const Colour c = classify_UYV(currentFramePixel, nnmcVYU);
         #endif

         *saliencyPixel = c;

         // only update the histogram where it is used
         if (c == cGOAL_BLUE || c == cGOAL_YELLOW) {
            ++xHistogram[c];
            ++yHistogram[c];
         }

         // increment for the next pixel down
         ++saliencyPixel;
         yHistogram += cNUM_COLOURS;
         currentFramePixel += SALIENCY_DENSITY * IMAGE_COLS * 2;
      }
      // TODO(jayen): only when off-nao is connected
      const Colour* const saliencyRowEnd =
      saliencyPixel + IMAGE_ROWS/SALIENCY_DENSITY - bodyRow;
      while (saliencyPixel < saliencyRowEnd) {
         *saliencyPixel = cBACKGROUND;
         ++saliencyPixel;
      }

      // increment for the next pixel to the right
      currentFramePixel -= bodyRow * SALIENCY_DENSITY * IMAGE_COLS * 2;
      currentFramePixel += SALIENCY_DENSITY * 2;

      xHistogram += cNUM_COLOURS;
      ++stop;
   } while (saliencyPixel < saliencyEnd);
}

void Vision::loadColourCalibration(const char *filename) {
   int fd, rr;
   fd = open(filename, O_RDONLY);
   llog(INFO) << "Loading calibration file " << filename << endl;
   // error checking for the calibration file. Loads the file to nnmc
   if (fd < 0) {
      llog(FATAL) << "Error opening calibration file " << filename << endl;
      throw runtime_error("Error opening calibration file");
   } else {
      nnmc = boost::shared_array<uint8_t>(new uint8_t[MAXY*MAXU*MAXV]);
      rr = read(fd, nnmc.get(), MAXY*MAXU*MAXV);
      if (rr == MAXY*MAXU*MAXV) {
         llog(INFO) << "nnmc loaded from " << filename << endl;
         isColourCalibrationLoaded = true;

         nnmcVYU = boost::shared_array<Colour>(new Colour[1<<24]);
         for (int v = 0; v < MAXV; ++v) {
            for (int y = 0; y < MAXY; ++y) {
               for (int u = 0; u < MAXU; ++u) {
                  for (int voff = 0; voff < 256/MAXV; ++voff) {
                     for (int yoff = 0; yoff < 256/MAXY; ++yoff) {
                        for (int uoff = 0; uoff < 256/MAXU; ++uoff) {
                           nnmcVYU[(((v << (8 - MAXV_POW)) + voff) << 16) |
                                   (((y << (8 - MAXY_POW)) + yoff) <<  8) |
                                    ((u << (8 - MAXU_POW)) + uoff)] =
                                 (Colour) (nnmc[(y << (MAXU_POW + MAXV_POW)) |
                                                (u <<  MAXV_POW) |
                                                 v] & ~MAYBE_BIT);
                        }
                     }
                  }
               }
            }
         }
      } else {
         llog(FATAL) << "nnmc too small: " << filename << endl;
         throw runtime_error("nnmc too small!");
      }
   }
   close(fd);
}

bool Vision::getIsColourCalibrationLoaded(void) {
   return isColourCalibrationLoaded;
}

boost::shared_ptr<RobotCamera> Vision::camera
   = boost::shared_ptr<RobotCamera>((RobotCamera *)NULL);
