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

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <utility>
#include <string>
#include "perception/vision/RobotCamera.hpp"
#include "perception/vision/NaoCamera.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/ImageToRR.hpp"
#include "perception/vision/yuv.hpp"
#include "perception/vision/RegionBuilder.hpp"
#include "perception/vision/BallDetection.hpp"
#include "perception/vision/FieldLineDetection.hpp"
#include "perception/vision/GoalDetection.hpp"
#include "perception/vision/RobotDetection.hpp"
#include "perception/vision/FieldEdgeDetection.hpp"
#include "utils/Line.hpp"
#include "utils/Timer.hpp"

/* Helps me see
*/
class Vision {
   friend class VisionAdapter;
   friend class CalibrationTab;
   friend class VisionTab;
   friend class OverviewTab;
   friend class Tab;
   friend class CameraPoseTab;

   friend class GoalDetection;
   friend class BallDetection;
   friend class RobotDetection;

   public:
      /**
       * Constructor for vision module. Initialises the camera
       * Only displays an error if camera not initated
       * @param foo A dummy parameter to demonstrate that you can
       *    pass initialisation values to constructor if desired
       **/
      Vision(bool dumpframes = false, int dumprate = 1000,
         std::string dumpfile = "dump.yuv", bool visionEnabled = true);

      /* Destructor */
      ~Vision();

   private:
      /**
       * Camera object, talks to v4l or NaoQi
       **/
      static boost::shared_ptr<RobotCamera> camera;
      /**
       * Pointer to the frame currently being processed
       **/
      uint8_t const* currentFrame;

      /**
       * C-Space lookup table
       **/
      boost::shared_array<uint8_t> nnmc;

      /**
       * Calibration table optimized for saliency scan
       */
      boost::shared_array<Colour> nnmcVYU;

      /**
       * Boolean set to true if colour calibration file is loaded.
       **/
      bool isColourCalibrationLoaded;

      /**
       * Histograms indicating how many pixels of each colour appear
       * in each row and column of the image
       **/
      XHistogram xhistogram[IMAGE_COLS/SALIENCY_DENSITY][cNUM_COLOURS];
      YHistogram yhistogram[IMAGE_ROWS/SALIENCY_DENSITY][cNUM_COLOURS];
      /**
       * C-Space representation of the pixels in the saliency scan
       **/
      Colour saliency[IMAGE_COLS/SALIENCY_DENSITY]
         [IMAGE_ROWS/SALIENCY_DENSITY];

      /**
       * Seed value for rand_r
       **/
      unsigned int seed;

      /**
       * which camera is in use
       **/
      WhichCamera whichCamera;

      /**
       * A nice wall clock
       **/
      Timer timer;

      /**
       * Settings related to recording frames to disk
       **/
      bool dumpframes;
      int dumprate;
      std::string dumpfile;
      bool visionEnabled;

      /**
       * The major sections of vision processing used by process frame
       **/
      FieldEdgeDetection fieldEdgeDetection;
      FieldLineDetection fieldLineDetection;
      RegionBuilder regionBuilder;
      BallDetection ballDetection;
      GoalDetection goalDetection;
      RobotDetection robotDetection;
      CameraToRR convRR;

      /**
       * Get a frame from the camera and put a pointer to it
       * in currentFrame
       **/
      void getFrame();

      /**
       * Processes a single frame, reading it from the camera
       * finding all interesting objects, and running sanity checks
       **/
      void processFrame();

      /**
       * Returns the colour of a pixel in the image. j is the
       * y value of the pixel, i is the x value
       **/
      inline Colour getColour(uint16_t j, uint16_t i);

      /**
       * Subsample the image, forming a colour histogram
       * in the x-axis and y-axis, for each colour in
       * the C-PLANE.
       **/
      void saliencyScan();

      /**
       * Classifies a certain set of yuv values
       * given a colour calibration file
       * @param y y value of pixel
       * @param u u value of pixel
       * @param v v value of pixel
       * @return the classified colour of the yuv values
       **/
      inline Colour classify(uint8_t y, uint8_t u, uint8_t v);

      /**
       * Classifies a pixel of a YUV422 image
       * given a colour calibration file
       * @param pixel address of Y component of pixel.  assumes image is 4-byte
       *              aligned
       * @return the classified colour of the yuv values
       **/
      inline static const Colour classify(const uint8_t *const pixel,
                                          const uint8_t *const nnmc);

      /**
       * Classifies a pixel of a YUV422 image
       * given a colour calibration file
       * @param pixel address of Y component of pixel.  assumes image and pixel
       *              are 4-byte aligned
       * @return the classified colour of the yuv values
       **/
      inline static const Colour classifyYU_V(const uint8_t *const pixel,
                                              const uint8_t *const nnmc);

      /**
       * Classifies a pixel of a YUV422 image
       * given a colour calibration file
       * @param pixel address of Y component of pixel.  assumes image is 4-byte
       *              aligned and pixel is 2-byte offset
       * @return the classified colour of the yuv values
       **/
      inline static const Colour classify_UYV(const uint8_t *const pixel,
                                              const uint8_t *const nnmc);

      /**
       * Classifies a pixel of a YUV422 image
       * given a colour calibration file
       * @param pixel address of Y component of pixel.  assumes image is 4-byte
       *              aligned and pixel is 2-byte offset
       * @return the classified colour of the yuv values
       **/
      inline static const Colour classify_UYV(const uint8_t *const pixel,
                                              const Colour *const nnmc);

      /**
       * Returns the y, u, v values of a pixel in the image
       * Used when do not want to use the classified colours
       **/
      inline PixelValues getPixelValues(uint16_t row, uint16_t col);

      /**
       * Loads the specified calibration file. Or the default
       * @param filename file to be loaded
       **/
      void loadColourCalibration(
            const char *filename = "/home/nao/data/nnmc.cal");

      /**
       * Returns true if colour calibration is loaded
       * @return bool indicating if colour calibration is loaded
       **/
      bool getIsColourCalibrationLoaded(void);
};

