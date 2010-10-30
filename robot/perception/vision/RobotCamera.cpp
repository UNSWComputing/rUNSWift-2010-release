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

#include "perception/vision/RobotCamera.hpp"
#include "utils/Timer.hpp"
#include "utils/log.hpp"

using namespace std;

RobotCamera::RobotCamera() : dumpFile(0) {
   imageSize = IMAGE_WIDTH * IMAGE_HEIGHT * 2;
}

bool RobotCamera::startRecording(const char *filename, uint32_t frequency_ms) {
   this->frequency_ms = frequency_ms;
   if (dumpFile != NULL) {
      fclose(dumpFile);
   }
   dumpFile = fopen(filename, "w");
   llog(INFO) << "Starting camera dump to file: " << filename << endl;
   return dumpFile != NULL;
}

void RobotCamera::stopRecording() {
   if (dumpFile != NULL) {
      fclose(dumpFile);
      dumpFile = NULL;
   }
   llog(INFO) << "Finishing camera dump to file" << endl;
}

void RobotCamera::writeFrame(const uint8_t*& image) {
   static Timer t;
   if (dumpFile != NULL && image != NULL) {
      if (t.elapsed_ms() >= frequency_ms) {
         t.restart();
         llog(DEBUG3) << "Writing frame to dumpFile" << endl;
         int written = fwrite(image, imageSize, 1, dumpFile);
         llog(DEBUG3) << "wrote " << written << " frames" << endl;
         fflush(dumpFile);
      }
   }
}
