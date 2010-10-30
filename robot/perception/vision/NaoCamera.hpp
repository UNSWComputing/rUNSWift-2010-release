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

#include <alvision/alvisiondefinitions.h>
#include <linux/videodev2.h>
#include <string>

#include "perception/vision/RobotCamera.hpp"

typedef enum {
   IO_METHOD_READ,
   IO_METHOD_MMAP,
   IO_METHOD_USERPTR,
   NUM_IO_METHODS
} IOMethod;

// only four frame buffers are supported on the Nao V3
#define NUM_FRAME_BUFFERS 4

// there are 17 controls on the Nao V3+ that we care about setting
#define NUM_CONTROLS 17

// these are undocumented controls available in the Nao V3+
#define V4L2_CID_AUTO_EXPOSURE_CORRECTION           (V4L2_CID_BASE+32)
#define V4L2_CID_SET_DEFAULT_PARAMETERS             (V4L2_CID_BASE+33)
#define V4L2_CID_EXPOSURE_CORRECTION                (V4L2_CID_BASE+34)
#define V4L2_CID_AUTO_EXPOSURE_CORRECTION_ALGORITHM (V4L2_CID_BASE+35)

/**
 * The IDs of the controls we set on the Nao
 */
extern __u32 controlIds[NUM_CONTROLS];

/**
 * The initial values for each camera for the controls we set on the Nao
 */
extern __s32 (*controlValues)[NUM_CONTROLS];

/**
 * NaoCamera provides methods for the vision module to interact with the nao camera
 */
class NaoCamera : public RobotCamera {
  public:
   /**
    * Constructor
    * opens the device, calibrates it, and sets it up for streaming
    *
    * @param filename the device or file to get images from
    * @param method the io method to use
    * @see IOMethod
    * @param format the format of the image
    * @see AL::kVGA
    * @param initialCamera which camera to start with (top or bottom)
    * @see WhichCamera
    */
   NaoCamera(const char *filename = "/dev/video0",
             const IOMethod method = IO_METHOD_MMAP,
             const int format = AL::kVGA,
             const WhichCamera initialCamera = TOP_CAMERA);

   /**
    * Destructor
    * closes the device
    */
   virtual ~NaoCamera();

   /**
    * tells the caller the current image dimensions
    *
    * @param width will be set to the image's width
    * @param height will be set to the image's height
    * @return if a correct format has been set
    */
   bool imgDimensions(int *const width, int *const height);

   /**
    * Get the image and convert it to the requested colourSpace (currently only
    * kYUVColorSpace (YUV422) is supported)
    *
    * @return a pointer to the image if the image was fetched successfully.
    * NULL, otherwise.
    */
   const uint8_t *get(const int colourSpace);

   /**
    * If applicable, changes the current camera being used.  If necesary,
    * disables the video stream momentarily so the stream can switch inputs.
    *
    * @param whichCamera specifies the top or bottom camera to switch to
    * @return whether the set succeeds
    * @sideeffect stops capturing, switches the camera, then starts again
    */
   bool setCamera(WhichCamera whichCamera);

   /**
    * Gets the current camera
    *
    * @return either the top or bottom camera
    */
   WhichCamera getCamera();

   /**
    * Sets a camera control (eg. hue)
    *
    * @param control the control id to set.
    * @param value the value to set the control to
    * @returns true if success, false otherwise
    */
   bool setControl(const uint32_t id, const int32_t value);

  private:
   /**
    * Actually reads the frame from the camera (or does the appropriate ioctl
    * call if not using the read io method)
    *
    * @return true if successful, false otherwise (if asked to read again)
    */
   const uint8_t *read_frame();

   /**
    * Calibrate the camera.  Turn off the auto-corrections.
    *
    * @param camera which camera settings to use. does not switch cameras.
    */
   void setCameraSettings(const WhichCamera camera);

   /**
    * Initialise this class
    */
   bool init_camera();

   /**
    * initializes the buffers for streaming i/o mode
    */
   void init_buffers(void);

   /**
    * prepares the buffers and device for capture
    */
   void start_capturing(void);

   /**
    * turns off the data stream
    */
   void stop_capturing(void);

   /**
    * cleans up the buffers
    */
   void uninit_buffers(void);

   /**
    * open the device
    */
   void open_device(void);

   /**
    * closes the device
    */
   void close_device(void);

   /**
    * open the i2cbus device
    */
   void open_i2cbus(void);

   /**
    * closes the i2cbus device
    */
   void close_i2cbus(void);

   /**
    * set mmap to write to the image buffer
    */
   void init_mmap(void);

   /**
    * set mmap to write to the image buffer
    */
   void init_userp(void);

   /**
    * just clears the image buffer
    */
   void init_read(void);

   /**
    * Tries to set the current camera being used.  DOES NOT disables the video
    * stream while switching inputs.
    *
    * @param whichCamera specifies the top or bottom camera to switch to
    * @return whether the set succeeds
    * @sideeffect may switch the camera
    */
   bool immediateSetCamera(WhichCamera whichCamera);

   /**
    * file name for camera
    */
   std::string filename;

   /**
    * device file handle for camera.  -1 indicates uninitialised
    */
   int fd;

   /**
    * v4l device?  true indicates that fd points to a v4l device.
    * false probably indicates a regular file
    */
   bool v4lDeviceP;

   /**
    * the location and length for each saved frame
    */
   struct Buffer {
      /**
       * the location for each frame.  in kernel space in mmap mode
       */
      uint8_t *start;

      /**
       * the length for each frame
       */
      size_t length;
   };

   /**
    * stores the location and length for each posisble saved frame
    */
   Buffer buffers[NUM_FRAME_BUFFERS];

   /**
    * the actual number of frames we're saving.  device dependent
    */
   unsigned int n_buffers;

   /**
    * the io method we are using right now
    */
   IOMethod io;

   /**
    * device handle for i2c bus.  used for switching cameras.
    */
   int i2cBus;

   int format;

   // Keep track of the current camera. if unkown should be set
   // to whichCameraError
   WhichCamera currentCamera;

   struct v4l2_queryctrl queryctrl;
   struct v4l2_querymenu querymenu;

   /**
    * When dealing with mmap'ed buffers, we should
    * 1. Dequeue a buffer and write its pointer to the blackboard
    * 2. Allow perception to process the image
    * 3. Enqueue the buffer again so it can be reused
    *
    * Before we were not allowing perception to process the current frame
    * before enqueuing the buffer again, which could have potentially have
    * lead to split images
    */
   struct v4l2_buffer lastDequeued;
};
