Using libo3d3xx
===============

A useful exercise for learning how to use the library would be to look at the
code for the command-line utilities provided with `libo3d3xx`. In particular,
the `o3d3xx-viewer` application would be instructive. That said, using the
library basically boils down to:

	#include "o3d3xx.h"

	// instantiate the camera with default ip, xmlrpc port, and password
	o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();

	// associate the camera with a frame grabber
	o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam);

	// create an buffer for holding the image data from the camera
	o3d3xx::ImageBuffer::Ptr buff = std::make_shared<o3d3xx::ImageBuffer>();

	// now stream the data
	while (true)
	{
	  // block for .5 sec waiting for data from the camera
      if (! fg->WaitForFrame(buff.get(), 500))
	    {
	      continue;
        }

      // Now access to the current frame is available as:
	  //
	  // 1. A PCL point cloud
	  // buff->Cloud()
	  //
	  // 2. OpenCV images
	  // buff->DepthImage()
	  // buff->ConfidenceImage()
	  // buff->AmplitudeImage()
	  //
	  // Use the power of these open-source library to build your application
	  //
    }
