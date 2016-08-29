#include <iostream>
#include <memory>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"
#include "o3d3xx_image.h"

int main(int argc, const char **argv)
{
  o3d3xx::Logging::Init();
  int N = 100;
  std::cout << "Running crash test..." << std::endl;

  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>();
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();
  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(cam, o3d3xx::IMG_UVEC);
  std::cout << "Getting unit vectors..." << std::endl;
  if (! fg->WaitForFrame(img.get(), 1000))
    {
      std::cerr << "Timeout waiting for unit vectors!" << std::endl;
      return -1;
    }

  img->Organize(); // <-- forces parsing of the bytes

  //
  // Now: Pick one of: (A), (B), or (C) below...
  //

  //  (A) - this works OK
  img.reset(new o3d3xx::ImageBuffer());
  fg.reset(new o3d3xx::FrameGrabber(cam, o3d3xx::IMG_RDIS));

  // (B) - this works OK ... probably preferred over (A)
  //img = std::make_shared<o3d3xx::ImageBuffer>();
  //fg = std::make_shared<o3d3xx::FrameGrabber>(cam, o3d3xx::IMG_RDIS);

  // (C) - seg fault! -- duplicates the issue
  //fg.reset(new o3d3xx::FrameGrabber(cam, o3d3xx::IMG_RDIS));

  std::cout << "Running loop: " << N << std::endl;
  for (int i = 0; i < N; ++i)
    {
      if (! fg->WaitForFrame(img.get(), 1000))
        {
          std::cerr << "Timeout on Frame: " << i << std::endl;
          continue;
        }

      img->Organize(); // <-- force parsing of hte bytes
      std::cout << "Frame: " << i << std::endl;
    }

  std::cout << "Done." << std::endl;
  return 0;
}
