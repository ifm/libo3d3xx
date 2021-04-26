#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"
#include "o3d3xx_image.h"

using namespace pcl;
using namespace pcl::io;
using namespace std;
using namespace cv;

#define BOLD    "\033[1m"
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

string PCDFilenameBase;

int getSavedPCD_(int id, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_xyzi)
{

    if (PCDFilenameBase.empty())
    {
        cout<<RED<<"PCDFilenameBase not set !!"<<endl;
        exit(-1);
    }
    std::string filename;

    filename = PCDFilenameBase + std::to_string(id) + ".pcd";

    if (pcl::io::loadPCDFile (filename, *cloud_xyzi) == -1) //load the file
    {
        cout<<RED<<"Error!! Couldn't read file"<<filename<<RESET<<endl;
        exit (-1);
    }
    cout<<GREEN<<"Loaded image "<<filename<<RESET<<"\n";
    return 1;
}

void testCapture(string camIP)
{

    o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>(camIP);
    o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();
    o3d3xx::FrameGrabber::Ptr fg = std::make_shared<o3d3xx::FrameGrabber>(cam, o3d3xx::IMG_AMP|o3d3xx::IMG_RDIS|o3d3xx::IMG_CART);
    o3d3xx::Logging::Init();
    fg->SWTrigger();
    if (! fg->WaitForFrame(img.get(), 1000))
    {
        std::cerr << "Timeout waiting for camera!" << std::endl;
        exit (-1);
    }
    cout<<"Captured pcd"<<endl;
}

int main(int argc, char** argv)
{
    testCapture("192.168.0.25");
    return 1;
}
