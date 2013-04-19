#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl/io/openni_grabber.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <boost/filesystem.hpp>
#include "config.hpp"
using namespace std;
namespace fs = boost::filesystem;
using namespace util;

// parameters
int downsample = 3;
////


vector<double> stamps;
int sizes[2] = {480, 640};
cv::Mat rgb_mat(2, sizes, CV_8UC3);
cv::Mat depth_mat(2, sizes,CV_16UC1);

fs::path rgbd_dir;

void callback (const boost::shared_ptr<openni_wrapper::Image>& rgb, const boost::shared_ptr<openni_wrapper::DepthImage>& depth, float constant) {
  static int total_counter=0, save_counter=0;
  if (total_counter % downsample == 0) {
    stamps.push_back(ros::Time::now().toSec());
    const XnDepthPixel* depth_data = depth->getDepthMetaData().Data(); //unsigned short

    rgb->fillRGB(640, 480, rgb_mat.data, 640*3);
    for (int i=0; i < 480; ++i) {
      for (int j=0; j < 640; ++j) {
        cv::Vec3b& pix = rgb_mat.at<cv::Vec3b>(i,j);
        swap(pix[0], pix[2]);
      }
    }


    depth->fillDepthImageRaw(640, 480, (unsigned short*) depth_mat.data);

    cv::imwrite( (rgbd_dir / (boost::format("depth%.2i.png")%save_counter).str()).string(), depth_mat);
    cv::imwrite( (rgbd_dir / (boost::format("rgb%.2i.jpg")%save_counter).str()).string(), rgb_mat);
    save_counter++;
  }

  total_counter++;

}




int main(int argc, char** argv) {

  Config config;
  config.add(new Parameter<int>("downsample", &downsample, "ratio to downsample by"));
  string outdir;
  config.add(new Parameter<string>("out", &outdir, "output directory"));
  CommandParser(config).read(argc, argv);



  ros::init(argc, argv, "record_rgbd_video");
  ros::NodeHandle nh;
  bool time_ok = ros::Time::waitForValid(ros::WallDuration(1));
  pcl::OpenNIGrabber interface;
    // connect callback function for desired signal. In this case its a point cloud with color values
  boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float)> f(&callback);
  boost::signals2::connection c = interface.registerCallback (f);

  
  if (outdir.empty()) rgbd_dir = fs::unique_path("rgbd_%%%%%%%");
  else rgbd_dir = outdir;

  create_directories(rgbd_dir);
    // start receiving point clouds
  printf("saving data to %s\n", rgbd_dir.string().c_str());

  cv::Size size(640, 480);

  interface.start();
  printf("press ctrl-c to stop\n");
    
  while (ros::ok()) usleep(1e6 * .01);

  interface.stop();

  ofstream stampfile((rgbd_dir / "stamps.txt").string().c_str());
  stampfile << setprecision(20);
  for (int i=0; i < stamps.size(); ++i) {
    stampfile << stamps[i] << endl;
  }
  stampfile.close();

  printf("done\n");
}
