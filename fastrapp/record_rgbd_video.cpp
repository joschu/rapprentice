#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <boost/filesystem.hpp>
#include "config.hpp"
using namespace std;
namespace fs = boost::filesystem;
using namespace util;

// parameters
int downsample = 3;
bool on_prompt = false;
bool on_triangle = false;
bool verbose = true;
////

bool frame_requested = false; // only relevant when wait_for_prompt = true;

vector<double> stamps;
int sizes[2] = {480, 640};
cv::Mat rgb_mat(2, sizes, CV_8UC3);
cv::Mat depth_mat(2, sizes,CV_16UC1);

fs::path rgbd_dir;

const int TRIANGLE_BUTTON = 12;

void joy_callback(const sensor_msgs::Joy& joy) {
  static int prev_button=0;
  int cur_button = joy.buttons[TRIANGLE_BUTTON];
  if (cur_button && !prev_button) frame_requested = true;
  prev_button = cur_button;
}

void callback (const boost::shared_ptr<openni_wrapper::Image>& rgb, const boost::shared_ptr<openni_wrapper::DepthImage>& depth, float constant) {
  static int total_counter=0, save_counter=0;
  if ( (on_prompt && frame_requested) || (on_triangle && frame_requested) || (!on_prompt && !on_triangle && (total_counter % downsample == 0)) ) {
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

    bool success1 = cv::imwrite( (rgbd_dir / (boost::format("depth%05d.png")%save_counter).str()).string(), depth_mat);
    bool success2 = cv::imwrite( (rgbd_dir / (boost::format("rgb%05d.jpg")%save_counter).str()).string(), rgb_mat);
    if (!success1 || !success2) throw std::runtime_error("failed to write image");
    if (verbose) printf("saved rgb/depth images %i\n", save_counter);


    save_counter++;

    frame_requested = false;
  }

  total_counter++;

}




int main(int argc, char** argv) {

  Config config;
  config.add(new Parameter<int>("downsample", &downsample, "ratio to downsample by"));
  config.add(new Parameter<bool>("on_prompt", &on_prompt, "only acquire image after user presses enter"));
  config.add(new Parameter<bool>("on_triangle", &on_triangle, "only acquire image after user presses triangle"));
  config.add(new Parameter<bool>("verbose", &verbose, "verbose"));
  string outdir;
  config.add(new Parameter<string>("out", &outdir, "output directory"));
  CommandParser(config).read(argc, argv);

  printf("record_rgbd_video: ROS_MASTER_URI=%s\n", (getenv("ROS_MASTER_URI") != NULL) ? getenv("ROS_MASTER_URI") : "null" );


  ros::init(argc, argv, "record_rgbd_video");
  ros::NodeHandle nh;
  bool time_ok = ros::Time::waitForValid(ros::WallDuration(1));
  pcl::OpenNIGrabber interface;
    // connect callback function for desired signal. In this case its a point cloud with color values
  boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float)> f(&callback);
  boost::signals2::connection c = interface.registerCallback (f);


  ros::Subscriber joy_sub;
  if (on_triangle) joy_sub = nh.subscribe("/joy", 1, joy_callback);
  
  if (outdir.empty()) rgbd_dir = fs::unique_path("rgbd_%%%%%%%");
  else rgbd_dir = outdir;

  create_directories(rgbd_dir);
    // start receiving point clouds
  printf("saving data to %s\n", rgbd_dir.string().c_str());

  cv::Size size(640, 480);

  interface.start();
    
  if (on_prompt) {
    while (ros::ok()) {
      printf("press enter to acquire next frame\n");
      cin.get();
      frame_requested = true;
    }
  }
  else {
    ros::spin();
  }

  interface.stop();

  ofstream stampfile((rgbd_dir / "stamps.txt").string().c_str());
  stampfile << setprecision(20);
  for (int i=0; i < stamps.size(); ++i) {
    stampfile << stamps[i] << endl;
  }
  stampfile.close();

  printf("rgbd video: done\n");
}
