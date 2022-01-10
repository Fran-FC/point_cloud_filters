#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class SimpleScan
{
private:
  // auxiliar variables
  int scan_counter_;
  bool reduce_frequency_;
  // values read from config file
  int reduce_points_factor_;
  int reduce_frequency_factor_;
  
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  void scanCallback(const sensor_msgs::PointCloud2::Ptr& scan)
  {
    // Reduce frequency block
    if (reduce_frequency_)
    {
      if (scan_counter_ != 1)
      {
        --scan_counter_;
        return;
      }
      scan_counter_ = reduce_frequency_factor_;
    }

    int k = 0;
    for (int i = 0; i < scan->width; i += reduce_points_factor_)
    {
      for (int j = 0; j < scan->point_step; j++)
      {
        scan->data[k] = scan->data[j + (scan->point_step * i)];
        ++k;
      }
    }
    scan->width = std::floor(scan->width / reduce_points_factor_);
    scan->row_step = scan->point_step * scan->width;

    scan->data.resize(scan->row_step);
    pub_.publish(scan);

    return;
  }

public:
  SimpleScan()
  {
    reduce_frequency_ = false;

    // parameters hardcoded, ToDo: read from yaml
    reduce_points_factor_ = 2;
    reduce_frequency_factor_ = 2;

    if (reduce_frequency_factor_ > 1) {
      reduce_frequency_ = true;
      scan_counter_ = reduce_frequency_factor_;
    }

    sub_ = nh_.subscribe("velodyne_points2", 10, &SimpleScan::scanCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_scan_filtered", 1000);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_filter");
  SimpleScan s;
  ros::spin();
  return 0;
}
