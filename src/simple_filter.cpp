#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class SimplePointCloudScan
{
public:
  float frequency_;

  SimplePointCloudScan()
  {
    if (!ros::param::get("/simple_point_cloud_filter/reduce_points_factor", reduce_points_factor_))
      reduce_points_factor_ = 1;
    if (!ros::param::get("/simple_point_cloud_filter/frequency", frequency_))
      frequency_ = 1.0;

    ROS_INFO("Frequency: %.1f Hz", frequency_);
    ROS_INFO("Resolution reduce factor: %d", reduce_points_factor_);

    sub_ = nh_.subscribe("scan", 1, &SimplePointCloudScan::scanCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_scan_filtered", 10);
  }

private:

  // values read from config file
  int reduce_points_factor_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  void scanCallback(const sensor_msgs::PointCloud2::Ptr& scan)
  {
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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_point_cloud_filter");
  SimplePointCloudScan s;

  ros::Rate rate(s.frequency_);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}
