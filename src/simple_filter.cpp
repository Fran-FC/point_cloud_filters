#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class SimplePointCloudScan
{
public:
  SimplePointCloudScan()
  {
    reduce_frequency_ = false;

    if (!ros::param::get("/simple_point_cloud_filter/reduce_points_factor", reduce_points_factor_))
      reduce_points_factor_ = 1;
    if (!ros::param::get("/simple_point_cloud_filter/reduce_frequency_factor", reduce_frequency_factor_))
      reduce_frequency_factor_ = 1;

    ROS_INFO("Frequency factor: %d", reduce_frequency_factor_);
    ROS_INFO("Resolution factor: %d", reduce_points_factor_);

    if (reduce_frequency_factor_ > 1)
    {
      reduce_frequency_ = true;
      scan_counter_ = reduce_frequency_factor_;
    }

    sub_ = nh_.subscribe("scan", 10, &SimplePointCloudScan::scanCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_scan_filtered", 1000);
  }

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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_point_cloud_filter");
  SimplePointCloudScan s;
  ros::spin();
  return 0;
}
