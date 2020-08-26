// #include <ros/ros.h>
#include <image_transport/image_transport.h>


namespace publish_image_folder
{
  class PublishImageFolder
  {
  private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_priv_nh;
    image_transport::ImageTransport m_it;
    image_transport::Publisher m_it_pub_image;

    std::string m_topic_pub_image;
    std::string m_image_folder;
    std::string m_window_name;

    bool m_running;
    bool m_debug_window;
    int m_rate;

    int initialize();
    int readParameters();
    void initializePublishers();
    void startReadingFolder();

  public:
    PublishImageFolder(const ros::NodeHandle &nh_ = ros::NodeHandle(), const ros::NodeHandle &priv_nh_ = ros::NodeHandle("~"));
    ~PublishImageFolder() = default;
    bool start();
    void stop();
  };
} // namespace publish_image_folder
