#include <publish_image_folder.h>
#include <nodelet/nodelet.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>

namespace publish_image_folder
{

  PublishImageFolder::PublishImageFolder(const ros::NodeHandle &nh, const ros::NodeHandle &priv_nh) : m_nh(nh),
                                                                                                      m_priv_nh(priv_nh),
                                                                                                      m_it(nh),
                                                                                                      m_topic_pub_image(""),
                                                                                                      m_running(false),
                                                                                                      m_debug_window(true),
                                                                                                      m_rate(1)
  {
  }

  bool PublishImageFolder::start()
  {
    if (m_running)
    {
      ROS_ERROR("[PublishImageFolder] is already running!");
      return false;
    }
    if (!initialize())
    {
      ROS_ERROR("[PublishImageFolder] Initialization failed!");
      return false;
    }
    m_running = true;

    startReadingFolder();

    return true;
  }

  void PublishImageFolder::stop()
  {
    if (!m_running)
    {
      ROS_ERROR("[PublishImageFolder] is not running!");
      return;
    }
    m_running = false;

    if (m_debug_window)
    {
      cv::destroyWindow(m_window_name);
    }

    m_it_pub_image.shutdown();

    m_nh.shutdown();
    m_priv_nh.shutdown();
  }

  int PublishImageFolder::initialize()
  {
    if (!readParameters())
    {
      ROS_ERROR("[PublishImageFolder] Could not read parameters!");
    }

    m_window_name = "publish_image_folder";

    if (m_debug_window)
    {
      cv::namedWindow(m_window_name);
      cv::startWindowThread();
    }

    initializePublishers();

    return true;
  }

  int PublishImageFolder::readParameters()
  {
    return m_priv_nh.getParam("rate", m_rate) &&
           m_priv_nh.getParam("debug_window", m_debug_window) &&
           m_priv_nh.getParam("image_folder", m_image_folder) &&
           m_priv_nh.getParam("topic_pub_image", m_topic_pub_image);
  }

  void PublishImageFolder::initializePublishers()
  {
    ROS_INFO("[PublishImageFolder] Initializing Publishers");
    m_it_pub_image = m_it.advertise("camera/image", 1);

  }


  void PublishImageFolder::startReadingFolder()
  {
    // cv::Mat image;
    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    // std::filesystem::directory_iterator(m_image_folder);
    boost::filesystem::path p(m_image_folder);




    if (!(exists(p)))
    {
      ROS_ERROR("[PublishImageFolder] The given image folder path does not exist!");
      return;
    }

    if (!(is_directory(p)))
    {
      ROS_ERROR("[PublishImageFolder] The given image folder path is not a directory");
      return;
    }

    boost::filesystem::directory_iterator it{p};

    // boost::filesystem::directory_entry &entry;

    cv::Mat image;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(m_rate);
    while (m_nh.ok())
    {
      ROS_INFO("[PublishImageFolder] Publishing %s", it->path().string().c_str());

      image = cv::imread(it->path().string(), cv::IMREAD_COLOR);

      if (m_debug_window)
        cv::imshow(m_window_name, image);
      

      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      m_it_pub_image.publish(msg);

      it++;
      ros::spinOnce();
      loop_rate.sleep();
    }

    return;
  }

class PublishImageFolderNodelet : public nodelet::Nodelet
{
private:
  PublishImageFolder *publish_image_folder_ptr;

public:
  PublishImageFolderNodelet() : Nodelet(), publish_image_folder_ptr(nullptr)
  {
    ROS_INFO("[PublishImageFolderNodelet] Constructor call");
  }

  ~PublishImageFolderNodelet() override
  {
    ROS_INFO("[PublishImageFolderNodelet] Destructor call");
    if (publish_image_folder_ptr)
    {
      publish_image_folder_ptr->stop();
      delete publish_image_folder_ptr;
    }
  }

  void onInit() override
  {
    ROS_INFO("[PublishImageFolderNodelet] onInit");
    publish_image_folder_ptr = new PublishImageFolder(getNodeHandle(), getPrivateNodeHandle());
    if (!publish_image_folder_ptr->start())
    {
      delete publish_image_folder_ptr;
      publish_image_folder_ptr = nullptr;
      throw nodelet::Exception("[PublishImageFolderNodelet] Could not start nodelet");
    }
  }
};

} // namespace publish_image_folder
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(publish_image_folder::PublishImageFolderNodelet, nodelet::Nodelet)
