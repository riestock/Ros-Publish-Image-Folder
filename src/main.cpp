#include <publish_image_folder.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_image_folder_main", ros::init_options::AnonymousName);

  if (!ros::ok())
  {
    ROS_ERROR("ros::ok failed!");
    return -1;
  }

  publish_image_folder::PublishImageFolder PublishImageFolderNode;
  if (PublishImageFolderNode.start())
  {
    ros::spin();

    PublishImageFolderNode.stop();
  }

  ros::shutdown();
  return 0;
}
