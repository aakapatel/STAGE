#include "explorer_base.h"

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>

#define SQ(x) (x * x)


GraphExploration::GraphExploration (const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private) {

  initializeAttributes(); 

}

void GraphExploration::initializeAttributes () {

  visualization_ = new Visualization (nh_, nh_private_);



}
