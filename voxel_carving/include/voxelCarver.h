#include "ros/ros.h"
#include "common.h"
#include "utils.h"
#include "rendering.h"
#include "voxel_carving.h"
#include "voxel_colouring.h"
#include <json/json.h>
#include <cassert>
#include <regex>
#include <vector>
#include "red_msgs/ImageData.h"
#include "red_msgs/VoxelCarving.h"
#include "std_srvs/Empty.h"


class VoxelCarver
{
    public:
        VoxelCarver(ros::NodeHandle& n);
        bool onNewImageCallback(red_msgs::ImageData::Request& req, 
                red_msgs::ImageData::Response& res);
        bool carveCallback(std_srvs::Empty::Request& req,
                std_srvs::Empty::Response& res);
    
    private:
        std::vector<cv::Mat> poses;
        std::vector<cv::Mat> silhouettes;
        std::vector<cv::Mat> images;
        ros::NodeHandle& n;
        ros::ServiceServer carveService;
        ros::ServiceServer onNewImageService;
        
};