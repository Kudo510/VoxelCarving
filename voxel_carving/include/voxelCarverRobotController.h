#pragma once
#include "robot_controllers/controller/controller.h"
#include "robot_controllers/VoxelConfig.h"
#include "sensor_msgs/Image.h"
#include "red_msgs/frankX_end_effector_T.h"
#include "red_msgs/ImageData.h"
#include "red_msgs/ArucoImage.h"
#include "red_msgs/VoxelCarving.h"
#include "std_srvs/Empty.h"
#include <numeric>

struct ImageData
{
    std::string encoding;
    std::vector<uint8_t, std::allocator<uint8_t>> data;
    uint32_t width;
    uint32_t height;
};

struct VoxelCarvingParams
{
    Pose center = {0.4,0.0,0.1};
    double radius;
    double limit_angle;
    int number_of_pictures;
    double height_of_camera;
};

enum class State
{
    SUCCESS,
    NOT_REACHED,
    FINISHED
};

enum class RotationAxis
{
    X,
    Y,
    Z
};



class VoxelCarvingController : public Controller
{
private:

    ros::NodeHandle& n_voxel;
    ros::ServiceClient endEffectorTMatrixService;
    ros::Subscriber cameraSubscriber;
    ros::ServiceClient imageSaverService;
    ros::ServiceClient imageProcesserService;

    ros::ServiceClient onNewMarkerImageService;
    ros::ServiceClient carveService;
    
    ImageData imageData;
    VoxelCarvingParams voxelCarvingParams;
    
    void callback_image(const sensor_msgs::Image::ConstPtr& msg);
    
    std::vector<Frame> generatePoints(const Pose& center, double radius, 
        int numberOfPoints, RotationAxis axis = RotationAxis::Z,
        double minAngle= -M_PI_2, double maxAngle = M_PI_2);
    
    Frame calculatePoint(const Pose& center, RotationAxis axis, double radius, double angle);
    Orientation calculateOrientation(
        const Pose& targetObjectPosition , const Pose& cameraPosition, const RotationAxis axis);
    
    template <class F = float, class U = unsigned char>
            std::vector<F> linspace(F init, F end, U size) 
    {
        F inc = (end - init) / static_cast<F>(size - 1); // Calculating the increment.
        std::vector<F> ret(size);
        ret[0] = init;
        ret[size - 1] = end; 
        F aux = end;
        for (size_t i = size - 1; i >= 1; --i, aux -= inc) 
            ret[i] = aux;
        return ret;
    }

    // objects and functions used for the dynamic reconfiguration server
    void reconfigure_voxel_params(robot_controllers::VoxelConfig &config, uint32_t level);
    dynamic_reconfigure::Server<robot_controllers::VoxelConfig>* voxel_param_server;
    dynamic_reconfigure::Server<robot_controllers::VoxelConfig>::CallbackType voxel_param_callback;


public:

    VoxelCarvingController(ros::NodeHandle& n_voxel, ros::NodeHandle& n_controller);
    ~VoxelCarvingController();

    void takeScreenshot(std::string file);
    void processImage();
    void performVoxelCarving(std::string carvingParamsFile);
    void moveAroundAxis();
    void moveAndSaveImages(std::string file);
    
};

