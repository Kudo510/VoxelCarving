#include "robot_controllers/voxelCarvingController/voxelCarvingController.h"

VoxelCarvingController::VoxelCarvingController(ros::NodeHandle& n_voxel, ros::NodeHandle& n_controller) 
    : n_voxel(n_voxel), Controller(n_controller)
{
    
    this->cameraSubscriber = n_voxel.subscribe("/camera/color/image_raw", 1000, 
            &VoxelCarvingController::callback_image, this);

    ros::service::waitForService("/frankX_service_node/frankX_end_effector_T", ros::Duration(3.0));
    this->endEffectorTMatrixService = 
        n_voxel.serviceClient<red_msgs::frankX_end_effector_T>("/frankX_service_node/frankX_end_effector_T");

    ros::service::waitForService("/marker_detector/save_ar_image", ros::Duration(3.0));
    this->imageSaverService = 
        n_voxel.serviceClient<red_msgs::ImageData>("/marker_detector/save_ar_image");

    ros::service::waitForService("/marker_detector/process_ar_image", ros::Duration(3.0));
    this->imageProcesserService = 
        n_voxel.serviceClient<red_msgs::ArucoImage>("/marker_detector/process_ar_image");

    ros::service::waitForService("/voxel_carver/on_new_image", ros::Duration(3.0));
    this->onNewMarkerImageService = 
        n_voxel.serviceClient<red_msgs::ImageData>("/voxel_carver/on_new_image");

    ros::service::waitForService("/voxel_carver/carve", ros::Duration(3.0));
    this->carveService = 
        n_voxel.serviceClient<std_srvs::Empty>("/voxel_carver/carve");

    

    this->voxel_param_server =
        new dynamic_reconfigure::Server<robot_controllers::VoxelConfig>(n_voxel);
    this->voxel_param_callback = 
        boost::bind(&VoxelCarvingController::reconfigure_voxel_params, this, _1, _2);
    this->voxel_param_server->setCallback(this->voxel_param_callback);
    
}

void VoxelCarvingController::reconfigure_voxel_params(robot_controllers::VoxelConfig &config, uint32_t level)
{
    this->voxelCarvingParams.center.x = config.x;
    this->voxelCarvingParams.center.y = config.y;
    this->voxelCarvingParams.center.z = config.z;

    this->voxelCarvingParams.limit_angle = config.limit_angle;
    this->voxelCarvingParams.number_of_pictures = config.number_of_pictures;
    this->voxelCarvingParams.radius = config.radius;
    this->voxelCarvingParams.height_of_camera = config.camera_height;

    ROS_INFO("voxel params updated!");
}

VoxelCarvingController::~VoxelCarvingController()
{
    free(this->voxel_param_server);
}

void VoxelCarvingController::callback_image(const sensor_msgs::Image::ConstPtr& msg)
{
    this->imageData.data = msg->data;
    this->imageData.encoding = msg->encoding;
    this->imageData.height = msg->height;
    this->imageData.width = msg->width;
}

void VoxelCarvingController::takeScreenshot(std::string file)
{   
    red_msgs::frankX_end_effector_T pose_args;
    this->endEffectorTMatrixService.call(pose_args);

    //ROS_INFO("image width: %d", this->imageData.width);
    //ROS_INFO("image height: %d", this->imageData.height);
    
    red_msgs::ImageData image_args;
    image_args.request.transform = pose_args.response.transform;
    image_args.request.data = imageData.data;
    image_args.request.file = file;
    ROS_INFO("calling save image service");
    this->imageSaverService.call(image_args);

}


Orientation VoxelCarvingController::calculateOrientation(
        const Pose& targetObjectPosition , const Pose& cameraPosition, const RotationAxis axis)
{
    double relativeX = cameraPosition.x - targetObjectPosition.x;
    double relativeY = cameraPosition.y - targetObjectPosition.y;
    double relativeZ = cameraPosition.z - targetObjectPosition.z;
    double radius = sqrt(relativeX*relativeX+relativeY*relativeY);

    double a = 0;
    double b = 0;
    double c = 0;


    switch (axis)
    {
    case RotationAxis::X:
        a = 0;
        b = std::atan(relativeX / relativeZ);
        c = - std::atan(relativeY / relativeZ);
        break;
    case RotationAxis::Y:
        a = M_PI/2;
        b = std::atan(relativeY / relativeZ);
        c = std::atan(relativeX / relativeZ);
        break;
    case RotationAxis::Z:
        a = std::atan2(relativeY, relativeX);
        b = M_PI/2 - std::atan2(relativeZ, radius);
        c = 0;
        //a = std::atan2(relativeY, relativeX);
        //b = 0;
        //c = M_PI/2 - std::atan2(relativeZ, radius);
        break;
    
    default:
        break;
    }

    ROS_INFO("relative calculated: [X: %f, Y: %f, Z: %f] ",
        relativeX, relativeY, relativeZ);

    return {a, b, c};
}

Frame VoxelCarvingController::calculatePoint(const Pose& center, RotationAxis axis, double radius, double angle)
{
    Frame currentPoint;
    currentPoint.pose.x = center.x;
    currentPoint.pose.y = center.y;
    currentPoint.pose.z = center.z + this->voxelCarvingParams.height_of_camera;

    switch (axis)
    {
    case RotationAxis::X:
        currentPoint.pose.y += radius * std::sin(angle);
        currentPoint.pose.z += radius * std::cos(angle);
        break;
    case RotationAxis::Y:
        currentPoint.pose.z += radius * std::sin(angle + M_PI/2);
        currentPoint.pose.x += radius * std::cos(angle + M_PI/2);
        break;
    case RotationAxis::Z:
        currentPoint.pose.y += radius * std::sin(angle);
        currentPoint.pose.x += radius * std::cos(angle);
        break;
    
    default:
        break;
    }

    Orientation newOrientation = calculateOrientation(center,currentPoint.pose,axis);
  
    currentPoint.orientation.a = newOrientation.a;
    currentPoint.orientation.b = newOrientation.b;
    currentPoint.orientation.c = newOrientation.c;

    return currentPoint;
}

std::vector<Frame> VoxelCarvingController::generatePoints(const Pose& center, double radius, 
         int numberOfPoints, RotationAxis axis, 
            double minAngle, double maxAngle)
{   
    // create linspace of values
    std::vector<Frame> points;
    points.reserve(numberOfPoints);
    std::vector<double> angles = linspace<double,int>(minAngle,maxAngle,numberOfPoints);

    for(auto angle : angles)
    {
        Frame point = calculatePoint(center,axis,radius,angle);
        ROS_INFO("point calculated: [X: %f, Y: %f, Z: %f] ", point.pose.x, point.pose.y, point.pose.z);
        ROS_INFO("orientation calculated: [a: %f, b: %f, c: %f] ", 
            point.orientation.a, point.orientation.b, point.orientation.c);
        points.push_back(point);
    }

    return points;
}

void VoxelCarvingController::moveAroundAxis()
{

    auto points = this->generatePoints(this->voxelCarvingParams.center,
        this->voxelCarvingParams.radius, this->voxelCarvingParams.number_of_pictures/3,
        RotationAxis::X, -1.57, 
        1.57 );

    for(Frame point : points)
    {
        this->robotParams.x = point.pose.x;
        this->robotParams.y = point.pose.y;
        this->robotParams.z = point.pose.z;

        this->moveCartesian();
        
        this->robotParams.a = point.orientation.a;
        this->robotParams.b = point.orientation.b;
        this->robotParams.c = point.orientation.c;

        this->moveCartesian();
        
        
    }

}



void VoxelCarvingController::moveAndSaveImages(std::string file)
{
    Pose center_for_z = {0.45,- 0.05,0.33};
    Pose center_for_x = {0.4,0.0,0.33};
    double radius = 0.07;
    
    // do full arc around X axis
    std::vector<Frame> points_x = this->generatePoints(center_for_x,
        radius, this->voxelCarvingParams.number_of_pictures/8,
        RotationAxis::X, -1.57, 
        1.57 );

    for(Frame point : points_x)
    {
        this->robotParams.x = point.pose.x;
        this->robotParams.y = point.pose.y;
        this->robotParams.z = point.pose.z;

        this->moveCartesian();
        
        this->robotParams.a = point.orientation.a;
        this->robotParams.b = point.orientation.b;
        this->robotParams.c = point.orientation.c;

        this->moveCartesian();
        
        this->takeScreenshot(file);
        
    }

    

    Pose center_for_y = {0.48,-0.05,0.33};
    
    //do arc around Y axis
    /*
    auto points_y = this->generatePoints(center_for_y,
        radius, this->voxelCarvingParams.number_of_pictures/8,
        RotationAxis::Y, - M_PI/2, M_PI/2);

    for(Frame point : points_y)
    {
        this->robotParams.x = point.pose.x;
        this->robotParams.y = point.pose.y;
        this->robotParams.z = point.pose.z;

        //this->moveCartesian();
        
        this->robotParams.a = point.orientation.a;
        this->robotParams.b = point.orientation.b;
        this->robotParams.c = point.orientation.c;

        this->moveCartesian();
        
        //this->takeScreenshot(file);
        
    }
    */

    this->moveHome();

    
    // do arc around Z axis

    std::vector<Frame> points_z_1 = this->generatePoints(center_for_z,
        radius, this->voxelCarvingParams.number_of_pictures/2,
        RotationAxis::Z,   -0.85 *  M_PI , 
         0.85 * M_PI );

    for(Frame point : points_z_1)
    {
        this->robotParams.x = point.pose.x;
        this->robotParams.y = point.pose.y;
        this->robotParams.z = point.pose.z;

        //this->moveCartesian();
        
        this->robotParams.a = point.orientation.a;
        this->robotParams.b = point.orientation.b;
        this->robotParams.c = point.orientation.c;

        this->moveCartesian();
        
        this->takeScreenshot(file);
        
    }

}


void VoxelCarvingController::processImage()
{
    red_msgs::ArucoImage processer_params;
    this->imageProcesserService.call(processer_params);
    red_msgs::ImageData new_image_params;
    new_image_params.request.data = processer_params.response.image;
    new_image_params.request.transform = processer_params.response.transform;
    new_image_params.request.segmented_image = processer_params.response.segmented_image;

    this->onNewMarkerImageService.call(new_image_params);
}


void VoxelCarvingController::performVoxelCarving(std::string carvingParamsFile)
{
   
    Pose center = {this->voxelCarvingParams.center.x,this->voxelCarvingParams.center.y,this->voxelCarvingParams.center.z};
    double radius = this->voxelCarvingParams.radius;

    this->moveHome();

    
    // do arc around Z axis

    std::vector<Frame> points_z_1 = this->generatePoints(center,
        radius, this->voxelCarvingParams.number_of_pictures/2,
        RotationAxis::Z,   -0.85 *  M_PI , 
         0.85 * M_PI );

    for(Frame point : points_z_1)
    {
        this->robotParams.x = point.pose.x;
        this->robotParams.y = point.pose.y;
        this->robotParams.z = point.pose.z;

        this->moveCartesian();
        
        this->robotParams.a = point.orientation.a;
        this->robotParams.b = point.orientation.b;
        this->robotParams.c = point.orientation.c;

        this->moveCartesian();

        this->processImage();
        
        
    }
    std_srvs::Empty req;
    this->carveService.call(req);
}
