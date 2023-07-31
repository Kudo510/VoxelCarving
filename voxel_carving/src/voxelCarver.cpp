#include "voxelCarver.h"

VoxelCarver::VoxelCarver(ros::NodeHandle& n) : n(n)
{
    this->carveService = 
        n.advertiseService("/voxel_carver/carve", &VoxelCarver::carveCallback, this);
    this->onNewImageService = 
        n.advertiseService("/voxel_carver/on_new_image", &VoxelCarver::onNewImageCallback, this);
}

bool VoxelCarver::onNewImageCallback(red_msgs::ImageData::Request& req, 
        red_msgs::ImageData::Response& res)
{   
    ROS_INFO("image saved on carver node");

    cv::Mat shilhouette(720,1280,CV_8UC3,req.segmented_image.data());
    this->silhouettes.emplace_back(shilhouette.clone());

    cv::Mat image(720,1280,CV_8UC3,req.data.data());
    this->images.emplace_back(image.clone());



    return true;
}


bool VoxelCarver::carveCallback(std_srvs::Empty::Request& req,
                std_srvs::Empty::Response& res)
{

    ROS_INFO("start carving");

    for( auto image : this->images)
    {
        
        cv::imshow("debug", image);
        cv::waitKey();
    }

    /*
        std::string config_path = req.config_path;
        

        // Read the JSON file
        std::ifstream ifs(config_path);
        assert(ifs);
        Json::Value config;
        ifs >> config;
        ifs.close();

        // Retrieve the values from the JSON
        std::string obj_path = config["obj_path"].asString();
        std::string matrices_path = config["matrices_path"].asString();
        std::string image_folder = config["image_folder"].asString();
        std::string silhouette_folder = config["silhouette_folder"].asString();
        bool use_masks = config["use_masks"].asBool();

        int voxel_dim = config["voxel_dim"].asInt();
        int voxel_size = config["voxel_size"].asInt();

        float start_x = config["start_x"].asFloat();
        float start_y = config["start_y"].asFloat();
        float start_z = config["start_z"].asFloat();

        // Intrinsics 
        float fx = config["fx"].asFloat();
        float fy = config["fy"].asFloat();
        float cx = config["cx"].asFloat();
        float cy = config["cy"].asFloat();

        // Voxel Grid 
        float xmin = config["xmin"].asFloat();
        float xmax = config["xmax"].asFloat();
        float ymin = config["ymin"].asFloat();
        float ymax = config["ymax"].asFloat();
        float zmin = config["zmin"].asFloat();
        float zmax = config["zmax"].asFloat();

        std::vector<camera> cameras;
        std::string extension = getFileExtension(matrices_path);

        cv::FileStorage viff_fs;
        std::vector<cv::Mat> matrices;

        matrices = readMatricesFromFile(matrices_path);
        

        std::string img_ext = use_masks ? ".png" : ".jpg";
        auto files = use_masks ? std::filesystem::directory_iterator(silhouette_folder) : std::filesystem::directory_iterator(image_folder);
        std::map<int, std::filesystem::directory_entry> image_paths;
        // Extract and print the indices from the sorted directory entries
        std::regex numericRegex;
        if (img_ext == ".png")
        {
            numericRegex = R"(image[_-](\d+)\.png)";
        }
        else
        {
            numericRegex = R"(image[_-](\d+)\.jpg)";
        }

        for (const auto &file : files)
        {
            if (file.is_regular_file() && file.path().extension() == img_ext)
            {
                std::string fileName = file.path().filename().string();
                // Extract the index using regex
                std::smatch match;
                if (std::regex_search(fileName, match, numericRegex) && match.size() > 1)
                {
                    int index = std::stoi(match[1]);
                    image_paths[index - 1] = file;
                }
            }
        }
        std::cout << "Found " << image_paths.size() << " images\n";

        // Get the size and index the elements
        for (int i = 0; i < image_paths.size(); i++)
        {
            auto entry = image_paths[i];
            if (entry.is_regular_file() && entry.path().extension() == img_ext)
            {
                std::cout << "Processing Image: " << entry.path() << std::endl;

                cv::Mat img = cv::imread(entry.path());

                cv::Mat silhouette;
                if (!use_masks)
                {
                    cv::cvtColor(img, silhouette, cv::COLOR_BGR2HSV);
                    cv::inRange(silhouette, cv::Scalar(0, 0, 30), cv::Scalar(255, 255, 255), silhouette); // binary image -black 0; while =1
                    std::string silhouette_path = replaceSubstring(entry.path(), "original_images", "segmented_images");
                    silhouette_path = replaceSubstring(silhouette_path, ".jpg", ".png");
                    if (!fileExists(silhouette_path))
                    {
                        cv::imwrite(silhouette_path, silhouette);
                    }
                }
                else
                {
                    cv::cvtColor(img, silhouette, cv::COLOR_BGR2GRAY);
                }

                cv::Mat P;
                cv::Mat K, R, t;

                K = cv::Mat::eye(3, 3, CV_32FC1);
                K.at<float>(0, 0) = fx;
                K.at<float>(1, 1) = fy;
                K.at<float>(0, 2) = cx;
                K.at<float>(1, 2) = cy;

                if (extension == "xml")
                {
                    std::stringstream smat;
                    smat << "viff" << std::setfill('0') << std::setw(3) << i << "_matrix";
                    viff_fs[smat.str()] >> P;
                    cv::decomposeProjectionMatrix(P, K, R, t);
                    // DecomposeProjectionMatrix instrinsics need to be overwritten?
                    // K.at<float>(0, 0) = fx;
                    // K.at<float>(1, 1) = fy;
                    // K.at<float>(0, 2) = cx;
                    // K.at<float>(1, 2) = cy;
                }
                else
                {
                    const cv::Mat &secondMatrix = matrices[i]; // Get the second matrix
                    cv::Mat T(3, 4, CV_64F);
                    T = secondMatrix(cv::Rect(0, 0, 4, 3)); // only take the first 3 row

                    cv::gemm(K, T, 1.0, cv::Mat(), 0.0, P);
                }

                camera c;
                c.Image = img;
                c.P = P;
                c.K = K;
                c.R = R;
                c.t = t;
                std::cout << P << std::endl << std::endl;
                c.Silhouette = silhouette;
                cameras.push_back(c);
            }
        }

        // Bounding Box for object
        float bbwidth = std::abs(xmax - xmin) * 1.15;
        float bbheight = std::abs(ymax - ymin) * 1.15;
        float bbdepth = std::abs(zmax - zmin) * 1.05;

        startParams params;
        params.startX = xmin - std::abs(xmax - xmin) * 0.15;
        params.startY = ymin - std::abs(ymax - ymin) * 0.15;
        params.startZ = 0.0f;
        params.voxelWidth = bbwidth / VOXEL_DIM;
        params.voxelHeight = bbheight / VOXEL_DIM;
        params.voxelDepth = bbdepth / VOXEL_DIM;

        // 3 dimensional voxel grid 
        float *fArray = new float[VOXEL_SIZE];
        std::fill_n(fArray, VOXEL_SIZE, 1000.0f);

        std::cout << "Carving Voxel Grid now...\n";
        // carving model for every given camera image 
        for (int i = 0; i < image_paths.size(); i++)
        {
            carve(fArray, params, cameras.at(i));
        }
        std::cout << "Voxel Carving Complete \n";

        // TODO: Voxel Colouring 

        // show example of segmented image 
        cv::Mat original, segmented;
        cv::resize(cameras.at(0).Image, original, cv::Size(640, 480));
        cv::resize(cameras.at(0).Silhouette, segmented, cv::Size(640, 480));
        cv::imshow("Sample Image", original);
        cv::imshow("Sample Silhouette", segmented);

        renderModel(fArray, params, obj_path);

        return 0;
        */
       return true;
}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"voxel_carver");
    ros::NodeHandle n("~");

    VoxelCarver voxelCarver(n);

    ROS_INFO("carver spinning");
    ros::spin();

    return 0;
}


