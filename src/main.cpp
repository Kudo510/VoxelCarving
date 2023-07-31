#include "common.h"
#include "utils.h"
#include "rendering.h"
#include "voxel_carving.h"
#include "voxel_colouring.h"
#include <json/json.h>
#include <cassert>
#include <regex>
#include <random>

std::vector<voxel> VOXELS;

int main(int argc, char *argv[])
{

    std::string config_path = "";
    if (argc >= 2)
    {
        config_path = argv[1];
    }
    else
    {
        // No base path provided, handle the error
        std::cerr << "Error: Config path not provided." << std::endl;
        return 1; // Exit with a non-zero status code indicating an error
    }

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

    /* Intrinsics */
    float fx = config["fx"].asFloat();
    float fy = config["fy"].asFloat();
    float cx = config["cx"].asFloat();
    float cy = config["cy"].asFloat();

    /* Voxel Grid */
    float xmin = config["xmin"].asFloat();
    float xmax = config["xmax"].asFloat();
    float ymin = config["ymin"].asFloat();
    float ymax = config["ymax"].asFloat();
    float zmin = config["zmin"].asFloat();
    float zmax = config["zmax"].asFloat();

    const Json::Value &skip_frames_array = config["skip_frames"];
    std::vector<int> skip_frames;
    // Iterate over the array and extract integer values
    for (unsigned int i = 0; i < skip_frames_array.size(); ++i)
    {
        if (skip_frames_array[i].isInt())
        {
            skip_frames.push_back(skip_frames_array[i].asInt());
        }
    }

    std::vector<camera> cameras;
    std::string extension = getFileExtension(matrices_path);

    cv::FileStorage viff_fs;
    std::vector<cv::Mat> matrices;

    if (extension == "xml")
    {
        // viff file and squirrel dataset
        viff_fs = cv::FileStorage(matrices_path, cv::FileStorage::READ);
    }
    else
    {
        matrices = readMatricesFromFile(matrices_path);
    }

    std::string img_ext = ".jpg";
    auto files = std::filesystem::directory_iterator(image_folder);
    std::map<int, std::filesystem::directory_entry> image_paths;
    // Extract and print the indices from the sorted directory entries
    std::regex numericRegex;
    numericRegex = R"(image[_-]*(\d+)\.jpg)";

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
                if (std::find(skip_frames.begin(), skip_frames.end(), index) == skip_frames.end())
                    image_paths[index - 1] = file;
            }
        }
    }
    std::cout << "Found " << image_paths.size() << " images after filtering skipped frames\n";

    // Get the size and index the elements
    for (auto kv : image_paths)
    {
        int i = kv.first;
        // if (std::find(skip_frames.begin(), skip_frames.end(), i - 1) != skip_frames.end())
        // continue;
        auto entry = image_paths[i];
        if (entry.is_regular_file() && entry.path().extension() == img_ext)
        {
            std::cout << "Processing Image: " << entry.path() << std::endl;

            cv::Mat img = cv::imread(entry.path());

            cv::Mat silhouette;
            std::string silhouette_path = replaceSubstring(entry.path(), "original_images", "segmented_images");
            silhouette_path = replaceSubstring(silhouette_path, ".jpg", ".png");

            if (use_masks)
            {
                silhouette = cv::imread(silhouette_path);
                cv::cvtColor(silhouette, silhouette, cv::COLOR_BGR2GRAY); // Convert to grayscale // Apply thresholding  -need to change here our gaonl is to do the same as the template- siluhette need to be a black and white
            }
            else
            {
                cv::cvtColor(img, silhouette, cv::COLOR_BGR2HSV);
                cv::inRange(silhouette, cv::Scalar(0, 0, 30), cv::Scalar(255, 255, 255), silhouette); // binary image -black 0; while =1
                if (!fileExists(silhouette_path))
                {
                    cv::imwrite(silhouette_path, silhouette);
                }
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
                K.at<float>(0, 0) = fx;
                K.at<float>(1, 1) = fy;
                K.at<float>(0, 2) = cx;
                K.at<float>(1, 2) = cy;
            }
            else
            {
                cv::Mat secondMatrix = matrices[i]; // Get the second matrix
                cv::Mat T(3, 4, CV_64F);
                cv::invert(secondMatrix, T);
                T = T(cv::Rect(0, 0, 4, 3)); // only take the first 3 row

                cv::gemm(K, T, 1.0, cv::Mat(), 0.0, P);
            }

            camera c;
            c.Image = img;
            c.P = P;
            c.K = K;
            c.R = R;
            c.t = t;

            c.Silhouette = silhouette;
            c.image_name = entry.path().filename().string();
            cameras.push_back(c);
        }
    }

    // Bounding Box for object
    float bbwidth = std::abs(xmax - xmin) * 1.15;
    float bbheight = std::abs(ymax - ymin) * 1.15;
    float bbdepth = std::abs(zmax - zmin) * 1.05;

    startParams params;
    // params.startX = xmin - std::abs(xmax - xmin) * 0.15;
    // params.startY = ymin - std::abs(ymax - ymin) * 0.15;
    // params.startZ = zmin - std::abs(zmax - zmin) * 0.15;

    // params.voxelWidth = bbwidth / VOXEL_DIM;
    // params.voxelHeight = bbheight / VOXEL_DIM;
    // params.voxelDepth = bbdepth / VOXEL_DIM;

    // Override 3D bbox
    params.startX = xmin;
    params.startY = ymin;
    params.startZ = zmin;

    params.voxelWidth = 100.0 / VOXEL_DIM;
    params.voxelHeight = 100.0 / VOXEL_DIM;
    params.voxelDepth = 100.0 / VOXEL_DIM;

    /* 3 dimensional voxel grid */
    float *fArray = new float[VOXEL_SIZE];
    unsigned char *colourData = new unsigned char[VOXEL_SIZE * 3];
    std::fill_n(fArray, VOXEL_SIZE, 1000.0f);

    std::cout << "Carving Voxel Grid now...\n";
    /* carving model for every given camera image */
    for (int i = 0; i < image_paths.size(); i++)
    {
        carve(fArray, params, cameras.at(i), VOXELS);
    }
    std::cout << "Voxel Carving Complete \n";

    /** TODO: Voxel Colouring **/
    // std::cout << "Colouring Voxel Grid...\n";
    // colourVoxels(cameras, VOXELS);
    // std::cout << "Colouring completed \n";

    /** VTK **/
    // renderColoredVoxels(VOXELS);
    // renderColoredVoxels(VOXELS, params);
    /*****/

    /* show example of segmented image */
    cv::Mat original,
        segmented;
    cv::resize(cameras.at(0).Image, original, cv::Size(640, 480));
    cv::resize(cameras.at(0).Silhouette, segmented, cv::Size(640, 480));
    cv::imshow("Sample Image", original);
    cv::imshow("Sample Silhouette", segmented);

    renderModel(fArray, params, obj_path, VOXELS);

    return 0;
}
