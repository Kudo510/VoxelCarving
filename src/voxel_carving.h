#include "common.h"

void carve(float fArray[], startParams params, camera cam, std::vector<voxel> &voxels)
{
    cv::Mat silhouette, distImage;
    cv::Canny(cam.Silhouette, silhouette, 0, 255);
    cv::bitwise_not(silhouette, silhouette);
    // cv::imshow("hello", silhouette);
    // cv::waitKey();
    cv::distanceTransform(silhouette, distImage, cv::DIST_L2, 3);

    // cv::Mat projectedImage;           // Create an image to store the projected voxel
    // cam.Image.copyTo(projectedImage); // Initialize the projected image with the segmentation mask

    for (int i = 0; i < VOXEL_DIM; i++)
    {
        for (int j = 0; j < VOXEL_DIM; j++)
        {
            for (int k = 0; k < VOXEL_DIM; k++)
            {

                /* calc voxel position inside camera view frustum */
                voxel v;
                v.xpos = params.startX + i * params.voxelWidth;
                v.ypos = params.startY + j * params.voxelHeight;
                v.zpos = params.startZ + k * params.voxelDepth;
                v.value = 1.0f;
                float dist = -1.0f;

                // coord im = project(cam, v);

                cv::Mat voxel_coord(4, 1, CV_32F);
                voxel_coord.at<float>(0) = v.xpos;
                voxel_coord.at<float>(1) = v.ypos;
                voxel_coord.at<float>(2) = v.zpos;
                voxel_coord.at<float>(3) = 1.0f;

                cv::Mat pixel_coord;
                cv::gemm(cam.P, voxel_coord, 1.0, cv::Mat(), 0.0, pixel_coord);
                // cv::Mat pixel_coord = cam.P * voxel_coord;

                coord im;
                im.x = pixel_coord.at<float>(0) / pixel_coord.at<float>(2);
                im.y = pixel_coord.at<float>(1) / pixel_coord.at<float>(2);

                /* test if projected voxel is within image coords */
                if (im.x > 0 && im.y > 0 && im.x < IMG_WIDTH && im.y < IMG_HEIGHT)
                {
                    dist = distImage.at<float>(im.y, im.x);
                    if ((int)cam.Silhouette.at<uchar>(im.y, im.x) == OUTSIDE)
                    {
                        dist *= -1.0f;
                    }
                    else
                    {
                        // int px = static_cast<int>(im.x);
                        // int py = static_cast<int>(im.y);
                        // projectedImage.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 0, 255); // Set color as red (BGR)
                    }
                }
                if (dist < fArray[i * VOXEL_SLICE + j * VOXEL_DIM + k])
                {
                    fArray[i * VOXEL_SLICE + j * VOXEL_DIM + k] = dist;
                    v.value = dist;
                }
                voxels.push_back(v);
            }
        }
    }
    // Save the final image after projecting all voxels to disk
    // cv::imwrite("../assets/projected_voxels/" + cam.image_name, projectedImage);
}