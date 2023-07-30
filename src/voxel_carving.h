#include "common.h"

void carve(float fArray[], startParams params, camera cam)
{

    cv::Mat silhouette, distImage;
    cv::Canny(cam.Silhouette, silhouette, 0, 255);
    cv::bitwise_not(silhouette, silhouette);
    cv::imshow("hello",silhouette);
    cv::waitKey();
    cv::distanceTransform(silhouette, distImage, cv::DIST_L2, 3);

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

                cv::Mat pixel_coord = cam.P * voxel_coord;
                coord im;
                im.x = pixel_coord.at<float>(0) / pixel_coord.at<float>(2);
                im.y = pixel_coord.at<float>(1) / pixel_coord.at<float>(2);

                /* test if projected voxel is within image coords */
                if (im.x > 0 && im.y > 0 && im.x < IMG_WIDTH && im.y < IMG_HEIGHT)
                {
                    dist = distImage.at<float>(im.y, im.x);
                    if (cam.Silhouette.at<uchar>(im.y, im.x) == OUTSIDE)
                    {
                        dist *= -1.0f;
                    }
                }
                else{
                    //std::cout << "problem";
                }

                if (dist < fArray[i * VOXEL_SLICE + j * VOXEL_DIM + k])
                {
                    fArray[i * VOXEL_SLICE + j * VOXEL_DIM + k] = dist;
                }
            }
        }
    }
}