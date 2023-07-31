// /*** UMAIR: **/

// void calculateColorStats(cv::Mat &image, std::vector<coord> &pixels, cv::Mat &colorMean, cv::Mat &colorStandardDeviation)
// {
//     cv::Mat pixelColors(pixels.size(), 1, CV_8UC3);

//     for (size_t i = 0; i < pixels.size(); i++)
//     {
//         cv::Vec3b pixelColor = image.at<cv::Vec3b>(pixels[i].y, pixels[i].x);
//         pixelColors.at<cv::Vec3b>(i, 0) = pixelColor;
//     }

//     cv::Scalar sum;
//     cv::accumulate(pixelColors, sum);

//     cv::Scalar mean;
//     cv::meanStdDev(pixelColors, colorMean, colorStandardDeviation);
// }

// void voxelColoring(std::vector<cv::Mat> &images, std::array<float, VOXEL_SIZE> &voxels, std::vector<cv::Mat> &occlusionBitmaps, std::vector<camera> &cameras, std::vector<startParams> &params, double threshold)
// {
//     for (const auto &image : images)
//     {
//         occlusionBitmaps.push_back(cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, cv::Scalar(0)));
//     }

//     for (int layer = 0; layer < VOXEL_DIM; layer++)
//     {
//         for (int i = layer * VOXEL_SLICE; i < (layer + 1) * VOXEL_SLICE; i++)
//         {
//             int voxelIndex = layer * VOXEL_SLICE + i;

//             voxel &V = voxels[voxelIndex];

//             for (int j = 0; j < images.size(); j++)
//             {
//                 camera &cam = cameras[j];
//                 startParams &param = params[j];

//                 std::vector<coord> pixels;

//                 // Find the set P(i) of pixels
//                 coord im = projectVoxel(cam, V);

//                 // Determine the pixels in P(i)
//                 int xStart = std::max(0, static_cast<int>(im.x - param.startX) / param.voxelWidth);
//                 int yStart = std::max(0, static_cast<int>(im.y - param.startY) / param.voxelHeight);
//                 int xEnd = std::min(IMG_WIDTH - 1, static_cast<int>(im.x - param.startX + param.voxelWidth) / param.voxelWidth);
//                 int yEnd = std::min(IMG_HEIGHT - 1, static_cast<int>(im.y - param.startY + param.voxelHeight) / param.voxelHeight);

//                 for (int x = xStart; x <= xEnd; x++)
//                 {
//                     for (int y = yStart; y <= yEnd; y++)
//                     {
//                         pixels.push_back(coord{x, y});
//                     }
//                 }

//                 cv::Scalar colorMean;
//                 double colorStandardDeviation;
//                 calculateColorStats(cam.Image, pixels, colorMean, colorStandardDeviation);

//                 if (colorStandardDeviation < threshold)
//                 {
//                     voxels[voxelIndex] = colorMean[0];
//                     for (int j = 0; j < images.size(); j++)
//                     {
//                         occlusionBitmaps[j].setTo(255, cam.Silhouette);
//                     }
//                 }
//                 else
//                 {
//                     voxels[voxelIndex] = OUTSIDE;
//                 }
//             }
//         }
//     }
// }

// /*********/