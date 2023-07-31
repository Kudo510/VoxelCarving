#ifndef COMMON_H
#define COMMON_H

// Header contents go here

#include <iostream>
#include <filesystem>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vtkSmartPointer.h>
#include <vtkStructuredPoints.h>
#include <vtkPointData.h>
#include <vtkPLYWriter.h>
#include <vtkFloatArray.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkMarchingCubes.h>
#include <vtkCleanPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkOBJExporter.h>

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkColorTransferFunction.h>
#include <vtkVolumeProperty.h>
#include <vtkSampleFunction.h>
#include <vtkPiecewiseFunction.h>
#include <vtkImageData.h>
#include <stdlib.h>
#include <numeric> // std::iota
// #include <boost/filesystem.hpp>

#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkPoints.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkDataSetMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

const int IMG_WIDTH = 1280;
const int IMG_HEIGHT = 720;
const int VOXEL_DIM = 128;
const int VOXEL_SIZE = VOXEL_DIM * VOXEL_DIM * VOXEL_DIM;
const int VOXEL_SLICE = VOXEL_DIM * VOXEL_DIM;
const int OUTSIDE = 0;

struct voxel
{
    float xpos;
    float ypos;
    float zpos;
    float res;
    float value;

    // Colour
    float red = -1;
    float green = -1;
    float blue = -1;
};

struct coord
{
    int x;
    int y;
};

struct startParams
{
    float startX;
    float startY;
    float startZ;
    float voxelWidth;
    float voxelHeight;
    float voxelDepth;
};

struct camera
{
    cv::Mat Image;
    cv::Mat P;
    cv::Mat K;
    cv::Mat R;
    cv::Mat t;
    cv::Mat Silhouette;
    std::string image_name;
};

coord project(camera cam, voxel v)
{

    coord im;

    /* project voxel into camera image coords */
    float z = cam.P.at<float>(2, 0) * v.xpos +
              cam.P.at<float>(2, 1) * v.ypos +
              cam.P.at<float>(2, 2) * v.zpos +
              cam.P.at<float>(2, 3);

    im.y = (cam.P.at<float>(1, 0) * v.xpos +
            cam.P.at<float>(1, 1) * v.ypos +
            cam.P.at<float>(1, 2) * v.zpos +
            cam.P.at<float>(1, 3)) /
           z;

    im.x = (cam.P.at<float>(0, 0) * v.xpos +
            cam.P.at<float>(0, 1) * v.ypos +
            cam.P.at<float>(0, 2) * v.zpos +
            cam.P.at<float>(0, 3)) /
           z;

    return im;
}

#endif // COMMON_H