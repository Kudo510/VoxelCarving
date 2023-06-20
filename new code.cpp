#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <fstream>

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

const int IMG_WIDTH = 1280;
const int IMG_HEIGHT = 960;
const int VOXEL_DIM = 128;
const int VOXEL_SIZE = VOXEL_DIM * VOXEL_DIM * VOXEL_DIM;
const int VOXEL_SLICE = VOXEL_DIM * VOXEL_DIM;
const int OUTSIDE = 0;

struct voxel {
    float xpos;
    float ypos;
    float zpos;
    float res;
    float value;
};

struct coord {
    int x;
    int y;
};

struct startParams {
    float startX;
    float startY;
    float startZ;
    float voxelWidth;
    float voxelHeight;
    float voxelDepth;
};

struct camera {
    cv::Mat Image;
    cv::Mat P;
    cv::Mat K;
    cv::Mat R;
    cv::Mat t;
    cv::Mat Silhouette;
};

void exportModel(char* filename, vtkPolyData* polyData) {

    /* exports 3d model in ply format */
    vtkSmartPointer<vtkPLYWriter> plyExporter = vtkSmartPointer<vtkPLYWriter>::New();
    plyExporter->SetFileName(filename);
    plyExporter->SetInputData(polyData);
    plyExporter->Update();
    plyExporter->Write();
}


coord project(camera cam, voxel v) {

    coord im;

    /* project voxel into camera image coords */
    float z = cam.P.at<float>(2, 0) * v.xpos +
        cam.P.at<float>(2, 1) * v.ypos +
        cam.P.at<float>(2, 2) * v.zpos +
        cam.P.at<float>(2, 3);

    im.y = (cam.P.at<float>(1, 0) * v.xpos +
        cam.P.at<float>(1, 1) * v.ypos +
        cam.P.at<float>(1, 2) * v.zpos +
        cam.P.at<float>(1, 3)) / z;

    im.x = (cam.P.at<float>(0, 0) * v.xpos +
        cam.P.at<float>(0, 1) * v.ypos +
        cam.P.at<float>(0, 2) * v.zpos +
        cam.P.at<float>(0, 3)) / z;

    return im;
}

void renderModel(float fArray[], startParams params) {
    /* Create vtk visualization pipeline from voxel grid (float array) */
    vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
    imageData->SetDimensions(VOXEL_DIM, VOXEL_DIM, VOXEL_DIM);
    imageData->SetSpacing(params.voxelDepth, params.voxelHeight, params.voxelWidth);
    imageData->SetOrigin(params.startZ, params.startY, params.startX);

    vtkSmartPointer<vtkFloatArray> scalarArray = vtkSmartPointer<vtkFloatArray>::New();
    scalarArray->SetNumberOfComponents(1);
    scalarArray->SetArray(fArray, VOXEL_SIZE, 1);

    imageData->GetPointData()->SetScalars(scalarArray);

    /* Create iso-surface with marching cubes algorithm */
    vtkSmartPointer<vtkMarchingCubes> mcSource = vtkSmartPointer<vtkMarchingCubes>::New();
    mcSource->SetInputData(imageData);
    mcSource->ComputeNormalsOn();
    mcSource->SetValue(0, 0.5); // isosurface value (0.5 for this example)

    /* Recreate mesh topology and merge vertices */
    vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
    cleanPolyData->SetInputConnection(mcSource->GetOutputPort());

    /* Usual render stuff */
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(0.45, 0.45, 0.9);
    renderer->SetBackground2(0.0, 0.0, 0.0);
    renderer->GradientBackgroundOn();

    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cleanPolyData->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    /* Visible light properties */
    actor->GetProperty()->SetSpecular(0.15);
    actor->GetProperty()->SetInterpolationToPhong();
    renderer->AddActor(actor);

    renderWindow->Render();
    interactor->Start();
    // Export the rendered model as an OBJ file
    vtkSmartPointer<vtkOBJExporter> exporter = vtkSmartPointer<vtkOBJExporter>::New();
    exporter->SetInput(renderWindow);
    exporter->SetFilePrefix("F:\\second semseter\\3D Scanning & Motion Capture\\Final project\\Voxel-Carving\\");  // Specify the desired file path and prefix
    exporter->Write();
}


void carve(float fArray[], startParams params, camera cam) {
    cv::Mat silhouette, distImage;
    cv::Canny(cam.Silhouette, silhouette, 0, 255);
    cv::bitwise_not(silhouette, silhouette);
    cv::distanceTransform(silhouette, distImage, cv::DIST_L2, 3);

    for (int i = 0; i < VOXEL_DIM; i++) {
        for (int j = 0; j < VOXEL_DIM; j++) {
            for (int k = 0; k < VOXEL_DIM; k++) {

                /* calc voxel position inside camera view frustum */
                voxel v;
                v.xpos = params.startX + i * params.voxelWidth;
                v.ypos = params.startY + j * params.voxelHeight;
                v.zpos = params.startZ + k * params.voxelDepth;
                v.value = 1.0f;

                coord im = project(cam, v);
                float dist = -1.0f;

                /* test if projected voxel is within image coords */
                if (im.x > 0 && im.y > 0 && im.x < IMG_WIDTH && im.y < IMG_HEIGHT) {
                    dist = distImage.at<float>(im.y, im.x);
                    if (cam.Silhouette.at<uchar>(im.y, im.x) == OUTSIDE) {
                        dist *= -1.0f;
                    }
                }

                if (dist < fArray[i * VOXEL_SLICE + j * VOXEL_DIM + k]) {
                    fArray[i * VOXEL_SLICE + j * VOXEL_DIM + k] = dist;
                }

            }
        }
    }
}

cv::Mat getRowFromColumnB(const std::string& filename, int row) {
    std::ifstream file(filename);
    std::string line, value;

    // Skip the rows until reaching the desired row
    for (int i = 0; i < row; ++i) {
        std::getline(file, line);
    }

    // Read the desired row and extract value from column B
    std::getline(file, line);
    std::stringstream ss(line);
    for (int i = 0; i < 1; ++i) {
        std::getline(ss, value, ',');
    }

    // Create a cv::Mat object from the value
    cv::Mat rowMat(1, 1, CV_64F);
    rowMat.at<double>(0, 0) = std::stod(value);

    return rowMat;
}
cv::Mat extractRows(const cv::Mat& matrix, int numRows) {
    cv::Rect roi(0, 0, matrix.cols, numRows);
    cv::Mat extractedRows = matrix(roi).clone();

    return extractedRows;
}
/*
cv::Mat getCellValue(const std::string& filename, int row, int col) {
    std::ifstream file(filename);
    std::string line, value;

    // Skip the rows until reaching the desired row
    for (int i = 0; i < row; ++i) {
        std::getline(file, line);
    }

    // Read the desired row and extract the value from the desired column
    std::getline(file, line);
    std::stringstream ss(line);
    for (int i = 0; i < col; ++i) {
        std::getline(ss, value, ',');
    }

    // Create a cv::Mat object from the value
    cv::Mat cellMat(1, 1, CV_64F);
    cellMat.at<double>(0, 0) = std::stod(value);

    return cellMat;
}*/

cv::Mat getCellValue(const std::string& filename, int row) {
    std::ifstream file(filename);
    std::string line;

    // Skip the rows until reaching the desired row
    for (int i = 0; i < 20; ++i) {
        std::getline(file, line);
    }
    std::cout << "Value in line" << line;

    // Read the desired row and extract the value from the second column (column B)
    std::getline(file, line);
    std::stringstream ss(line);
    std::string value;
    std::getline(ss, value, '\t'); // Skip the first column (column A)
    std::getline(ss, value, '\t'); // Read the value in the second column (column B)

    // Remove any leading/trailing spaces or brackets from the value
    value.erase(std::remove(value.begin(), value.end(), ' '), value.end());
    value.erase(std::remove(value.begin(), value.end(), '['), value.end());
    value.erase(std::remove(value.begin(), value.end(), ']'), value.end());

    // Split the value into individual numbers
    std::stringstream num_ss(value);
    std::vector<double> nums;
    double num;
    while (num_ss >> num) {
        nums.push_back(num);
    }

    // Create a cv::Mat object from the numbers
    cv::Mat cellMat(1, nums.size(), CV_64F);
    for (int i = 0; i < nums.size(); ++i) {
        cellMat.at<double>(0, i) = nums[i];
    }

    return cellMat;
}

int main(int argc, char* argv[]) {

    /* acquire camera images, silhouettes and camera matrix */
    std::vector<camera> cameras;
    cv::FileStorage fs("F:\\second semseter\\3D Scanning & Motion Capture\\Final project\\Voxel-Carving\\assets\\viff.xml", cv::FileStorage::READ);
    //std::string filepath = "F:\\second semseter\\3D Scanning & Motion Capture\\Final project\\VoxelCarvingWithRoboticArm\\data\\red_block.csv"; // Replace with the actual file path
    //
    //int targetRow = 1;
    //int targetCol = 3;

    //cv::Mat cellValue = getCellValue(filepath, targetRow);
    //std::cout << "Value in block B2 (cv::Mat):\n" << cellValue << std::endl;



    for (int i = 0; i < 36; i++) {

        /* camera image */
        std::stringstream simg;
        simg << "F:\\second semseter\\3D Scanning & Motion Capture\\Final project\\Voxel-Carving\\assets\\image_" << i << ".jpg";
        //simg << "F:\\second semseter\\3D Scanning & Motion Capture\\Final project\\Voxel-Carving\\assets\\image_" << i << ".jpg";
        cv::Mat img = cv::imread(simg.str());

        /* silhouette */
        cv::Mat silhouette;
        cv::cvtColor(img, silhouette, cv::COLOR_BGR2HSV);
        cv::inRange(silhouette, cv::Scalar(0, 0, 30), cv::Scalar(255, 255, 255), silhouette);
        //std::cout << "Silhouette" << silhouette.rows << silhouette.cols;
        /* camera matrix */
        std::stringstream smat;
        smat << "viff" << std::setfill('0') << std::setw(3) << i << "_matrix";
        cv::Mat P;
        fs[smat.str()] >> P;
        //std::cout << "Image shape: " << P.rows << " rows x " << P.cols << " columns" << std::endl;

        /* decompose proj matrix to cam- and rot matrix and trans vect */
        cv::Mat K, R, t;
        cv::decomposeProjectionMatrix(P, K, R, t);
        K = cv::Mat::eye(3, 3, CV_32FC1);
        //K.at<float>(0, 0) = 920.88464355; /* fx */
        //K.at<float>(1, 1) = 924.34155273; /* fy */
        //K.at<float>(0, 2) = 613.90703152; /* cx */
        //K.at<float>(1, 2) = 389.5955547; /* cy */
        K.at<float>(0, 0) = 1680.2631413061415; /* fx */
        K.at<float>(1, 1) = 1676.1202869984309; /* fy */
        K.at<float>(0, 2) = 621.59194200994375; /* cx */
        K.at<float>(1, 2) = 467.7223229477861; /* cy */
        camera c;
        c.Image = img;
        c.P = P;
        c.K = K;
        c.R = R;
        c.t = t;
        c.Silhouette = silhouette;

        cameras.push_back(c);
    }

    /* bounding box dimensions of squirrel */
    float xmin = -6.21639, ymin = -10.2796, zmin = -14.0349;
    float xmax = 7.62138, ymax = 12.1731, zmax = 12.5358;

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

    /* 3 dimensional voxel grid */
    float* fArray = new float[VOXEL_SIZE];
    std::fill_n(fArray, VOXEL_SIZE, 1000.0f);

    /* carving model for every given camera image */
    for (int i = 0; i < 36; i++) {
        carve(fArray, params, cameras.at(i));
    }

    /* show example of segmented image */
    cv::Mat original, segmented;
    cv::resize(cameras.at(1).Image, original, cv::Size(640, 480));
    cv::resize(cameras.at(1).Silhouette, segmented, cv::Size(640, 480));
    cv::imshow("Squirrel", original);
    cv::imshow("Squirrel Silhouette", segmented);

    renderModel(fArray, params);

    return 0;
}
