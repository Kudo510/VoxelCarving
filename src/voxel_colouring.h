// /*** UMAIR: **/
#include "common.h"
#include <random>
#include <GL/glut.h> // Make sure to include the appropriate OpenGL headers

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkGlyph3D.h>
#include <vtkSphereSource.h>
#include <vtkNamedColors.h>
#include <vtkPolyDataMapper.h>

void colourVoxels(std::vector<camera> &cameras, std::vector<voxel> &voxels)
{
    for (auto &voxel : voxels)
    {
        float runningSumR = 0.0;
        float runningSumG = 0.0;
        float runningSumB = 0.0;
        int numContributingCameras = 0;

        for (auto &camera : cameras)
        {
            cv::Mat voxel_coord(4, 1, CV_32F);
            voxel_coord.at<float>(0) = voxel.xpos;
            voxel_coord.at<float>(1) = voxel.ypos;
            voxel_coord.at<float>(2) = voxel.zpos;
            voxel_coord.at<float>(3) = 1.0f;

            cv::Mat pixel_coord;
            cv::gemm(camera.P, voxel_coord, 1.0, cv::Mat(), 0.0, pixel_coord);
            // cv::Mat pixel_coord = cam.P * voxel_coord;

            coord im;
            im.x = pixel_coord.at<float>(0) / pixel_coord.at<float>(2);
            im.y = pixel_coord.at<float>(1) / pixel_coord.at<float>(2);

            if (im.x >= 0 && im.x < IMG_WIDTH &&
                im.y >= 0 && im.y < IMG_HEIGHT &&
                camera.Silhouette.at<uchar>(im.y, im.x) != 0 && voxel.value <= 0)
            {
                cv::Vec3b pixelColor = camera.Image.at<cv::Vec3b>(im.y, im.x);
                runningSumB += pixelColor[0];
                runningSumG += pixelColor[1];
                runningSumR += pixelColor[2];
                numContributingCameras++;
            }
        }

        if (numContributingCameras > 0)
        {
            voxel.red = runningSumR / numContributingCameras;
            voxel.green = runningSumG / numContributingCameras;
            voxel.blue = runningSumB / numContributingCameras;
            // std::cout << "Voxel (" << voxel.xpos << ", " << voxel.ypos << ", " << voxel.zpos << ") has colour " << voxel.red << " " << voxel.green << " " << voxel.blue << "\n";
        }
    }
}

// Function to render the colored voxel object using VTK
void renderColoredVoxels(std::vector<voxel> &voxels)
{
    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();

    // Create VTK objects
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colorsArray = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colorsArray->SetNumberOfComponents(3);
    vtkSmartPointer<vtkCellArray> voxelsCells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

    // Populate points and colors arrays
    for (auto &voxel : voxels)
    {
        points->InsertNextPoint(voxel.xpos, voxel.ypos, voxel.zpos);
        unsigned char color[3] = {static_cast<unsigned char>(voxel.red),
                                  static_cast<unsigned char>(voxel.green),
                                  static_cast<unsigned char>(voxel.blue)};
        colorsArray->InsertNextTypedTuple(color);
    }

    // Populate voxel cells
    for (vtkIdType i = 0; i < voxels.size(); ++i)
    {
        vtkIdType voxelIndices[1] = {i};
        voxelsCells->InsertNextCell(1, voxelIndices);
    }

    // Connect the points and colors to the polyData
    polyData->SetPoints(points);
    polyData->GetPointData()->SetScalars(colorsArray);
    polyData->SetVerts(voxelsCells);

    // Create a mapper
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    // Create an actor
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    // Create a renderer and add the actor
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);
    renderer->SetBackground(colors->GetColor3d("White").GetData());

    // Create a render window
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    // Create an interactor
    vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

    // Start the visualization
    renderWindow->Render();
    interactor->Start();
}
