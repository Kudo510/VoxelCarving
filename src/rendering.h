#include "common.h"
#include <vtkCubeSource.h>

void renderModel(float fArray[], startParams params, std::string obj_path, std::vector<voxel> &voxels)
{
    // Create vtk visualization pipeline from voxel grid (float array)
    vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
    imageData->SetDimensions(VOXEL_DIM, VOXEL_DIM, VOXEL_DIM);
    imageData->SetSpacing(params.voxelDepth, params.voxelHeight, params.voxelWidth);
    imageData->SetOrigin(params.startZ, params.startY, params.startX);

    vtkSmartPointer<vtkFloatArray> scalarArray = vtkSmartPointer<vtkFloatArray>::New();
    scalarArray->SetNumberOfComponents(1);
    scalarArray->SetArray(fArray, VOXEL_SIZE, 1);

    imageData->GetPointData()->SetScalars(scalarArray);

    // Create iso-surface with marching cubes algorithm
    vtkSmartPointer<vtkMarchingCubes> mcSource = vtkSmartPointer<vtkMarchingCubes>::New();
    mcSource->SetInputData(imageData);
    mcSource->ComputeNormalsOn();
    mcSource->SetValue(0, 0.5); // isosurface value (0.5 for this example)

    // Recreate mesh topology and merge vertices
    vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
    cleanPolyData->SetInputConnection(mcSource->GetOutputPort());

    // Set the color values for the object voxels
    vtkSmartPointer<vtkUnsignedCharArray> pointColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    pointColors->SetNumberOfComponents(3);
    pointColors->SetNumberOfTuples(cleanPolyData->GetOutput()->GetNumberOfPoints());

    for (vtkIdType i = 0; i < cleanPolyData->GetOutput()->GetNumberOfPoints(); ++i)
    {
        voxel &currentVoxel = voxels[i];

        unsigned char color[3] = {static_cast<unsigned char>(currentVoxel.red),
                                  static_cast<unsigned char>(currentVoxel.green),
                                  static_cast<unsigned char>(currentVoxel.blue)};

        pointColors->SetTuple3(i, color[0], color[1], color[2]);
    }

    cleanPolyData->GetOutput()->GetPointData()->SetScalars(pointColors);

    // Usual render stuff
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

    // Visible light properties
    actor->GetProperty()->SetSpecular(0.15);
    actor->GetProperty()->SetInterpolationToPhong();

    renderer->AddActor(actor);

    renderWindow->Render();
    interactor->Start();

    // Export the rendered model as an OBJ file
    vtkSmartPointer<vtkOBJExporter> exporter = vtkSmartPointer<vtkOBJExporter>::New();
    exporter->SetInput(renderWindow);
    exporter->SetFilePrefix(obj_path.c_str()); // Specify the desired file path and prefix
    exporter->Write();
}
