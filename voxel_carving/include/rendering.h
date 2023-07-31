#include "common.h"
#include <vtkCubeSource.h>

void renderModel(float fArray[], startParams params, std::string obj_path)
{
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

    /* Create a 3D bounding box */
    vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
    cubeSource->SetBounds(params.startX, params.startX + params.voxelWidth * VOXEL_DIM,
                          params.startY, params.startY + params.voxelHeight * VOXEL_DIM,
                          params.startZ, params.startZ + params.voxelDepth * VOXEL_DIM);

    vtkSmartPointer<vtkPolyDataMapper> outlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    outlineMapper->SetInputConnection(cubeSource->GetOutputPort());

    vtkSmartPointer<vtkActor> outlineActor = vtkSmartPointer<vtkActor>::New();
    outlineActor->SetMapper(outlineMapper);
    outlineActor->GetProperty()->SetColor(1.0, 1.0, 1.0);        // Set color to white
    outlineActor->GetProperty()->SetRepresentationToWireframe(); // Render the bounding box as wireframe

    renderer->AddActor(outlineActor);

    /* Visible light properties */
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