## Voxel Carving with Robotic Arm

This repository contains an implementation of 3D reconstruction via Voxel Carving with the help of a Robotic Arm to capture images of an object.

### Environment Setup

This codebase requires the following libraries and packages to be installed:

- C++ Compiler (supporting C++11 or later)
- OpenCV (version 2 or later)
- JSON (<https://github.com/nlohmann/json>)
- VTK (Visualization Toolkit)
- Python3
- ROS (Robot Operating System)
  
### Requirements

This codebase requires a dataset of the target object in a specific format, as well as a configuration file for necessary parameters. The dataset must be placed in the following format:

- assets/
  - <dataset_name>/
    - original_images/
      - image_1.jpg
      - ...
    - segmented_images/
      - image_1.png
      - ...
    - pose.txt

`pose.txt` contains the camera poses for each frame. Each row is a 16x1 row vector representing the 4x4 Transformation matrix.

#### Configuration File

This section explains each parameter present in the configuration file used for voxel carving:

1. **obj_path**: The path to the directory containing the 3D model of the object to be voxel carved. This model is usually represented as an OBJ file format.

2. **matrices_path**: The path to the file containing camera poses or matrices. These camera poses represent the positions and orientations of the camera in the scene during image capture. Each row in the file corresponds to a different camera pose.

3. **image_folder**: The path to the directory containing the original images of the scene. These images are typically RGB images captured from the real world.

4. **silhouette_folder**: The path to the directory containing the segmented images or silhouettes of the object in the scene. These images represent binary masks where the object pixels are set to 1 (white) and the background pixels are set to 0 (black).

5. **skip_frames**: A list of frame numbers (or indices) that need to be skipped during the voxel carving process. These frames might contain invalid data or occlusions that could negatively impact the final result.

6. **use_masks**: A boolean parameter indicating whether to use the segmented images/masks (silhouettes) to assist in the voxel carving process. When set to true, the algorithm takes the segmentation information into account while reconstructing the object. Otherwise, a default OpenCV method will be applied for segmentation

7. **fx**: The focal length of the camera along the x-axis. This parameter is used in the camera projection model to transform 3D points to 2D image coordinates.

8. **fy**: The focal length of the camera along the y-axis. Similar to 'fx', this parameter is also used in the camera projection model.

9. **cx**: The x-coordinate of the principal point of the camera. The principal point is the point where the optical axis intersects the image plane.

10. **cy**: The y-coordinate of the principal point of the camera. Similar to 'cx', this parameter also represents the principal point's position on the image plane.

11. **xmin**: The minimum value for the x-coordinate of the voxel carving bounding box. This defines the lower limit of the volume to be carved in the x-axis.

12. **xmax**: The maximum value for the x-coordinate of the voxel carving bounding box. This defines the upper limit of the volume to be carved in the x-axis.

13. **ymin**: The minimum value for the y-coordinate of the voxel carving bounding box. This defines the lower limit of the volume to be carved in the y-axis.

14. **ymax**: The maximum value for the y-coordinate of the voxel carving bounding box. This defines the upper limit of the volume to be carved in the y-axis.

15. **zmin**: The minimum value for the z-coordinate of the voxel carving bounding box. This defines the lower limit of the volume to be carved in the z-axis.

16. **zmax**: The maximum value for the z-coordinate of the voxel carving bounding box. This defines the upper limit of the volume to be carved in the z-axis.

Make sure to adjust these parameters according to your specific dataset and requirements to achieve the desired voxel carving results.  
Compiling and Running the Code

### Running the program

Once you have installed the required libraries, you can compile and run the codebase using your preferred build system or IDE.

```

mkdir build && cd build

cmake -DCOMPILE_ARUCO=OFF ..

make -j$(nproc)

./VoxelCarving ../red_block.json

```

### Result


https://github.com/Kudo510/VoxelCarving/assets/68633914/ab455a0b-42e3-4244-b9bd-26bd41ee582c




