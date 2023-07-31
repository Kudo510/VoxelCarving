import open3d as o3d
import rospkg
import csv
import numpy as np
import ast
from scipy.spatial.transform import Rotation as R



class CameraPoseShower:

    def __init__(self) -> None:

        self.poses = []
        self.width = 1280
        self.height = 720
        self.cx = 629.14694662
        self.cy = 314.33765115
        self.fx = 923.65667725
        self.fy = 919.3928833
        self.tx = 0.0
        self.ty = 0.0
        self.tz = 0.0
        """
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('voxel_carving')+"/../data/speriamo.txt"
        with open(file_path, mode='r') as file:
            for line in file:
                string_list = line.rstrip('\n')
                list  = ast.literal_eval(string_list)
                pose = np.array(list)
                pose = np.reshape(pose, (4,4))
                self.poses.append(pose)
        """

    def get_camera_geometry(self,pose):
        # Compute the camera frustum points in camera coordinate system
        frustum_points =   10 * np.array([[0.0, 0.0, 0.0],  # Camera center
                                [-1.0, -1.0, 1.0],  # Top-left corner
                                [1.0, -1.0, 1.0],  # Top-right corner
                                [1.0, 1.0, 1.0],  # Bottom-right corner
                                [-1.0, 1.0, 1.0], # Bottom-left corner
                                [ 0.0 , 0.0 , 10 ]
                                ])  # center projected
        
    
        center_world = np.dot(pose[:3, :3], frustum_points.T).T + pose[:3, 3]
        # Transform frustum points to world coordinate system using camera pose
        
        #print(frustum_points)
        frustum_points_world = np.dot(pose[:3, :3], frustum_points.T).T + pose[:3, 3]
        #print(frustum_points_world)
        # Create a line geometry for the camera frustum
        lines = [[0, 1], [0, 2], [0, 3], [0, 4] , [1, 2], [2, 3], [3, 4], [4, 1]]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(frustum_points_world)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        return line_set
    
    def display_camera_poses(self, cameraPoses, points):
        visualizer = o3d.visualization.Visualizer()
        visualizer.create_window()

        # Add the world axis
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        visualizer.add_geometry(coordinate_frame)
        
        # add pose to visualizer
       
        visualizer.create_window(window_name="Camera Pose", width=800, height=600)

        for cameraPose in cameraPoses:
            camera_geometry = self.get_camera_geometry(cameraPose)
            visualizer.add_geometry(camera_geometry)
   
        view_control = visualizer.get_view_control()
        view_control.set_up([0, 0, 1])
        view_control.set_lookat([0.5, 0.5, 0.5])
        view_control.set_front([-1.0, -1.0, -1.0])

        for point in points:                       
            coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=6, origin=point)
            visualizer.add_geometry(coordinate_frame)

            # Set the view control (optional)
        view_control = visualizer.get_view_control()
        view_control.set_up([0, -1, 0])  # Set the up direction
        view_control.set_front([0, 0, -1])  # Set the front direction

            # Run the visualization
        visualizer.run()
            #vis.destroy_window()

    
    def display_world_frame(self):

                # Create a coordinate frame
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])

        # Create a visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        # Add the coordinate frame to the visualization window
        vis.add_geometry(coordinate_frame)

        # Set the view control (optional)
        view_control = vis.get_view_control()
        view_control.set_up([0, -1, 0])  # Set the up direction
        view_control.set_front([0, 0, -1])  # Set the front direction

        # Run the visualization
        vis.run()
        #vis.destroy_window()

    def display_poses(self):

        visualizer = o3d.visualization.Visualizer()
        visualizer.create_window()

        # Add the world axis
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        visualizer.add_geometry(coordinate_frame)
        
        # add poses to visualizer
        for camera_index,pose in enumerate(self.poses):
            
            cc = [ ["s"]]

            #pose[:3,:3] = pose[:3,:3] @ rotation
            #print(ee_to_camera @ camera_to_ee)
            visualizer.create_window(window_name="Camera Pose", width=800, height=600)
            camera_geometry = self.get_camera_geometry(pose)
            if camera_index == 0:
                colors = [[1, 0, 0] for _ in range(len(camera_geometry.lines))]
                camera_geometry.colors = o3d.utility.Vector3dVector(colors)
            visualizer.add_geometry(camera_geometry)

        # add cube to visualizer
        vertices = 0.005 * np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
                     [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]])
        shift_vector = [0.48,0.0,0.35]
        vertices += shift_vector
        faces = np.array([[0, 1, 2], [0, 2, 3], [1, 5, 6], [1, 6, 2],
                        [5, 4, 7], [5, 7, 6], [4, 0, 3], [4, 3, 7],
                        [3, 2, 6], [3, 6, 7], [0, 4, 5], [0, 5, 1]])
        cube_mesh = o3d.geometry.TriangleMesh()
        cube_mesh.vertices = o3d.utility.Vector3dVector(vertices)
        cube_mesh.triangles = o3d.utility.Vector3iVector(faces)
        cube_orientation = np.eye(3)  # Identity matrix to maintain the cube's original orientation
        cube_mesh.rotate(cube_orientation, center=(0.5, 0.5, 0.5))
        cube_mesh.compute_vertex_normals()

        visualizer.add_geometry(cube_mesh)
 
        view_control = visualizer.get_view_control()
        view_control.set_up([0, 0, 1])
        view_control.set_lookat([0.5, 0.5, 0.5])
        view_control.set_front([-1.0, -1.0, -1.0])
        visualizer.run()


        

if __name__ == "__main__":
    c = CameraPoseShower()
    c.display_poses()
