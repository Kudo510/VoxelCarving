import sys
import rospkg
import numpy as np
import cv2

if __name__ == "__main__":
    
    rospack = rospkg.RosPack()
    path_read = rospack.get_path("voxel_carving")+"/data/red_block_correct_pose.txt"
    path_write = rospack.get_path("voxel_carving")+"/data/red_block_correct_pose_2.txt"

    ee_to_camera_fake = np.array([[1, 0, 0, 0.307, 0, 1, 0, 0, 0, 0, 1, 0.487, 0, 0, 0, 1]]).reshape(4,4)
    camera_to_ee_fake = np.linalg.inv(ee_to_camera_fake)
    ee_to_camera_true = np.array([[1, 0, 0, 0.01, 0, 1, 0, 0.02, 0, 0, 1, 0.039978, 0, 0, 0, 1]]).reshape(4,4)
    
    with open(path_read) as f_read:
        with open(path_write, "w") as f_write:
            for line in f_read:
                pose = line.strip('[]\n').split(",")
                pose = [float(value) for value in pose]
                pose = np.array(pose)
                pose = np.reshape(pose, (4, 4))
                pose = pose @ camera_to_ee_fake @ ee_to_camera_true
                pose = pose.flatten()
                f_write.write("[ ")
                for count,value in enumerate(pose):
                    f_write.write(str(value))
                    if(count!=15):
                        f_write.write(", ")
                f_write.write("]\n")

        K = cv2.intris
