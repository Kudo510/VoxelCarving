import cv2 as cv
import numpy as np
import rospkg
from PIL import Image

rospack = rospkg.RosPack()
path = rospack.get_path("voxel_carving")+"/../images/red_block_correct_pose"
segmented_images_path = rospack.get_path("voxel_carving")+"/../segmented_images/red_block_correct_pose"

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720

def is_green(pixel):
    b,g,r = pixel
    green_threshold = 100
    is_green = (g - r) > green_threshold and (g - b) > green_threshold
    return is_green

def calculate_green_screen_center(image):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    lower_green = np.array([36,25,25])  # Lower threshold for green color (B, G, R)
    upper_green = np.array([70,255,255])  # Upper threshold for green color (B, G, R)
    mask = cv.inRange(hsv, lower_green, upper_green)
    masked_image = cv.bitwise_and(image, image, mask=mask)  

    count = 0
    x_total = 0
    y_total = 0
    for x in range(IMAGE_WIDTH):
        for y in range(IMAGE_HEIGHT):
            if (masked_image[y,x]!=[0,0,0]).all():
                x_total+=x
                y_total+=y
                count+=1

    return int(x_total/count), int(y_total/count)

def perform_clustering(image, k):
    pixel_vals = image.reshape((-1,3))
    # Convert to float type
    pixel_vals = np.float32(pixel_vals)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 500, 0.85)
    # then perform k-means clustering with number of clusters defined as 3
    #also random centres are initially choosed for k-means clustering
    retval, labels, centers = cv.kmeans(pixel_vals, k, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)
    # convert data into 8-bit values
    centers = np.uint8(centers)
    segmented_data = centers[labels.flatten()]
    # reshape data into the original image dimensions
    clustered_image = segmented_data.reshape((image.shape))
    return clustered_image

def select_region_of_interest(image, center, width = 550, height = 450):
    x,y = center
    x_min = int(x - width/2)
    x_max = int(x + width/2)
    y_min = int(y - height/2)
    y_max = int(y + height/2)
    # Create a mask of the same size as the image, initialized with all white pixels
    mask = np.zeros_like(image, dtype=np.uint8) 
    # Set pixels outside the window to black in the mask
    mask[y_min:y_max, x_min:x_max] = 255
    # Apply the mask to the image
    result = cv.bitwise_and(image, mask)
    return result

def convert_green_to_black(image):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    lower_green = np.array([30,0,0])  # Lower threshold for green color (B, G, R)
    upper_green = np.array([75,255,255])  # Upper threshold for green color (B, G, R)
    green_mask = cv.inRange(hsv, lower_green, upper_green)
    image[green_mask != 0] = (0, 0, 0)
    return image


def perform_segmentation(image):
    intermediate_clustered_image = perform_clustering(image, k = 7)
    roi_center = calculate_green_screen_center(intermediate_clustered_image)
    roi = select_region_of_interest(intermediate_clustered_image,roi_center)
    segmented_image = perform_clustering(roi, k = 3)
    binary_segmented_image = convert_green_to_black(segmented_image)
    
    mask = np.zeros_like(image)
    mask[binary_segmented_image != 0] = 255
    segmented_image = cv.bitwise_and(image, mask)

    return segmented_image


if __name__ == "__main__":
    images = []
    for i in range(16):
        image = cv.imread(path + "/image"+str(i*16+1)+".jpg")
        segmented_image = perform_segmentation(image)
        images.append(segmented_image)
        segmented_imageRGB = cv.cvtColor(segmented_image, cv.COLOR_BGR2RGB)
        im = Image.fromarray(segmented_imageRGB)
        im.save(segmented_images_path + '/image' + str(i*16+1) + '.png')
        print("image" + str(i) + " done!")


    for image in images:
        pass
