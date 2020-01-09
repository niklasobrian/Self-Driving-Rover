import numpy as np
import cv2
from matplotlib import pyplot as plt
from tkinter import*
import numpy as np
import IPCamAccess


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image
    # mask = cv2.warpPerspective(np.ones_like(img[:, :, 0]), M, (img.shape[1], img.shape[0]))
    # Removed mask because it occasionally creates seemingly arbitrary errors
    # return warped, mask (whenever mask is used)
    # Mask can be initialized by returning "mask" in the "rock_thresh" section
    return warped


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(80, 80, 80)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def obstacle_thresh(img, rgb_thresh=(120, 120, 120)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[below_thresh] = 1
    # Return the binary image
    return color_select

def rock_thresh(img, threshold_low=(95, 0, 0), threshold_high=(210, 210, 55)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > threshold_low[0]) & (img[:,:,0] < threshold_high[0])  \
                   & (img[:,:,1] > threshold_low[1]) & (img[:,:,1] < threshold_high[1]) \
                   & (img[:,:,2] > threshold_low[2]) & (img[:,:,2] < threshold_high[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles



# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()

    Rover.img = IPCamAccess.IpCamState()
    #Rover.img = cv2.imread('C:/Users/OB/Documents/-1-Plugg/Udacity robotcis/Pollutionator/calibration_images/example_grid.jpg')

    # 1) Define source and destination points for perspective transform
   # xpos, ypos = Rover.pos
    #yaw = Rover.yaw

    dst_size = 65
    # Equates to "box size"
    bottom_offset = 130
    source = np.float32([[125, 488], [274, 472], [257, 396], [138, 407]])
    destination = np.float32([[Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              [Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset], ])




    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
 #   plt.show(warped)
    Rover.warped=warped
    # Note: add mask back to this point whenever used

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    threshed_ground = color_thresh(warped)
    threshed_obstacle = obstacle_thresh(warped)
    threshed_rock = rock_thresh(warped)


    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
    #      Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    #      Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:, :, 2] = threshed_ground * 255
    Rover.vision_image[:, :, 0] = threshed_obstacle * 255
    Rover.vision_image[:, :, 1] = threshed_rock * 255

    # 5) Convert map image pixel values to rover-centric coords
    ground_xpix, ground_ypix = rover_coords(threshed_ground)
    plt.plot(ground_xpix, ground_ypix, '.')
    plt.ylim(-320, 320)
    plt.xlim(0, 480)
    # plt.show()

    colorsel = color_thresh(warped, rgb_thresh=(80, 80, 80))  # Threshold the warped image
    xpix, ypix = rover_coords(colorsel)  # Convert to rover-centric coords
    rock_xpix, rock_ypix = rover_coords(rock_thresh)
    distances, angles = to_polar_coords(xpix, ypix)  # Convert to polar coords
    avg_angle = np.mean(angles)  # Compute the average angle

    # 6) Convert rover-centric pixel values to world coordinates
 #   x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
  #  obs_xworld, obs_yworld = pix_to_world(obs_xpix, obs_ypix, xpos, ypos, yaw, world_size, scale)
 #   rock_xworld, rock_yworld = pix_to_world(rock_xpix, rock_ypix, xpos, ypos, yaw, world_size, scale)

    # 8) Convert rover-centric pixel positions to polar coordinates
    rover_dist, rover_angles = to_polar_coords(xpix, ypix)
    rock_dist, rock_angle = to_polar_coords(rock_xpix, rock_ypix)
    # Update Rover pixel distances and angles
    Rover.nav_dists = rover_dist
    Rover.nav_angles = rover_angles
    print(rover_angles)

    # Do some plotting
    fig = plt.figure(figsize=(12, 9))
    plt.subplot(221)
    plt.imshow(Rover.img)
    plt.subplot(222)
    plt.imshow(warped)
    plt.subplot(223)
    plt.imshow(colorsel, cmap='gray')
    plt.subplot(224)
    plt.plot(xpix, ypix, '.')
    plt.ylim(-240, 240)
    plt.xlim(0, 640)
    arrow_length = 100
    x_arrow = arrow_length * np.cos(avg_angle)
    y_arrow = arrow_length * np.sin(avg_angle)
    plt.arrow(0, 0, x_arrow, y_arrow, color='red', zorder=2, head_width=10, width=2)
    plt.show()






