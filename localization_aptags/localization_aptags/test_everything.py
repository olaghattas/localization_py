import numpy as np
import cv2
import os

# # Camera intrinsic parameters
# # cam_matrix = np.array([[fx, 0, cx],
# #                        [0, fy, cy],
# #                        [0, 0, 1]])

def plot_on_image(x_1 ,y_1, x_real, y_real ):
    # Step 1: Read the image using OpenCV

    ### SPECIFIC PATH
    path = os.environ["HOME"] + "/smart-home/src/smart-home/external/human_pose_detection/pose_detection/pose_images_for_debug/"  # Replace with the actual path to your image
    # image_name = '1618_611' + ".png"
    # image_name = '1618_611' + ".png"
    image_name = '724_636' + ".png"
    image_path = path + image_name
    image = cv2.imread(image_path)

    #Draw the point on the image
    point_color = (0, 0, 255)  # Red color in BGR format
    point_radius = 5          # Radius of the point
    thickness = -1            # Thickness -1 fills the circle
    cv2.circle(image, (x_real, y_real), point_radius, point_color, thickness)

    # projected pt
    # Draw the point on the image
    point_color = (255, 0, 0)  # Blue color in BGR format
    point_radius = 5          # Radius of the point
    thickness = -1            # Thickness -1 fills the circle
    cv2.circle(image, (int(x_1), int(y_1)), point_radius, point_color, thickness)

    # knee_camera_coordinates = np.array([[ 0.78, 0.09, 2.53, 1]]).T
    # old point
    # point_color = (0, 255, 0)  # Blue color in BGR format
    # point_radius = 5          # Radius of the point
    # thickness = -1            # Thickness -1 fills the circle
    # cv2.circle(image, (int(923.51479367), int(396.05308768)), point_radius, point_color, thickness)


    # Step 4: Display or save the modified image
    cv2.imshow("Image with Point", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def project_with_matrix_multi():
    K = np.array([[921.6875610351562, 0.0, 639.358154296875, 0.0],
                   [0.0, 921.7409057617188, 363.2638854980469, 0.0],
                   [0, 0, 1, 0.0]])


    knee_pixel_x = 933
    knee_pixel_y = 498
    # print('knee_2D_px', knee_2D_px)

    # in optical frame coordinates S1

    knee_camera_coordinates = np.array([[0.81, 0.36, 2.5, 1]]).T

    dst = K @ knee_camera_coordinates
    x = dst[0]
    y = dst[1]
    w = dst[2]
    if w != 0:
        points_2d = (x / w, y / w)
    else:
        points_2d = (0.0, 0.0)

    # Compare the projected point with the detected knee joint
    projected_knee_pixel_x = points_2d[0]
    projected_knee_pixel_y = points_2d[1]
    # print("Data type of projected_knee_pixel_x:", type(projected_knee_pixel_x))
    # print("Data type of projected_knee_pixel_y:", type(projected_knee_pixel_y))
    # print("Projected knee pixel x:", projected_knee_pixel_x)
    # print("Projected knee pixel y:", projected_knee_pixel_y)


    # Calculate the pixel coordinate difference
    pixel_diff_x = projected_knee_pixel_x - knee_pixel_x
    pixel_diff_y = projected_knee_pixel_y - knee_pixel_y
    plot_on_image(projected_knee_pixel_x, projected_knee_pixel_y)
    # Print the results
    print("Detected knee joint in image (x, y):", knee_pixel_x, knee_pixel_y)
    print("Projected knee joint in image (x, y):", projected_knee_pixel_x, projected_knee_pixel_y)
    print("Pixel coordinate difference (x, y):", pixel_diff_x, pixel_diff_y)

def project_with_cv2():
    # K = np.array([[1759, 0.0, 0],
    #               [0.0, 989, 0.0],
    #               [0, 0, 1]])
    K = np.array([[1510.42359328571, 0.0, 956.274149759222],
                  [0.0, 1514.99036925224, 546.045450379061],
                  [0, 0, 1]])


    # ## 1 advil
    # knee_pixel_x = 1618
    # knee_pixel_y = 611
    # knee_camera_coordinates = np.array([[[1.3, 0.15, 2.92]]]).T

    # ## cream
    knee_pixel_x = 724
    knee_pixel_y = 636
    knee_camera_coordinates = np.array([[[-0.4, 0.15, 2.8]]]).T

    # ## SPF
    # knee_pixel_x = 1126
    # knee_pixel_y = 673
    # knee_camera_coordinates = np.array([[[0.3, 0.15, 2.28]]]).T

    # dist_coeffs = np.array([k1, k2, p1, p2, k3])
    dist_coeffs = np.array([[0.0177522271029360, -0.245387961307194, 0, 0, 0]], dtype=np.float32)

    # Extract rotation vector (rvec) and translation vector (tvec)
    # # rvec = cv2.Rodrigues(w_T_cam[:3, :3])[0]
    # # tvec = w_T_cam[:3, 3].reshape(3, 1)

    # Project the knee joint onto the image plane
    points_3d = knee_camera_coordinates.T
    points_2d, _ = cv2.projectPoints(points_3d, np.zeros((3, 1)), np.zeros((3, 1)), K, dist_coeffs)

    # Compare the projected point with the detected knee joint
    projected_knee_pixel_x = points_2d[0, 0, 0]
    projected_knee_pixel_y = points_2d[0, 0, 1]
    print('z', points_2d)

    # Calculate the pixel coordinate difference
    pixel_diff_x = projected_knee_pixel_x - knee_pixel_x
    pixel_diff_y = projected_knee_pixel_y - knee_pixel_y
    plot_on_image(projected_knee_pixel_x, projected_knee_pixel_y, knee_pixel_x, knee_pixel_y)

    # Print the results
    print("Detected knee joint in image (x, y):", knee_pixel_x, knee_pixel_y)
    print("Projected knee joint in image (x, y):", projected_knee_pixel_x, projected_knee_pixel_y)
    print("Pixel coordinate difference (x, y):", pixel_diff_x, pixel_diff_y)

def project_with_librealsense_rsutil():
    knee_pixel_x = 724
    knee_pixel_y = 636

    fx = 1510.42359328571
    fy = 1514.99036925224

    # px = 639.358154296875
    # py = 363.2638854980469

    px = 956.274149759222
    py = 546.045450379061
    # print('knee_2D_px', knee_2D_px)

    # in optical frame coordinates S1
    knee_camera_coordinates = [-0.4, 0.15, 2.8]
    x = knee_camera_coordinates[0]/knee_camera_coordinates[2]
    y = knee_camera_coordinates[1]/knee_camera_coordinates[2]

    dist_coeffs = np.array([0.0177522271029360, -0.245387961307194, 0, 0, 0], dtype=np.float32)
    r2 = x*x + y*y
    f = 1 + dist_coeffs[0]*r2 + dist_coeffs[1]*r2*r2 + dist_coeffs[4]*r2*r2*r2
    x = f * x
    y = f * x
    dx = x + 2*dist_coeffs[2]*x*y + dist_coeffs[3]*(r2 + 2*x*x)
    dy = y + 2*dist_coeffs[3]*x*y + dist_coeffs[2]*(r2 + 2*y*y)
    x = dx
    y = dy

    # Compare the projected point with the detected knee joint
    projected_knee_pixel_x = x * fx + px
    projected_knee_pixel_y = y * fy + py

    # Calculate the pixel coordinate difference
    pixel_diff_x = projected_knee_pixel_x - knee_pixel_x
    pixel_diff_y = projected_knee_pixel_y - knee_pixel_y
    plot_on_image(projected_knee_pixel_x, projected_knee_pixel_y, knee_pixel_x, knee_pixel_y)

    # Print the results
    print("Detected knee joint in image (x, y):", knee_pixel_x, knee_pixel_y)
    print("Projected knee joint in image (x, y):", projected_knee_pixel_x, projected_knee_pixel_y)
    print("Pixel coordinate difference (x, y):", pixel_diff_x, pixel_diff_y)


# project_with_cv2()
project_with_librealsense_rsutil()
