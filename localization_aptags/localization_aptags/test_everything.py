import numpy as np
# import cv2
#
#
# # Camera intrinsic parameters
# # cam_matrix = np.array([[fx, 0, cx],
# #                        [0, fy, cy],
# #                        [0, 0, 1]])
#
# # cam_matrix = np.array([[921.6875610351562, 0.0, 639.358154296875],
# #               [0.0, 921.7409057617188, 363.2638854980469],
# #               [0, 0, 1]], dtype=np.float32)
#
#
# K = np.array([[921.6875610351562, 0.0, 639.358154296875, 0.0],
#                [0.0, 921.7409057617188, 363.2638854980469, 0.0],
#                [0, 0, 1, 0.0]])
#
# # K = np.array([[921.6875610351562, 0.0, 0, 0.0],
# #               [0.0, 921.7409057617188, 0.0, 0.0],
# #               [0, 0, 1, 0.0]])
# # dist_coeffs = np.array([k1, k2, p1, p2, k3])
# dist_coeffs = np.array([[0, 0, 0, 0, 0]], dtype=np.float32)
#
# # Detected knee joint pixel coordinates in the image
# knee_pixel_x = 235
# knee_pixel_y = 396
#
# cam_T_ap13 = np.array([[0.0, 1.0, 0.0, 0.19],
#                        [0.0, 0.0, -1.0, -0.15],
#                        [-1.0, 0.0, 0.0, 1.29],
#                        [0.0, 0.0, 0.0, 1.0]])
#
# w_T_ap13 = np.array([[-1.0, 0.0, 0.0, 1.29],
#                     [0.0, 1.0, 0.0, 0.19],
#                     [0.0, 0.0, 1.0, 0.35],
#                     [0.0, 0.0, 0.0, 1.0]])
#
#
# w_T_cam = w_T_ap13 @ np.linalg.inv(cam_T_ap13) # camera extrinsics
# print('w_T_cam', w_T_cam)
#
# height = 720
# width = 1280
#
# knee_camera_coordinates = np.array([[2.1, -1.16, 0.44,1]]).T
#
# projected_homog_point = K @ w_T_cam @ knee_camera_coordinates
# _2d_calculated = [projected_homog_point[0] / projected_homog_point[2], projected_homog_point[1] / projected_homog_point[2]]
# print('_2d_calculated', _2d_calculated)
#
# # what the real number dhould be of the coordinates
# knee_2D_px = np.array([[235, 396]])
# print('knee_2D_px', knee_2D_px)
#
#
# #
# # # Extract rotation vector (rvec) and translation vector (tvec)
# # rvec = cv2.Rodrigues(w_T_cam[:3, :3])[0]
# # tvec = w_T_cam[:3, 3].reshape(3, 1)
# #
# # # Project the knee joint onto the image plane
# # points_3d = knee_camera_coordinates.T
# # points_2d, _ = cv2.projectPoints(points_3d, rvec, tvec, cam_matrix, dist_coeffs)
# #
# # # Compare the projected point with the detected knee joint
# # projected_knee_pixel_x = points_2d[0, 0, 0]
# # projected_knee_pixel_y = points_2d[0, 0, 1]
# #
# # # Calculate the pixel coordinate difference
# # pixel_diff_x = projected_knee_pixel_x - knee_pixel_x
# # pixel_diff_y = projected_knee_pixel_y - knee_pixel_y
# #
# # # Print the results
# # print("Detected knee joint in image (x, y):", knee_pixel_x, knee_pixel_y)
# # print("Projected knee joint in image (x, y):", projected_knee_pixel_x, projected_knee_pixel_y)
# # print("Pixel coordinate difference (x, y):", pixel_diff_x, pixel_diff_y)

K = np.array([[921.6875610351562, 0.0, 639.358154296875, 0.0],
               [0.0, 921.7409057617188, 363.2638854980469, 0.0],
               [0, 0, 1, 0.0]])


knee_pixel_x = 235
knee_pixel_y = 396
# print('knee_2D_px', knee_2D_px)

knee_camera_coordinates = np.array([[ -1.16, -0.24, 2.09, 1]]).T

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

# Calculate the pixel coordinate difference
pixel_diff_x = projected_knee_pixel_x - knee_pixel_x
pixel_diff_y = projected_knee_pixel_y - knee_pixel_y

# Print the results
print("Detected knee joint in image (x, y):", knee_pixel_x, knee_pixel_y)
print("Projected knee joint in image (x, y):", projected_knee_pixel_x, projected_knee_pixel_y)
print("Pixel coordinate difference (x, y):", pixel_diff_x, pixel_diff_y)
