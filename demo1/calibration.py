import numpy as np
import cv2 as cv
import glob
import pickle


################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

chessboardSize = (8, 5)
frameSize = (640, 480)


# termination criteria
criteria = (
    cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER,
    30,
    0.001
)


# prepare object points
objp = np.zeros(
    (chessboardSize[0] * chessboardSize[1], 3),
    np.float32
)

objp[:, :2] = np.mgrid[
    0:chessboardSize[0],
    0:chessboardSize[1]
].T.reshape(-1, 2)

size_of_chessboard_squares_mm = 20
objp = objp * size_of_chessboard_squares_mm


# Arrays to store object points and image points
objpoints = []
imgpoints = []


images = glob.glob('images/*.png')

print("Images found:", len(images))

for image in images:

    img = cv.imread(image)

    if img is None:
        print("Could not read image:", image)
        continue

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv.findChessboardCorners(
        gray, chessboardSize, None
    )

    # If found, add object points and refined image points
    if ret == True:

        objpoints.append(objp.copy())

        corners2 = cv.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria
        )

        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(
            img, chessboardSize, corners2, ret
        )

        cv.imshow('img', img)
        cv.waitKey(1000)

    else:
        print("Chessboard corners not found:", image)


cv.destroyAllWindows()


if len(objpoints) == 0:
    raise RuntimeError(
        "No chessboard corners found. Check images and chessboardSize."
    )


############## CALIBRATION #######################################################

ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints, imgpoints, frameSize, None, None
)


# Save the camera calibration result
with open("calibration.pkl", "wb") as f:
    pickle.dump((cameraMatrix, dist), f)

with open("cameraMatrix.pkl", "wb") as f:
    pickle.dump(cameraMatrix, f)

with open("dist.pkl", "wb") as f:
    pickle.dump(dist, f)


############## UNDISTORTION #####################################################

img = cv.imread('cali5.png')

if img is None:
    raise FileNotFoundError("Could not find cali5.png")

h, w = img.shape[:2]

newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(
    cameraMatrix, dist, (w, h), 1, (w, h)
)


# Undistort
dst = cv.undistort(
    img, cameraMatrix, dist, None, newCameraMatrix
)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]

cv.imwrite('caliResult1.png', dst)


# Undistort with Remapping
h_img, w_img = img.shape[:2]

mapx, mapy = cv.initUndistortRectifyMap(
    cameraMatrix,
    dist,
    None,
    newCameraMatrix,
    (w_img, h_img),
    5
)

dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]

cv.imwrite('caliResult2.png', dst)


############## Reprojection Error ################################################

mean_error = 0

for i in range(len(objpoints)):

    imgpoints2, _ = cv.projectPoints(
        objpoints[i],
        rvecs[i],
        tvecs[i],
        cameraMatrix,
        dist
    )

    error = cv.norm(
        imgpoints[i], imgpoints2, cv.NORM_L2
    ) / len(imgpoints2)

    mean_error += error

print("total error: {}".format(mean_error / len(objpoints)))
