import cv2
import os

os.makedirs('images', exist_ok=True)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

num = 0

while cap.isOpened():

    succes, img = cap.read()

    if not succes:
        print("Error: Could not read frame.")
        break

    k = cv2.waitKey(5)

    if k == 27:
        break
    elif k == ord('s'):
        cv2.imwrite('images/img' + str(num) + '.png', img)
        print("image saved!")
        num += 1

    cv2.imshow('Img', img)

# Release and destroy all windows before termination
cap.release()
cv2.destroyAllWindows()
