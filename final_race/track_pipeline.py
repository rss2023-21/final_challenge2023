import cv2
import numpy as np
import time 
from homography import get_homography_matrix

# homography matrix (from lab4 visual servoing)
M = get_homography_matrix()

def get_filtered_lines(img):
    # crop image to only include the bottom half 
    # (where the track is)
    height, width, channels = img.shape
    height_cutoff = int(height*0.4)
    img[:int(height*0.4), 0:width] = 0
    # img = img[height_cutoff:, :]

    width = 400
    height = 300
    img_out = cv2.warpPerspective(img, M, (width, height))
    
    # flip vertically
    img_out = cv2.flip(img_out, 0)

    # cv2.imwrite('outputs/homography{0}.png'.format(i), img_out)

    # use homographied image
    img = img_out

    # convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # threshold white color
    thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]
    # cv2.imwrite('thresh.png', thresh)

    # hough line transform
    lines = cv2.HoughLinesP(thresh, rho=1, theta=np.pi/180, threshold=50, minLineLength=150, maxLineGap=10)
    lines_filtered = []

    if lines is None:
        return img, lines_filtered

    for line in lines:
        x1, y1, x2, y2 = line[0]

        # if line is too horizontal, ignore it
        if abs(y1 - y2) < 40:
            continue

        # if slope is small, ignore it
        slope = (1.0 * y2 - y1) / (1.0 * x2 - x1)
        if abs(slope) < 1:
            continue

        # y = mx + b
        # -y/m = x + b/m
        # -b/m = x + y/m
        # y = 0 implies x = -b/m
        x_intercept = x1 + (y1 / slope)
        # y_intercept = y1 - (slope * x1)
        lines_filtered.append([slope, x_intercept])

        thickness = 1 # adjust this to the desired line thickness
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), thickness)

    lines_filtered = np.array(lines_filtered)
    # cluster lines?

    return img, lines_filtered


if __name__ == '__main__':
    # start timing
    start = time.time()
    for i in range(1, 9):
        if i != 7: 
            continue
        
        img = cv2.imread('samples/track{0}.png'.format(i))
        
        img, lines = get_filtered_lines(img)

        # save img to output.png
        cv2.imwrite('outputs/output{0}.png'.format(i), img)

    # end timing
    end = time.time()
    print((end - start) / 8)