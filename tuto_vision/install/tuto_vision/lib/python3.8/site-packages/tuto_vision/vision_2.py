import cv2
import numpy as np
from matplotlib import pyplot as plt

# Initialize the camera
cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the range of colors for black and orange bottles
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 30])
    lower_orange = np.array([5, 50, 50])
    upper_orange = np.array([15, 255, 255])

    # Create masks for the black and orange colors
    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

    # Combine the masks to create a single image
    combined_mask = cv2.bitwise_or(mask_black, mask_orange)

    # Use the k-means method to segment the image
    Z = combined_mask.reshape((-1,3))
    Z = np.float32(Z)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 2
    ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)

    center = np.uint8(center)
    res = center[label.flatten()]
    kmeans_result = res.reshape((combined_mask.shape))

    # Use the centroid_histogram() function to detect the black and orange bottles

    def centroid_histogram(clt):
     numLabels = np.arange(0, len(np.unique(clt.labels_)) + 1)
     (hist, _) = np.histogram(clt.labels_, bins=numLabels)

     # normalize the histogram, such that it sums to one
     hist = hist.astype("float")
     hist /= hist.sum()

     return hist

    def plot_colors(hist, centroids):
     bar = np.zeros((50, 300, 3), dtype="uint8")
     startX = 0

     # loop over the percentage of each cluster and the color of
     # each cluster
     for (percent, color) in zip(hist, centroids):
        # plot the relative percentage of each cluster
        endX = startX + (percent * 300)
        cv2.rectangle(bar, (int(startX), 0), (int(endX), 50),
                      color.astype("uint8").tolist(), -1)
        startX = endX

     return bar

    black_centroid, black_counts = centroid_histogram(kmeans_result, lower_black, upper_black)
    orange_centroid, orange_counts = centroid_histogram(kmeans_result, lower_orange, upper_orange)

    # Use the plot_colors() function to draw circles around the detected bottles
    plot_colors(frame, black_centroid, black_counts)
    plot_colors(frame, orange_centroid, orange_counts)

    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
