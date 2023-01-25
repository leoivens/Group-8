import cv2
import numpy as np

# Load the template images of the bottles (orange and black)
orange_bottle = cv2.imread('orange_bottle.jpeg', 0)
black_bottle = cv2.imread('black_bottle.jpeg', 0)

# Create a list of templates to search for
templates = [orange_bottle, black_bottle]

# Start the camera
cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert the frame to grayscale for template matching
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Iterate through the templates
    for template in templates:
        # Perform multi-scale template matching
        res = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
        threshold = 0.8  # Set a threshold for a match
        loc = np.where(res >= threshold)

        # Draw a rectangle around the matched region
        for pt in zip(*loc[::-1]):
            cv2.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0,255,0), 2)

    # Display the resulting frame
    cv2.imshow('frame', frame)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
