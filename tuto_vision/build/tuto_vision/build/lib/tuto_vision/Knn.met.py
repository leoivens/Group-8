import cv2
import numpy as np
from sklearn.neighbors import KNeighborsClassifier

basedir_data = "./tuto_vision/"
rel_path = basedir_data + "cifar-100-python/"

#Désérialiser les fichiers image afin de permettre l’accès aux données et aux labels:

def unpickle(file):
    import pickle
    with open(file, 'rb') as fo:
        dict = pickle.load(fo,encoding='bytes')
    return dict

X = unpickle(rel_path + 'data_batch_1')
img_data = X[b'data']
img_label_orig = img_label = X[b'labels']
img_label = np.array(img_label).reshape(-1, 1)


# Load training data
orange_bottles = []
black_bottles = []

for i in range(1, 11):
    orange_bottles.append(cv2.imread('orange_bottle_' + str(i) + '.jpg'))
    black_bottles.append(cv2.imread('black_bottle_' + str(i) + '.jpg'))

# Extract features from training data
orange_features = []
black_features = []

for i in range(10):
    orange_hsv = cv2.cvtColor(orange_bottles[i], cv2.COLOR_BGR2HSV)
    black_hsv = cv2.cvtColor(black_bottles[i], cv2.COLOR_BGR2HSV)

    orange_features.append(np.ravel(orange_hsv))
    black_features.append(np.ravel(black_hsv))

# Create labels for training data
orange_labels = np.zeros(10)
black_labels = np.ones(10)

# Combine features and labels
features = np.concatenate((orange_features, black_features))
labels = np.concatenate((orange_labels, black_labels))

# Create k-NN classifier
knn = KNeighborsClassifier(n_neighbors=3)

# Train classifier
knn.fit(features, labels)

# Capture image from camera
cap = cv2.VideoCapture(0)
ret, frame = cap.read()

# Convert image to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# Flatten image
flattened_image = np.ravel(hsv)

# Predict class
prediction = knn.predict([flattened_image])

# Display prediction
if prediction == 0:
    print("The image is of an orange bottle.")
else:
    print("The image is of a black bottle.")

# Release camera
cap.release()
