import cv2
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
import time

# Define mouse callback function for drawing the ROI
def draw_rectangle(event, x, y, flags, param):
    global ix, iy, drawing, ex, ey
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            ex, ey = x, y
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        ex, ey = x, y

# Function to detect features using SIFT within a given image
def detect_features_SIFT(image):
    if image is None or image.size == 0:
        return np.empty((0, 2))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    keypoints, _ = sift.detectAndCompute(gray, None)
    return np.float32([kp.pt for kp in keypoints]) if keypoints is not None else np.empty((0, 2))

# Function to detect features using Shi Tomasi Corner detector
def detect_features_Shi_Tomasi_corner(image):
    if image is None or image.size == 0:
        return np.empty((0, 2))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Use goodFeaturesToTrack
    # You can adjust the parameters as needed
    corners = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.01, minDistance=10) # TODO: Play with these parameters

    return np.float32(corners).reshape(-1, 2) if corners is not None else np.empty((0, 2))

# Function to track features using CUDA-accelerated KLT tracker
def track_features(prev_frame, current_frame, prev_features, cuda_lk):
    current_gray_cuda = cv2.cuda.cvtColor(cv2.cuda_GpuMat(current_frame), cv2.COLOR_BGR2GRAY)
    prev_gray_cuda = cv2.cuda.cvtColor(cv2.cuda_GpuMat(prev_frame), cv2.COLOR_BGR2GRAY)

    prev_features_reshaped = prev_features.reshape(1, -1, 2).astype(np.float32)
    prev_features_cuda = cv2.cuda_GpuMat()
    prev_features_cuda.upload(prev_features_reshaped)

    new_features_cuda, status_cuda, _ = cuda_lk.calc(prev_gray_cuda, current_gray_cuda, prev_features_cuda, None)

    new_features = new_features_cuda.download()
    status = status_cuda.download()

    if new_features.size == 0 or status.size == 0:
        return np.empty((0, 2))

    new_features = new_features.reshape(-1, 2)
    valid = status.flatten() == 1
    return new_features[valid] if valid.any() else np.empty((0, 2))

# Define state transition and measurement functions for the UKF
# - The state transition function must accurately represent how you expect people or vehicles to move. For vehicles, you might include models
#   that account for smoother trajectories, while for people, a more flexible model might be necessary.
# - The measurement function should correlate with how you are measuring position (e.g., pixel coordinates in image space).
def state_transition_function(state, dt):
    # NOTE: This is a simple constant velocity model. May want to replace it with a more complex model for people and vehicles respectively
    x, y, vx, vy = state
    return [x + dt * vx, y + dt * vy, vx, vy]

def measurement_function(state):
    x, y, vx, vy = state
    return [x, y]

# Start video capture
cap = cv2.VideoCapture(0)

# Measure frame rate
frame_count = 0
start_time = time.time()

# Capture frames for a fixed duration (e.g., 5 seconds)
while time.time() - start_time < 5:
    ret, frame = cap.read()
    if ret:
        frame_count += 1
    else:
        break

# Calculate frame rate
elapsed_time = time.time() - start_time
frame_rate = frame_count / elapsed_time
print(f"Measured Frame Rate: {frame_rate:.2f} frames per second")

# Set dt for UKF based on the measured frame rate
dt = 1.0 / frame_rate

# Release the video capture
cap.release()

# Initialize Unscented Kalman Filter
points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2., kappa=-1)
ukf = UnscentedKalmanFilter(dim_x=4, dim_z=2, dt=dt, fx=state_transition_function, hx=measurement_function, points=points)

# Initial state - assuming starting at origin with no velocity
ukf.x = np.array([0., 0., 0., 0.])

# Initial covariance matrix - larger values indicate less confidence in the initial state
ukf.P *= 10.

# Process noise - adjust based on the expected variability in movement
# Higher for people due to erratic movements, lower for [off-road] vehicles
ukf.Q = np.diag([0.2, 0.2, 1, 1])  # Person [position variance, position variance, velocity variance, velocity variance]
#ukf.Q = np.diag([0.1, 0.1, 0.5, 0.5])  # Vehicle (variable terrain) [position variance, position variance, velocity variance, velocity variance]

# Measurement noise - depends on the accuracy of your measurements
# - If using camera-based tracking, factors like camera resolution, distance to the subject, and lighting conditions can affect this.
# - In scenarios with higher uncertainty in measurements (e.g., low visibility), increase the measurement noise covariance.
ukf.R = np.diag([0.1, 0.1])  # Adjust based on measurement uncertainty

# Initialize SIFT detector and CUDA KLT tracker
sift = cv2.SIFT_create()
lk_params = dict(winSize=(15, 15), maxLevel=2, iters=10) # TODO: Play with these KLT tracker parameters
cuda_lk = cv2.cuda.SparsePyrLKOpticalFlow_create(**lk_params)

# Start video capture from camera
cap = cv2.VideoCapture(0)

# Setup the window and mouse callback for ROI selection
cv2.namedWindow("Live Stream")
cv2.setMouseCallback("Live Stream", draw_rectangle)

drawing = False
ix, iy, ex, ey = -1, -1, -1, -1 # Initial and ending coordinates for drawing rectangle

while True:
    ret, frame = cap.read()
    if not ret:
        break
    if drawing or (ix != -1 and iy != -1):
        cv2.rectangle(frame, (ix, iy), (ex, ey), (0, 255, 0), 2)
    cv2.imshow("Live Stream", frame)
    if cv2.waitKey(1) & 0xFF == 13: # Enter key to finalize ROI
        break
cv2.destroyAllWindows()

# Calculate ROI coordinates and initialize tracking
x, y, w, h = min(ix, ex), min(iy, ey), abs(ex-ix), abs(ey-iy)
if w and h:
    roi = frame[y:y+h, x:x+w]
    
    # TODO: play with feature detector methods here
    prev_features = detect_features_SIFT(roi)
    # prev_features = detect_features_Shi_Tomasi_corner(roi)
    
    prev_features += (x, y)
    bbox_active = True
    prev_frame = frame.copy()
    
    # Initialize feature count related variables
    initial_feature_counts = []
    calculate_initial_features = True
    min_features_threshold = 0
    feature_threshold_delta = 40
    lost_threshold = 5
    lost_count = 0
    lost_frame_limit = 15  # Limit for how many frames to search for the subject after being lost
    
    # Define colors for bounding box
    color_tracked = (0, 255, 0)  # Green when features are tracked
    color_predicted = (0, 0, 255)  # Red when relying on prediction
    
else:
    bbox_active = False

# Main tracking loop
while bbox_active:
    ret, frame = cap.read()
    if not ret:
        break
    
    features = track_features(prev_frame, frame, prev_features, cuda_lk)#, (x, y, w, h))
    
    # Determine initial feature count threshold
    if calculate_initial_features:
        initial_feature_counts.append(features.shape[0])
        if len(initial_feature_counts) >= 5:  # Use the first 5 frames to calculate the average
            avg_initial_features = sum(initial_feature_counts) / len(initial_feature_counts)
            min_features_threshold = max(avg_initial_features - feature_threshold_delta, 0)
            calculate_initial_features = False
            print(f"Initial average features: {avg_initial_features}, threshold set to: {min_features_threshold}")
    else:
        # Debug: Print number of features detected
        print(f"Detected features: {len(features)}")

        # Debug: Visualize detected features
        for feature in features:
            cv2.circle(frame, (int(feature[0]), int(feature[1])), 2, (255, 0, 0), -1)

    # Check if features are detected and above the threshold
    if features.size > 0 and features.shape[1] == 2 and features.shape[0] >= min_features_threshold:
        
        # Features detected: Update UKF with measurements
        x_mean, y_mean = np.mean(features, axis=0)
        ukf.predict()
        ukf.update([x_mean, y_mean])
        x, y = int(x_mean), int(y_mean)  # Use measurements directly for bounding box
        # x, y = int(ukf.x[0]), int(ukf.x[1]) # Use filter prediction for bounding box
        prev_features = features.reshape(-1, 1, 2)
        lost_count = 0
        current_color = color_tracked  # Set color to green when tracked
    else:
        # Increment lost count and use Kalman Filter for prediction
        lost_count += 1
        ukf.predict()
        x, y = int(ukf.x[0]), int(ukf.x[1])
        
        if lost_count > lost_threshold:
            current_color = color_predicted  # Set color to red when predicted
        
        # Stop attempting to find the object if it's been lost for too long
        if lost_count > lost_frame_limit:
            print("Unable to relocate subject, stopping tracking.")
            break

    # Draw bounding box with the current color
    cv2.rectangle(frame, (int(x - w/2), int(y - h/2)), (int(x + w/2), int(y + h/2)), current_color, 2)

    # Display the frame
    cv2.imshow('CUDA KLT tracker with SIFT', frame)
    prev_frame = frame.copy()

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()