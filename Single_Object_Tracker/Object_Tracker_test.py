import cv2
import numpy as np

# Global variables for mouse callback
drawing = False  # Indicates if the mouse is being pressed and dragged
ix, iy = -1, -1  # Initial x, y coordinates for drawing rectangle
ex, ey = -1, -1  # Ending x, y coordinates for drawing rectangle

# Mouse callback function to draw the ROI on the frame
def draw_rectangle(event, x, y, flags, param):
    global ix, iy, drawing, ex, ey
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True  # Start drawing
        ix, iy = x, y
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            ex, ey = x, y
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False  # Stop drawing
        ex, ey = x, y

# Function to detect features within a given image
def detect_features(image):
    if image is None or image.size == 0:
        return np.empty((0, 2))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    keypoints, _ = sift.detectAndCompute(gray, None)
    return np.float32([kp.pt for kp in keypoints]) if keypoints is not None else np.empty((0, 2))

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

# Initialize Kalman Filter for tracking
def init_kalman(x, y, w, h):
    kalman = cv2.KalmanFilter(4, 2)
    kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    kalman.processNoiseCov = 1e-4 * np.eye(4, dtype=np.float32)
    kalman.measurementNoiseCov = 1e-1 * np.eye(2, dtype=np.float32)
    # The Kalman Filter's state is initialized with the center of the ROI and zero initial velocity
    kalman.errorCovPost = 1e-1 * np.eye(4, dtype=np.float32)
    # Initialize state with the center of the ROI
    kalman.statePost = np.array([[x + w/2], [y + h/2], [0], [0]], np.float32)
    return kalman

# Function for Kalman Filter prediction and correction
def kalman_predict_and_correct(kalman, measurement=None):
    # Predict the next state
    prediction = kalman.predict()

    # Correct with the measurement if available
    if measurement is not None:
        kalman.correct(np.array([[np.float32(measurement[0])], [np.float32(measurement[1])]]))

    # Return the predicted state
    return int(prediction[0]), int(prediction[1])

# Initialize SIFT detector and CUDA KLT tracker
sift = cv2.SIFT_create()
lk_params = dict(winSize=(15, 15), maxLevel=2, iters=10)
cuda_lk = cv2.cuda.SparsePyrLKOpticalFlow_create(**lk_params)

# Start video capture from the camera
cap = cv2.VideoCapture(0)

# Set up the window and mouse callback for ROI selection
cv2.namedWindow("Live Stream")
cv2.setMouseCallback("Live Stream", draw_rectangle)

# Loop for live streaming and drawing the ROI
while True:
    ret, frame = cap.read()
    if not ret:
        break
    if drawing or (ix != -1 and iy != -1):
        cv2.rectangle(frame, (ix, iy), (ex, ey), (0, 255, 0), 2)
    cv2.imshow("Live Stream", frame)
    if cv2.waitKey(1) & 0xFF == 13:  # Enter key to finalize ROI
        break
cv2.destroyAllWindows()

# Calculate ROI coordinates and initialize tracking
x, y, w, h = min(ix, ex), min(iy, ey), abs(ex-ix), abs(ey-iy)
if w and h:
    roi = frame[y:y+h, x:x+w]
    prev_features = detect_features(roi)
    prev_features += (x, y)
    bbox_active = True
    prev_frame = frame.copy()
    kalman = init_kalman(x, y, w, h)

    # Initialize feature count related variables
    initial_feature_counts = []
    calculate_initial_features = True
    min_features_threshold = 0
    feature_threshold_delta = 40
    lost_threshold = 5
    lost_count = 0
    lost_frame_limit = 300  # Limit for how many frames to search for the subject after being lost
    
    # Define colors for bounding box
    color_tracked = (0, 255, 0)  # Green when features are tracked
    color_predicted = (0, 0, 255)  # Red when relying on prediction

# Main tracking loop
while bbox_active:
    ret, frame = cap.read()
    if not ret:
        break

    features = track_features(prev_frame, frame, prev_features, cuda_lk)

    # Determine initial feature count threshold
    if calculate_initial_features:
        initial_feature_counts.append(features.shape[0])
        if len(initial_feature_counts) >= 5:  # Use the first 5 frames to calculate the average
            avg_initial_features = sum(initial_feature_counts) / len(initial_feature_counts)
            min_features_threshold = max(avg_initial_features - feature_threshold_delta, 0)
            calculate_initial_features = False
            print(f"Initial average features: {avg_initial_features}, threshold set to: {min_features_threshold}")

    # Check if features are detected and above the threshold
    if features.size > 0 and features.shape[1] == 2 and features.shape[0] >= min_features_threshold:
        # Features detected: Update Kalman Filter and use measurements for the bounding box
        x_mean, y_mean = np.mean(features, axis=0)
        kalman_predict_and_correct(kalman, (x_mean, y_mean))  # Update Kalman Filter
        x, y = int(x_mean), int(y_mean)  # Use measurements directly for bounding box
        prev_features = features.reshape(-1, 1, 2)
        lost_count = 0
        current_color = color_tracked  # Set color to green when tracked
    else:
        # Increment lost count and use Kalman Filter for prediction
        lost_count += 1
        x, y = kalman_predict_and_correct(kalman)
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