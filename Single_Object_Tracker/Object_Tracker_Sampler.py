import cv2

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

tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
tracker_type = tracker_types[0] # SELECT TRACKER HERE

if tracker_type == 'BOOSTING':
    tracker = cv2.legacy.TrackerBoosting_create()
if tracker_type == 'MIL':
    tracker = cv2.TrackerMIL_create() 
if tracker_type == 'KCF':
    tracker = cv2.TrackerKCF_create() 
if tracker_type == 'TLD':
    tracker = cv2.legacy.TrackerTLD_create() 
if tracker_type == 'MEDIANFLOW':
    tracker = cv2.legacy.TrackerMedianFlow_create() 
if tracker_type == 'GOTURN':
    tracker = cv2.TrackerGOTURN_create()
if tracker_type == 'MOSSE':
    tracker = cv2.legacy.TrackerMOSSE_create()
if tracker_type == "CSRT":
    tracker = cv2.TrackerCSRT_create()

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
bbox = min(ix, ex), min(iy, ey), abs(ex-ix), abs(ey-iy)
ret = tracker.init(frame, bbox)
# Start tracking
while True:
    ret, frame = cap.read()
    if not ret:
        print('something went wrong')
        break
    timer = cv2.getTickCount()
    ret, bbox = tracker.update(frame)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    if ret:
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    else:
        cv2.putText(frame, "Tracking failure detected", (100,80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    cv2.putText(frame, tracker_type + " Tracker", (100,20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
    cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
    cv2.imshow("Tracking", frame)
    
    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()