import numpy as np
import cv2
from face_tracker import face_tracker_pf

# setup initial location of window by using two corner points
corner_points = []
SET_ROI = False
PINK = (179, 102, 255)

def draw_point(frame, coords): 
    cv2.circle(frame, coords, radius=2, color=PINK, thickness=-1)

def draw_box(frame, corners):
    cv2.rectangle(frame, corners[0], corners[1], PINK, 3)

def mouse_drawing(event, x_temp, y_temp, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        global corner_points
        if not SET_ROI: 
            corner_points.append((x_temp, y_temp))

def get_state_ranges(cap): 
    if cap.isOpened(): 
        # ranges is [(upper_lim, lower_lim, standard_deviation_of_noise), ...]
        width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)   # float `width`
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)  # float `height`
        x_range = (0, width, 0.6)
        y_range = (0, height, 0.6)  #? 
        vx_range = (-width, 1.5 * width, 0.6)      #assume face can flash across the window in 1s
        vy_range = (-height, 1.5 * height, 0.6)
        hx_range = (0, width, 0.6)
        hy_range = (0, height, 0.6)
        at_dot_range = (0,2, 0.6)        #scale change
    return [x_range, y_range, vx_range, vy_range, hx_range, hy_range, at_dot_range]

def update_corner_points(corner_points, return_state): 
    center = np.array(return_state[:2]) 
    h_half = return_state[4]/2.0
    w_half = return_state[5]/2.0
    diagonal_half = np.array([h_half, w_half])
    corner_points[0] = tuple((center - diagonal_half).astype(np.int))
    corner_points[1] = tuple((center + diagonal_half).astype(np.int))

# set up video streaming
cv2.namedWindow("face_tracker")
cv2.setMouseCallback("face_tracker", mouse_drawing)
cap = cv2.VideoCapture(0)

# initialize face tracker
ranges = get_state_ranges(cap)
PARTICLE_NUM = 1000
SCALE_CHANGE_DISTURB = 0.0
VELOCITY_DISTURB = 70
FRAME_RATE = cap.get(cv2.CAP_PROP_FPS)
SIGMA_WEIGHT = 0.05      #quite important, too big will not distinuguish the right state, too small will make output weight far from zero.
SIGMA_CONTROL = 2.0     #should add enough randomness to the tracker
VALID_WEIGHT_LOWER_LIMIT = 0.0/PARTICLE_NUM # value should be decided based on total weight when target is lost. If set to zero, the tracker will never reset states randomly 

while True:
    rval, frame = cap.read()
    kernel = np.ones((5, 5), 'uint8')
    frame = cv2.erode(frame, kernel, iterations=1)
    frame = cv2.dilate(frame, kernel, iterations=1)

    # initialize ROI - we need two corner points
    if len(corner_points) < 2:  #for initialization
        for pt in corner_points: 
            draw_point(frame, pt)
    else: 
        if not SET_ROI: 
            SET_ROI = True
            tracker_input = {"ranges":ranges, "PARTICLE_NUM":PARTICLE_NUM, "SCALE_CHANGE_DISTURB":SCALE_CHANGE_DISTURB, "VELOCITY_DISTURB":VELOCITY_DISTURB, "FRAME_RATE":FRAME_RATE, "ROI_corner_points":corner_points, "SIGMA_WEIGHT" : SIGMA_WEIGHT, "SIGMA_CONTROL" : SIGMA_CONTROL, "VALID_WEIGHT_LOWER_LIMIT" : VALID_WEIGHT_LOWER_LIMIT, "initial_frame" : frame}
            tracker = face_tracker_pf(tracker_input)

        # after initializing ROI, run one iteration
        return_state = tracker.run_one_iteration(frame)
        update_corner_points(corner_points, return_state)
        print("state: ", return_state) 

        draw_box(frame, corner_points)

    cv2.imshow("face_tracker", frame)
    key = cv2.waitKey(20)  
    if key == 27: # exit on ESC
        break

