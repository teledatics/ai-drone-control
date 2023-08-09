# ai-drone-control
AI drone &amp; rover control system code repository. It includes the following modules:

1. Row Detection Algorithm
    - **Description:** Implementation of crop row detection algorithm based on novel statistical techniques.
    - **Status:** Complete
2. Crop Field Segmentation Net
    - **Description:** Convolutional Neural Network for recognizing and segmenting farm crop fields in real-time.
    - **Status:** In Progress
3. Rover Path Planning
    - **Description:** Various sub-modules for creating optimal global rover path plan based on drone visual field. Includes following:
        1. Image to GPS Coordinate Converter
            - **Description:** Converts drone image pixel coordinates to GPS coordinates based on drone's current position and orientation in 3-D space on the globe.
            - **Status:** Complete
        2. Rover Path Mapper
            - **Description:** Processes image data, using above modules and CV techniques, to identify valid rover paths and obstacles between paths.
            - **Status:** Pending
        3. Path Planner
            - **Description:** Algorithms for calculating optimal rover patrol routes based on outputs from above modules. Implements multi-traveling salesman and A-star solutions.
            - **Status:** In Progress
4. Drone Control
    - **Description:** Real-time drone control system code.
    - **Status:** Pending
5. Rover Control
    - **Description:** Real-time rover control system code.
    - **Status:** Pending