# ompl_rasp

Risk-Aware Shortest Path (ICAPS 2017 Submission 73)

## Dependencies

The RASP planner itself only requires OMPL and Boost. Running the provided examples requires OpenCV.

## Building and Running

To compile the library and the examples, run

    catkin build ompl_rasp

The main example can be run with

    ./devel/lib/ompl_rasp/vikings  # for the incremental RASP planner
    ./devel/lib/ompl_rasp/vikings --precompute true  # for the precomputation-based RASP planner

The examples will automatically pop up an OpenCV window that plots the path
found by the planner. Press any key to close the window.
