## Visionary_S_AP_BoxMeasurement
To measure the width and length of boxes.
### Description
The app uses the Shape3D.fitPlane() with LEASTSQUARES and RANSAC in combination

  Following steps are done for each image to calculate the dimensions of the box:
    1. Find connected flat regions in the image
    2. From the flat regions, retain only the ones which contain the center pixel - called box region
    3. Widen the box region by dilation and calculate its edges
    4. Find the largest connected region from the edge image
    5. Dilate and fill the holes in the identified region, get its borders
    6. In the point cloud, only retain the points that lie within the identified border and within the
       working range
    7. Rotate the point cloud to find the rotation angle where the point cloud is axis aligned
    8. From the point cloud of the box, find its bounding box and calculate the dimensions
### How to run
Start by running the app (F5) or debugging (F7+F10).
Set a breakpoint on the first row inside the main function to debug step-by-step.
See the results in the viewer on the DevicePage.
### Topics
View, Visionary, Stereo, Box, Measure, SICK-AppSpace