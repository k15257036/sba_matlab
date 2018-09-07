# sba_matlab
matlab edition of sparse bundle adjustment

This matlab code can be used in the situation like following:

1.You used two cameras which have been calibrated to take picture of an object, and got the feature points' 2d coordinates in the image.

2.And then you calulate the feature points' 3d coordinates according to the triangulation principle(Such as stereo_triangulation.m in matlab calibration toolbox.
Download address:http://www.vision.caltech.edu/bouguetj/calib_doc/download/index.html). 

3.Howerver, the 3d coordinates you got are just in a local coordinate system. So you take some point cloud registration and stitching methods to make them in a same global coordinate system.

4.There often exists stitching error which you want to reduce. An efficient way is bundle adjustment, or sparse bundle adjustment in this situation.

5.Usually, people take reprojection point as [x;y] and reduce reprojection error in a single image. In the code provided, you can reduce the reprojection error in both left and right image because we take reprojection point as [x_left;y_left;x_right;y_right].
 
 More details can be seen in main.m and bundle_adjustment.m
 
 
 Some data and example code are provided for testing. 
 If you have any question or suggestion, please feel free to send an email to tracytrunks@163.com
