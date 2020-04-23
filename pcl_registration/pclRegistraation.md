**Steps used in PCL registration algorithms**
http://pointclouds.org/documentation/tutorials/registration\_api.php#registration-api

 - identify the interest points aka keypoints that best represents in both datasets
 - at each keypoint compute a feature descriptor
 - from the set of feature descriptors together with their XYZ positions in both the datasets and estimate a set of correspondences based on the
   similarities between features and positions
 - reject the bad correspondence
 - from the obtained good correspondences estimate motion transformation
