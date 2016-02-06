Vector<KeyPoint>left_keypoints,right_keypoints; 
//declare variable which is left and right keypoints

// Detect keypoints in the left and right images
FastFeatureDetectorffd; #cv2.FastFeatureDetector_create()
ffd.detect(img1, left_keypoints); 
ffd.detect(img2, right_keypoints);
//find features in each image

vector<Point2f>left_points; //declare the left points thing
KeyPointsToPoints(left_keypoints,left_points); 
//transform left points

vector<Point2f>right_points(left_points.size()); 
//declare an empty rightpoints vector which is the size of the left points vector

// making sure images are grayscale  #now convert everything to grayscale
Mat prevgray,gray;
if (img1.channels() == 3) {
 cvtColor(img1,prevgray,CV_RGB2GRAY);
 cvtColor(img2,gray,CV_RGB2GRAY);
} else {
 prevgray = img1;
  gray = img2;
}


// Calculate the optical flow field:
// how each left_point moved across the 2 images
vector<uchar>vstatus; vector<float>verror; //status and error vectors
calcOpticalFlowPyrLK(prevgray, gray, left_points, right_points,
vstatus, verror);
//the two images and the left points, populates the right points vector, status and error are also assigned

// First, filter out the points with high error

vector<Point2f>right_points_to_find; 
//vector of points, the right points to find in the left points thing

vector<int>right_points_to_find_back_index; 
//index of right points

for (unsigned inti=0; i<vstatus.size(); i++) { //for the number of values
 if (vstatus[i] &&verror[i] < 12.0) { //if status is good and error is less than 12 (arbitrary?)
 // Keep the original index of the point in the
 // optical flow array, for future use
 right_points_to_find_back_index.push_back(i); //append index of point
 // Keep the feature point itself
right_points_to_find.push_back(j_pts[i]); //append point
} else {
 vstatus[i] = 0; // a bad flow, just ignore yo
}
}


// for each right_point see which detected feature it belongs to
Mat right_points_to_find_flat = Mat(right_points_to_find). //matrix conversion
reshape(1,to_find.size()); //flatten array //reshape matrix
vector<Point2f>right_features; // detected features //new matrix
KeyPointsToPoints(right_keypoints,right_features); //convert the right keypoints to points now
Mat right_features_flat = Mat(right_features).reshape(1,right_
features.size());


// Look around each OF point in the right image
// for any features that were detected in its area
// and make a match.
BFMatchermatcher(CV_L2); //choose the brute force default
vector<vector<DMatch>>nearest_neighbors; //new vector
matcher.radiusMatch(right_points_to_find_flat,right_features_flat,nearest_neighbors, 2.0f); //assign the new vector



// Check that the found neighbors are unique (throw away neighbors
// that are too close together, as they may be confusing)
std::set<int>found_in_right_points; // for duplicate prevention

for(inti=0;i<nearest_neighbors.size();i++) { //for the length of neighbors
DMatch _m;
if(nearest_neighbors[i].size()==1) {
 _m = nearest_neighbors[i][0]; // only one neighbor
} else if(nearest_neighbors[i].size()>1) {
 // 2 neighbors – check how close they are
 double ratio = nearest_neighbors[i][0].distance /
 nearest_neighbors[i][1].distance;
if(ratio < 0.7) { // not too close
 // take the closest (first) one
 _m = nearest_neighbors[i][0];
} else { // too close – we cannot tell which is better
 continue; // did not pass ratio test – throw away
}
} else {
 continue; // no neighbors... :(
}



// prevent duplicates
if (found_in_right_points.find(_m.trainIdx) == found_in_right_points.
end()) {
 // The found neighbor was not yet used:
 // We should match it with the original indexing
 // ofthe left point
 _m.queryIdx = right_points_to_find_back_index[_m.queryIdx];
 matches->push_back(_m); // add this match
 found_in_right_points.insert(_m.trainIdx);
 }
}
cout<<"pruned "<< matches->size() <<" / "<<nearest_neighbors.size()
<<" matches"<<endl;
