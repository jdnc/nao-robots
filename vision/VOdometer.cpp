#include "vision/VOdometer.h"
#define getImageTop() vblocks_.image->getImgTop()
#define getImageBottom() vblocks_.image->getImgBottom()
#define LOOK_BACK 7
#define MAX_POINTS 400
#define IMG_SIZE 640 * 480
#define THRESHOLD 
using namespace cv;
using namespace std;


VOdometer::VOdometer(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : DETECTOR_INITIALIZE, classifier_(classifier) {
  //TODO make it use the expr below
  //IMG_SIZE = iparams_.height * iparams_.width;
  // initialize prevImage to whatever its looking at now
  lastImageIndexTop = 0;
  lastImageIndexBottom = 0;
  cumlTurn = 0;
  validFeatures = 0;
  curTurn = 0; // is in radians
  cumDispX = 0;
  cumDispY = 0;
}

void VOdometer::calcOpticalFlow(){
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];
  //Mat outGray;
  vector<uchar> status;
  vector<float> angles;
  vector<float>xDisplacements;
  vector<float>yDisplacements;
  vector<float>xDisplacementsWithoutRot;
  vector<float>yDisplacementsWithoutRot;
  vector<Point2f>worldStart;
  vector<Point2f>worldEnd;
  vector<Point2f>worldEndWithoutRot;
  vector<float> err;
  float median_turn;
  TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.3);
  Size winSize = Size(3,3);
  Mat prevGray, curGray;
  getImage(curImage);        
  cvtColor(curImage, curGray, CV_BGR2GRAY);
  if (camera_ == Camera::TOP){
     // flow vectors in top camera for calculating rotation
     //cout << "hare Krishna"<< "lastImageIndexTop" << lastImageIndexTop<<endl;          
     if(lastImageIndexTop == LOOK_BACK){
       trackedTopFeatures.clear();
       trackedTopImages.clear();
       lastImageIndexTop = 0;
     }
     if(lastImageIndexTop == 0){
       // just do initialization -  no need for actual processing
       //cout << "Now here" << endl;
       prevGray =  curGray;
       //calculate per frame
       findFeaturesToTrack(prevGray, corners);
       lastImageIndexTop++;
       trackedTopFeatures.push_back(corners);
       trackedTopImages.push_back(curGray);
       return;
       }
       if(lastImageIndexTop >= 1){
         //corners = trackedTopFeatures.back();
         prevGray = trackedTopImages.back();
         findFeaturesToTrack(prevGray, corners);
         int id = tic();
         calcOpticalFlowPyrLK(prevGray, curGray, corners, outCorners,
                              status, err, winSize, 4, termcrit);
         //cout << "Time for PyrLK" << toc() << endl;
         int in = 0, out = 0;
         validFeatures = 0;
         tic();
         for(int i=0; i<outCorners.size(); i++){
         //cout << "out"<< out<<"in"<<in<<endl;
           out++;
           if (status[i] == 0){
            in++;
            corners[i] = outCorners[i] = Point2f(-1.0f, -1.0f);
           }
           else{
           validFeatures++;
           visionLog((8,"Index is %d start (%f , %f)end (%f, %f)",i, corners[i].x,corners[i].y,outCorners[i].x, outCorners[i].y));
           float ax = outCorners[i].x;
           float ay = outCorners[i].y;
           float bx = corners[i].x;
           float by = corners[i].y;
           //some confusion in signs
           float dx =  ax - bx;
           float dy = ay - by;
           visionLog((8,"dx %0.2f dy %0.2f ",dx, dy));
           float angle  =  abs(dx/iparams_.width * FOVx);
           angles.push_back(angle);
           }
        } 
        if (angles.size() > 80)  {
              sort( angles.begin(), angles.end());
              for(int i = 0; i< angles.size(); i++){
                visionLog((9, "index %d angle is %f deg\n",i, angles[i] * RAD_T_DEG ));
              }
              median_turn = angles[(int)(angles.size()/2)];   
              }
        else{
          median_turn = 0;
        }
        //cout<<"time for angle calculations" <<toc()<<endl;
        trackedTopImages.push_back(curGray);
        trackedTopFeatures.push_back(outCorners);
        lastImageIndexTop++;
        visionLog((7, "angle is %f deg and total angle is %f deg lastImageIndexTop %d",median_turn*180/3.14159, cumlTurn, lastImageIndexTop -1 ));
	//cout <<"angle"<<median_turn*180/3.14159<<"deg"<<endl;
	cumlTurn += median_turn*180/3.14159;
	//cout << "total angle change" << cumlTurn << endl;
       } 
     }
   else if (camera_ == Camera::BOTTOM){
      //flow vectors in bottom camera for calculating displacement
      //cout << "hare Krishna"<< "lastImageIndexBottom" << lastImageIndexBottom<<endl;          
     if(lastImageIndexBottom == LOOK_BACK){
       trackedBottomFeatures.clear();
       trackedBottomImages.clear();
       lastImageIndexBottom = 0;
     }
     if(lastImageIndexBottom == 0){
       // just do initialization -  no need for actual processing
       //cout << "Now here" << endl;
       prevGray =  curGray;
       //calculate per frame
       findFeaturesToTrack(prevGray, corners);
       lastImageIndexBottom++;
       trackedBottomFeatures.push_back(corners);
       trackedBottomImages.push_back(curGray);
       return;
       }
       if(lastImageIndexBottom >= 1){
         //corners = trackedTopFeatures.back();
         prevGray = trackedBottomImages.back();
         findFeaturesToTrack(prevGray, corners);
         calcOpticalFlowPyrLK(prevGray, curGray, corners, outCorners,
                            status, err, winSize, 4, termcrit);
         int in = 0, out = 0;
         validFeatures = 0;
         tic();
         for(int i=0; i<outCorners.size(); i++){
         //cout << "out"<< out<<"in"<<in<<endl;
           out++;
           if (status[i] == 0){
            in++;
            corners[i] = outCorners[i] = Point2f(-1.0f, -1.0f);
           }
           else{
             validFeatures++;
             visionLog((8,"Index is %d start (%f , %f)end (%f, %f)",i, corners[i].x,corners[i].y,outCorners[i].x, outCorners[i].y));
             // now remove rotational effects             
                   float r = curTurn;
                   float x = outCorners[i].x;
                   float y = outCorners[i].y;
                   Point2f out;
                   out.x = x*cos(r) - y * sin(r);
                   out.y = x*sin(r) + y * cos(r);                                            
             // obtain the two points on the ground plane
              Position p1 = cmatrix_.getWorldPosition((int)corners[i].x, (int)corners[i].y, 0);
              worldStart.push_back(Point2f(p1.x, p1.y));
              Position p2 = cmatrix_.getWorldPosition((int)outCorners[i].x, (int)outCorners[i].y, 0);
              worldEnd.push_back(Point2f(p2.x, p2.y));
              Position p3 = cmatrix_.getWorldPosition((int)out.x, (int)out.y, 0);  
              worldEndWithoutRot.push_back(Point2f(p3.x, p3.y));        
           }
        }
        //cout<<"time for disp calculations" <<toc()<<endl;
        //if(worldEnd.size() > 80)
        {
        // calculate net x and y displacements 
        for(int i = 0; i < worldEnd.size(); i++){
          float dx, dy;
          float dx_nr, dy_nr;
          dx =  (worldEnd[i].x - worldStart[i].x);
          dy =  (worldEnd[i].y - worldStart[i].y);
          dx_nr = (worldEndWithoutRot[i].x - worldStart[i].x);
          dy_nr = (worldEndWithoutRot[i].y - worldStart[i].y);
          xDisplacements.push_back(dx); 
          yDisplacements.push_back(dy);
          xDisplacementsWithoutRot.push_back(dx_nr);
          yDisplacementsWithoutRot.push_back(dy_nr);
          visionLog((10,"xdisp %f ydisp %f, rot removed xdisp %f ydisp %f", dx, dy, dx_nr, dy_nr));
        } 
        // print both the resultant displacements
        float net_x, net_y, net_x_nr, net_y_nr;
        net_x = getMedian(xDisplacements);
        net_y = getMedian(yDisplacements);
        ball->xDisp = net_x;
        ball->yDisp = net_y;
        net_x_nr = getMedian(xDisplacementsWithoutRot);
        net_y_nr = getMedian(yDisplacementsWithoutRot);
        //cout <<"Displacements "<<"x "<<net_x<<"y "<<net_y<<" no rot x "<<net_x_nr<<" y "<<net_y_nr << endl;
        visionLog((11,"xdisp %f ydisp %f, rot removed xdisp %f ydisp %f", net_x, net_y, net_x_nr, net_y_nr));
        trackedBottomImages.push_back(curGray);
        trackedBottomFeatures.push_back(outCorners);
        lastImageIndexBottom++;
        cumDispX += net_x_nr;
        cumDispY +=net_y_nr;
        int maxDispX = 0;
        int maxDispY = 0;
        for(int i=0; i<xDisplacements.size(); i++){
          /*if(maxDispX < xDisplacements[i])*/ maxDispX += xDisplacements[i];
          /*f(maxDispY < yDisplacements[i])*/ maxDispY += yDisplacements[i];
        }
        if(!(xDisplacements.empty() || yDisplacements.empty())){
          maxDispX /= xDisplacements.size();
          maxDispY /= yDisplacements.size();
          cout << "Displacement X "<< net_x<<" Y "<< net_y<<endl;
          ball->xDisp = net_x;
          ball->yDisp = net_y;
        }
      }
      
    } 
     
}
}

/* find the features to track */
void VOdometer::findFeaturesToTrack(Mat &grayImage, vector<Point2f>& foundFeatures, bool filter){
  TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.3);
  Size winSize = Size(3,3);
  int id;
  tic();
  goodFeaturesToTrack(grayImage, foundFeatures, MAX_POINTS, 0.005, 20);
  //cout << "Time for good features" << toc() << endl;
  tic();
  cornerSubPix(grayImage, foundFeatures, winSize, Size(-1,-1), termcrit);
  //cout << "Time for sub pixels" << toc() << endl;
  return;
}

/* determine if need o reset the found points*/
bool VOdometer::needReset(){
  if (validFeatures < 50) {
    return true;
  }
  return false;
}

/* TODO Change this very bad results */ 
float VOdometer::getIncAngle(){
  float ax, ay, bx, by, dx, dy;
  vector<float> angles;
  if (trackedTopFeatures.size() > 2){
    outCorners = trackedTopFeatures[trackedTopFeatures.size() - 1];
    corners = trackedTopFeatures[trackedTopFeatures.size() - 2]; 
    for(int i = 0; i < outCorners.size(); i++){
      if(corners[i] == Point2f(-1.0f, -1.0f) || outCorners[i] == Point2f(-1.0f, -1.0f)){
        continue;
      }
      ax = outCorners[i].x;
      ay = outCorners[i].y;
      bx = corners[i].x;
      by = corners[i].y;
      //some confusion in signs
      dx =  ax - bx;
      dy = ay - by;
      //visionLog((8,"dx %0.2f dy%0.2f ",i, dx, dy));
      float angle  =  dx/iparams_.width * FOVx;
      angles.push_back(angle);
    }
    if (angles.size() > 0) {
      for(int i = 0; i< angles.size(); i++){
        //visionLog((9, "index %d angle is %f deg\n",i, angles[i] * RAD_T_DEG ));
      }
      sort( angles.begin(), angles.end() );
      float median_turn = angles[(int)(angles.size()/2)];
      return median_turn;
    }
    // print out all the angels found DEBUGGING only
    
  }
  else
   return 0.0f;
}

/* fetches the images -  no changes in this required */
void VOdometer::getImage(Mat& image){
  if (vblocks_.image->loaded_)
  {
    unsigned char * rawImage;
    rawImage = new unsigned char[iparams_.rawSize];
    if(camera_ == Camera::TOP){
    memcpy(rawImage, vblocks_.image->getImgTop(), iparams_.rawSize);
    }
    else{
    memcpy(rawImage, vblocks_.image->getImgBottom(), iparams_.rawSize);
    }
    image = color::rawToMat(rawImage, iparams_);
    delete [] rawImage;
  }
  return;
}

float VOdometer::getMedian(vector<float> & v){
  if(v.size() <= 0)
    return 0.0f;
  sort(v.begin(), v.end());
  return v[(int)(v.size()/2)];  
}
