//
//  face.mm
//  facecircle
//
//  Created by Kazuyuki Tanimura on 11/29/14.
//  Copyright (c) 2014 Kazuyuki Tanimura. All rights reserved.
//

#import "facecircle-Bridging-Header.h"


@interface Face()
{
  cv::CascadeClassifier cascade;
  cv::KalmanFilter KF;
}
@end

@implementation Face: NSObject

- (id)init
{
  self = [super init];

  // import cascading file
  NSBundle *bundle = [NSBundle mainBundle];
  NSString *path = [bundle pathForResource:@"haarcascade_frontalface_alt" ofType:@"xml"];
  std::string cascadeName = (char *)[path UTF8String];

  if (!cascade.load(cascadeName)) {
    return nil;
  }

  // Initialize Kalman Filter with
  // 4 dynamic parameters and 2 measurement parameters,
  // where my measurement is: 2D location of object,
  // and dynamic is: 2D location and 2D velocity.
  // http://stackoverflow.com/questions/18403918/opencv-kalman-filter-prediction-without-new-observtion
  // http://opencvexamples.blogspot.com/2014/01/kalman-filter-implementation-tracking.html#.VIRDi6TF95w
  // TODO add accelerations later for smoother stabilization
  // http://stackoverflow.com/questions/17836267/kalmanfilter6-2-0-transition-matrix
  KF.init(10, 5, 0);
  KF.transitionMatrix = *(cv::Mat_<float>(10, 10) <<
                          1,0,0,0,0,1,0,0,0,0,
                          0,1,0,0,0,0,1,0,0,0,
                          0,0,1,0,0,0,0,1,0,0,
                          0,0,0,1,0,0,0,0,1,0,
                          0,0,0,0,1,0,0,0,0,1,
                          0,0,0,0,0,1,0,0,0,0,
                          0,0,0,0,0,0,1,0,0,0,
                          0,0,0,0,0,0,0,1,0,0,
                          0,0,0,0,0,0,0,0,1,0,
                          0,0,0,0,0,0,0,0,0,1);
  KF.statePre.at<float>(0) = 0; // x
  KF.statePre.at<float>(1) = 0; // y
  KF.statePre.at<float>(2) = 72; // TODO get a realistic initial width
  KF.statePre.at<float>(3) = 96; // TODO get a realistic initial height
  KF.statePre.at<float>(4) = 0; // offsetY
  KF.statePre.at<float>(5) = 0;
  KF.statePre.at<float>(6) = 0;
  KF.statePre.at<float>(7) = 0;
  KF.statePre.at<float>(8) = 0;
  KF.statePre.at<float>(9) = 0;
  setIdentity(KF.measurementMatrix);
  setIdentity(KF.processNoiseCov, cv::Scalar::all(0.005)); //adjust this for faster convergence - but higher noise
  setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));
  setIdentity(KF.errorCovPost, cv::Scalar::all(.1));

  return self;
}

- (void)convertYUVSampleBuffer:(CMSampleBufferRef)sampleBuffer toGrayscaleMat:(cv::Mat &)mat
{
  // http://mkonrad.net/2014/06/24/cvvideocamera-vs-native-ios-camera-apis.html
  CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);

  // lock the buffer
  CVPixelBufferLockBaseAddress(imageBuffer, 0);

  // get the address to the image data
  void *dataAddress = CVPixelBufferGetBaseAddress(imageBuffer);

  // get image properties
  int w = (int)CVPixelBufferGetWidth(imageBuffer);
  int h = (int)CVPixelBufferGetHeight(imageBuffer);

  // create the cv mat
  mat = cv::Mat(h, w, CV_8UC4, dataAddress, CVPixelBufferGetBytesPerRow(imageBuffer));


  // unlock again
  CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
}

// http://stackoverflow.com/questions/19068085/shift-image-content-with-opencv
- (void)shiftImage:(cv::Mat &)mat x:(int)offsetx y:(int)offsety
{
  cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
  cv::warpAffine(mat,mat,trans_mat,mat.size());
}

- (UIImage *)processFace:(CMSampleBufferRef)sampleBuffer camera:(AVCaptureDevice*)device
{
  // create grayscale TODO: is it possible to process only updated pixels not the entire image?
  cv::Mat mat;
  [self convertYUVSampleBuffer:sampleBuffer toGrayscaleMat:mat];
  cv::Mat greyMat;
  cv::cvtColor(mat, greyMat, CV_BGR2GRAY);

  // detect faces
  std::vector<cv::Rect> faces;
  cascade.detectMultiScale(greyMat, faces, 1.1, 2, CV_HAAR_DO_CANNY_PRUNING | CV_HAAR_DO_ROUGH_SEARCH | CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(40, 40));

  cv::Mat tmpMat = mat;
  cv::Rect roi = cv::Rect(0, 0, mat.cols, mat.rows);

  if (faces.size() > 0) {
    cv::Rect r = faces[0];

    // calculate params
    int offsetY = (mat.rows - r.height) * 0.5 - r.y;
    int newHeight = r.width * 2;
    int newY = 0;
    if (mat.rows < newHeight) {
      newHeight = mat.rows;
    } else {
      newY = (mat.rows - newHeight) * 0.5;
    }

    // Kalman filter predict, to update the internal statePre variable
    KF.predict();
    // Kalman measure
    cv::Mat_<float> measurement(5,1);
    measurement(0) = r.x;
    measurement(1) = newY;
    measurement(2) = MIN(r.width, mat.cols - r.x);
    measurement(3) = MIN(newHeight, mat.rows - newY);
    measurement(4) = offsetY;
    // Kalman estimate
    cv::Mat estimated = KF.correct(measurement);

    // vertical shift
    [self shiftImage:mat x:0 y:estimated.at<float>(4)];

    // crop
    // http://www.plosone.org/article/info%3Adoi%2F10.1371%2Fjournal.pone.0093369
    roi.x = MAX(estimated.at<float>(0), 0);
    roi.y = MAX(estimated.at<float>(1), 0);
    roi.width = MAX(MIN(estimated.at<float>(2), mat.cols - roi.x), 20);
    roi.height = MAX(MIN(estimated.at<float>(3), mat.rows - roi.y), 20);
    tmpMat = mat(roi);

    // http://stackoverflow.com/questions/17698431/extracting-background-image-using-grabcut
    cv::Mat maskMat = cv::Mat::ones(roi.size(), CV_8U) * cv::GC_PR_BGD;
    cv::Point seedPoint = cv::Point(roi.width * 0.5, roi.height * 0.5);
    cv::ellipse(maskMat, seedPoint, cv::Size(roi.width * 0.20, roi.height * 0.20), 0, 0, 360, cv::Scalar(cv::GC_FGD), -1);
    cv::Mat bgModel,fgModel;
    cv::cvtColor(tmpMat, tmpMat, CV_BGRA2BGR);
    int border = MIN(MIN(20, roi.width/2), roi.height/2);
    cv::Rect rect = cv::Rect(border,border,roi.width-border*2,roi.height-border*2);
    cv::grabCut(tmpMat, maskMat, rect, bgModel, fgModel, 1, cv::GC_INIT_WITH_MASK);
    cv::Mat foreground(tmpMat.size(),CV_8UC3,cv::Scalar(0,0,0));
    cv::Mat maskMat2;
    cv::compare(maskMat,cv::GC_FGD,maskMat2,cv::CMP_EQ);
    tmpMat.copyTo(foreground, maskMat2); // bg pixels not copied
    cv::compare(maskMat,cv::GC_PR_FGD,maskMat2,cv::CMP_EQ);
    tmpMat.copyTo(foreground, maskMat2); // bg pixels not copied
    tmpMat = foreground;
  }

  // flip the preview
  cv::flip(tmpMat, tmpMat, 1);

  // convert mat to UIImage
  cv::cvtColor(tmpMat, tmpMat, CV_BGR2RGB);
  return MatToUIImage(tmpMat);
}

@end