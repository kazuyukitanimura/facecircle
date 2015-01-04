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
  cv::Mat previousMask;
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  cv::Ptr<cv::DescriptorMatcher > matcher;
  cv::Mat previousDescriptors;
  std::vector<cv::KeyPoint> previousKeypoints;
  int shiftx, shifty;
}
@end

@implementation Face: NSObject

- (id)init
{
  self = [super init];

  // import cascading file
  NSBundle *bundle = [NSBundle mainBundle];
  NSString *path = [bundle pathForResource:@"lbpcascade_frontalface" ofType:@"xml"];
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

  previousMask.create(1, 1, CV_8UC1);
  detector = new cv::OrbFeatureDetector();
  extractor = new cv::OrbDescriptorExtractor();
  matcher = new cv::BFMatcher(cv::NORM_HAMMING, false);
  shiftx = 0;
  shifty = 0;

  return self;
}

- (void)convertYUVSampleBuffer:(CMSampleBufferRef)sampleBuffer toGrayscaleMat:(cv::Mat &)mat
{
  // http://mkonrad.net/2014/06/24/cvvideocamera-vs-native-ios-camera-apis.html
  CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);

  // lock the buffer
  CVPixelBufferLockBaseAddress(imageBuffer, 0);

  // get the address to the image data
  uint8_t *yDataAddress = (uint8_t *) CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 0);
  uint8_t *uvDataAddress = (uint8_t *) CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 1);

  // get image properties
  int w = (int)CVPixelBufferGetWidth(imageBuffer);
  int h = (int)CVPixelBufferGetHeight(imageBuffer);

  // create the cv mat
  // 8 bit unsigned chars for grayscale data
  // the first plane contains the grayscale data
  // therefore we use <imgBufAddr> as source
  // http://en.wikipedia.org/wiki/YUV
  // http://kentaroid.com/kcvpixelformattype%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6%E3%81%AE%E8%80%83%E5%AF%9F/
  // http://stackoverflow.com/questions/8476821/repeated-scene-items-in-ios-yuv-video-capturing-output
  // http://msdn.microsoft.com/en-us/library/windows/desktop/dd206750(v=vs.85).aspx
  mat.create(h, w, CV_8UC1);
  for (uint32_t i = 0; i < h; i++) {
    for (uint32_t j = 0; j < w; j++) {
      uint32_t t = (i >> 1) * w + (j & -2); // TODO upconvert
      uint32_t y = yDataAddress[i * w + j]; // 16 - 235
      uint32_t u = uvDataAddress[t]; // 16 -240
      uint32_t v = uvDataAddress[t + 1]; // 16 -240
      mat.data[(i + 1) * w - j - 1] = y;
    }
  }

  // unlock again
  CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
}

// http://stackoverflow.com/questions/19068085/shift-image-content-with-opencv
- (void)shiftImage:(cv::Mat &)mat x:(int)offsetx y:(int)offsety
{
  cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
  cv::warpAffine(mat,mat,trans_mat,mat.size());
}
- (void)tilt:(cv::Mat &)src toMat:(cv::Mat &)dst orientation:(UIInterfaceOrientation)orientation
{
  // perspective transform
  cv::Point2f srcQuad[4], dstQuad[4];
  // before transform
  int tiltY = 0;
  int tiltX = 0;
  if (orientation == UIInterfaceOrientationPortrait) {
    tiltY = 10;
  } else if (orientation == UIInterfaceOrientationPortraitUpsideDown) {
    tiltY = -10;
  } else if (orientation == UIInterfaceOrientationLandscapeLeft) {
    tiltX = -6;
  } else if (orientation == UIInterfaceOrientationLandscapeRight) {
    tiltX = 6;
  }
  srcQuad[0] = cv::Point2f(-tiltY, -tiltX); // upper left
  srcQuad[1] = cv::Point2f(src.cols + tiltY, tiltX); // upper right
  srcQuad[2] = cv::Point2f(src.cols - tiltY, src.rows - tiltX); // lower right
  srcQuad[3] = cv::Point2f(tiltY, src.rows + tiltX); // lower left
  // after trasnform
  tiltX = abs(tiltX);
  dstQuad[0] = cv::Point2f(2 * tiltX, 2 * tiltY); // upper left
  dstQuad[1] = cv::Point2f(src.cols - 2 * tiltX, 2 * tiltY); // upper right
  dstQuad[2] = cv::Point2f(src.cols - 2 * tiltX, src.rows - 2 * tiltY); // lower right
  dstQuad[3] = cv::Point2f(2 * tiltX, src.rows - 2 * tiltY); // lower left
  // transform
  cv::Mat warp_matrix = cv::getPerspectiveTransform(srcQuad, dstQuad);
  cv::warpPerspective(src, dst, warp_matrix, src.size());
}

- (void)connectDots:(cv::Mat &)mat
{
  cv::Mat tmpMat(mat.size(), mat.type());
  // connect if 0-255-0 sequence is found
  for (uint32_t y = 1; y < mat.rows-1; y++) {
    for (uint32_t x = 1; x < mat.cols-1; x++) {
      // 0 1 2
      // 7   3
      // 6 5 4
      uchar dot0 = mat.at<uchar>(y-1, x-1);
      uchar dot1 = mat.at<uchar>(y-1, x);
      uchar dot2 = mat.at<uchar>(y-1, x+1);
      uchar dot3 = mat.at<uchar>(y, x+1);
      uchar dot4 = mat.at<uchar>(y+1, x+1);
      uchar dot5 = mat.at<uchar>(y+1, x);
      uchar dot6 = mat.at<uchar>(y+1, x-1);
      uchar dot7 = mat.at<uchar>(y, x-1);
      tmpMat.at<uchar>(y, x) = mat.at<uchar>(y, x)
      & (dot1 | dot5)
      & (dot3 | dot7)
      & (dot0 | dot4)
      & (dot2 | dot6)
      & (dot1 | dot4)
      & (dot1 | dot6)
      & (dot0 | dot5)
      & (dot2 | dot5)
      & (dot0 | dot3)
      & (dot3 | dot6)
      & (dot2 | dot7)
      & (dot4 | dot7)
      //& (dot1 | dot3 | dot5 | dot7)
      //& (dot0 | dot7 | dot2 | dot3)
      //& (dot6 | dot7 | dot4 | dot3)
      //& (dot0 | dot5 | dot1 | dot6)
      //& (dot2 | dot1 | dot5 | dot4)
      ;
    }
  }
  // connect if vertical 0-255-255-0 sequence is found
  if (tmpMat.rows > 3) {
    for (uint32_t y = 1; y < tmpMat.rows-2; y+=2) {
      for (uint32_t x = 0; x < tmpMat.cols; x++) {
        uchar dot0 = tmpMat.at<uchar>(y-1, x);
        uchar dot1 = tmpMat.at<uchar>(y, x);
        uchar dot2 = tmpMat.at<uchar>(y+1, x);
        uchar dot3 = tmpMat.at<uchar>(y+2, x);
        mat.at<uchar>(y, x) = dot1 & (dot0 | dot3);
        mat.at<uchar>(y+1, x) = dot2 & (dot0 | dot3);
      }
    }
  }
  // connect if horizonatal 0-255-255-0 sequence is found
  if (tmpMat.cols > 3) {
    for (uint32_t x = 1; x < tmpMat.cols-2; x+=2) {
      for (uint32_t y = 0; y < tmpMat.rows; y++) {
        uchar dot0 = tmpMat.at<uchar>(y, x-1);
        uchar dot3 = tmpMat.at<uchar>(y, x+2);
        mat.at<uchar>(y, x) &= (dot0 | dot3);
        mat.at<uchar>(y, x+1) &= (dot0 | dot3);
      }
    }
  }
}

// http://schima.hatenablog.com/entry/2013/10/25/202418
void sauvolaFast(const cv::Mat &src, cv::Mat &dst, int kernelSize, double k, double r)
{
  dst.create(src.size(), src.type());

  cv::Mat srcWithBorder;
  int borderSize = kernelSize / 2 + 1;
  int kernelPixels = kernelSize * kernelSize;
  cv::copyMakeBorder(src, srcWithBorder, borderSize, borderSize, borderSize, borderSize, cv::BORDER_REPLICATE);

  cv::Mat sum, sqSum;
  cv::integral(srcWithBorder, sum, sqSum);
  for (int y = 0; y < src.rows; y++) {
    for (int x = 0; x < src.cols; x++) {
      int kx = x + kernelSize;
      int ky = y + kernelSize;
      double sumVal = sum.at<int>(ky, kx) - sum.at<int>(ky, x) - sum.at<int>(y, kx) + sum.at<int>(y, x);
      double sqSumVal = sqSum.at<double>(ky, kx) - sqSum.at<double>(ky, x) - sqSum.at<double>(y, kx) + sqSum.at<double>(y, x);

      double mean = sumVal / kernelPixels;
      double var = (sqSumVal / kernelPixels) - (mean * mean);
      if (var < 0.0) {
        var = 0.0;
      }
      double stddev = sqrt(var);
      // original
      double threshold = mean * (1 + k * (stddev / r - 1));
      //Phansalkar
      //double p = 2.0;
      //double q = 10.0;
      //double threshold = mean * (1 + std::pow(p, -q * mean) + k * (stddev / r - 1));
      // http://research.ijcaonline.org/volume51/number6/pxc3881362.pdf
      //double minVal;
      //double maxVal;
      //cv::minMaxLoc(srcWithBorder(cv::Rect(x, y, kernelSize, kernelSize)), &minVal, &maxVal);
      //double threshold = 0.95 * (mean + (maxVal - minVal)/(1 - src.at<uchar>(y, x)));

      if (src.at<uchar>(y, x) < threshold) {
        dst.at<uchar>(y, x) = 0;
      } else {
        dst.at<uchar>(y, x) = 255;
      }
    }
  }
}

// https://opencv-code.com/quick-tips/sharpen-image-with-unsharp-mask/
// Perform in-place unsharp masking operation
void unsharpMask(cv::Mat& im)
{
  cv::Mat tmp;
  cv::GaussianBlur(im, tmp, cv::Size(5, 5), 5);
  cv::addWeighted(im, 1.5, tmp, -0.5, 0, im);
}

- (UIImage *)processFace:(CMSampleBufferRef)sampleBuffer camera:(AVCaptureDevice*)device orientation:(UIInterfaceOrientation)orientation
{
  // create grayscale TODO: is it possible to process only updated pixels not the entire image?
  cv::Mat mat;
  [self convertYUVSampleBuffer:sampleBuffer toGrayscaleMat:mat];

  // detect faces
  std::vector<cv::Rect> faces;
  cascade.detectMultiScale(mat, faces, 1.1, 2, CV_HAAR_DO_CANNY_PRUNING | CV_HAAR_DO_ROUGH_SEARCH | CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(40, 40));

  cv::Mat tmpMat = mat;
  cv::Rect roi = cv::Rect(0, 0, mat.cols, mat.rows);

  if (faces.size() == 0) {
    shiftx = 0;
    shifty = 0;
    return nil;
  }
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
  cv::Mat_<float> measurement(5, 1);
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
  roi.width = MAX(MIN(estimated.at<float>(2), mat.cols - roi.x), 1);
  roi.height = MAX(MIN(estimated.at<float>(3), mat.rows - roi.y), 1);
  tmpMat = mat(roi);

  if (tmpMat.size().area() > 10) {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<cv::DMatch> matches;
    detector->detect(tmpMat, keypoints);
    extractor->compute(tmpMat, keypoints, descriptors);
    if (!descriptors.empty() && !keypoints.empty() && !previousDescriptors.empty() && !previousKeypoints.empty()) {
      matcher->match(previousDescriptors, descriptors, matches);
      if (matches.size() >= 4) {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        for (uint32_t i = 0; i < matches.size(); i++) {
          srcPoints.push_back(previousKeypoints[matches[i].queryIdx].pt);
          dstPoints.push_back(keypoints[matches[i].trainIdx].pt);
        }
        cv::Mat H = cv::findHomography(srcPoints, dstPoints, CV_RANSAC);
        //std::cout << "H = "<< std::endl << " "  << H << std::endl << std::endl;
        int dx = H.at<double>(0, 2);
        int dy = H.at<double>(1, 2);
        if (abs(dx) < 5 && abs(dy) < 5) {
          shiftx -= dx;
          shifty -= dy;
        } else {
          shiftx *= 0.5;
          shifty *= 0.5;
        }
        //[self shiftImage:tmpMat x:shiftx y:shifty];
      }
    }
    descriptors.copyTo(previousDescriptors);
    previousKeypoints = keypoints;
  }

  // Exposure settings
  // TODO change this to observer for device.adjustingExposure
  // http://cocoadays.blogspot.com/2011
  if (device.exposurePointOfInterestSupported && !device.adjustingExposure) {
    NSError *error;
    if ([device lockForConfiguration:&error]) {
      cv::Point maxLoc;
      cv::Mat blurMat;
      cv::GaussianBlur(tmpMat, blurMat, cv::Size(15, 15), 0);
      cv::minMaxLoc(blurMat, NULL, NULL, NULL, &maxLoc);
      device.exposurePointOfInterest = CGPointMake((maxLoc.x + roi.x) / mat.cols, (maxLoc.y + roi.y - estimated.at<float>(4)) / mat.rows);
      device.exposureMode = AVCaptureExposureModeContinuousAutoExposure;
      //device.exposureMode = AVCaptureExposureModeLocked;
      //device.exposureMode = AVCaptureExposureModeAutoExpose;
      [device unlockForConfiguration];
    }
  }

  // adaptive histogram equalization
  // http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_histograms/py_histogram_equalization/py_histogram_equalization.html
  cv::createCLAHE(4.0, cv::Size(4, 4))->apply(tmpMat, tmpMat);

  cv::Mat tmpMat2, tmpMat3;
  //cv::medianBlur(tmpMat, tmpMat, 9);
  //cv::bilateralFilter(tmpMat, tmpMat2, 3, 8, 8);
  //cv::adaptiveBilateralFilter(tmpMat, tmpMat2, cv::Size(3,3), 4, 4);
  //tmpMat2.copyTo(tmpMat);
  //cv::equalizeHist(tmpMat, tmpMat);
  //cv::Laplacian(tmpMat, tmpMat2, CV_8UC1);
  //cv::cvtColor(tmpMat, tmpMat, CV_GRAY2BGR);
  //cv::pyrMeanShiftFiltering(tmpMat, tmpMat4, 15, 40);
  //double threshold = cv::threshold(tmpMat, tmpMat3, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU) * 1.5;
  //cv::Mat sobel_x, sobel_y;
  //cv::Sobel(tmpMat, sobel_x, CV_8UC1, 1, 0);
  //cv::convertScaleAbs(sobel_x, sobel_x);
  //cv::Sobel(tmpMat, sobel_y, CV_8UC1, 0, 1);
  //cv::convertScaleAbs(sobel_y, sobel_y);
  //cv::addWeighted(sobel_x, 0.5, sobel_y, 0.5, 0, tmpMat);
  //cv::Scharr(tmpMat, tmpMat, CV_8UC1, 1, 0);
  //cv::Mat tmpMat2;
  //cv::Sobel(tmpMat, tmpMat2, CV_8UC1, 1, 0);
  //cv::adaptiveThreshold(tmpMat, tmpMat, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 13, 13);
  //cv::threshold(tmpMat, tmpMat, 127, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
  //cv::threshold(tmpMat2, tmpMat3, 200, 255, CV_THRESH_BINARY_INV);
  //cv::bitwise_or(tmpMat2, tmpMat3, tmpMat2);
  //cv::distanceTransform(tmpMat, tmpMat, CV_DIST_L2, 5);
  //unsharpMask(tmpMat);
  //sauvolaFast(tmpMat, tmpMat4, 15, 0.05, 100);
  //cv::bitwise_and(tmpMat4, tmpMat3, tmpMat2);

  int morph_size = 3;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));

  // noise reduction
  cv::GaussianBlur(tmpMat, tmpMat2, cv::Size(3,3), 0);

  // edge detection
  double minVal, maxVal;
  cv::minMaxLoc(tmpMat2(cv::Rect(roi.width * 0.3, roi.height * 0.3, roi.width * 0.4, roi.height * 0.4)), &minVal, &maxVal);
  double threshold = maxVal - minVal;
  cv::Canny(tmpMat2, tmpMat3, threshold * 0.7, threshold, 3, true);
  cv::bitwise_not(tmpMat3, tmpMat2);

  // connect broken edges
  [self connectDots:tmpMat2];
  cv::morphologyEx(tmpMat2, tmpMat2, cv::MORPH_OPEN, cv::Mat());

  // create the mask
  cv::Point seedPoint = cv::Point(roi.width * 0.5, roi.height * 0.5);
  cv::ellipse(tmpMat2, seedPoint, cv::Size(roi.width * 0.22, roi.height * 0.22), 0, 0, 360, cv::Scalar(255, 255, 255), CV_FILLED);
  cv::floodFill(tmpMat2, seedPoint, cv::Scalar(128, 128, 128));
  cv::compare(tmpMat2, cv::Scalar(128, 128, 128), tmpMat3, cv::CMP_EQ);
  cv::morphologyEx(tmpMat3, tmpMat3, cv::MORPH_CLOSE, element);
  //cv::erode(tmpMat2, tmpMat2, element);
  //cv::dilate(tmpMat2, tmpMat2, element);
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(tmpMat3, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  drawContours(tmpMat2, contours, -1, cv::Scalar(128, 128, 128), CV_FILLED);
  cv::compare(tmpMat2, cv::Scalar(128, 128, 128), tmpMat3, cv::CMP_EQ);
  cv::morphologyEx(tmpMat3, tmpMat3, cv::MORPH_CLOSE, element);
  tmpMat2.setTo(cv::Scalar(255, 255, 255));

  // stabilize the mask
  cv::resize(previousMask, previousMask, tmpMat3.size(), cv::INTER_LANCZOS4);
  cv::addWeighted(tmpMat3, 0.25, previousMask, 0.75, 0, previousMask);
  cv::threshold(previousMask, tmpMat3, 224, 255, CV_THRESH_BINARY);

  cv::dilate(tmpMat3, tmpMat3, cv::Mat()); // make the mask slightly larger

  // apply the mask
  tmpMat.copyTo(tmpMat2, tmpMat3);

  // flip the preview
  //cv::flip(tmpMat2, mat, 1);

  [self tilt:tmpMat2 toMat:tmpMat3 orientation:orientation];

  // convert mat to UIImage TODO: create my own MatToUIImage and add color
  return MatToUIImage(tmpMat3);
}

@end