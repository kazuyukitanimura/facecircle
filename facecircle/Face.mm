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
  KF.statePre.at<float>(0) = 0;
  KF.statePre.at<float>(1) = 0;
  KF.statePre.at<float>(2) = 72; // TODO get a realistic initial width
  KF.statePre.at<float>(3) = 96; // TODO get a realistic initial height
  KF.statePre.at<float>(4) = 0;
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
      // Y = ( (  66 * R + 129 * G +  25 * B + 128) >> 8) +  16
      // U = ( ( -38 * R -  74 * G + 112 * B + 128) >> 8) + 128
      // V = ( ( 112 * R -  94 * G -  18 * B + 128) >> 8) + 128
      // R = clip(( 298 * C           + 409 * E + 128) >> 8)
      // G = clip(( 298 * C - 100 * D - 208 * E + 128) >> 8)
      // B = clip(( 298 * C + 516 * D           + 128) >> 8)
      uint32_t t = (i >> 1) * w + (j & -2); // TODO upconvert
      uint32_t y = yDataAddress[i * w + j];
      uint32_t u = uvDataAddress[t];
      uint32_t v = uvDataAddress[t + 1];
      uint32_t C = y - 16;
      uint32_t D = u - 128;
      uint32_t E = v - 128;
      uint32_t R = 0xFF & (( 298 * C           + 409 * E + 128) >> 8);
      uint32_t G = 0xFF & (( 298 * C - 100 * D - 208 * E + 128) >> 8);
      uint32_t B = 0xFF & (( 298 * C + 516 * D           + 128) >> 8);
      mat.data[(i + 1) * w - j - 1] = y /*+ (v >> 4) + (u >> 4) ;*/ - ((129 * G) >> 8); // remove R
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
  for(int y = 0; y < src.rows; y++) {
    for(int x = 0; x < src.cols; x++) {
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
  cv::GaussianBlur(im, tmp, cv::Size(5,5), 5);
  cv::addWeighted(im, 1.5, tmp, -0.5, 0, im);
}

- (UIImage *)processFace:(CMSampleBufferRef)sampleBuffer camera:(AVCaptureDevice*)device
{
  // create grayscale TODO: is it possible to process only updated pixels not the entire image?
  cv::Mat mat;
  [self convertYUVSampleBuffer:sampleBuffer toGrayscaleMat:mat];

  // detect faces
  std::vector<cv::Rect> faces;
  cascade.detectMultiScale(mat, faces, 1.1, 2, CV_HAAR_DO_CANNY_PRUNING | CV_HAAR_DO_ROUGH_SEARCH | CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(40, 40));

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
    measurement(2) = r.width;
    measurement(3) = newHeight;
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
    // tmpMat = mat(cv::Rect(r.x, newY, r.width, newHeight));

    // Exposure settings
    if (device.exposurePointOfInterestSupported) {
      NSError *error;
      if ([device lockForConfiguration:&error]) {
        cv::Point maxLoc;
        cv::minMaxLoc(mat(cv::Rect(r.x, r.y, r.width, r.height)), NULL, NULL, NULL, &maxLoc);
        device.exposurePointOfInterest = CGPointMake(maxLoc.x / mat.cols, maxLoc.y / mat.rows);
        device.exposureMode = AVCaptureExposureModeContinuousAutoExposure;
        [device unlockForConfiguration];
      }
    }
  }

  //cv::medianBlur(tmpMat, tmpMat, 9);
  //cv::GaussianBlur(tmpMat, tmpMat, cv::Size(3,3), 0);
  //cv::Mat matMat;
  //cv::bilateralFilter(tmpMat, matMat, 15, 80, 80);
  //cv::adaptiveBilateralFilter(tmpMat, matMat, cv::Size(3,3), 15);
  //cv::equalizeHist(tmpMat, tmpMat);
  //cv::Laplacian(tmpMat, tmpMat, CV_8UC1);
  //cv::Canny(tmpMat, tmpMat, 200, 180);
  //cv::Mat sobel_x, sobel_y;
  //cv::Sobel(tmpMat, sobel_x, CV_8UC1, 1, 0);
  //cv::convertScaleAbs(sobel_x, sobel_x);
  //cv::Sobel(tmpMat, sobel_y, CV_8UC1, 0, 1);
  //cv::convertScaleAbs(sobel_y, sobel_y);
  //cv::addWeighted(sobel_x, 0.5, sobel_y, 0.5, 0, tmpMat);
  //cv::Scharr(tmpMat, tmpMat, CV_8UC1, 1, 0);
  //cv::Mat tmpMat2;
  //cv::Sobel(tmpMat, tmpMat2, CV_8UC1, 1, 0);
  //cv::bitwise_not(tmpMat, tmpMat);
  //cv::adaptiveThreshold(tmpMat, tmpMat, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 13, 13);
  //cv::threshold(tmpMat, tmpMat, 127, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
  //cv::distanceTransform(tmpMat, tmpMat, CV_DIST_L2, 5);
  //unsharpMask(tmpMat);
  cv::Mat tmpMat2;
  sauvolaFast(tmpMat, tmpMat2, 15, 0.05, 100);
  int morph_size = 1;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size+1), cv::Point( morph_size, morph_size));
  cv::morphologyEx(tmpMat2, tmpMat2, cv::MORPH_OPEN, element);
  //cv::erode(tmpMat2, tmpMat2, element);
  //cv::dilate(tmpMat2, tmpMat2, element);
  cv::Point seedPoint = cv::Point(roi.width * 0.5, roi.height * 0.5);
  cv::ellipse(tmpMat2, seedPoint, cv::Size(roi.width * 0.20, roi.height * 0.20), 0, 0, 360, cv::Scalar( 255, 255, 255), -1); // create white spot
  cv::floodFill(tmpMat2, seedPoint, cv::Scalar(128,128,128));

  /*
  cv::MSER mser;
  cv::vector<cv::KeyPoint> mser_features;
  mser.detect(tmpMat, mser_features);
  for(int i=0;i<mser_features.size();i++){
    cv::circle(tmpMat , mser_features[i].pt, 1, cv::Scalar(0,0,255), 3);
  }
  */

  // TODO: LineSegmentDetector for opencv 3.0
  //std::vector<cv::Vec4i> lines;
  //cv::HoughLinesP(tmpMat2, lines, 1, CV_PI/180, 100, 5, 1);
  //for (size_t i = 0; i < lines.size(); i++) {
  //  cv::Vec4i line = lines[i];
  //  cv::line(tmpMat, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(128,128,128));
  //}

  /*
  cv::cvtColor(tmpMat, tmpMat, CV_8UC3);
  cv::Mat fgdModel;
  fgdModel.setTo(0);
  cv::Mat bgdModel;
  bgdModel.setTo(0);
  cv::grabCut(tmpMat, tmpMat, cv::Rect(previousROI), bgdModel, fgdModel, cv::GC_INIT_WITH_MASK);
  */

  // TODO: wrap with findContours()

  // flip the preview
  //cv::flip(tmpMat2, mat, 1);

  // convert mat to UIImage TODO: create my own MatToUIImage and add color
  return MatToUIImage(tmpMat2);
}

@end