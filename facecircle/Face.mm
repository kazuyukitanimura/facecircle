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
  cv::Rect previousROI;
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

  previousROI = cv::Rect(0, 0, 1, 1);

  return self;
}

- (void)convertYUVSampleBuffer:(CMSampleBufferRef)sampleBuffer toGrayscaleMat:(cv::Mat &)mat
{
  // http://mkonrad.net/2014/06/24/cvvideocamera-vs-native-ios-camera-apis.html
  CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);

  // lock the buffer
  CVPixelBufferLockBaseAddress(imageBuffer, 0);

  // get the address to the image data
  void *imageBufferAddress = CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 0);

  // get image properties
  int w = (int)CVPixelBufferGetWidth(imageBuffer);
  int h = (int)CVPixelBufferGetHeight(imageBuffer);

  // create the cv mat
  // 8 bit unsigned chars for grayscale data
  // the first plane contains the grayscale data
  // therefore we use <imgBufAddr> as source
  mat.create(h, w, CV_8UC1);
  memcpy(mat.data, imageBufferAddress, w * h);

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
      //double threshold = mean * (1 + k * (stddev / r - 1));
      double p = 2.0;
      double q = 10.0;
      double threshold = mean * (1 + std::pow(p, -q * mean) + k * (stddev / r - 1)); //Phansalkar

      if (src.at<uchar>(y, x) < threshold) {
        dst.at<uchar>(y, x) = 0;
      } else {
        dst.at<uchar>(y, x) = 255;
      }
    }
  }
}

- (UIImage *)processFace:(CMSampleBufferRef)sampleBuffer
{
  // create grayscale TODO: is it possible to process only updated pixels not the entire image?
  cv::Mat mat;
  [self convertYUVSampleBuffer:sampleBuffer toGrayscaleMat:mat];

  // detect faces
  std::vector<cv::Rect> faces;
  cascade.detectMultiScale(mat, faces, 1.1, 2, CV_HAAR_DO_CANNY_PRUNING | CV_HAAR_DO_ROUGH_SEARCH | CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(40, 40));

  cv::Mat tmpMat = mat;
  if (faces.size() > 0) {
    cv::Rect r = faces[0];

    // vertical shift TODO: shift after crop for performance
    int offsety = (mat.rows - r.height) * 0.5 - r.y;
    [self shiftImage:mat x:0 y:offsety];

    // crop
    // http://www.plosone.org/article/info%3Adoi%2F10.1371%2Fjournal.pone.0093369
    int newHeight = r.width * 2;
    int newY = 0;
    if (mat.rows < newHeight) {
      newHeight = mat.rows;
    } else {
      newY = (mat.rows - newHeight) * 0.5;
    }
    // Simple stabilizer
    // TODO use Kalman Filter to stablize http://nghiaho.com/?p=2093
    previousROI.x = (r.x + previousROI.x) * 0.5;
    previousROI.y = (newY + previousROI.x) * 0.5;
    previousROI.width = MIN((r.width + previousROI.width) * 0.5, mat.cols - previousROI.x);
    previousROI.height = MIN((newHeight + previousROI.height) * 0.5, mat.rows - previousROI.y);
    tmpMat = mat(previousROI);
    // tmpMat = mat(cv::Rect(r.x, newY, r.width, newHeight));
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
  cv::Mat tmpMat2;
  sauvolaFast(tmpMat, tmpMat2, 11, 0.05, 100);
  int morph_size = 1;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size+1), cv::Point( morph_size, morph_size));
  cv::morphologyEx(tmpMat2, tmpMat2, cv::MORPH_OPEN, element);
  //cv::erode(tmpMat2, tmpMat2, element);
  //cv::dilate(tmpMat2, tmpMat2, element);
  cv::Point seedPoint = cv::Point(previousROI.width * 0.5, previousROI.height * 0.5);
  // search pixel
  int amount = seedPoint.y * 0.5;
  for (int y = -amount; y <= amount; y++) {
    if (tmpMat2.at<uchar>(seedPoint.x, seedPoint.y + y) == 255) {
      seedPoint.y = seedPoint.y + y;
      break;
    }
  }
  cv::floodFill(tmpMat2, seedPoint, cv::Scalar(128,128,128));


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
  cv::flip(tmpMat2, mat, 1);

  // convert mat to UIImage TODO: create my own MatToUIImage and add color
  return MatToUIImage(mat);
}

@end