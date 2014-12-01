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

  // http://stackoverflow.com/questions/9939843/cgbitmapcontextcreate-for-cv-8uc3-to-use-in-opencv
}

- (UIImage *)processFace:(CMSampleBufferRef)sampleBuffer
{
  // create grayscale
  cv::Mat mat;
  [self convertYUVSampleBuffer:sampleBuffer toGrayscaleMat:mat];

  // detect faces
  std::vector<cv::Rect> faces;
  cascade.detectMultiScale(mat, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(50, 50));

  // draw circles on faces
  std::vector<cv::Rect>::const_iterator r = faces.begin();
  for(; r != faces.end(); ++r) {
    cv::Point center;
    int radius;
    center.x = cv::saturate_cast<int>(r->x + r->width*0.5);
    center.y = cv::saturate_cast<int>(r->y + r->height*0.5);
    radius = cv::saturate_cast<int>(MAX(r->width, r->height) * 0.5);
    cv::circle(mat, center, radius, cv::Scalar(255,255,255), 3, 8, 0 );
  }

  cv::Mat mirror;
  cv::flip(mat, mirror, 1);

  // convert mat to UIImage
  return MatToUIImage(mirror);
}

@end