//
//  Use this file to import your target's public headers that you would like to expose to Swift.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import <CoreMedia/CoreMedia.h>

#ifdef __cplusplus
#import <opencv2/opencv.hpp>
#import <opencv2/highgui/ios.h>
#endif

@interface Face: NSObject

- (id)init;
#ifdef __cplusplus
- (void)convertYUVSampleBuffer:(CMSampleBufferRef)sampleBuffer toGrayscaleMat:(cv::Mat &)mat;
- (void)shiftImage:(cv::Mat &)img x:(int)offsetx y:(int)offsety;
void sauvolaFast(const cv::Mat &src, cv::Mat &dst, int kernelSize, double k, double r);
void unsharpMask(cv::Mat& im);
- (void)searchPixel:(cv::Mat &)mat seedPoint:(cv::Point &)point;
#endif
- (UIImage *)processFace:(CMSampleBufferRef)sampleBuffer;

@end