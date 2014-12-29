//
//  Use this file to import your target's public headers that you would like to expose to Swift.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import <CoreMedia/CoreMedia.h>
#import <AVFoundation/AVFoundation.h>

#ifdef __cplusplus
#import <opencv2/opencv.hpp>
#import <opencv2/highgui/ios.h>
//#import "EGBS.h"
#endif

@interface Face: NSObject

- (id)init;
#ifdef __cplusplus
- (void)convertYUVSampleBuffer:(CMSampleBufferRef)sampleBuffer toGrayscaleMat:(cv::Mat &)mat;
- (void)shiftImage:(cv::Mat &)img x:(int)offsetx y:(int)offsety;
- (void)tilt:(cv::Mat &)src toMat:(cv::Mat &)dst orientation:(UIInterfaceOrientation)orientation;
- (void)connectDots:(cv::Mat &)mat;
void sauvolaFast(const cv::Mat &src, cv::Mat &dst, int kernelSize, double k, double r);
void unsharpMask(cv::Mat& im);
#endif
- (UIImage *)processFace:(CMSampleBufferRef)sampleBuffer camera:(AVCaptureDevice*)device orientation:(UIInterfaceOrientation)orientation;

@end