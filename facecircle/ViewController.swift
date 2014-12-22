//
//  ViewController.swift
//  facecircle
//
//  Created by Kazuyuki Tanimura on 11/27/14.
//  Copyright (c) 2014 Kazuyuki Tanimura. All rights reserved.
//

import UIKit
import AVFoundation

class ViewController: UIViewController, AVCaptureVideoDataOutputSampleBufferDelegate {
  
  let captureSession = AVCaptureSession()
  var captureDevice: AVCaptureDevice!
  var startButton: UIButton!
  var stopButton: UIButton!
  let videoDataOutput = AVCaptureVideoDataOutput()
  let imageView = UIImageView()
  let face = Face()
  
  let UIInterface2VideoOrientation: [UIInterfaceOrientation: AVCaptureVideoOrientation] = [
    .Portrait: .Portrait,
    .PortraitUpsideDown: .PortraitUpsideDown,
    .LandscapeRight: .LandscapeRight,
    .LandscapeLeft: .LandscapeLeft,
  ]

  override func viewDidLoad() {
    super.viewDidLoad()

    captureSession.sessionPreset = AVCaptureSessionPresetLow
    // get all devices
    let devices = AVCaptureDevice.devices()
    // get front camera
    for device in devices {
      if (device.position == AVCaptureDevicePosition.Front) {
        captureDevice = device as? AVCaptureDevice
      }
    }
    // return if no camera found
    if (captureDevice? == nil) {
      UIAlertView(title: "Missing Camera", message: "No camera device found", delegate: nil, cancelButtonTitle: "OK").show()
      return
    }
    // set the front camera
    let videoInput = AVCaptureDeviceInput.deviceInputWithDevice(captureDevice, error: nil) as AVCaptureDeviceInput
    captureSession.addInput(videoInput)
    // camera setup
    var lockError: NSError?
    if captureDevice.lockForConfiguration(&lockError) {
      if let error = lockError {
        println("lock error: \(error.localizedDescription)")
        return
      } else {
        captureDevice.activeVideoMinFrameDuration = CMTimeMake(1, 15) // 15 FPS
        if (captureDevice.autoFocusRangeRestrictionSupported) {
          captureDevice.autoFocusRangeRestriction = .Near
        }
        /*
        if (captureDevice.isFocusModeSupported(.Locked)) {
          println("lock")
        }
        if (captureDevice.focusPointOfInterestSupported) {
          println("focus")
        }
        if (captureDevice.exposurePointOfInterestSupported) {
          println("exposure")
        }
        */
        captureDevice.unlockForConfiguration()
      }
    }
    // set microphone
    let audioCaptureDevice = AVCaptureDevice.devicesWithMediaType(AVMediaTypeAudio)
    let audioInput = AVCaptureDeviceInput.deviceInputWithDevice(audioCaptureDevice[0] as AVCaptureDevice, error: nil) as AVCaptureDeviceInput
    captureSession.addInput(audioInput)

    // set video data output
    videoDataOutput.videoSettings = [ kCVPixelBufferPixelFormatTypeKey: kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange ]
    // ignore delayed frames
    videoDataOutput.alwaysDiscardsLateVideoFrames = true
    // set up delegate
    let queue: dispatch_queue_t = dispatch_queue_create("frame.queue",  nil)
    videoDataOutput.setSampleBufferDelegate(self, queue: queue)
    captureSession.addOutput(videoDataOutput)

    // create a layer for imageView
    imageView.contentMode = .ScaleAspectFit
    self.view.addSubview(imageView)

    // start the session
    captureSession.startRunning()
    // set up the UI
    startButton = UIButton(frame: CGRectMake(0,0,120,50))
    stopButton = UIButton(frame: CGRectMake(0,0,120,50))
    startButton.backgroundColor = UIColor.redColor();
    stopButton.backgroundColor = UIColor.grayColor();
    startButton.layer.masksToBounds = true
    stopButton.layer.masksToBounds = true
    startButton.setTitle("END", forState: .Normal)
    stopButton.setTitle("MUTE", forState: .Normal)
    startButton.layer.cornerRadius = 20.0
    stopButton.layer.cornerRadius = 20.0
    startButton.addTarget(self, action: "onClickButton:", forControlEvents: .TouchUpInside)
    stopButton.addTarget(self, action: "onClickButton:", forControlEvents: .TouchUpInside)
    self.view.addSubview(startButton)
    self.view.addSubview(stopButton)
    resetPosition(interfaceOrientation)
  }

  func onClickButton(sender: UIButton) {
    if (sender == startButton) {
    } else if (sender == stopButton) {
    }
  }

  func resetPosition(toInterfaceOrientation: UIInterfaceOrientation) {
    startButton.layer.position = CGPoint(x: self.view.bounds.width/2 - 70, y:self.view.bounds.height-50)
    stopButton.layer.position = CGPoint(x: self.view.bounds.width/2 + 70, y:self.view.bounds.height-50)
    for connection in videoDataOutput.connections {
      if let conn = connection as? AVCaptureConnection {
        if (conn.supportsVideoOrientation) {
          conn.videoOrientation = UIInterface2VideoOrientation[toInterfaceOrientation] ?? .Portrait
        }
        if (conn.supportsVideoStabilization) {
          conn.preferredVideoStabilizationMode = .Auto
        }
      }
    }
    imageView.frame = self.view.bounds
  }
  
  func captureOutput(captureOutput: AVCaptureOutput!, didOutputSampleBuffer sampleBuffer: CMSampleBuffer!, fromConnection connection: AVCaptureConnection!) {
    dispatch_async(dispatch_get_main_queue(), {
      self.imageView.image = self.face.processFace(sampleBuffer, camera: self.captureDevice, orientation: self.interfaceOrientation)
    })
  }
  
  override func willAnimateRotationToInterfaceOrientation(toInterfaceOrientation: UIInterfaceOrientation, duration: NSTimeInterval) {
    resetPosition(toInterfaceOrientation)
  }
  
  /*
  override func shouldAutorotate() -> Bool {
    return true
  }

  override func supportedInterfaceOrientations() -> Int {
    if UIDevice.currentDevice().userInterfaceIdiom == .Phone {
      return Int(UIInterfaceOrientationMask.AllButUpsideDown.rawValue)
    } else {
      return Int(UIInterfaceOrientationMask.All.rawValue)
    }
  }

  func captureOutput(captureOutput: AVCaptureFileOutput!, didFinishRecordingToOutputFileAtURL outputFileURL: NSURL!, fromConnections connections: [AnyObject]!, error: NSError!) {
    println("didFinishRecordingToOutputFileAtURL")
  }

  func captureOutput(captureOutput: AVCaptureFileOutput!, didStartRecordingToOutputFileAtURL fileURL: NSURL!, fromConnections connections: [AnyObject]!) {
    println("didStartRecordingToOutputFileAtURL")
  }
  */

  override func didReceiveMemoryWarning() {
    super.didReceiveMemoryWarning()
    // Dispose of any resources that can be recreated.
  }


}

