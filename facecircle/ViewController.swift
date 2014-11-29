//
//  ViewController.swift
//  facecircle
//
//  Created by Kazuyuki Tanimura on 11/27/14.
//  Copyright (c) 2014 Kazuyuki Tanimura. All rights reserved.
//

import UIKit
import AVFoundation
import AssetsLibrary

class ViewController: UIViewController, AVCaptureFileOutputRecordingDelegate {
  
  var captureSession: AVCaptureSession!
  var captureDevice: AVCaptureDevice?
  var videoOutput: AVCaptureMovieFileOutput!
  var startButton: UIButton!
  var stopButton: UIButton!
  var videoLayer: AVCaptureVideoPreviewLayer!
  
  let UIInterface2VideoOrientation: [UIInterfaceOrientation: AVCaptureVideoOrientation] = [
    .Portrait: .Portrait,
    .PortraitUpsideDown: .PortraitUpsideDown,
    .LandscapeRight: .LandscapeRight,
    .LandscapeLeft: .LandscapeLeft,
  ]

  override func viewDidLoad() {
    super.viewDidLoad()

    // create a session
    captureSession = AVCaptureSession()
    // get all devices
    let devices = AVCaptureDevice.devices()
    // get front camera
    for device in devices {
      if (device.position == AVCaptureDevicePosition.Front) {
        captureDevice = device as? AVCaptureDevice
      }
    }
    // return if no camera found
    if (captureDevice == nil) {
      UIAlertView(title: "Missing Camera", message: "No camera device found", delegate: nil, cancelButtonTitle: "OK").show()
      return
    }
    // set the front camera
    let videoInput = AVCaptureDeviceInput.deviceInputWithDevice(captureDevice!, error: nil) as AVCaptureDeviceInput
    captureSession.addInput(videoInput)
    // set microphone
    let audioCaptureDevice = AVCaptureDevice.devicesWithMediaType(AVMediaTypeAudio)
    let audioInput = AVCaptureDeviceInput.deviceInputWithDevice(audioCaptureDevice[0] as AVCaptureDevice, error: nil) as AVCaptureDeviceInput
    captureSession.addInput(audioInput)
    // set video output
    videoOutput = AVCaptureMovieFileOutput()
    captureSession.addOutput(videoOutput)
    // create a layer for video
    videoLayer = AVCaptureVideoPreviewLayer.layerWithSession(captureSession) as AVCaptureVideoPreviewLayer
    videoLayer.videoGravity = AVLayerVideoGravityResizeAspectFill
    self.view.layer.addSublayer(videoLayer)
    // start the session
    captureSession.startRunning()
    // set up the UI
    startButton = UIButton(frame: CGRectMake(0,0,120,50))
    stopButton = UIButton(frame: CGRectMake(0,0,120,50))
    startButton.backgroundColor = UIColor.redColor();
    stopButton.backgroundColor = UIColor.grayColor();
    startButton.layer.masksToBounds = true
    stopButton.layer.masksToBounds = true
    startButton.setTitle("START", forState: .Normal)
    stopButton.setTitle("STOP", forState: .Normal)
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
      let paths = NSSearchPathForDirectoriesInDomains(.DocumentDirectory, .UserDomainMask, true)
      let documentsDirectory = paths[0] as String
      let filePath = "\(documentsDirectory)/test.mp4"
      let fileURL = NSURL(fileURLWithPath: filePath)!
      
      videoOutput.startRecordingToOutputFileURL(fileURL, recordingDelegate: self)
    } else if (sender == stopButton) {
      videoOutput.stopRecording()
    }
  }
  
  func resetPosition(toInterfaceOrientation: UIInterfaceOrientation) {
    startButton.layer.position = CGPoint(x: self.view.bounds.width/2 - 70, y:self.view.bounds.height-50)
    stopButton.layer.position = CGPoint(x: self.view.bounds.width/2 + 70, y:self.view.bounds.height-50)
    videoLayer.connection.videoOrientation = UIInterface2VideoOrientation[toInterfaceOrientation] ?? .Portrait
    videoLayer.frame = self.view.bounds
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
  */
  
  override func willAnimateRotationToInterfaceOrientation(toInterfaceOrientation: UIInterfaceOrientation, duration: NSTimeInterval) {
    resetPosition(toInterfaceOrientation)
  }

  func captureOutput(captureOutput: AVCaptureFileOutput!, didFinishRecordingToOutputFileAtURL outputFileURL: NSURL!, fromConnections connections: [AnyObject]!, error: NSError!) {
    println("didFinishRecordingToOutputFileAtURL")
    // create AssetsLibrary
    let assetsLib = ALAssetsLibrary()
    
    // save video
    assetsLib.writeVideoAtPathToSavedPhotosAlbum(outputFileURL, completionBlock: nil)
  }
  
  func captureOutput(captureOutput: AVCaptureFileOutput!, didStartRecordingToOutputFileAtURL fileURL: NSURL!, fromConnections connections: [AnyObject]!) {
    println("didStartRecordingToOutputFileAtURL")
  }

  override func didReceiveMemoryWarning() {
    super.didReceiveMemoryWarning()
    // Dispose of any resources that can be recreated.
  }


}

