/*
HCFI MIT License
Copyright (c) <2017> <Carlos Castellanos, Digital/Experiemntal Media Lab, Kansas STate University>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include "ofMain.h"
#include "ofxOsc.h"               // Include the main ofxOSC header 
#include "ofxGrt.h"               // Include the main ofxGrt header
#include "ofxKinectForWindows2.h" // Include KinectforWindows2 header

#define MAX_BODY_TRACK 2          // tracking a maximum of two bodies
#define NUM_DIMENSIONS 4          // number of dimensions (inputs) to teh classifier
#define SEND_PORT 5001            // OSC send port
#define HOST "localhost"          // OSC host
#define WIN_WIDTH 1920
#define WIN_HEIGHT 1080

using namespace GRT; // State that we want to use the GRT namespace

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		// OSC sender object
		ofxOscSender oscSender;

		// Kinect object
		ofxKFW2::Device kinect;

		// Create some variables for gesture recognition
		LabelledTimeSeriesClassificationData trainingData;    // store our training data
		MatrixDouble timeseries;                              // store a single training sample
		GestureRecognitionPipeline pipelines[MAX_BODY_TRACK]; // wrapper for our classifiers and any pre/post processing modules
		bool recordGesture;                                   // flag that keeps track of when we should record training data
		UINT trainingClassLabel;                              // holds the current label for when we are training the classifier
		string infoText;                                      // used to draw some info messages to the main app window
		
};
