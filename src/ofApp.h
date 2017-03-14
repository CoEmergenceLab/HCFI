#pragma once

#include "ofMain.h"
#include "ofxOsc.h"               // Include the main ofxOSC header 
#include "ofxGrt.h"               // Include the main ofxGrt header
#include "ofxKinectForWindows2.h" // Include KinectforWindows2 header

#define MAX_BODY_TRACK 2          // tracking a maximum of two bodies
#define NUM_DIMENSIONS 4          // number of dimensions (inputs) to teh classifier
#define SEND_PORT 5001            // OSC send port
#define HOST "localhost"          // OSC host

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
		GestureRecognitionPipeline pipelines[MAX_BODY_TRACK];  // wrapper for our classifiers and any pre/post processing modules
		bool recordGesture;                                   // flag that keeps track of when we should record training data
		UINT trainingClassLabel;                              // holds the current label for when we are training the classifier
		string infoText;                                      // used to draw some info messages to the main app window
		
};
