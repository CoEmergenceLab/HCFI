#pragma once

#include "ofMain.h"
#include "ofxOsc.h"               // Include the main ofxOSC header 
#include "ofxGrt.h"               // Include the main ofxGrt header
#include "ofxKinectForWindows2.h" // Include KinectforWindows2 header
#define SEND_PORT 5001
#define HOST "localhost"

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
		TimeSeriesClassificationData trainingData; // store our training data
		GestureRecognitionPipeline pipeline;       // wrapper for our classifier and any pre/post processing modules
		bool record;                               // flag that keeps track of when we should record training data
		UINT trainingClassLabel;                   // holds the current label for when we are training the classifier
		string infoText;                           // used to draw some info messages to the main app window
		UINT numPipelines;						   // number of GestureRecognitionPipeline objects (corresponds to the # of people we want to track)
		
};
