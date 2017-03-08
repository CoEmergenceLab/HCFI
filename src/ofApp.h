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

		ofxOscSender oscSender;
		
};
