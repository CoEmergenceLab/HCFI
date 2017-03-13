#include "ofApp.h"

int kinectPreviewWidth = 640;
int kinectPreviewHeight = 480;

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetFrameRate(60);

	// ==== GRT ==== //
	//Initialize the training and info variables
	infoText = "";
	trainingClassLabel = 1;
	record = false;

	//The input to the training data will be the distance from the left & right hands to the head as well as
	//the left and right hand states, so we set the number of dimensions to 4
	trainingData.setNumDimensions(4);

	//Initialize the DTW classifier
	DTW dtw;

	//Turn on null rejection, this lets the classifier output the predicted class label of 0 when the likelihood of a gesture is low
	dtw.enableNullRejection(true);

	//Set the null rejection coefficient to 3, this controls the thresholds for the automatic null rejection
	//You can increase this value if you find that your real-time gestures are not being recognized
	//If you are getting too many false positives then you should decrease this value
	dtw.setNullRejectionCoeff(3);

	//Turn on the automatic data triming, this will remove any sections of none movement from the start and end of the training samples
	dtw.enableTrimTrainingData(true, 0.1, 90);

	//Offset the timeseries data by the first sample, this makes your gestures (more) invariant to the location the gesture is performed
	dtw.setOffsetTimeseriesUsingFirstSample(true);

	//Add the classifier to the pipeline (after we do this, we don't need the DTW classifier anymore)
	pipeline.setClassifier(dtw);

	// add this pipeline to the pipelines vector
	pipelines.push_back(pipeline);

	// ==== Kinect ==== //
	// initialize the Kinect
	kinect.open();
	kinect.initDepthSource();
	kinect.initInfraredSource();
	kinect.initColorSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	// ==== OSC ==== //
	// open an outgoing connection to HOST:SEND_PORT
	oscSender.setup(HOST, SEND_PORT);

	// set the dimesions of the app window
	ofSetWindowShape(1920, 1080);
}

//--------------------------------------------------------------
void ofApp::update(){
	kinect.update();

	// Getting joint positions (skeleton tracking)
	{
		auto bodies = kinect.getBodySource()->getBodies();
		unsigned int numTracked = 0;
		for (auto body : bodies) {
			// only tracking up to MAX_BODY_TRACK
			if(body.tracked) numTracked++; // make sure the body is actually being tracked
			if (numTracked > 0 && numTracked <= MAX_BODY_TRACK) {
				auto joints = body.joints;
				// get hand positions
				float handLeftX = joints[JointType_HandLeft].getPosition().x;
				float handLeftY = joints[JointType_HandLeft].getPosition().y;
				float handLeftZ = joints[JointType_HandLeft].getPosition().z;
				float handRightX = joints[JointType_HandRight].getPosition().x;
				float handRightY = joints[JointType_HandRight].getPosition().y;
				float handRightZ = joints[JointType_HandRight].getPosition().z;
				// get hand states (open=2, closed=3, lasso=4, notracked=1, unknown=0)
				int handLeftState = body.leftHandState;
				int handRightState = body.rightHandState;
				// get head position
				float headX = joints[JointType_Head].getPosition().x;
				float headY = joints[JointType_Head].getPosition().y;
				float headZ = joints[JointType_Head].getPosition().z;
				// get eucidian distance between hands and head
				float distLeft = ofDist(handLeftX, handLeftY, handLeftZ, headX, headY, headZ);
				float distRight = ofDist(handRightX, handRightY, handRightZ, headX, headY, headZ);
			}
		}
	}

}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
