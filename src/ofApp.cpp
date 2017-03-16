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

#include "ofApp.h"

unsigned int kinectPreviewWidth = 640;
unsigned int kinectPreviewHeight = 480;

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetFrameRate(30);

	// ==== GRT ==== //
	//Initialize the training and info variables
	infoText = "";
	trainingClassLabel = 1;
	recordGesture = false;

	//The input to the training data will be the distance from the left & right hands to the head as well as
	//the left and right hand states, so we set the number of dimensions to 4
	trainingData.setNumDimensions(NUM_DIMENSIONS);

	//Initialize the DTW classifier
	DTW dtw;

	//Turn on null rejection, this lets the classifier output the predicted class label of 0 when the likelihood of a gesture is low
	dtw.enableNullRejection(true);

	//Set the null rejection coefficient to 2, this controls the thresholds for the automatic null rejection
	//You can increase this value if you find that your real-time gestures are not being recognized
	//If you are getting too many false positives then you should decrease this value
	dtw.setNullRejectionCoeff(2);

	//Turn on the automatic data triming, this will remove any sections of none movement from the start and end of the training samples
	dtw.enableTrimTrainingData(true, 0.1, 90);

	//Offset the timeseries data by the first sample, this makes your gestures (more) invariant to the location the gesture is performed
	dtw.setOffsetTimeseriesUsingFirstSample(true);

	//Add the classifier to the first pipeline (after we do this, we don't need the DTW classifier anymore)
	pipelines[0].setClassifier(dtw);


	// ==== Kinect ==== //
	// initialize the Kinect
	kinect.open();
	kinect.initDepthSource();
	kinect.initInfraredSource();
	kinect.initColorSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();
	numTracked = 0;

	// ==== OSC ==== //
	// open an outgoing connection to HOST:SEND_PORT
	oscSender.setup(HOST, SEND_PORT);

	// set the dimesions of the app window
	ofSetWindowShape(WIN_WIDTH, WIN_HEIGHT);
}

//--------------------------------------------------------------
void ofApp::update(){
	float distLeft = 0.0;
	float distRight = 0.0;
	float handLeftState = 0;
	float handRightState = 0;

	// ==== get the Kinect data first ==== //
	kinect.update();

	// Get joint positions (skeleton tracking)
	{
		auto bodies = kinect.getBodySource()->getBodies();
		unsigned int trackCount = 0;
		for (auto body : bodies) {
			// only tracking up to MAX_BODY_TRACK
			if (body.tracked) { // make sure the body is actually being tracked
				trackCount++;
				if (trackCount > 0 && trackCount <= MAX_BODY_TRACK) {
					auto joints = body.joints;
					// get hand positions
					float handLeftX = joints[JointType_HandLeft].getPosition().x;
					float handLeftY = joints[JointType_HandLeft].getPosition().y;
					float handLeftZ = joints[JointType_HandLeft].getPosition().z;
					float handRightX = joints[JointType_HandRight].getPosition().x;
					float handRightY = joints[JointType_HandRight].getPosition().y;
					float handRightZ = joints[JointType_HandRight].getPosition().z;
					// get hand states (open=2, closed=3, lasso=4, notracked=1, unknown=0)
					handLeftState = body.leftHandState;
					handRightState = body.rightHandState;
					//ofLog(OF_LOG_NOTICE, "left hand state: " + ofToString(handLeftState));
					// get head position
					float headX = joints[JointType_Head].getPosition().x;
					float headY = joints[JointType_Head].getPosition().y;
					float headZ = joints[JointType_Head].getPosition().z;
					// get eucidian distance between hands and head
					distLeft = ofDist(handLeftX, handLeftY, handLeftZ, headX, headY, headZ);
					distRight = ofDist(handRightX, handRightY, handRightZ, headX, headY, headZ);
				}
			}
		}
		// update the global variable numTracked with the number of bodies just tracked
		numTracked = trackCount;
	}


	// ==== process the GRT data next === //
	// store the relevant data taken from the kinect
	VectorDouble sample(NUM_DIMENSIONS);
	sample[0] = distLeft;
	sample[1] = distRight;
	sample[2] = handLeftState;
	sample[3] = handRightState;

	// If we are recording training data, then add the current sample to the training data set
	if (recordGesture) {
		timeseries.push_back(sample);
	}

	// If pipelines[0] has been trained, then run the prediction for all the pipelines
	if (pipelines[0].getTrained()) {
		for (unsigned int i = 0; i < numTracked; i++) {
			pipelines[i].predict(sample);
		}
	}

	
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);

	// === GRT text display === //
	string text;
	int textX = 20;
	int textY = 20;

	// Draw the training info
	ofSetColor(255);
	text = "------------------- TrainingInfo -------------------";
	ofDrawBitmapString(text, textX, textY);

	if (recordGesture) ofSetColor(255, 0, 0);
	else ofSetColor(255);
	textY += 15;
	text = recordGesture ? "RECORDING" : "Not Recording";
	ofDrawBitmapString(text, textX, textY);

	ofSetColor(255);
	textY += 15;
	text = "TrainingClassLabel: " + ofToString(trainingClassLabel);
	ofDrawBitmapString(text, textX, textY);

	textY += 15;
	text = "NumTrainingSamples: " + ofToString(trainingData.getNumSamples());
	ofDrawBitmapString(text, textX, textY);

	// Draw the prediction info
	textY += 30;
	text = "------------------- Prediction Info -------------------";
	ofDrawBitmapString(text, textX, textY);

	textY += 15;
	text = pipelines[0].getTrained() ? "Model Trained: YES" : "Model Trained: NO";
	ofDrawBitmapString(text, textX, textY);

	// draw the prediction info for each body/pipeline
	for (int i = 0; i < numTracked; i++) {
		textY += 15;
		text = "Body " + ofToString(i);
		ofDrawBitmapString(text, textX, textY);

		textY += 15;
		text = "PredictedClassLabel: " + ofToString(pipelines[i].getPredictedClassLabel());
		ofDrawBitmapString(text, textX, textY);

		textY += 15;
		text = "Likelihood: " + ofToString(pipelines[i].getMaximumLikelihood());
		ofDrawBitmapString(text, textX, textY);

		textY += 15;
		text = "SampleRate: " + ofToString(ofGetFrameRate(), 2);
		ofDrawBitmapString(text, textX, textY);

		textY += 5;
		text = "_____________________________";
		ofDrawBitmapString(text, textX, textY);
	}

	// Draw the info text
	textY += 30;
	text = "InfoText: " + infoText;
	ofDrawBitmapString(text, textX, textY);

	// Draw the timeseries data (while recording)
	if (recordGesture) {
		textX += 200;
		ofSetColor(90, 120, 255);
		for (UINT i = 0; i<timeseries.getNumRows(); i++) {
			double lDist = timeseries[i][0];
			double rDist = timeseries[i][1];
			double lHandState = timeseries[i][2];
			double rHandState = timeseries[i][3];

			// draw left hand distance
			text = "Left Hand Distance: " + ofToString(lDist);
			ofDrawBitmapString(text, textX, textY);
			textY += 15;
			// draw right hand distance
			text = "Right Hand Distance: " + ofToString(rDist);
			ofDrawBitmapString(text, textX, textY);
			textY += 15;
			// draw left hand state
			text = "Left Hand State: " + ofToString(lHandState);
			ofDrawBitmapString(text, textX, textY);
			textY += 15;
			// draw right hand state
			text = "Right Hand State: " + ofToString(rHandState);
			ofDrawBitmapString(text, textX, textY);
			textY += 15;
		}
	}

	// Draw the timeseries data (during prediction, for all pipelines/bodies)
	if (pipelines[0].getTrained()) {
		for (UINT i = 0; i < numTracked; i++) {
			textX += 200;
			// Draw the data in the DTW input buffer
			DTW *dtw = pipelines[i].getClassifier< DTW >();

			if (dtw != NULL) {
				vector< VectorDouble > inputData = dtw->getInputDataBuffer();
				for (UINT i = 0; i<inputData.size(); i++) {
					double lDist = inputData[i][0];
					double rDist = inputData[i][1];
					double lHandState = inputData[i][2];
					double rHandState = inputData[i][3];

					// draw left hand distance
					text = "Left Hand Distance: " + ofToString(lDist);
					ofDrawBitmapString(text, textX, textY);
					textY += 15;
					// draw right hand distance
					text = "Right Hand Distance: " + ofToString(rDist);
					ofDrawBitmapString(text, textX, textY);
					textY += 15;
					// draw left hand state
					text = "Left Hand State: " + ofToString(lHandState);
					ofDrawBitmapString(text, textX, textY);
					textY += 15;
					// draw right hand state
					text = "Right Hand State: " + ofToString(rHandState);
					ofDrawBitmapString(text, textX, textY);
					textY += 15;
				}
			}
		}
	}

	// === Kinect video/tracking display === //
	// Draw the kinect image sources
	kinect.getBodyIndexSource()->draw(WIN_WIDTH-kinectPreviewWidth, 0, kinectPreviewWidth, kinectPreviewHeight);
	kinect.getBodySource()->drawProjected(WIN_WIDTH-kinectPreviewWidth, 0, kinectPreviewWidth, kinectPreviewHeight, ofxKFW2::ProjectionCoordinates::DepthCamera);

	// === OSC data === //
	//TODO
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

	infoText = "";

	switch (key) {
	case 'r':
		recordGesture = !recordGesture;
		if (!recordGesture) {
			trainingData.addSample(trainingClassLabel, timeseries);

			// clear the timeseries for the next recording
			timeseries.clear();
		}
		break;
	case '[':
		if (trainingClassLabel > 1)
			trainingClassLabel--;
		break;
	case ']':
		trainingClassLabel++;
		break;
	case 't':
		// train the first pipeline and make copies of the rest for prediction
		if (pipelines[0].train(trainingData)) {
			for (int i = 1; i < numTracked; i++) {
				GestureRecognitionPipeline p(pipelines[0]);
				pipelines[i] = p;
			}
			infoText = "Pipeline Trained";
		}
		else infoText = "WARNING: Failed to train pipeline";
		break;
	case 's':
		if (trainingData.saveDatasetToFile("TrainingData.txt")) {
			infoText = "Training data saved to file";
		}
		else infoText = "WARNING: Failed to save training data to file";
		break;
	case 'l':
		if (trainingData.loadDatasetFromFile("TrainingData.txt")) {
			infoText = "Training data saved to file";
		}
		else infoText = "WARNING: Failed to load training data from file";
		break;
	case 'c':
		trainingData.clear();
		infoText = "Training data cleared";
		break;
	default:
		break;
	}
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
