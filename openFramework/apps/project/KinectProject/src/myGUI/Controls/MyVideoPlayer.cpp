#ifdef USE_OFXOPENCV

#include "MyButton.h"
#include "MySlider.h"
#include "MyVideoPlayer.h"
#include "MyScrollList.h"

#include "ofEvents.h"

MyVideoPlayer::MyVideoPlayer(string _name, float _x, float _y, float _previewW, float _previewH):
MyPanel(_name, _x, _y)
{
	previewW = _previewW;
	previewH = _previewH;
	
	setSize(BUTTONS_WIDTH + BUTTON_PADDING_X + previewW, BUTTON_PADDING_Y + previewH);
	adjustVertices();
	vid = new ofVideoPlayer();
	
	bPaused = true;
	currentVideo = -1;
	
	
	float buttonW = (previewW / 3);
	prevButton = new MyButton("Prev", BUTTONS_WIDTH + BUTTON_PADDING_X*2, previewH + 30 + BUTTON_PADDING_Y , buttonW, 50);
	addControl(prevButton);
	ofAddListener(prevButton->release, this, &MyVideoPlayer::onPrevReleased);
	
	playToggle = new MyToggle("Play", BUTTONS_WIDTH +BUTTON_PADDING_X*2+ buttonW, previewH + 30 + BUTTON_PADDING_Y , buttonW, 50);
	playToggle->setStateText("", "");
	addControl(playToggle);
	ofAddListener(playToggle->turnOn, this, &MyVideoPlayer::onPlayTurnOn);
	ofAddListener(playToggle->turnOff, this, &MyVideoPlayer::onPlayTurnOff);
	
	nextButton = new MyButton("Next", BUTTONS_WIDTH + BUTTON_PADDING_X*2+buttonW*2, previewH + 30 + BUTTON_PADDING_Y , buttonW, 50);
	addControl(nextButton);
	ofAddListener(nextButton->release, this, &MyVideoPlayer::onNextReleased);
	
	videoPosition = videoDuration = 0.0f;
	videoPosSlider = new MySlider("", BUTTONS_WIDTH + BUTTON_PADDING_X*2, previewH + 5 + BUTTON_PADDING_Y, _previewW, 20, &videoPosition, 0.0f, videoDuration);
	videoPosSlider->setInfoMode(INFO_MODE_TIME);
	addControl(videoPosSlider);
	ofAddListener(videoPosSlider->drag, this, &MyVideoPlayer::onVidPosDrag);
	
	loopToggle = new MyToggle("Loop", BUTTON_PADDING_X*3 + previewW + BUTTONS_WIDTH, BUTTON_PADDING_Y, BUTTONS_WIDTH * 0.5f, 20);
	ofAddListener(loopToggle->turnOn, this, &MyVideoPlayer::onLoopTurnOn);
	ofAddListener(loopToggle->turnOff, this, &MyVideoPlayer::onLoopTurnOff);
	addControl(loopToggle);
	loopToggle->turnOnToggle();
	
	volumeVal = 100.0f;
	volumeSlider = new MySlider("Volume", BUTTON_PADDING_X*3 + previewW + BUTTONS_WIDTH,30 + BUTTON_PADDING_Y, BUTTONS_WIDTH * 0.5f, 20, &volumeVal, 0.0f, MAX_VOLUME);
	addControl(volumeSlider);
	ofAddListener(volumeSlider->drag, this, &MyVideoPlayer::onVolumeDrag);
	
	videoList = new MyScrollList("Videos", BUTTON_PADDING_X, BUTTON_PADDING_Y, BUTTONS_WIDTH, previewH + 50 + BUTTON_PADDING_Y);
	addControl(videoList);
	ofAddListener(videoList->itemSelected, this, &MyVideoPlayer::onVideoSelected);
}

MyVideoPlayer::~MyVideoPlayer(){
	vid->close();
	delete vid;
}

void MyVideoPlayer::update(){
	videoList->update();
	vid->update();
	if(vid->bLoaded)videoPosition = vid->getPosition() * videoDuration;
}

void MyVideoPlayer::draw(){
	MyPanel::draw();
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	vid->draw(x + BUTTONS_WIDTH + BUTTON_PADDING_X*2, y + BUTTON_PADDING_Y, previewW, previewH);
	ofNoFill();
	ofSetColor(colors.mainColorPress.r, colors.mainColorPress.g, colors.mainColorPress.b);
	ofRect(x + BUTTONS_WIDTH + BUTTON_PADDING_X*2, y + BUTTON_PADDING_Y, previewW, previewH);
}

void MyVideoPlayer::drawVideo(float _x, float _y, float _w, float _h){
	vid->draw(_x, _y, _w, _h);
}

void MyVideoPlayer::addVideo(string _path, string _name){
	videoFiles.push_back(_path);
	videoNames.push_back(_name);
	videoList->addItem(_name);
	if(videoList->getNumItems() == 1){
		videoList->selectItem(0);
	}
}

void MyVideoPlayer::loadVideosFromFolder(string path){
	videoFiles.clear();
	videoNames.clear();
	ofxDirList dir;
	int numFiles = dir.listDir(path);
	for(int i=0; i<numFiles; i++){
		string fileName = dir.getName(i);
		string::size_type index = fileName.rfind(".");
		string nameWithoutExtension = fileName.substr(0, index);
		addVideo(dir.getPath(i), nameWithoutExtension);
	}
}

void MyVideoPlayer::play(){
	playToggle->turnOnToggle();
}

void MyVideoPlayer::pause(){
	playToggle->turnOffToggle();
}

void MyVideoPlayer::next(){
	ControlEventArgs e(this);
	onNextReleased(e);
}

void MyVideoPlayer::prev(){
	ControlEventArgs e(this);
	onPrevReleased(e);
}

void MyVideoPlayer::goToVideo(int vidIndex){
	if(vidIndex == currentVideo)return;
	if(vidIndex < videoFiles.size()){
		currentVideo = vidIndex;
		vid->loadMovie(videoFiles[currentVideo]);
		initVideo(vid);
	}
}

void MyVideoPlayer::goToVideo(string videoName){
	for(int i=0; i<videoFiles.size(); i++){
		if(strcmp(videoName.c_str(), videoNames[i].c_str()) == 0){
			goToVideo(i);
			return;
		}
	}
}

void MyVideoPlayer::setVolume(float _val){
	volumeVal = MAX(0.0f, MIN(MAX_VOLUME, _val));
	if(vid->bLoaded){
		vid->setVolume(volumeVal);
	}
}

void MyVideoPlayer::enable(){
	if(isEnabled())return;
	ofAddListener(playToggle->turnOn, this, &MyVideoPlayer::onPlayTurnOn);
	ofAddListener(prevButton->release, this,&MyVideoPlayer::onPrevReleased);
	ofAddListener(nextButton->release, this, &MyVideoPlayer::onNextReleased);
	bEnabled = true;
}

void MyVideoPlayer::disable(){
	if(isDisabled())return;
	ofRemoveListener(playToggle->turnOn, this, &MyVideoPlayer::onPlayTurnOn);
	ofRemoveListener(prevButton->release, this,&MyVideoPlayer::onPrevReleased);
	ofRemoveListener(nextButton->release, this, &MyVideoPlayer::onNextReleased);
	bEnabled = false;
}

void MyVideoPlayer::setListHeight(float height){
	videoList->setSize(videoList->width, height);
	videoList->adjustVertices();
	if(videoList->y + videoList->height > this->height - 5){
		this->setSize(this->width, videoList->y + videoList->height - 20);
		this->adjustVertices();
	}
	videoList->calcNumVisibleItems();
}

void MyVideoPlayer::setLoop(bool _bLoop){
	bLoop = _bLoop;
	if(vid->bLoaded){
		vid->setLoopState(bLoop ? OF_LOOP_NORMAL : OF_LOOP_NONE);
	}
}

int MyVideoPlayer::getCurrentVideoIndex(){
	return currentVideo;	
}

string MyVideoPlayer::getCurrentVideoName(){
	if(currentVideo < 0)return "";
	return videoNames[currentVideo];
}

string MyVideoPlayer::getCurrentVideoPath(){
	if(currentVideo < 0)return "";
	return videoFiles[currentVideo];
}

int MyVideoPlayer::getNumVideos(){
	return videoFiles.size();
}

void MyVideoPlayer::onPlayTurnOn(ControlEventArgs& e){
	bPaused = false;
	vid->setPaused(bPaused);
}

void MyVideoPlayer::onPlayTurnOff(ControlEventArgs& e){
	bPaused = true;
	vid->setPaused(bPaused);
}

void MyVideoPlayer::onNextReleased(ControlEventArgs& e){
	if(currentVideo < videoFiles.size()-1){
		videoList->selectItem(currentVideo+1);
	}
}

void MyVideoPlayer::onPrevReleased(ControlEventArgs& e){
	if(currentVideo > 0){
		videoList->selectItem(currentVideo-1);
	}
}

void MyVideoPlayer::onVideoSelected(ControlEventArgs& e){
	cout << "video selected: " + e.control->getName() << endl;
	goToVideo(e.control->getName());
}

void MyVideoPlayer::onVidPosDrag(ControlEventArgs& e){
	float vidPos = videoPosition / videoDuration;
	vid->setPosition(videoPosition / videoDuration);
}

void MyVideoPlayer::onLoopTurnOn(ControlEventArgs& e){
	setLoop(true);
}

void MyVideoPlayer::onLoopTurnOff(ControlEventArgs& e){
	setLoop(false);
}

void MyVideoPlayer::onVolumeDrag(ControlEventArgs& e){
	setVolume(volumeVal);
}

ofVideoPlayer* MyVideoPlayer::getVideo(){
	return vid;	
}

void MyVideoPlayer::addControlEvents(){
	
}

void MyVideoPlayer::removeControlEvents(){
	
}

void MyVideoPlayer::addMovieEvents(){
	
}

void MyVideoPlayer::removeMovieEvents(){
	
}

void MyVideoPlayer::addEvents(){
	addControlEvents();
	removeMovieEvents();
}

void MyVideoPlayer::removeEvents(){
	removeControlEvents();
	removeMovieEvents();
}

void MyVideoPlayer::initVideo(ofVideoPlayer* _vid){
	if(_vid == NULL)return;
	if(!_vid->bLoaded)return;
	_vid->play();
	_vid->setPaused(bPaused);
	_vid->setLoopState(bLoop ? OF_LOOP_NORMAL : OF_LOOP_NONE);
	_vid->setVolume(volumeVal);
	videoDuration = _vid->getDuration();
	videoPosition = 0.0f;
	videoPosSlider->setMaxVal(videoDuration);
	videoPosSlider->setName(videoNames[currentVideo]);
}

void MyVideoPlayer::loadFromXML(ofxXmlSettings& xml){
	loopToggle->loadFromXML(xml);
	volumeSlider->loadFromXML(xml);
}

void MyVideoPlayer::saveToXML(ofxXmlSettings& xml){
	loopToggle->saveToXML(xml);
	volumeSlider->saveToXML(xml);
}

#endif







