#pragma once

#include "ofMain.h"

class Draggable{
public:
	
	Draggable(){
		pos.x = ofRandomWidth();
		pos.y = ofRandomHeight();
		target = pos;
		radius = 50;
		radiusSQ = radius * radius;
		bDrag = false;
		bOver = false;
	}
	
	void update(){
		pos += (target - pos) * 0.5f;
	}
	
	void draw(){
		if(bDrag){
			glColor4f(1.0f, 0.0f, 0.0f, 0.0f);
		}else{
			if(!bOver){
				glColor4f(1.0f, 1.0f, 0.0f, 0.7f);
			}else{
				glColor4f(0.0f, 1.0f, 0.0f, 0.7f);
			}
		}
		ofCircle(pos.x, pos.y, radius * 0.8);
		glColor3f(1.0f, 1.0f, 1.0f);
		glBegin(GL_LINES);
		glVertex2f(pos.x, pos.y);
		glVertex2f(target.x, target.y);
		glEnd();
	}
	
	void goTo(const ofPoint& target){
		this->target = target;
	}
	
	void setDrag(bool bDrag){
		this->bDrag = bDrag;
		if(bDrag){
			target = pos;
		}
	}
	
	bool isDrag(){
		return bDrag;
	}
	
	bool isInside(const ofPoint& pt){
		float dx = pos.x - pt.x;
		float dy = pos.y - pt.y;
		bOver = dx*dx + dy*dy < radiusSQ;
		return bOver;
	}
	
protected:
	ofPoint pos, target;
	float radius, radiusSQ;
	bool bDrag;
	bool bOver;
};