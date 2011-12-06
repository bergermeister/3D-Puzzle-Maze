#include "MyButton.h"

MyButton::MyButton(string _name,float _x, float _y, float _w, float _h): ControlBase(_name){
	type = CONTROL_TYPE_BUTTON;
	setPos(_x, _y);
	setSize(_w, _h);
	adjustVertices();
	setDefaultColors();
}

MyButton::~MyButton(){
	
}




