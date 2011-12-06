#pragma once

#include "ControlBase.h"

class MyButton: public ControlBase{
protected:
	
public:
	MyButton(string _name, float _x, float _y, float _w, float _h);
	~MyButton();

};
