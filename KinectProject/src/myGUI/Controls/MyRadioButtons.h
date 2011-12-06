#pragma once

#include "MyPanel.h"
#include "MyToggle.h"

class MyRadioButtons:public MyPanel{
protected:
	vector<MyToggle*>buttons;
	bool bAlwaysOn;
	virtual void onToggleTurnOn(ControlEventArgs& e);
public:
	MyRadioButtons(string _name, float _x, float _y, bool _bAlwaysOn = false);
	virtual ~MyRadioButtons();
	
	virtual void clear();
	virtual void clearToggles();
	
	virtual void addToggle(MyToggle* b);
	virtual vector<MyToggle*>& getRadioButtons();
	
	virtual void onRollOut();
	virtual void alwaysOn(bool _bAlways);
	virtual bool alwaysOn();
	
	virtual void onPress(int x, int y, int button);
	
	virtual void enable();
	virtual void disable();
	
	
};
