#pragma once

class ControlBase;
class ControlEventArgs{
public:
	ControlBase* control;
	
	ControlEventArgs(ControlBase* _control){
		control = _control;
	}
};
