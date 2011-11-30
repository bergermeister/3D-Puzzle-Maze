#pragma once

class ControlColorScheme{
public:
	ofColor mainColorIdle, mainColorOver, mainColorPress;
	ofColor secColorIdle, secColorOver, secColorPress;
	ofColor outlineColor;
	ofColor textColor;
	
	ControlColorScheme(){
		mainColorIdle.r = 20;
		mainColorIdle.g = 20;
		mainColorIdle.b = 20;
		
		mainColorOver.r = 40;
		mainColorOver.g = 40;
		mainColorOver.b = 40;
		
		mainColorPress.r = 130;
		mainColorPress.g = 130;
		mainColorPress.b = 150;
		
		secColorIdle.r = 100;
		secColorIdle.g = 100;
		secColorIdle.b = 100;
		
		secColorOver.r = 120;
		secColorOver.g = 120;
		secColorOver.b = 120;
		
		secColorPress.r = 180;
		secColorPress.g = 180;
		secColorPress.b = 180;
		
		outlineColor.r = 100;
		outlineColor.g = 100;
		outlineColor.b = 100;
		
		textColor.r = 240;
		textColor.g = 240;
		textColor.b = 240;
	}
};