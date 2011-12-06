#pragma once

#include "ControlBase.h"
#include "MyButton.h"
#include "MyRadioButtons.h"

#define ITEMS_PADDING_X 10
#define ITEMS_PADDING_Y 30
#define ITEMS_HEIGHT 20

class MyScroller;
class MyButton;

class MyScrollList: public MyButton{
protected:
	vector<MyToggle*>items;
	float togglesWidth;
	float totalItemsHeight;
	int topVisibleItem, bottomVisibleItem;
	int maxVisibleItems;
	MyScroller *scroller;
	bool bShowScroller;
	virtual void onScrollerDrag(ControlEventArgs& e);
	virtual void onItemSelected(ControlEventArgs& e);
	int itemReleased;
	bool bNotifyItemSelected;
public:
	
	MyScrollList(string _name, float _x, float _y, float _w, float _h);
	~MyScrollList();
	
	virtual void addItem(string _itemName);
	virtual void update();
	virtual void draw();
	virtual void selectItem(string _itemName);
	virtual void selectItem(unsigned int itemId);
	virtual void setDefaultColors();
	virtual int getNumItems();
	virtual void adjustVertices();
	virtual void calcNumVisibleItems();
	
	ofEvent<ControlEventArgs> itemSelected;
	
protected:
	
	//internal events
	virtual void onPress(int x, int y, int button);
	virtual void onRelease(int x, int y, int button);
	virtual void onReleaseOutside(int x, int y, int button);
	virtual void onMouseMove(int x, int y);
	virtual void onRollOut();
	virtual void onDragOver(int x, int y, int button);
	virtual void onDragOutside(int x, int y, int button);
	
	virtual void adjustItemsPosition();
	virtual void adjustScrollPosition();
	
};

