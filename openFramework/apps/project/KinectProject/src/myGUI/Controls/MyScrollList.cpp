#include "MyScrollList.h"
#include "MyScroller.h"
#include "MyButton.h"

MyScrollList::MyScrollList(string _name, float _x, float _y, float _w, float _h):
MyButton(_name, _x, _y, _w, _h)
{
	scroller = NULL;
	setPos(_x, _y);
	setSize(_w, _h);
	adjustVertices();
	togglesWidth = width - ITEMS_PADDING_X * 2 - 10;
	setDefaultColors();
	topVisibleItem = 0;
	bottomVisibleItem = 0;
	calcNumVisibleItems();
	scroller = new MyScroller(x - ITEMS_PADDING_X + width, y + ITEMS_PADDING_Y, 10, height - ITEMS_PADDING_Y*2);
	bShowScroller = false;
	ofAddListener(scroller->drag, this, &MyScrollList::onScrollerDrag);
	bNotifyItemSelected = false;
	itemReleased = -1;
}


MyScrollList::~MyScrollList(){
	
}

void MyScrollList::addItem(string _itemName){
	MyToggle * item;
	if(items.size() == 0){
	    item = new MyToggle(_itemName, ITEMS_PADDING_X + x, ITEMS_PADDING_Y + y, togglesWidth, ITEMS_HEIGHT);
	} else {
		float posY = items.back()->y + ITEMS_HEIGHT;
		item = new MyToggle(_itemName, 10 + x, posY, togglesWidth, ITEMS_HEIGHT);
	}
	item->setStateText("", "");
	items.push_back(item);
	totalItemsHeight += ITEMS_HEIGHT;
	if(items.size() <= maxVisibleItems){
		topVisibleItem = 0;
		bottomVisibleItem = items.size();
	}  else {
		bShowScroller = true;
		topVisibleItem = (int)(scroller->getPosition() * (items.size() - maxVisibleItems));
		bottomVisibleItem = topVisibleItem + maxVisibleItems;
	}
}

void MyScrollList::update(){
	ControlBase::update();
	for(int i=topVisibleItem; i<bottomVisibleItem; i++){
		items[i]->update();
	}
	if(bShowScroller){
		scroller->update();
	}
	if(items.size() > 0){
		if(bNotifyItemSelected){
			bNotifyItemSelected = false;
			if(itemReleased > -1){
				ControlEventArgs e(items[itemReleased]);
				ofNotifyEvent(itemSelected, e, this);
			}
		}
	}
}

void MyScrollList::draw(){
	ControlBase::draw();
	for(int i=topVisibleItem; i<bottomVisibleItem; i++){
		items[i]->draw();
	}
	if(bShowScroller)scroller->draw();
}

void MyScrollList::selectItem(string _itemName){
	for(int i=0; i<items.size(); i++){
		if(items[i]->getName() == _itemName){
			selectItem(i);
			return;
		}
	}
}

void MyScrollList::selectItem(unsigned int _itemId){
	if(_itemId < items.size()){
		if(items[_itemId]->isOn()) return;
		for(int i=0; i<items.size(); i++){
			if(i == _itemId)continue;
			if(items[i]->isOn()){
				items[i]->turnOffToggle();
			}
		}
		items[_itemId]->turnOnToggle();
		itemReleased = _itemId;
		bNotifyItemSelected = true;
		if(items.size() > maxVisibleItems){
			//if(_itemId >= bottomVisibleItem or _itemId < topVisibleItem){
			topVisibleItem = _itemId - (int)(maxVisibleItems*0.5f);
			if(topVisibleItem < 0)topVisibleItem = 0;
			if(topVisibleItem > items.size() - maxVisibleItems){
				topVisibleItem = items.size() -maxVisibleItems; 
			}
			bottomVisibleItem = topVisibleItem + maxVisibleItems;
			adjustItemsPosition();
			adjustScrollPosition();
			//}
		}
	}
}

void MyScrollList::setDefaultColors(){
	colors.mainColorOver = colors.mainColorPress = colors.mainColorIdle;
}

int MyScrollList::getNumItems(){
	return items.size();	
}

void MyScrollList::adjustVertices(){
	//x - ITEMS_PADDING_X + width, y + ITEMS_PADDING_Y, 10, height - ITEMS_PADDING_Y*2
	if(scroller != NULL){
		scroller->x = x - (ITEMS_PADDING_X*1.5) + width;
		scroller->y = y + ITEMS_PADDING_Y;
		scroller->width = 10;
		scroller->height = height - ITEMS_PADDING_Y*2;
		scroller->adjustVertices();
	}
	ControlBase::adjustVertices();
}

void MyScrollList::calcNumVisibleItems(){
	maxVisibleItems = (int)((height - ITEMS_PADDING_Y*2) / ITEMS_HEIGHT);
}

void MyScrollList::onPress(int x, int y, int button){
	ofMouseEventArgs e;
	e.x = x;
	e.y = y;
	e.button = button;
	for(int i=topVisibleItem; i<bottomVisibleItem; i++){
		items[i]->_mousePressed(e);
	}
	scroller->_mousePressed(e);
}

void MyScrollList::onRelease(int _x, int _y, int _button){
	ofMouseEventArgs e;
	e.x = _x;
	e.y = _y;
	e.button = _button;
		scroller->_mouseReleased(e);
	
	itemReleased = -1;
	for(int i=topVisibleItem; i<bottomVisibleItem; i++){
		if(items[i]->hitTest(_x, _y)){
			itemReleased = i;
			break;
		} 
	}
	if(itemReleased == -1) return;
	if(items[itemReleased]->isOn())return;
	for(int i=topVisibleItem; i<bottomVisibleItem; i++){
		if(i != itemReleased){
			if(items[i]->isOn()) items[i]->turnOffToggle();
		}
	}
	bNotifyItemSelected = true;
	for(int i=topVisibleItem; i<bottomVisibleItem; i++){
		items[i]->_mouseReleased(e);
	}
}

void MyScrollList::onRollOut(){
	for(int i=topVisibleItem; i<bottomVisibleItem; i++){
		if(items[i]->getState() == CONTROL_STATE_OVER){
			items[i]->setState(CONTROL_STATE_IDLE);
		}
	}
	scroller->setState(CONTROL_STATE_IDLE);
}

void MyScrollList::onReleaseOutside(int x, int y, int button){
	onRelease(x, y, button);
}

void MyScrollList::onMouseMove(int x, int y){
	ofMouseEventArgs e;
	e.x = x;
	e.y = y;
	e.button = 0;
	for(int i=topVisibleItem; i<bottomVisibleItem; i++){
		items[i]->_mouseMoved(e);
	}
	scroller->_mouseMoved(e);
}

void MyScrollList::onDragOver(int x, int y, int button){
	ofMouseEventArgs e;
	e.x = x;
	e.y = y;
	e.button = button;
	for(int i=topVisibleItem; i<bottomVisibleItem; i++){
		items[i]->_mouseDragged(e);
	}
	scroller->_mouseDragged(e);
}


void MyScrollList::onDragOutside(int x, int y, int button){
	onDragOver(x, y, button);
}

void MyScrollList::onScrollerDrag(ControlEventArgs& e){
	topVisibleItem = (int)(scroller->getPosition() * (items.size() - maxVisibleItems));
	bottomVisibleItem = topVisibleItem + maxVisibleItems;
	adjustItemsPosition();
}

void MyScrollList::onItemSelected(ControlEventArgs& e){
	int itemIndex = 0;
	for(int i=0; i<items.size(); i++){
		if(items[i] == e.control){
			itemIndex = i;
		} else {
			if(items[i]->isOn()){
				items[i]->turnOffToggle();
			}
		}
	}
	if(itemIndex > bottomVisibleItem){
		bottomVisibleItem = itemIndex;
		topVisibleItem = bottomVisibleItem - maxVisibleItems;
	} else if(itemIndex < topVisibleItem){
		topVisibleItem = itemIndex;
		bottomVisibleItem = topVisibleItem + maxVisibleItems;
	}
	ofNotifyEvent(itemSelected, e, this);
}

void MyScrollList::adjustItemsPosition(){
	for(int i=0; i<maxVisibleItems; i++){
		items[topVisibleItem + i]->y = y + ITEMS_PADDING_Y + (ITEMS_HEIGHT * i);
		items[topVisibleItem + i]->adjustVertices();
	}
}

void MyScrollList::adjustScrollPosition(){
	float pos = ((float)topVisibleItem) / (items.size() - maxVisibleItems);
	scroller->setPosition(pos);
}


