/*
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2017 Sensnology AB
* Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*/
#ifndef List_h
#define List_h

/******************************************
List: data structure to dinamically store different types of objects
*/

template<typename T> class List {
public:
	typedef T* iterator;
	List() {
		_internalArray = NULL;
		_endPosition = 0;
		_allocBlocks = 0;
	}
	~List() {
		delete[] _internalArray;
		_internalArray = NULL;
		_endPosition = 0;
		_allocBlocks = 0;
	}
	void push(T item) {
		if (_endPosition == _allocBlocks) _AllocOneBlock(false);
		_internalArray[_endPosition] = item;
		++_endPosition;
	}
	void pop() {
		if (_endPosition == 0) return;
		--_endPosition;
		_DeAllocOneBlock(false);
	}
	T get(int position = 1) {
		position = position -1;
		if (position > _endPosition) position = _endPosition;
		return _internalArray[position];
	}
	void clear() {
		T* newArray = NULL;
		if (_allocBlocks > 0) newArray = new T[_allocBlocks];
		delete[] _internalArray;
		_internalArray = newArray;
		_endPosition = 0;
	}
	inline iterator begin() { return _internalArray; }
	inline iterator end() { return _internalArray + _endPosition; }
	inline bool empty() { return (_endPosition == 0); }
	inline int size() { return _endPosition; }
	void allocateBlocks(int alloc) {
		_allocBlocks = alloc;
		T* newArray = new T[_allocBlocks];
		for (int i = 0; i < _endPosition; ++i) newArray[i] = _internalArray[i];
		delete[] _internalArray;
		_internalArray = newArray;
	}

private:
	T* _internalArray;
	int _endPosition;
	int _allocBlocks;
	void _AllocOneBlock(bool shiftItems) {
		++_allocBlocks;
		T* newArray = new T[_allocBlocks];
		for (int i = 0; i < _endPosition; ++i) newArray[shiftItems ? (i + 1) : i] = _internalArray[i];
		delete[] _internalArray;
		_internalArray = newArray;
	}
	void _DeAllocOneBlock(bool shiftItems) {
		--_allocBlocks;
		if (_allocBlocks == 0) {
			delete[] _internalArray;
			_internalArray = NULL;
			return;
		}
		T* newArray = new T[_allocBlocks];
		for (int i = 0; i < _endPosition; ++i) newArray[i] = _internalArray[shiftItems ? (i + 1) : i];
		delete[] _internalArray;
		_internalArray = newArray;
	}
};

#endif