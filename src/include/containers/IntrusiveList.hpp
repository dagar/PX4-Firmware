/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file IntrusiveList.hpp
 *
 * An intrusive linked list.
 */

#pragma once

#include <stdlib.h>

template<typename T>
class IntrusiveListNode
{
public:

	T next() const { return _next; }
	T prev() const { return _prev; }

	void setNext(T next) { _next = next; }
	void setPrev(T prev) { _prev = prev; }

	// join prev and next
	void remove()
	{
		if (_prev) {
			_prev->setNext(_next);
			_prev = nullptr;
		}

		if (_next) {
			_next->setPrev(_prev);
			_next = nullptr;
		}
	}

private:

	T _prev{nullptr};
	T _next{nullptr};
};

template<typename T>
class IntrusiveList
{
public:

	class Iterator
	{
	public:
		explicit Iterator(T node): _node(node) {}
		Iterator() = delete;

		T operator*() { return _node; }
		bool operator!=(const Iterator &it) const { return value() != it.value(); }

		Iterator &operator++()
		{
			_node = _node->next();
			return *this;
		}

		Iterator &operator--()
		{
			_node = _node->prev();
			return *this;
		}

		const T &value() const { return _node; }

	private:
		T _node;
	};

	Iterator begin() { return Iterator{front()}; }
	Iterator end() { return Iterator{back()}; }

	void push_front(T newNode)
	{
		newNode->setNext(front());
		newNode->setPrev(nullptr);
		_head = newNode;
	}

	void push_back(T newNode)
	{
		newNode->setNext(nullptr);
		newNode->setPrev(back());
		_tail = newNode;
	}

	void remove(const T node)
	{
		if (_head == node) {
			_head = _head->next();
		}

		if (_tail == node) {
			_tail = _tail->prev();
		}

		node->remove();
	}

	bool empty() const { return _head == nullptr; }

	size_t size()
	{
		size_t i = 0;

		for (const auto node : *this) {
			(void)node; // unused
			i++;
		}

		return i;
	}

	T front() const { return _head; }
	T back() const { return _tail; }

protected:

	T _head{nullptr};
	T _tail{nullptr};

};
