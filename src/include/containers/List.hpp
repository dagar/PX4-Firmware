/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file List.hpp
 *
 * An intrusive linked list.
 */

#pragma once

#include <stdlib.h>

template<typename T>
class ListNode
{
public:

	T next() const { return _next; }
	T prev() const { return _prev; }

	void setNext(T new_next) { _next = new_next; }
	void setPrev(T new_prev) { _prev = new_prev; }

	bool operator!=(const ListNode<T> &other) const { return (_next != other.next()) && (_prev != other.prev()); }

private:

	T _prev{nullptr};
	T _next{nullptr};
};

template<typename T>
class List
{
public:

	struct Iterator {
		T node;
		Iterator(T v) : node(v) {}

		operator T() const         { return node; }
		operator T &()               { return node; }
		T operator* () const         { return node; }
		Iterator &operator++ ()      { if (node) { node = node->next(); }; return *this; }

		//bool operator!=(const Iterator &other) const { return *node != *other.node; }
	};

	Iterator begin() { return Iterator(front()); }
	Iterator end() { return Iterator(nullptr); }

	void push_front(T newNode)
	{
		if (empty()) {
			// list is empty, new node becomes head
			_head = newNode;
			_tail = nullptr;

		} else {
			// new node inserted before current head
			newNode->setNext(nullptr);
			newNode->setNext(_head);
			_head->setPrev(newNode);
			_head = newNode;
		}
	}

	void push_back(T newNode)
	{
		if (empty()) {
			// list is empty, new node becomes head
			newNode->setPrev(nullptr);
			newNode->setNext(nullptr);
			_head = newNode;
			_tail = newNode;

		} else {
			// new node inserted after current tail
			_tail->setNext(newNode);
			newNode->setPrev(_tail);
			newNode->setNext(nullptr);
			_tail = newNode;
		}
	}

	void remove(T node)
	{
		// connect previous and next nodes
		T prev = node->prev();
		T next = node->next();

		if (prev) {
			prev->setNext(next);
		}

		if (next) {
			next->setPrev(prev);
		}

		// special case if node is head
		if (_head == node) {
			_head = next;
		}

		// special case if node is tail
		if (_tail == node) {
			_tail = prev;
		}

		// clear node's previous and next
		node->setNext(nullptr);
		node->setPrev(nullptr);
	}

	void deleteNode(T node)
	{
		remove(node);
		delete node;
	}

	void clear()
	{
		auto node = _head;

		while (node != nullptr) {
			auto next = node->next();
			delete node;
			node = next;
		}

		_head = nullptr;
		_tail = nullptr;
	}

	bool empty() const { return (_head == nullptr); }

	size_t size()
	{
		size_t sz = 0;

		for (auto n : *this) {
			(void)n; // unused
			sz++;
		}

		return sz;
	}

	T front() const { return _head; }
	T back() const { return _tail; }

private:

	T _head{nullptr};
	T _tail{nullptr};

};
