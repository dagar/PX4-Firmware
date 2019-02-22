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
 * @file List.hpp
 *
 * A linked list.
 */

#pragma once

template<typename T>
class ListNode
{
public:

	void setSibling(T sibling) { _sibling = sibling; }
	const T getSibling() const { return _sibling; }

	void removeSibling() { _sibling = nullptr; }

protected:

	T _sibling{nullptr};

};

template<typename T>
class List
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
			_node = _node->getSibling();
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
		newNode->setSibling(front());
		_head = newNode;
	}

//	void push_back(T newNode)
//	{
//		newNode->setSibling(back());
//		_tail = newNode;
//	}

	void pop_front()
	{
		T &oldHead = _head;
		_head = _head->getSibling();
		oldHead->removeSibling();
	}

//	void pop_back()
//	{
//		T& oldTail = _tail;
//		_tail = _head->getSibling();
//		oldHead.removeSibling();
//	}

	void add(T newNode)
	{
		newNode->setSibling(getHead());
		_head = newNode;
	}

	bool remove(const T removeNode)
	{
		if (removeNode == _head) {
			_head = nullptr;
			return true;

		} else if (removeNode == _tail) {
			// TODO: find node pointing to tail and update
			_tail = nullptr;
			return true;

		} else {
			for (T node = _head; node != nullptr; node = node->getSibling()) {
				// is sibling the node to remove?
				if (node->getSibling() == removeNode) {
					// replace sibling
					auto sibling = node->getSibling();

					if (sibling != nullptr) {
						node->setSibling(sibling->getSibling());

					} else {
						node->removeSibling();
					}

					return true;
				}
			}
		}

		return false;
	}

	const T getHead() const { return _head; }

	bool empty() const { return _head == nullptr; }

	T front() const { return _head; }
	T back() const { return _tail; }

	// range based for loop
	// comparison operator?

	// cbegin
	// cend

//	T begin() { return _head; }
//	T end() { return _tail; }
//
//	const T begin() const { return _head; }
//	const T end() const { return _tail; }

	//iterator operator++() { ++ptr; return *this; }

protected:

	T _head{nullptr};
	T _tail{nullptr};

};
