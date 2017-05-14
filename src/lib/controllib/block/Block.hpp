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
 * @file Block.h
 *
 * Controller library code
 */

#pragma once

#include <containers/List.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <controllib/block/BlockParam.hpp>

#include <forward_list>

namespace control
{

static constexpr uint8_t maxChildrenPerBlock = 100;
static constexpr uint8_t maxParamsPerBlock = 100;
static constexpr uint8_t maxSubscriptionsPerBlock = 100;
static constexpr uint8_t maxPublicationsPerBlock = 100;
static constexpr uint8_t blockNameLengthMax = 40;

// forward declaration
class SuperBlock;

/**
 */
class __EXPORT Block : public ListNode<Block *>
{
public:
	Block(SuperBlock *parent, const char *name);

	virtual ~Block() = default;

	Block(const control::Block &) = delete;
	Block operator=(const control::Block &) = delete;

	virtual void updateParams();
	virtual void updateSubscriptions();
	virtual void updatePublications();

	void getName(char *name, size_t n);

	virtual void setDt(float dt) { _dt = dt; }
	float getDt() { return _dt; }

	void addParam(BlockParamInt *param) { /*_paramsInt.push_front(param);*/ }
	void addParam(BlockParamFloat *param) { /*_paramsFloat.push_front(param);*/ }
	void addParam(BlockParamExtInt *param) { /*_paramsExtInt.push_front(param);*/ }
	void addParam(BlockParamExtFloat *param) { /*_paramsExtFloat.push_front(param);*/ }

protected:

	SuperBlock *getParent() { return _parent; }
	List<uORB::SubscriptionNode *> &getSubscriptions() { return _subscriptions; }
	List<uORB::PublicationNode *> &getPublications() { return _publications; }

	SuperBlock *_parent;
	const char *_name;

	float _dt{0.0f};

	List<uORB::SubscriptionNode *> _subscriptions;
	List<uORB::PublicationNode *> _publications;

	//std::forward_list<BlockParamInt *> _paramsInt;
	//std::forward_list<BlockParamFloat *> _paramsFloat;
	//std::forward_list<BlockParamExtInt *> _paramsExtInt;
	//std::forward_list<BlockParamExtFloat *> _paramsExtFloat;

};

class __EXPORT SuperBlock : public Block
{
public:
	friend class Block;

	SuperBlock(SuperBlock *parent, const char *name) :
		Block(parent, name),
		_children()
	{
	}

	virtual ~SuperBlock() = default;

	virtual void setDt(float dt);

	virtual void updateParams()
	{
		Block::updateParams();

		if (getChildren().getHead() != nullptr) { updateChildParams(); }
	}

	virtual void updateSubscriptions()
	{
		Block::updateSubscriptions();

		if (getChildren().getHead() != nullptr) { updateChildSubscriptions(); }
	}

	virtual void updatePublications()
	{
		Block::updatePublications();

		if (getChildren().getHead() != nullptr) { updateChildPublications(); }
	}

protected:

	List<Block *> &getChildren() { return _children; }

	void updateChildParams();
	void updateChildSubscriptions();
	void updateChildPublications();

	List<Block *> _children;
};

} // namespace control
