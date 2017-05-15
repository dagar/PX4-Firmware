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
 * @file Block.cpp
 *
 * Controller library code
 */

#include "Block.hpp"
#include "BlockParam.hpp"

#include <cmath>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

namespace control
{

Block::Block(SuperBlock *parent, const char *name) :
	_parent(parent),
	_name(name)
{
	if (getParent() != nullptr) {
		getParent()->getChildren().add(this);
	}
}

void Block::getName(char *name, size_t n)
{
	if (getParent() == nullptr) {
		strncpy(name, _name, n);
		// ensure string is terminated
		name[n - 1] = '\0';

	} else {
		char parentName[blockNameLengthMax];
		getParent()->getName(parentName, n);

		if (strcmp(_name, "") == 0) {
			strncpy(name, parentName, n);
			// ensure string is terminated
			name[n - 1] = '\0';

		} else {
			snprintf(name, n, "%s_%s", parentName, _name);
		}
	}
}

void Block::updateParams()
{
	// update int32 params
	//BlockParamInt *paramInt = _paramsInt.getHead();
	//int count = 0;


	for (const auto &dummy : _paramsInt) {
		dummy->update();
	}

	for (const auto &dummy : _paramsFloat) {
		dummy->update();
	}

	for (const auto &dummy : _paramsExtInt) {
		dummy->update();
	}

	for (const auto &dummy : _paramsExtFloat) {
		dummy->update();
	}

//
//	for (auto it = _paramsInt2.begin(); it != _paramsInt2.end(); ++it) {
//
//	}
//
//
//	while (paramInt != nullptr) {
//		if (count++ > maxParamsPerBlock) {
//			char name[blockNameLengthMax];
//			getName(name, blockNameLengthMax);
//			PX4_ERR("exceeded max int params for block: %s\n", name);
//			break;
//		}
//
//		//printf("updating param: %s\n", param->getName());
//		paramInt->update();
//		//paramInt = paramInt->getSibling();
//	}


//	// update float params
//	BlockParamFloat *paramFloat = _paramsFloat.getHead();
//	count = 0;
//
//	while (paramFloat != nullptr) {
//		if (count++ > maxParamsPerBlock) {
//			char name[blockNameLengthMax];
//			getName(name, blockNameLengthMax);
//			PX4_ERR("exceeded max float params for block: %s\n", name);
//			break;
//		}
//
//		//printf("updating param: %s\n", param->getName());
//		paramFloat->update();
//		paramFloat = paramFloat->getSibling();
//	}
}

void Block::updateSubscriptions()
{
	uORB::SubscriptionNode *sub = getSubscriptions().getHead();
	int count = 0;

	while (sub != nullptr) {
		if (count++ > maxSubscriptionsPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			PX4_ERR("exceeded max subscriptions for block: %s\n", name);
			break;
		}

		sub->update();
		sub = sub->getSibling();
	}
}

void Block::updatePublications()
{
	uORB::PublicationNode *pub = getPublications().getHead();
	int count = 0;

	while (pub != nullptr) {
		if (count++ > maxPublicationsPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			PX4_ERR("exceeded max publications for block: %s\n", name);
			break;
		}

		pub->update();
		pub = pub->getSibling();
	}
}

void SuperBlock::setDt(float dt)
{
	Block::setDt(dt);
	Block *child = getChildren().getHead();
	int count = 0;

	while (child != nullptr) {
		if (count++ > maxChildrenPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			PX4_ERR("exceeded max children for block: %s\n", name);
			break;
		}

		child->setDt(dt);
		child = child->getSibling();
	}
}

void SuperBlock::updateChildParams()
{
	Block *child = getChildren().getHead();
	int count = 0;

	while (child != nullptr) {
		if (count++ > maxChildrenPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			PX4_ERR("exceeded max children for block: %s\n", name);
			break;
		}

		child->updateParams();
		child = child->getSibling();
	}
}

void SuperBlock::updateChildSubscriptions()
{
	Block *child = getChildren().getHead();
	int count = 0;

	while (child != nullptr) {
		if (count++ > maxChildrenPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			PX4_ERR("exceeded max children for block: %s\n", name);
			break;
		}

		child->updateSubscriptions();
		child = child->getSibling();
	}
}

void SuperBlock::updateChildPublications()
{
	Block *child = getChildren().getHead();
	int count = 0;

	while (child != nullptr) {
		if (count++ > maxChildrenPerBlock) {
			char name[blockNameLengthMax];
			getName(name, blockNameLengthMax);
			PX4_ERR("exceeded max children for block: %s\n", name);
			break;
		}

		child->updatePublications();
		child = child->getSibling();
	}
}


} // namespace control

template class List<uORB::SubscriptionNode *>;
template class List<uORB::PublicationNode *>;
//template class List<control::BlockParamInt *>;
//template class List<control::BlockParamFloat *>;
