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
 * @file Blockparam.cpp
 *
 * Controller library code
 */

#include "BlockParam.hpp"

#include "Block.hpp"

#include <cstring>

namespace control
{

template <class T>
void BlockParam<T>::init(Block *parent, const char *name, bool parent_prefix)
{
	// 16 character param name + NULL terminated
	constexpr size_t name_length = 17;
	char fullname[name_length];

	if (parent == nullptr) {
		strncpy(fullname, name, name_length);

	} else {
		char parentName[name_length];
		parent->getName(parentName, name_length);

		if (strcmp(name, "") == 0) {
			strncpy(fullname, parentName, name_length);
			// ensure string is terminated
			fullname[sizeof(fullname) - 1] = '\0';

		} else if (parent_prefix) {
			snprintf(fullname, name_length, "%s_%s", parentName, name);

		} else {
			strncpy(fullname, name, name_length);
			// ensure string is terminated
			fullname[sizeof(fullname) - 1] = '\0';
		}
	}

	_handle = param_find(fullname);

	if (_handle == PARAM_INVALID) {
		PX4_ERR("error finding param: %s\n", fullname);

	} else {
		// handle valid

		// check that param type matches BlockParam
		if (block_type() != param_type(_handle)) {
			PX4_ERR("error BlockParam type mismatch: %s\n", fullname);
		}

		// add to Block
		if (parent != nullptr) {
			parent->addParam(this);
		}

		update();
	}
};

template <> param_type_e BlockParam<int32_t>::block_type() const { return PARAM_TYPE_INT32; }
template <> param_type_e BlockParam<int32_t &>::block_type() const { return PARAM_TYPE_INT32; }

template <> param_type_e BlockParam<float>::block_type() const { return PARAM_TYPE_FLOAT; }
template <> param_type_e BlockParam<float &>::block_type() const { return PARAM_TYPE_FLOAT; }

template <>
BlockParam<int32_t>::BlockParam(Block *parent, const char *name, bool parent_prefix) : _val(0)
{
	init(parent, name, parent_prefix);
}

template <>
BlockParam<float>::BlockParam(Block *parent, const char *name, bool parent_prefix) : _val(0.0f)
{
	init(parent, name, parent_prefix);
}

template <>
BlockParam<int32_t &>::BlockParam(Block *parent, const char *name, bool parent_prefix, int32_t &external_val)
	: _val(external_val)
{
	init(parent, name, parent_prefix);
}

template <>
BlockParam<float &>::BlockParam(Block *parent, const char *name, bool parent_prefix, float &external_val)
	: _val(external_val)
{
	init(parent, name, parent_prefix);
}

} // namespace control
