/****************************************************************************
 *
 *  Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <unit_test.h>
#include <containers/Queue.hpp>
#include <float.h>
#include <math.h>

class testContainer : public QueueNode<testContainer *>
{
public:
	int i{0};
};

class QueueTest : public UnitTest
{
public:
	virtual bool run_tests();

	bool test_push_front();

};

bool QueueTest::run_tests()
{
	ut_run_test(test_push_front);

	return (_tests_failed == 0);
}

bool QueueTest::test_push_front()
{
	Queue<testContainer *> list1;

	// size should be 0 initially
	ut_compare("size initially 0", list1.size(), 0);
	ut_assert_true(list1.empty());

	// insert 100
	for (int i = 0; i < 100; i++) {
		testContainer *t = new testContainer();
		t->i = i;
		list1.push_front(t);

		ut_compare("size increasing with i", list1.size(), i + 1);
		ut_assert_true(!list1.empty());
	}

	// verify full size (100)
	ut_assert_true(list1.size() == 100);

	int i = 99;

	for (auto t : list1) {
		// verify all elements were inserted in order
		ut_compare("stored i", i, t->i);
		i--;
	}

	// delete all elements
	list1.clear();

	// verify list has been cleared
	ut_assert_true(list1.empty());
	ut_assert_true(list1.size() == 0);

	return true;
}

ut_declare_test_c(test_Queue, QueueTest)
