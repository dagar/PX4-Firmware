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

/**
 * @file test_List.cpp
 * Tests flight path smoothing algorithms.
 */

#include <unit_test.h>
#include <containers/List.hpp>
#include <float.h>
#include <math.h>

class testContainer : public ListNode<testContainer *>
{
public:
	int i{0};
};

class ListTest : public UnitTest
{
public:
	virtual bool run_tests();

	bool test_push_front();
	bool test_push_back();
	bool test_range_based_for();

};

bool ListTest::run_tests()
{
	ut_run_test(test_push_front);
	ut_run_test(test_push_back);
	ut_run_test(test_range_based_for);

	return (_tests_failed == 0);
}

bool ListTest::test_push_front()
{
	List<testContainer *> list1;

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

bool ListTest::test_push_back()
{
	List<testContainer *> list1;

	// size should be 0 initially
	ut_compare("size initially 0", list1.size(), 0);
	ut_assert_true(list1.empty());

	// insert 100
	for (int i = 0; i < 100; i++) {
		testContainer *t = new testContainer();
		t->i = i;
		list1.push_back(t);

		//ut_compare("size increasing with i", list1.size(), i+1);
		ut_assert_true(!list1.empty());
	}

	// first element should be 0, last 99
	ut_compare("first 0", list1.front()->i, 0);
	ut_compare("last 99", list1.back()->i, 99);

	// verify full size (100)
	ut_compare("size check", list1.size(), 100);

	// verify all elements were inserted in order
	int i = 0;
	auto t1 = list1.front();

	while (t1 != nullptr) {
		ut_compare("check count", i, t1->i);
		t1 = t1->next();
		i++;
	}

	// verify all elements were inserted in order
	i = 0;

	for (auto t2 : list1) {
		// verify all elements were inserted in order
		ut_compare("check count", i, t2->i);
		i++;
	}

	// delete all elements
	list1.clear();

	// verify list has been cleared
	ut_assert_true(list1.empty());
	ut_compare("size check", list1.size(), 0);


	// prevent same thing from being inserted twice
//	traversing the list
//	edge cases:
//	traversing an empty list
//	traversing list where 1+ stored values are NULL
//	operations (if applicable):
//	deleting from the list
//	inserting into the list
//	inserting a sub-list into the linked list
//	traversing the list backwards (if doubly-linked list)
//	concurrency tests (if applicable):
//	race condition tests


	return true;
}

bool ListTest::test_range_based_for()
{
	List<testContainer *> list1;

	// size should be 0 initially
	ut_compare("size initially 0", list1.size(), 0);
	ut_assert_true(list1.empty());

	// insert 100
	for (int i = 0; i < 100; i++) {
		testContainer *t = new testContainer();
		t->i = i;

		list1.push_back(t);

		ut_compare("size increasing with i", list1.size(), i + 1);
		ut_assert_true(!list1.empty());
	}

	// first element should be 0, last 99
	ut_compare("first 0", list1.front()->i, 0);
	ut_compare("last 99", list1.back()->i, 99);

	// verify all elements were inserted in order
	int i = 0;
	auto t1 = list1.front();

	while (t1 != nullptr) {
		ut_compare("check count", i, t1->i);
		t1 = t1->next();
		i++;
	}

	// verify full size (100)
	ut_assert_true(list1.size() == 100);

	i = 0;

	for (auto t2 : list1) {
		ut_compare("range based for i", i, t2->i);
		i++;
	}

	// verify full size (100)
	ut_compare("size check", list1.size(), 100);

	// delete all elements
	list1.clear();

	// verify list has been cleared
	ut_assert_true(list1.empty());
	ut_compare("size check", list1.size(), 0);

	return true;
}

ut_declare_test_c(test_List, ListTest)
