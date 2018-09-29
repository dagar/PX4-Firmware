
#include <unit_test.h>

#include <bitset.hpp>

class BitsetTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool constructTest();
	bool setAllTest();
	bool setRandomTest();

};

bool BitsetTest::run_tests()
{
	ut_run_test(constructTest);
	ut_run_test(setAllTest);
	ut_run_test(setRandomTest);

	return (_tests_failed == 0);
}


ut_declare_test_c(test_bitset, BitsetTest)

bool BitsetTest::constructTest()
{
	bitset<10> test_bitset1;

	ut_compare("bitset size 10", test_bitset1.size(), 10);
	ut_compare("bitset init count 0", test_bitset1.count(), 0);

	for (int i = 0; i < test_bitset1.size(); i++) {
		ut_compare("bitset not set by default", test_bitset1[i], false);
	}

	return true;
}

bool BitsetTest::setAllTest()
{
	bitset<100> test_bitset2;

	ut_compare("bitset size 100", test_bitset2.size(), 100);
	ut_compare("bitset init count 0", test_bitset2.count(), 0);

	for (int i = 0; i < test_bitset2.size(); i++) {
		ut_compare("bitset not set by default", test_bitset2[i], false);
	}

	// set all
	for (int i = 0; i < test_bitset2.size(); i++) {
		test_bitset2.set(i, true);
	}

	// check count
	ut_compare("bitset count", test_bitset2.count(), 100);

	// verify all set
	for (int i = 0; i < test_bitset2.size(); i++) {
		ut_compare("bitset not true", test_bitset2[i], true);
	}

	// set all back to false
	for (int i = 0; i < test_bitset2.size(); i++) {
		test_bitset2.set(i, false);
	}

	// check count
	ut_compare("bitset count", test_bitset2.count(), 0);

	// verify all no longer set
	for (int i = 0; i < test_bitset2.size(); i++) {
		ut_compare("bitset not false", test_bitset2[i], false);
	}

	return true;
}

bool BitsetTest::setRandomTest()
{
	bitset<999> test_bitset3;

	ut_compare("bitset size 999", test_bitset3.size(), 999);
	ut_compare("bitset init count 0", test_bitset3.count(), 0);

	for (int i = 0; i < test_bitset3.size(); i++) {
		ut_compare("bitset not set by default", test_bitset3[i], false);
	}

	// random set and verify 100 elements
	const int random_test_size = 5;
	int random_array[random_test_size] = { 3, 1, 4, 5, 9 };

	// set random elements
	for (auto x : random_array) {
		test_bitset3.set(x, true);
		ut_less_than("invalid test element range", x, test_bitset3.size());
	}

	// check count
	ut_compare("bitset count", test_bitset3.count(), random_test_size);

	// check that only random elements are set
	for (int i = 0; i < test_bitset3.size(); i++) {

		// is i in the random test array
		// if so it should be set
		bool i_in_random = false;

		for (auto x : random_array) {
			if (i == x) {
				i_in_random = true;
			}
		}

		if (i_in_random) {
			ut_compare("bitset true", test_bitset3[i], true);

		} else {
			ut_compare("bitset false", test_bitset3[i], false);
		}
	}

	// set all back to false
	for (int i = 0; i < test_bitset3.size(); i++) {
		test_bitset3.set(i, false);
	}

	// check count
	ut_compare("bitset count", test_bitset3.count(), 0);

	// verify all no longer set
	for (int i = 0; i < test_bitset3.size(); i++) {
		ut_compare("bitset not false", test_bitset3[i], false);
	}

	return true;
}
