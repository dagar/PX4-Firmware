
#include <unit_test.h>

#include <px4_eigen.h>

#include <Eigen/Dense>
using namespace Eigen;

class EigenTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool eigenTests();
};

bool EigenTest::run_tests()
{
	ut_run_test(eigenTests);

	return (_tests_failed == 0);
}


ut_declare_test_c(test_eigen, EigenTest)

bool EigenTest::eigenTests()
{
	MatrixXd m(2, 2);
	m(0, 0) = 3;
	m(1, 0) = 2.5;
	m(0, 1) = -1;
	m(1, 1) = m(1, 0) + m(0, 1);

	VectorXd v(2);
	v(0) = 4;
	v(1) = v(0) - 1;

	Matrix3f x;
	x = Matrix3f::Zero();
	x = Matrix3f::Ones();
	x = Matrix3f::Constant(1);
	x = Matrix3f::Random();
	x.setZero();
	x.setOnes();
	x.setConstant(42);
	x.setRandom();

	return 1;
}
