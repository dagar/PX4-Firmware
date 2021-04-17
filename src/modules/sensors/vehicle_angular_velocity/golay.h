
namespace gram_sg
{

struct SavitzkyGolayFilterConfig {
	//! Window size is 2*m+1
	unsigned m = 5;

	//! Time at which the filter is applied
	// For real-time, should be t=m
	int t = 5;

	//! Polynomial order
	unsigned n = 3;

	//! Derivation order (0 for no derivation)
	unsigned s = 0;

	//! Time step
	double dt = 1;

	/**
	 * @brief Construct a filter with default weights
	 */
	SavitzkyGolayFilterConfig() = default;

	/**
	 * @brief Construct a filter with the specified configuration
	 *
	 * @param m Window size is `2*m+1`
	 * @param t Time at which the filter is applied
	 * - `t=m` for real-time filtering.
	 *   This uses only past information to determine the filter value and thus
	 *   does not introduce delay. However, this comes at the cost of filtering
	 *   accuracy as no future information is available.
	 * - `t=0` for smoothing. Uses both past and future information to determine the optimal
	 *   filtered value
	 * @param n Polynamial order
	 * @param s Derivation order
	 * - `0`: No derivation
	 * - `1`: First order derivative
	 * - `2`: Second order derivative
	 * @param dt Time step
	 */
	SavitzkyGolayFilterConfig(unsigned m, int t, unsigned n, unsigned s, double dt = 1.) :
		m(m),
		t(t),
		n(n),
		s(s),
		dt(dt)
	{
	}

	/**
	 * @brief Time at which the filter is evaluated
	 */
	int data_point() const { return t; }

	/**
	 * @brief Derivation order
	 */
	unsigned derivation_order() const { return s; }

	/**
	 * @brief Polynomial order
	 */
	unsigned order() const { return n; }

	/**
	 * @brief Full size of the filter's window `2*m+1`
	 */
	unsigned window_size() const { return 2 * m + 1; }

	/**
	 * @brief Time step
	 */
	double time_step() const { return dt; }
};

struct SavitzkyGolayFilter {

	SavitzkyGolayFilter(unsigned m, int t, unsigned n, unsigned s, double dt = 1.) :
		conf_(m, t, n, s, dt)
	{
		// Compute weights for the time window 2*m+1, for the t'th least-square
		// point of the s'th derivative
		weights_ = ComputeWeights(static_cast<int>(conf_.m), conf_.t, static_cast<int>(conf_.n), static_cast<int>(conf_.s));
		dt_ = std::pow(conf_.time_step(), conf_.derivation_order());
	}

	/**
	 * @brief Apply Savitzky-Golay convolution to the data x should have size 2*m+1
	 * As the function only applies a convolution, it runs in O(2m+1), and should
	 * be rather fast.
	 *
	 * @param v Container with the data to be filtered.
	 * Should have 2*m+1 elements
	 * Type of elements needs to be compatible with multiplication by a scalar,
	 * and addition with itself
	 * Common types would be std::vector<double>, std::vector<Eigen::VectorXd>, boost::circular_buffer<Eigen::Vector3d>...
	 *
	 * @return Filtered value according to the precomputed filter weights.
	 */
	template<typename ContainerT>
	typename ContainerT::value_type filter(const ContainerT &v) const
	{
		assert(v.size() == weights_.size() && v.size() > 0);
		using T = typename ContainerT::value_type;
		T res = weights_[0] * v[0];

		for (size_t i = 1; i < v.size(); ++i) {
			res += weights_[i] * v[i];
		}

		return res / dt_;
	}

	std::vector<double> weights() const { return weights_; }
	SavitzkyGolayFilterConfig config() const { return conf_; }

private:
	SavitzkyGolayFilterConfig conf_;

	std::vector<double> weights_;

	/*!
	 * @brief Calculates the generalized factorial \f$ (a)(a-1)(a-2)...(a-b+1) \f$
	 */
	double GenFact(const int a, const int b)
	{
		double gf = 1.;

		for (int j = (a - b) + 1; j <= a; j++) {
			gf *= j;
		}

		return gf;
	}

	/*!
	 * @brief Calculates the Gram Polynomial (s=0) or its sth derivative
	 *  evaluated at i, order k, over 2m+1 points
	 */
	double GramPoly(const int i, const int m, const int k, const int s)
	{
		if (k > 0) {
			return (4. * k - 2.) / (k * (2. * m - k + 1.)) * (i * GramPoly(i, m, k - 1, s) + s * GramPoly(i, m, k - 1, s - 1))
			       - ((k - 1.) * (2. * m + k)) / (k * (2. * m - k + 1.)) * GramPoly(i, m, k - 2, s);

		} else {
			if (k == 0 && s == 0) {
				return 1.;

			} else {
				return 0.;
			}
		}
	}

	/*!
	 * Weight Calculates the weight of the ith data point for the t'th
	 * Least-Square point of the s'th derivative, over `2m+1` points, order n
	 */
	double Weight(const int i, const int t, const int m, const int n, const int s)
	{
		double w = 0;

		for (int k = 0; k <= n; ++k) {
			w = w + (2 * k + 1) * (GenFact(2 * m, k) / GenFact(2 * m + k + 1, k + 1)) * GramPoly(i, m, k, 0) * GramPoly(t, m, k, s);
		}

		return w;
	}

	/*!
	 * Computes the weights for each data point over the window of size `2*m+1`,
	 * evaluated at time t, with order n and derivative s
	 */
	std::vector<double> ComputeWeights(const int m, const int t, const int n, const int s)
	{
		std::vector<double> weights(2 * m + 1);

		for (int i = 0; i < 2 * m + 1; ++i) {
			weights[i] = Weight(i - m, t, m, n, s);
		}

		return weights;
	}

	double dt_{0.f};
};
}
