// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <matrix/math.hpp>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: compute_sideslip_h_and_k
 *
 * Args:
 *     state: Matrix25_1
 *     P: Matrix24_24
 *     innov_var: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     H: Matrix24_1
 *     K: Matrix24_1
 */
template <typename Scalar>
void ComputeSideslipHAndK(const matrix::Matrix<Scalar, 25, 1>& state,
                          const matrix::Matrix<Scalar, 24, 24>& P, const Scalar innov_var,
                          const Scalar epsilon, matrix::Matrix<Scalar, 24, 1>* const H = nullptr,
                          matrix::Matrix<Scalar, 24, 1>* const K = nullptr) {
  // Total ops: 485

  // Input arrays

  // Intermediate terms (46)
  const Scalar _tmp0 = std::pow(state(2, 0), Scalar(2));
  const Scalar _tmp1 = std::pow(state(3, 0), Scalar(2));
  const Scalar _tmp2 = 1 - 2 * _tmp1;
  const Scalar _tmp3 = -2 * _tmp0 + _tmp2;
  const Scalar _tmp4 = -state(22, 0) + state(4, 0);
  const Scalar _tmp5 = 2 * state(0, 0);
  const Scalar _tmp6 = _tmp5 * state(3, 0);
  const Scalar _tmp7 = 2 * state(1, 0);
  const Scalar _tmp8 = _tmp7 * state(2, 0);
  const Scalar _tmp9 = _tmp6 + _tmp8;
  const Scalar _tmp10 = -state(23, 0) + state(5, 0);
  const Scalar _tmp11 = _tmp5 * state(2, 0);
  const Scalar _tmp12 = -_tmp11;
  const Scalar _tmp13 = _tmp7 * state(3, 0);
  const Scalar _tmp14 = _tmp12 + _tmp13;
  const Scalar _tmp15 = _tmp10 * _tmp9 + _tmp14 * state(6, 0) + _tmp3 * _tmp4;
  const Scalar _tmp16 =
      _tmp15 + epsilon * (2 * math::min<Scalar>(0, (((_tmp15) > 0) - ((_tmp15) < 0))) + 1);
  const Scalar _tmp17 = Scalar(1.0) / (_tmp16);
  const Scalar _tmp18 = 2 * state(2, 0) * state(3, 0);
  const Scalar _tmp19 = _tmp7 * state(0, 0);
  const Scalar _tmp20 = std::pow(state(0, 0), Scalar(2));
  const Scalar _tmp21 = std::pow(state(1, 0), Scalar(2));
  const Scalar _tmp22 = -_tmp21;
  const Scalar _tmp23 = _tmp20 + _tmp22;
  const Scalar _tmp24 = _tmp17 * (_tmp10 * (_tmp18 - _tmp19) + _tmp4 * (_tmp11 + _tmp13) +
                                  state(6, 0) * (-_tmp0 + _tmp1 + _tmp23));
  const Scalar _tmp25 = -_tmp13;
  const Scalar _tmp26 = -_tmp20;
  const Scalar _tmp27 = _tmp0 - _tmp1;
  const Scalar _tmp28 = _tmp2 - 2 * _tmp21;
  const Scalar _tmp29 = -_tmp6;
  const Scalar _tmp30 = _tmp29 + _tmp8;
  const Scalar _tmp31 = _tmp18 + _tmp19;
  const Scalar _tmp32 = _tmp30 * _tmp4 + _tmp31 * state(6, 0);
  const Scalar _tmp33 = (_tmp10 * _tmp28 + _tmp32) / std::pow(_tmp16, Scalar(2));
  const Scalar _tmp34 = _tmp33 * (_tmp10 * (-_tmp18 + _tmp19) + _tmp4 * (_tmp12 + _tmp25) +
                                  state(6, 0) * (_tmp21 + _tmp26 + _tmp27));
  const Scalar _tmp35 =
      _tmp17 * (_tmp10 * (_tmp29 - _tmp8) + _tmp4 * (_tmp0 + _tmp1 + _tmp22 + _tmp26) +
                state(6, 0) * (_tmp11 + _tmp25)) -
      _tmp33 * (_tmp10 * (_tmp23 + _tmp27) + _tmp32);
  const Scalar _tmp36 = _tmp3 * _tmp33;
  const Scalar _tmp37 = _tmp17 * _tmp30;
  const Scalar _tmp38 = -_tmp36 + _tmp37;
  const Scalar _tmp39 = _tmp33 * _tmp9;
  const Scalar _tmp40 = _tmp17 * _tmp28;
  const Scalar _tmp41 = -_tmp39 + _tmp40;
  const Scalar _tmp42 = -_tmp14 * _tmp33 + _tmp17 * _tmp31;
  const Scalar _tmp43 = _tmp36 - _tmp37;
  const Scalar _tmp44 = _tmp39 - _tmp40;
  const Scalar _tmp45 = Scalar(1.0) / (math::max<Scalar>(epsilon, innov_var));

  // Output terms (2)
  if (H != nullptr) {
    matrix::Matrix<Scalar, 24, 1>& _h = (*H);

    _h.setZero();

    _h(0, 0) = _tmp24;
    _h(1, 0) = -_tmp34;
    _h(2, 0) = _tmp35;
    _h(3, 0) = _tmp38;
    _h(4, 0) = _tmp41;
    _h(5, 0) = _tmp42;
    _h(21, 0) = _tmp43;
    _h(22, 0) = _tmp44;
  }

  if (K != nullptr) {
    matrix::Matrix<Scalar, 24, 1>& _k = (*K);

    _k(0, 0) =
        _tmp45 * (P(0, 0) * _tmp24 - P(0, 1) * _tmp34 + P(0, 2) * _tmp35 + P(0, 21) * _tmp43 +
                  P(0, 22) * _tmp44 + P(0, 3) * _tmp38 + P(0, 4) * _tmp41 + P(0, 5) * _tmp42);
    _k(1, 0) =
        _tmp45 * (P(1, 0) * _tmp24 - P(1, 1) * _tmp34 + P(1, 2) * _tmp35 + P(1, 21) * _tmp43 +
                  P(1, 22) * _tmp44 + P(1, 3) * _tmp38 + P(1, 4) * _tmp41 + P(1, 5) * _tmp42);
    _k(2, 0) =
        _tmp45 * (P(2, 0) * _tmp24 - P(2, 1) * _tmp34 + P(2, 2) * _tmp35 + P(2, 21) * _tmp43 +
                  P(2, 22) * _tmp44 + P(2, 3) * _tmp38 + P(2, 4) * _tmp41 + P(2, 5) * _tmp42);
    _k(3, 0) =
        _tmp45 * (P(3, 0) * _tmp24 - P(3, 1) * _tmp34 + P(3, 2) * _tmp35 + P(3, 21) * _tmp43 +
                  P(3, 22) * _tmp44 + P(3, 3) * _tmp38 + P(3, 4) * _tmp41 + P(3, 5) * _tmp42);
    _k(4, 0) =
        _tmp45 * (P(4, 0) * _tmp24 - P(4, 1) * _tmp34 + P(4, 2) * _tmp35 + P(4, 21) * _tmp43 +
                  P(4, 22) * _tmp44 + P(4, 3) * _tmp38 + P(4, 4) * _tmp41 + P(4, 5) * _tmp42);
    _k(5, 0) =
        _tmp45 * (P(5, 0) * _tmp24 - P(5, 1) * _tmp34 + P(5, 2) * _tmp35 + P(5, 21) * _tmp43 +
                  P(5, 22) * _tmp44 + P(5, 3) * _tmp38 + P(5, 4) * _tmp41 + P(5, 5) * _tmp42);
    _k(6, 0) =
        _tmp45 * (P(6, 0) * _tmp24 - P(6, 1) * _tmp34 + P(6, 2) * _tmp35 + P(6, 21) * _tmp43 +
                  P(6, 22) * _tmp44 + P(6, 3) * _tmp38 + P(6, 4) * _tmp41 + P(6, 5) * _tmp42);
    _k(7, 0) =
        _tmp45 * (P(7, 0) * _tmp24 - P(7, 1) * _tmp34 + P(7, 2) * _tmp35 + P(7, 21) * _tmp43 +
                  P(7, 22) * _tmp44 + P(7, 3) * _tmp38 + P(7, 4) * _tmp41 + P(7, 5) * _tmp42);
    _k(8, 0) =
        _tmp45 * (P(8, 0) * _tmp24 - P(8, 1) * _tmp34 + P(8, 2) * _tmp35 + P(8, 21) * _tmp43 +
                  P(8, 22) * _tmp44 + P(8, 3) * _tmp38 + P(8, 4) * _tmp41 + P(8, 5) * _tmp42);
    _k(9, 0) =
        _tmp45 * (P(9, 0) * _tmp24 - P(9, 1) * _tmp34 + P(9, 2) * _tmp35 + P(9, 21) * _tmp43 +
                  P(9, 22) * _tmp44 + P(9, 3) * _tmp38 + P(9, 4) * _tmp41 + P(9, 5) * _tmp42);
    _k(10, 0) =
        _tmp45 * (P(10, 0) * _tmp24 - P(10, 1) * _tmp34 + P(10, 2) * _tmp35 + P(10, 21) * _tmp43 +
                  P(10, 22) * _tmp44 + P(10, 3) * _tmp38 + P(10, 4) * _tmp41 + P(10, 5) * _tmp42);
    _k(11, 0) =
        _tmp45 * (P(11, 0) * _tmp24 - P(11, 1) * _tmp34 + P(11, 2) * _tmp35 + P(11, 21) * _tmp43 +
                  P(11, 22) * _tmp44 + P(11, 3) * _tmp38 + P(11, 4) * _tmp41 + P(11, 5) * _tmp42);
    _k(12, 0) =
        _tmp45 * (P(12, 0) * _tmp24 - P(12, 1) * _tmp34 + P(12, 2) * _tmp35 + P(12, 21) * _tmp43 +
                  P(12, 22) * _tmp44 + P(12, 3) * _tmp38 + P(12, 4) * _tmp41 + P(12, 5) * _tmp42);
    _k(13, 0) =
        _tmp45 * (P(13, 0) * _tmp24 - P(13, 1) * _tmp34 + P(13, 2) * _tmp35 + P(13, 21) * _tmp43 +
                  P(13, 22) * _tmp44 + P(13, 3) * _tmp38 + P(13, 4) * _tmp41 + P(13, 5) * _tmp42);
    _k(14, 0) =
        _tmp45 * (P(14, 0) * _tmp24 - P(14, 1) * _tmp34 + P(14, 2) * _tmp35 + P(14, 21) * _tmp43 +
                  P(14, 22) * _tmp44 + P(14, 3) * _tmp38 + P(14, 4) * _tmp41 + P(14, 5) * _tmp42);
    _k(15, 0) =
        _tmp45 * (P(15, 0) * _tmp24 - P(15, 1) * _tmp34 + P(15, 2) * _tmp35 + P(15, 21) * _tmp43 +
                  P(15, 22) * _tmp44 + P(15, 3) * _tmp38 + P(15, 4) * _tmp41 + P(15, 5) * _tmp42);
    _k(16, 0) =
        _tmp45 * (P(16, 0) * _tmp24 - P(16, 1) * _tmp34 + P(16, 2) * _tmp35 + P(16, 21) * _tmp43 +
                  P(16, 22) * _tmp44 + P(16, 3) * _tmp38 + P(16, 4) * _tmp41 + P(16, 5) * _tmp42);
    _k(17, 0) =
        _tmp45 * (P(17, 0) * _tmp24 - P(17, 1) * _tmp34 + P(17, 2) * _tmp35 + P(17, 21) * _tmp43 +
                  P(17, 22) * _tmp44 + P(17, 3) * _tmp38 + P(17, 4) * _tmp41 + P(17, 5) * _tmp42);
    _k(18, 0) =
        _tmp45 * (P(18, 0) * _tmp24 - P(18, 1) * _tmp34 + P(18, 2) * _tmp35 + P(18, 21) * _tmp43 +
                  P(18, 22) * _tmp44 + P(18, 3) * _tmp38 + P(18, 4) * _tmp41 + P(18, 5) * _tmp42);
    _k(19, 0) =
        _tmp45 * (P(19, 0) * _tmp24 - P(19, 1) * _tmp34 + P(19, 2) * _tmp35 + P(19, 21) * _tmp43 +
                  P(19, 22) * _tmp44 + P(19, 3) * _tmp38 + P(19, 4) * _tmp41 + P(19, 5) * _tmp42);
    _k(20, 0) =
        _tmp45 * (P(20, 0) * _tmp24 - P(20, 1) * _tmp34 + P(20, 2) * _tmp35 + P(20, 21) * _tmp43 +
                  P(20, 22) * _tmp44 + P(20, 3) * _tmp38 + P(20, 4) * _tmp41 + P(20, 5) * _tmp42);
    _k(21, 0) =
        _tmp45 * (P(21, 0) * _tmp24 - P(21, 1) * _tmp34 + P(21, 2) * _tmp35 + P(21, 21) * _tmp43 +
                  P(21, 22) * _tmp44 + P(21, 3) * _tmp38 + P(21, 4) * _tmp41 + P(21, 5) * _tmp42);
    _k(22, 0) =
        _tmp45 * (P(22, 0) * _tmp24 - P(22, 1) * _tmp34 + P(22, 2) * _tmp35 + P(22, 21) * _tmp43 +
                  P(22, 22) * _tmp44 + P(22, 3) * _tmp38 + P(22, 4) * _tmp41 + P(22, 5) * _tmp42);
    _k(23, 0) =
        _tmp45 * (P(23, 0) * _tmp24 - P(23, 1) * _tmp34 + P(23, 2) * _tmp35 + P(23, 21) * _tmp43 +
                  P(23, 22) * _tmp44 + P(23, 3) * _tmp38 + P(23, 4) * _tmp41 + P(23, 5) * _tmp42);
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
