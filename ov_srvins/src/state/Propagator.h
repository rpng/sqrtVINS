/*
 * Sqrt-VINS: A Sqrt-filter-based Visual-Inertial Navigation System
 * Copyright (C) 2025-2026 Yuxiang Peng
 * Copyright (C) 2025-2026 Chuchu Chen
 * Copyright (C) 2025-2026 Kejian Wu
 * Copyright (C) 2018-2026 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program. If not, see
 * <https://www.gnu.org/licenses/>.
 */





#ifndef OV_SRVINS_STATE_PROPAGATOR_H
#define OV_SRVINS_STATE_PROPAGATOR_H

#include <memory>
#include <mutex>

#include "utils/NoiseManager.h"

#include "utils/print.h"
#include "utils/sensor_data.h"

namespace ov_srvins {

class State;

/**
 * @brief Performs the state covariance and mean propagation using imu
 * measurements
 *
 * We will first select what measurements we need to propagate with.
 * We then compute the state transition matrix at each step and update the state
 * and covariance. For derivations look at @ref propagation page which has
 * detailed equations.
 */
class Propagator {

public:
  /**
   * @brief Default constructor
   * @param noises imu noise characteristics (continuous time)
   * @param gravity_mag Global gravity magnitude of the system (normally 9.81)
   */
  Propagator(NoiseManager noises, DataType gravity_mag);

  /**
   * @brief Stores incoming inertial readings
   * @param message Contains our timestamp and inertial information
   * @param oldest_time Time that we can discard measurements before (in IMU
   * clock)
   */
  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1);
  /**
   * @brief This will remove any IMU measurements that are older then the given
   * measurement time
   * @param oldest_time Time that we can discard measurements before (in IMU
   * clock)
   */
  void clean_old_imu_measurements(double oldest_time);

  /**
   * @brief Propagate state up to given timestamp and then clone
   *
   * This will first collect all imu readings that occured between the
   * *current* state time and the new time we want the state to be at.
   * If we don't have any imu readings we will try to extrapolate into the
   * future. After propagating the mean and covariance using our dynamics, We
   * clone the current imu pose as a new clone in our state.
   *
   * @param state Pointer to state
   * @param timestamp Time to propagate to and clone at (CAM clock frame)
   */
  void propagate_and_clone(std::shared_ptr<State> state, double timestamp);

  /**
   * @brief Gets what the state and its covariance will be at a given timestamp
   *
   * This can be used to find what the state will be in the "future" without
   * propagating it. This will propagate a clone of the current IMU state and
   * its covariance matrix. This is typically used to provide high frequency
   * pose estimates between updates.
   *
   * @param state Pointer to state
   * @param timestamp Time to propagate to (IMU clock frame)
   * @param state_plus The propagated state (q_GtoI, p_IinG, v_IinI, w_IinI)
   * @param covariance The propagated covariance (q_GtoI, p_IinG, v_IinI,
   * w_IinI)
   * @return True if we were able to propagate the state to the current timestep
   */
  bool fast_state_propagate(std::shared_ptr<State> state, double timestamp,
                            Eigen::Matrix<DataType, 13, 1> &state_plus,
                            Eigen::Matrix<DataType, 12, 12> &covariance);

  void get_imu_data(std::vector<ov_core::ImuData> &imu_data) {
    imu_data = imu_data_;
  }

  // Point to the same imu data
  std::shared_ptr<std::vector<ov_core::ImuData>> get_imu_data() {
    return std::shared_ptr<std::vector<ov_core::ImuData>>(&imu_data_,
                                                          [](auto *) {});
  }

protected:
  /**
   * @brief Propagates the state forward using the imu data and computes the
   * noise covariance and state-transition matrix of this interval.
   *
   * This function can be replaced with analytical/numerical integration or when
   * using a different state representation. This contains our state transition
   * matrix along with how our noise evolves in time. If you have other state
   * variables besides the IMU that evolve you would add them here. See the @ref
   * error_prop page for details on how this was derived.
   *
   * @param state Pointer to state
   * @param data_minus imu readings at beginning of interval
   * @param data_plus imu readings at end of interval
   * @param F State-transition matrix over the interval
   * @param Qd Discrete-time noise covariance over the interval
   */
  void predict_and_compute(std::shared_ptr<State> state,
                           const ov_core::ImuData &data_minus,
                           const ov_core::ImuData &data_plus,
                           Eigen::Matrix<DataType, 15, 15> &F,
                           Eigen::Matrix<DataType, 15, 15> &Qd);

  /**
   * @brief Discrete imu mean propagation.
   *
   * See @ref propagation for details on these equations.
   * \f{align*}{
   * \text{}^{I_{k+1}}_{G}\hat{\bar{q}}
   * &=
   * \exp\bigg(\frac{1}{2}\boldsymbol{\Omega}\big({\boldsymbol{\omega}}_{m,k}-\hat{\mathbf{b}}_{g,k}\big)\Delta
   * t\bigg)
   * \text{}^{I_{k}}_{G}\hat{\bar{q}} \\
   * ^G\hat{\mathbf{v}}_{k+1} &= \text{}^G\hat{\mathbf{v}}_{I_k} -
   * {}^G\mathbf{g}\Delta t
   * +\text{}^{I_k}_G\hat{\mathbf{R}}^\top(\mathbf{a}_{m,k} -
   * \hat{\mathbf{b}}_{\mathbf{a},k})\Delta t\\ ^G\hat{\mathbf{p}}_{I_{k+1}}
   * &= \text{}^G\hat{\mathbf{p}}_{I_k} + {}^G\hat{\mathbf{v}}_{I_k} \Delta t
   * - \frac{1}{2}{}^G\mathbf{g}\Delta t^2
   * + \frac{1}{2} \text{}^{I_k}_{G}\hat{\mathbf{R}}^\top(\mathbf{a}_{m,k} -
   * \hat{\mathbf{b}}_{\mathbf{a},k})\Delta t^2 \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat1 Angular velocity with bias removed
   * @param a_hat1 Linear acceleration with bias removed
   * @param w_hat2 Next angular velocity with bias removed
   * @param a_hat2 Next linear acceleration with bias removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   */
  void predict_mean_discrete(std::shared_ptr<State> state, double dt,
                             const Vec3 &w_hat1, const Vec3 &a_hat1,
                             const Vec3 &w_hat2, const Vec3 &a_hat2,
                             Vec4 &new_q, Vec3 &new_v, Vec3 &new_p);

  /**
   * @brief RK4 imu mean propagation.
   *
   * See this wikipedia page on [Runge-Kutta
   * Methods](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods). We are
   * doing a RK4 method, [this wolframe
   * page](http://mathworld.wolfram.com/Runge-KuttaMethod.html) has the forth
   * order equation defined below. We define function \f$ f(t,y) \f$ where y is
   * a function of time t, see @ref imu_kinematic for the definition of the
   * continous-time functions.
   *
   * \f{align*}{
   * {k_1} &= f({t_0}, y_0) \Delta t  \\
   * {k_2} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_1} ) \Delta t \\
   * {k_3} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_2} ) \Delta t \\
   * {k_4} &= f( {t_0} + {\Delta t}, y_0 + {k_3} ) \Delta t \\
   * y_{0+\Delta t} &= y_0 + \left( {{1 \over 6}{k_1} + {1 \over 3}{k_2} + {1
   * \over 3}{k_3} + {1 \over 6}{k_4}} \right) \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat1 Angular velocity with bias removed
   * @param a_hat1 Linear acceleration with bias removed
   * @param w_hat2 Next angular velocity with bias removed
   * @param a_hat2 Next linear acceleration with bias removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   */
  void predict_mean_rk4(std::shared_ptr<State> state, double dt,
                        const Vec3 &w_hat1, const Vec3 &a_hat1,
                        const Vec3 &w_hat2, const Vec3 &a_hat2, Vec4 &new_q,
                        Vec3 &new_v, Vec3 &new_p);

  /// Estimate for time offset at last propagation time
  double last_prop_time_offset_ = 0.0;
  bool have_last_prop_time_offset_ = false;

  /// Container for the noise values
  NoiseManager noises_;

  /// Our history of IMU messages (time, angular, linear)
  std::vector<ov_core::ImuData> imu_data_;

  std::mutex imu_data_mtx_;

  /// Gravity vector
  Vec3 gravity_;
};

} // namespace ov_srvins

#endif // OV_SRVINS_STATE_PROPAGATOR_H
