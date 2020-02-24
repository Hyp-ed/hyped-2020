/*
 * Author: Lukas Schaefer
 * Organisation: HYPED
 * Date: 31/03/2019
 * Description: Multivariate Kalman filter implementation to filter sensor measurement
 *              considering the dynamics of the system
 *
 *    Copyright 2019 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *    except in compliance with the License. You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software distributed under
 *    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *    either express or implied. See the License for the specific language governing permissions and
 *    limitations under the License.

 */

#include "kalman_multivariate.hpp"

namespace hyped {
    namespace utils {
        namespace math {
            KalmanMultivariate::KalmanMultivariate(unsigned int n, unsigned int m)
                : n_(n),
                  m_(m),
                  k_(0),
                  iteration_(0)
            {}

            KalmanMultivariate::KalmanMultivariate(unsigned int n, unsigned int m,
                                                   unsigned int k)
                : n_(n),
                  m_(m),
                  k_(k),
                  iteration_(0)
            {}

            void KalmanMultivariate::setDynamicsModel(MatrixXf& A, MatrixXf& Q)
            {
                A_ = A;
                Q_ = Q;
            }

            void KalmanMultivariate::setDynamicsModel(MatrixXf& A, MatrixXf& B, MatrixXf& Q)
            {
                A_ = A;
                B_ = B;
                Q_ = Q;
            }

            void KalmanMultivariate::setMeasurementModel(MatrixXf& H, MatrixXf& R)
            {
                H_ = H;
                R_ = R;
            }

            void KalmanMultivariate::setModels(MatrixXf& A, MatrixXf& Q, MatrixXf& H,
                                       MatrixXf& R)
            {
                setDynamicsModel(A, Q);
                setMeasurementModel(H, R);
            }

            void KalmanMultivariate::setModels(MatrixXf& A, MatrixXf& B, MatrixXf& Q,
                                       MatrixXf& H, MatrixXf& R)
            {
                setDynamicsModel(A, B, Q);
                setMeasurementModel(H, R);
            }

            void KalmanMultivariate::updateA(MatrixXf& A)
            {
                A_ = A;
            }

            void KalmanMultivariate::updateR(MatrixXf& R)
            {
                R_ = R;
            }

            void KalmanMultivariate::setInitial(VectorXf& x0, MatrixXf& P0)
            {
                x_ = x0;
                P_ = P0;
                I_ = MatrixXf::Identity(n_, n_);
            }

            void KalmanMultivariate::updateC() 
            {
                int new_window_size = std::min(iteration_, window_size_);
                int prev_window_size = std::min(iteration_ - 1, window_size_);

                // new_window_size will be smaller if less than 'window_size_' iterations happened
                if ((int)delta_zs_.size() > window_size_) {
                    VectorXf delta_z_old = delta_zs_.front();
                    delta_zs_.pop_front();
                    C_ = C_ - (delta_z_old * delta_z_old.transpose()) / (float)prev_window_size;
                    // assert((int)delta_zs.size() == window_size_);
                }

                if ((int)delta_zs_.size() > 0) {
                    C_ = C_ * (float)(prev_window_size / new_window_size) + 
                    (delta_zs_.back() * delta_zs_.back().transpose()) / (float)new_window_size;
                }
            }

            void KalmanMultivariate::updateQR() 
            {
                updateC();
                // TODO(Justas) figure this part out
                if (iteration_ >= window_size_) {
                    // we might want to just use the initial matrices for the first few iterations
                    Q_ = K_ * C_ * K_.transpose();
                    R_ = C_ - H_ * P_ * H_.transpose();
                } 
            }

            void KalmanMultivariate::predict_state()
            {
                x_ = A_ * x_;
            }

            void KalmanMultivariate::predict_state(VectorXf& u)
            {
                x_ = A_ * x_ + B_ * u;
            }

            void KalmanMultivariate::predict_covariance()
            {
                updateQR();
                P_ = A_ * P_ * A_.transpose() + Q_;
            }

            void KalmanMultivariate::correct(VectorXf& z)
            {
                K_ = (P_ * H_.transpose()) * (H_ * P_ * H_.transpose() + R_).inverse();
                x_ = x_ + K_ * (z - H_ * x_);
                P_ = (I_ - K_ * H_) * P_;
            }

            void KalmanMultivariate::filter(VectorXf& z)
            {
                iteration_++;

                predict_state();
                delta_zs_.push_back(z - H_ * x_);
                predict_covariance();
                
                correct(z);
            }

            void KalmanMultivariate::filter(VectorXf& u, VectorXf& z)
            {
                iteration_++;
                
                predict_state(u);
                delta_zs_.push_back(z - H_ * x_);
                predict_covariance();
                
                correct(z);
            }

            VectorXf& KalmanMultivariate::getStateEstimate()
            {
                return x_;
            }

            MatrixXf& KalmanMultivariate::getStateCovariance()
            {
                return P_;
            }

        }
    }
}
