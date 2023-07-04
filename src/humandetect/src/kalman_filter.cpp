#include "kalman_filter.hpp"

using namespace Eigen;

using namespace kalman_filter;

KalmanFilter::KalmanFilter(int d, double dt, std::vector<float> q, std::vector<float> r):
    state(2*d, 1),
    cov(2*d, 2*d),
    A(2*d, 2*d), 
    H(d, 2*d),
    Q(2*d, 2*d),
    R(d, d)
{
    pos_dim = d;
    dim = 2 * pos_dim;

    MatrixD combRow1(d, 2*d), combRow2(d, 2*d);
    combRow1 << MatrixXf::Identity(d, d), dt * MatrixXf::Identity(d, d);
    combRow2 << MatrixXf::Zero(d, d), MatrixXf::Identity(d, d);
    A << combRow1, combRow2;

    H << MatrixXf::Identity(d, d), MatrixXf::Zero(d, d);

    Q << MatrixXf::Zero(2*d, 2*d);
    for (int i = 0; i < 2*d; i++)
    {
        Q(i, i) = q[i];
    }

    R << MatrixXf::Zero(d, d);
    for (int i = 0; i < d; i++)
    {
        R(i, i) = r[i];
    }
}

void KalmanFilter::init(std::vector<float> pos)
{
    state << MatrixXf::Zero(dim, 1);
    for (int i = 0; i < pos_dim; i++)
    {
        state(i, 0) = pos[i];
    }
    cov << Q;
    is_init = true;
}

void KalmanFilter::predict()
{
    state = A * state;
    cov = A * cov * A.transpose() + Q;
}

void KalmanFilter::update(std::vector<float> pos)
{
    MatrixXf pos_(pos_dim, 1);
    for (int i = 0; i < pos_dim; i++)
    {
        pos_(i, 0) = pos[i];
    }

    MatrixXf state_obs = H * state;
    MatrixXf cov_obs = H * cov * H.transpose() + R;

    // MatrixXf Kgain = cov_obs.transpose().llt().solve(H * cov.transpose()).transpose();
    MatrixXf Kgain = cov_obs.llt().solve(H * cov).transpose();  // 利用cholesky分解求逆
    state += Kgain * (pos_ - state_obs);
    cov += -Kgain * cov_obs * Kgain.transpose();
}

float KalmanFilter::gating_distance(std::vector<float> pos)
{
    MatrixXf pos_(pos_dim, 1);
    for (int i = 0; i < pos_dim; i++)
    {
        pos_(i, 0) = pos[i];
    }

    MatrixXf state_obs = H * state;
    MatrixXf cov_obs = H * cov * H.transpose() + R;

    MatrixXf temp = cov_obs.transpose().llt().solve(pos_ - state_obs);
    MatrixXf dist = temp.transpose() * temp;
    return dist(0, 0);
}

void KalmanFilter::cal(std::vector<float> pos, bool valid)
{
    if (is_init)
    {
        predict();
        if (valid)
            update(pos);
    }
    else
    {
        if (valid)
            init(pos);
    }
}