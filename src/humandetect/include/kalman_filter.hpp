#pragma once
#include <vector>
#include <cstdlib>
#include <map>
#include <string>
#include <Eigen/Dense>

using namespace Eigen;

typedef Matrix<float, Dynamic, Dynamic> MatrixD;

namespace kalman_filter{
    
    class KalmanFilter
    {
    public:
        KalmanFilter(int d, double dt, std::vector<float> q, std::vector<float> r);
        void init(std::vector<float> pos);
        void predict();
        void update(std::vector<float> pos);
        float gating_distance(std::vector<float> pos);
        void cal(std::vector<float> pos, bool valid);
        int pos_dim;
        int dim;
        bool is_init;

        MatrixXf state;
        MatrixXf cov;

    private:
        MatrixXf A;
        MatrixXf H;
        MatrixXf Q;
        MatrixXf R;
    };

}