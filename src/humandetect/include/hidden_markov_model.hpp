#pragma once
#include <vector>
#include <cstdlib>
#include <Eigen/Dense>

using namespace Eigen;

class HMMFilter
{
public:
    HMMFilter(MatrixXf &a, MatrixXf &b);
    ~HMMFilter();
    void set(MatrixXf &a, MatrixXf &b);
    void init(int measure);
    void cal(int measure, bool valid);

    MatrixXf state;
    int dim;

private:
    MatrixXf A;
    MatrixXf B;

};