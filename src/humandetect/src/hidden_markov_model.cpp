#include "hidden_markov_model.hpp"

HMMFilter::HMMFilter(MatrixXf &a, MatrixXf &b):
    A(a.rows(), a.cols()),
    B(b.rows(), b.cols()),
    state(a.cols(), 1)
{
    set(a, b);
}

HMMFilter::~HMMFilter()
{

}

void HMMFilter::set(MatrixXf &a, MatrixXf &b)
{
    A << a;
    B << b;
    dim = a.cols();
    state << MatrixXf::Ones(a.cols(), 1);
    state /= a.cols();
}

void HMMFilter::init(int measure)
{
    state << MatrixXf::Zero(A.cols(), 1);
    state(measure, 0) = 1;
}

void HMMFilter::cal(int measure, bool valid)
{
    state = A * state;
    if (valid)
        for (int i = 0; i < state.rows(); i++)
        {
            state(i, 0) *= B(measure, i);
        }
    state /= state.sum();
}