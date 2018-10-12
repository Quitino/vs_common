#ifndef __VS_PCA_H__
#define __VS_PCA_H__
#include <Eigen/Dense>

namespace vs
{

inline bool PCA(const Eigen::MatrixXd& data, Eigen::MatrixXd& eig_val,
                Eigen::MatrixXd& eig_coef, Eigen::MatrixXd& center)
{
    int N = data.rows();
    if(N <= 1) return false;
    auto c = data.colwise().mean();
    Eigen::MatrixXd d = data.rowwise() - c;
    Eigen::MatrixXd cov = d.transpose() * d;
    cov /= N - 1;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(cov);
    eig_val = es.eigenvalues();
    eig_coef = es.eigenvectors();
    center = c.transpose();
    return true;
}

inline bool lineFit(const Eigen::MatrixXd& data, Eigen::MatrixXd& p1,
                    Eigen::MatrixXd& p2)
{
    int n = data.rows();
    int k = data.cols();
    Eigen::MatrixXd eig_val, eig_coef, center;
    PCA(data, eig_val, eig_coef, center);
    int last = k - 1;
    double lambda = eig_val(last);
    if(lambda < 0.1) return false;
    for(int i = 0; i < last; i++)
        if(eig_val(i) / lambda > 0.1)
            return false;
    auto dir = eig_coef.col(last);
    dir /= dir.norm();
    double dmin = 0, dmax = 0;
    for(int i = 0; i < n; i++)
    {
        double d = dir.dot(data.row(i).transpose() - center);
        if(d < dmin) dmin = d;
        else if(d > dmax) dmax = d;
    }
    p1 = center + dmin * dir;
    p2 = center + dmax * dir;
    return true;
}

} /* namespace vs */
#endif//__VS_PCA_H__