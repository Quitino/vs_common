#include "vs_geometry3d.h"
#include "vs_numeric.h"

namespace vs
{

Eigen::Matrix3d skew(const Eigen::Vector3d& w)
{
    Eigen::Matrix3d w_hat;
    w_hat << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
    return w_hat;
}

double rotDiff(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2)
{
    return fabs(Eigen::AngleAxisd(R1.transpose() * R2).angle());
}

bool checkRot(const Eigen::Matrix3d& R)
{
    return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-5;
}

Eigen::Vector4d weightMeanQuat(Eigen::Vector4d q1, Eigen::Vector4d q2,
                               double k1, double k2)
{
    Eigen::Vector4d res;
    if(k2 == 0)
    {
        k2 = 1 - k1;
    }
    else
    {
        double k = k1 + k2;
        k1 /= k;
        k2 /= k;
    }
    q1.normalize();
    q2.normalize();

    double dot = q1.dot(q2);
    if(dot < 0)
    {
        q2 = -q2;
        dot = -dot;
    }
    Eigen::Vector4d q;
    double theta = acos(clip(dot, -1, 1));
    if(fabs(theta) < 1e-5)
    {
        q = k1 * q1 + k2 * q2;
    }
    else
    {
        q = (sin(k1 * theta) * q1 + sin(k2 * theta) * q2) / sin(theta);
    }
    q.normalize();
    return q;
}

bool slerp(double t1, const Eigen::Vector4d& q1, double t2, const Eigen::Vector4d& q2,
           double t, Eigen::Vector4d& q)
{
    if(q1.norm() < 1e-5 || q2.norm() < 1e-5)
    {
        printf("[ERROR]slerp: bad input, quat norm is zero\n");
        return false;
    }
    else if(t1 > t2)
    {
        return lerp(t2, q2, t1, q1, t, q);
    }
    else if(t1 <= t && t <= t2)
    {
        double dt = t2 - t1;
        double k = dt < 1e-5 ? 0.5 : (t2 - t) / dt;
        q = weightMeanQuat(q1, q2, k);
        return true;
    }
    else
    {
        printf("[ERROR]slerp: bad input, t must range [t1,t2].\n");
        return false;
    }
}

Eigen::Vector4d quatMean(const std::vector<Eigen::Vector4d>& quats)
{
    if(quats.empty()) return Eigen::Vector4d();
    int sum = 1;
    Eigen::Vector4d mean_quat = quats[0];
    for(size_t i = 1; i < quats.size(); i++)
    {
        mean_quat = weightMeanQuat(mean_quat, quats[i], sum, 1);
    }
    return mean_quat;
}

Eigen::Matrix3d rotMean(const std::vector<Eigen::Matrix3d>& Rs)
{
    std::vector<Eigen::Vector4d> quats;
    for(const auto& R : Rs)
    {
        quats.push_back(Eigen::Quaterniond(R).coeffs());
    }
    auto q = quatMean(quats);
    return Eigen::Quaterniond(q[3], q[0], q[1], q[2]).toRotationMatrix();
}

Eigen::Matrix3d rotMeanSO3(const std::vector<Eigen::Matrix3d>& Rs)
{
    if(Rs.empty()) return Eigen::Matrix3d();
    auto R_ref = Rs[0];
    Eigen::Vector3d se3_sum(0, 0, 0);
    for(const auto& R : Rs)
    {
        auto v = logSO3(R_ref.transpose() * R);
        se3_sum += v;
    }
    return R_ref * expSO3(se3_sum / Rs.size());
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d& w)
{
    // get theta
    Eigen::Matrix3d w_x = skew(w);
    double theta = w.norm();
    // Handle small angle values
    double A, B;
    if(theta < 1e-12)
    {
        A = 1;
        B = 0.5;
    }
    else 
    {
        A = sin(theta)/theta;
        B = (1-cos(theta))/(theta*theta);
    }
    // compute so(3) rotation
    Eigen::Matrix3d R;
    if (theta == 0) R = Eigen::MatrixXd::Identity(3, 3);
    else R = Eigen::MatrixXd::Identity(3, 3) + A*w_x + B*w_x*w_x;
    return R;
}

Eigen::Vector3d logSO3(const Eigen::Matrix3d& R)
{
    // magnitude of the skew elements (handle edge case where we sometimes have a>1...)
    double a = 0.5*(R.trace()-1);
    double theta = (a > 1)? acos(1) : ((a < -1)? acos(-1) : acos(a));
    // Handle small angle values
    double D;
    if(theta < 1e-12) D = 0.5;
    else D = theta/(2*sin(theta));

    // calculate the skew symetric matrix
    Eigen::Matrix3d w_x = D*(R-R.transpose());
    // check if we are near the identity
    if (R != Eigen::MatrixXd::Identity(3, 3))
    {
        Eigen::Vector3d vec;
        vec << w_x(2, 1), w_x(0, 2), w_x(1, 0);
        return vec;
    }
    else
    {
        return Eigen::Vector3d::Zero();
    }
}

Eigen::Matrix4d expSE3(const Eigen::Matrix<double,6,1>& vec)
{
    // Precompute our values
    Eigen::Vector3d w = vec.head(3);
    Eigen::Vector3d u = vec.tail(3);
    double theta = sqrt(w.dot(w));
    Eigen::Matrix3d wskew;
    wskew << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

    // Handle small angle values
    double A, B, C;
    if(theta < 1e-12)
    {
        A = 1;
        B = 0.5;
        C = 1.0/6.0;
    }
    else
    {
        A = sin(theta)/theta;
        B = (1-cos(theta))/(theta*theta);
        C = (1-A)/(theta*theta);
    }

    // Matrices we need V and Identity
    Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d V = I_33 + B*wskew + C*wskew*wskew;

    // Get the final matrix to return
    Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
    mat.block(0,0,3,3) = I_33 + A*wskew + B*wskew*wskew;
    mat.block(0,3,3,1) = V*u;
    mat(3,3) = 1;
    return mat;
}

Eigen::Matrix<double,6,1> logSE3(const Eigen::Matrix4d& mat)
{
    // Get sub-matrices
    Eigen::Matrix3d R = mat.block(0,0,3,3);
    Eigen::Vector3d t = mat.block(0,3,3,1);

    // Get theta (handle edge case where we sometimes have a>1...)
    double a = 0.5*(R.trace()-1);
    double theta = (a > 1)? acos(1) : ((a < -1)? acos(-1) : acos(a));

    // Handle small angle values
    double A, B, D, E;
    if(theta < 1e-12)
    {
        A = 1;
        B = 0.5;
        D = 0.5;
        E = 1.0/12.0;
    }
    else
    {
        A = sin(theta)/theta;
        B = (1-cos(theta))/(theta*theta);
        D = theta/(2*sin(theta));
        E = 1/(theta*theta)*(1-0.5*A/B);
    }

    // Get the skew matrix and V inverse
    Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d wskew = D*(R-R.transpose());
    Eigen::Matrix3d Vinv = I_33 - 0.5*wskew+E*wskew*wskew;

    // Calculate vector
    Eigen::Matrix<double,6,1> vec;
    vec.head(3) << wskew(2, 1), wskew(0, 2), wskew(1, 0);
    vec.tail(3) = Vinv*t;
    return vec;
}

Eigen::Matrix4d Omega(Eigen::Vector3d w)
{
    Eigen::Matrix4d mat;
    mat.block(0, 0, 3, 3) = -skew(w);
    mat.block(3, 0, 1, 3) = -w.transpose();
    mat.block(0, 3, 3, 1) = w;
    mat(3, 3) = 0;
    return mat;
}

Eigen::Matrix<double, 6, 1> isom2vec(const Eigen::Isometry3d& T)
{
    auto ypr = T.linear().eulerAngles(2, 1, 0);
    if(ypr(2) < 0)
    {
        ypr = T.linear().transpose().eulerAngles(0, 1, 2);
        std::swap(ypr(0), ypr(2));
    }
    auto t = T.translation();
    Eigen::Matrix<double, 6, 1> res;
    res << t(0), t(1), t(2), ypr(0), ypr(1), ypr(2);
    return res;
}

Eigen::Vector3d eulerConjugate(const Eigen::Vector3d& rpy)
{
    return Eigen::Vector3d(vs::normalizeRad(rpy(0) - M_PI),
                           vs::normalizeRad(M_PI - rpy(1)),
                           vs::normalizeRad(rpy(2) - M_PI));
}

Eigen::Vector3d eulerAdjust(const Eigen::Vector3d& rpy, int type)
{
    switch(type)
    {
        case 0:
            if(fabs(rpy[0]) > M_PI_2) return eulerConjugate(rpy);
            break;
        case 1:
            if(rpy[0] < 0) return eulerConjugate(rpy);
            break;
        default:
            break;
    }
    return rpy;
}

Eigen::Matrix3d typicalRot(int type)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    switch(type)
    {
        case ROT_RDF2FLU: R << 0, 0, 1, -1, 0, 0, 0, -1, 0; break;
        case ROT_FLU2RDF: R << 0, -1, 0, 0, 0, -1, 1, 0, 0; break;
        case ROT_RDF2FRD: R << 0, 0, 1, 1, 0, 0, 0, 1, 0; break;
        case ROT_FRD2RDF: R << 0, 1, 0, 0, 0, 1, 1, 0, 0; break;
        case ROT_FLU2FRD: R << 1, 0, 0, 0, -1, 0, 0, 0, -1; break;
        case ROT_FRD2FLU: R << 1, 0, 0, 0, -1, 0, 0, 0, -1; break;
        case ROT_RBD2FLU: R << 0, -1, 0, -1, 0, 0, 0, 0, -1; break;
        case ROT_FLU2RBD: R << 0, -1, 0, -1, 0, 0, 0, 0, -1; break;
        case ROT_RBD2FRD: R << 0, -1, 0, 1, 0, 0, 0, 0, 1; break;
        case ROT_FRD2RBD: R << 0, 1, 0, -1, 0, 0, 0, 0, 1; break;
    }
    return R;
}

Eigen::Vector3d Rbw2rpy(const Eigen::Matrix3d& R_b_w)
{
    auto ypr = R_b_w.eulerAngles(2, 1, 0);
    return eulerAdjust(Eigen::Vector3d(ypr(2), ypr(1), ypr(0)), 0);
}

} /* namespace vs */