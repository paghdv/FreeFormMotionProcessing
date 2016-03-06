#ifndef UTILS_Q_H
#define UTILS_Q_H
#include <eigen3/Eigen/Dense>

static Eigen::Quaterniond qexp(const Eigen::Quaterniond &q)
{
    Eigen::Quaterniond qf=q;
    double r  = sqrt(q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
    double et = exp(q.w());
    double s  = r>0? et*sin(r)/r: 0;

    qf.w() =et*cos(r);
    qf.x()*=s;
    qf.y()*=s;
    qf.z()*=s;
    return qf;
}


static Eigen::Quaterniond qln(const Eigen::Quaterniond &q)
{
    Eigen::Quaterniond qf=q;
    double r  = sqrt(q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
    double t  = r>0? atan2(r,q.w())/r: 0;
    qf.w() =0.5*log(q.w()*q.w()+q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
    qf.x()*=t;
    qf.y()*=t;
    qf.z()*=t;
    return qf;
}

static void qscale(Eigen::Quaterniond &q, const double scale)
{
    q.w()*=scale;
    q.x()*=scale;
    q.y()*=scale;
    q.z()*=scale;
}

static Eigen::Quaterniond qpow(const Eigen::Quaterniond &q,const double n)
{
    //ln(q).scale(n).exp();
    Eigen::Quaterniond q1=qln(q);
    qscale(q1,n);
    return qexp(q1);
}

static Eigen::Quaterniond qinv(const Eigen::Quaterniond &q)
{
    Eigen::Quaterniond qf;
    double r  = q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w();

    qf.w() =  q.w()/r;
    qf.x() = -q.x()/r;
    qf.y() = -q.y()/r;
    qf.z() = -q.z()/r;
    return qf;
}


#endif // UTILS_Q_H
