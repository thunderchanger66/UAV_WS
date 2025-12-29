#include "ladrc.hpp"

LADRC::LADRC(double b0, double wc, double wo, double h) :
    b0_(b0), wc_(wc), wo_(wo), h_(h),
    v1_(0), v2_(0), z1_(0), z2_(0), z3_(0) {
        beta1 = 3 * wo;
        beta2 = 3 * wo * wo;
        beta3 = wo * wo * wo;
    }

void LADRC::td(double ref) {
    v1_ += h_ *  v2_;
    //zeta = 1, w_td = (0.5 - 1)w_c, 取1, w_td = wc_
    v2_ += h_ * (-2 * wc_ * v2_ - wc_ * wc_ * (v1_ - ref));
}

void LADRC::eso(double y, double u) {
    double e = y - z1_;

    z1_ += h_ * (z2_ + beta1 * e);
    z2_ += h_ * (z3_ + b0_ * u + beta2 * e);
    z3_ += h_ * (beta3 * e);
}

double LADRC::step(double ref, double y) {
    td(ref);

    // PD控制器，Kp = wc^2, Kd = 2wc
    double u0 = wc_ * wc_ * (v1_ - z1_) + 2 * wc_ * (v2_ - z2_);
    double u = (u0 - z3_) / b0_;

    eso(y, u);

    return u;
}