#pragma once

class LADRC {
public:
    LADRC(double b0, double wc, double wo, double h);
    double step(double ref, double y);

private:
    double b0_, wc_, wo_, h_;

    //TD
    double v1_, v2_;

    //ESO
    double z1_, z2_, z3_;
    double beta1 ,beta2, beta3;

    void td(double ref);
    void eso(double y, double u);
};