#ifndef QUARTICPOLYNOMIAL_H
#define QUARTICPOLYNOMIAL_H

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

class QuarticPolynomial
{
public:
    QuarticPolynomial() = default;
    QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double t);
    double getTmax();
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);

private:
    double tmax;
    double a0, a1, a2, a3, a4;
};

#endif // QUARTICPOLYNOMIAL_H
