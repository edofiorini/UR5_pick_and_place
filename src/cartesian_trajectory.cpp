#include "cartesian_trajectory.hpp"

#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace Eigen;

// PUBLIC METHODS

CartesianTrajectory::CartesianTrajectory(MatrixXd pi, MatrixXd pf, MatrixXd PHI_i, MatrixXd PHI_f, double ti, double tf, double Ts)
{
    length = (int)floor((tf - ti) / Ts);
    
    // Build data matrices
    std::cout << "Initializing trajectory matrices..." << std::endl;
    dataPosition = MatrixXd(6, length);
    dataVelocities = MatrixXd(6, length);
    dataAcceleration = MatrixXd(6, length);

    MatrixXd T(1, length);
    MatrixXd q(1, length);
    MatrixXd qd(1, length);
    MatrixXd qdd(1, length);

    //fifth_polinomials(T,q,qd,qdd,ti, tf, qi, dqi, ddqi, qf, dqf, ddqf, Ts);

    // Position in operationalspace with TIMING LAW
    MatrixXd p_tilde(3, length);
    MatrixXd dp_tilde(3, length);
    MatrixXd ddp_tilde(3, length);

    std::cout << "Lineat trajectory computation..." << std::endl;
    linear_tilde(T, p_tilde, dp_tilde, ddp_tilde, pi, pf, ti, tf, Ts);
    // std::cout << p_tilde << std::endl;
    //// std::cout<< "Linear_tilde: " << p_tilde << "\n" << std::endl;

    // int length2 = circular_length(pi, pf, Ts, c);
    // MatrixXd p_tilde1(3, length);
    // MatrixXd dp_tilde1(3, length);
    // MatrixXd ddp_tilde1(3, length);

    // std::cout << "Here 1" << std::endl;
    // circular_tilde(T, p_tilde1, dp_tilde1, ddp_tilde1, pi, pf, c, ti, tf, Ts, length2);
    //// std::cout << "Circular_tilde: " << dp_tilde1.row(0) << "\n" << std::endl;

    // linear motion - Position in operational space without TIMING LAW
    /*
    std::cout << "Path length computation..." << std::endl;
    MatrixXd support = pf - pi;
    double end = support.norm();

    int length1 = (int)floor(end / Ts);
    MatrixXd s(1, length1);
    int count = 0;
    for (float i = ti; i < length1; i++)
    {
      if (count >= length1)
      {
        break;
      }
      s(0, count) = i;
      count++;
    }*/

    // MatrixXd p(3, length1);
    // MatrixXd dp(3, length1);
    // MatrixXd ddp(3, length1);
    // std::cout << "Linear Motion" << std::endl;
    // linear_motion(T, p, dp, ddp, s, pi, pf, Ts, length1);
    // // std::cout << "Linear_motion: " << p << "\n" << std::endl;
    //// std::cout << s.cols() << " " << p.cols() << std::endl;

    // MatrixXd p1(3, length2);
    // MatrixXd dp1(3, length2);
    // MatrixXd ddp1(3, length2);
    // std::cout << "Circular Motion" << std::endl;
    // circular_motion(T, p1, dp1, ddp1, pi, pf, Ts, c, length2);
    // // std::cout << "Circular_motion: " << p1 << "\n"<< std::endl;
    // Frenet Frame to find the orientation of the end-effector
    // MatrixXd o_EE_t(3, 2);
    // MatrixXd o_EE_n(3, 2);
    // MatrixXd o_EE_b(3, 2);

    // @todo actually not used since orientation is fixed due to nan values
    // quaterions should be converted in euler angles to keep the same orientation
    // and the use of frenet frame

    // @todo frenet_frame doesn't work properly there is a problem with types
    // if you try different pi and pf you will find out a type problem
    // for now we can ignore it since we don't have an orientation

    //frenet_frame(p, dp, ddp, o_EE_t, o_EE_n, o_EE_b, PHI_i, PHI_f, length1);
    // frenet_frame(p, dp, ddp, o_EE_t, o_EE_n, o_EE_b, PHI_i, PHI_f, length1);

    // EE_orientation with the TIMING LAW
    std::cout << "Initializing orientation matrices..." << std::endl;
    MatrixXd o_tilde(3, length);
    MatrixXd do_tilde(3, length);
    MatrixXd ddo_tilde(3, length);
    EE_orientation(T, PHI_i, PHI_f, o_tilde, do_tilde, ddo_tilde, pi, pf, ti, tf, Ts);
    // std::cout << "o_tilde" << o_tilde << std::endl;

    // Put the 3d coordinate togheter
    std::cout << "Initializing position, velocity and acceleration matrices..." << std::endl;
    dataPosition.block(0, 0, 3, length) = p_tilde.block(0, 0, 3, length);
    dataPosition.block(3, 0, 3, length) = o_tilde.block(0, 0, 3, length);

    dataVelocities.block(0, 0, 3, length) = dp_tilde.block(0, 0, 3, length);
    dataVelocities.block(3, 0, 3, length) = do_tilde.block(0, 0, 3, length);

    dataAcceleration.block(0, 0, 3, length) = ddp_tilde.block(0, 0, 3, length);
    dataAcceleration.block(3, 0, 3, length) = ddo_tilde.block(0, 0, 3, length);
}

int CartesianTrajectory::get_length()
{
    return length;
}

// PRIVATE METHODS

void CartesianTrajectory::linear_tilde(MatrixXd &T, MatrixXd &p_tilde, MatrixXd &dp_tilde, MatrixXd &ddp_tilde, MatrixXd &pi, MatrixXd &pf, double ti, double tf, double Ts)
{

    double qi = 0;
    double dqi = 0;
    double ddqi = 0;
    MatrixXd support = pf - pi;
    double qf = (double)support.norm();
    double dqf = 0;
    double ddqf = 0;

    // int length = (int)floor((tf - ti) / Ts);
    MatrixXd s(1, length);
    MatrixXd sd(1, length);
    MatrixXd sdd(1, length);

    CartesianTrajectory::fifth_polinomials(T, s, sd, sdd, ti, tf, qi, dqi, ddqi, qf, dqf, ddqf, Ts);

    for (int i = 0; i < length; i++)
    {
        p_tilde.col(i) = pi + (support / support.norm()) * s.col(i);
        dp_tilde.col(i) = ((support) / support.norm()) * sd.col(i);
        ddp_tilde.col(i) = ((support) / support.norm()) * sdd.col(i);
    }
}

void CartesianTrajectory::fifth_polinomials(MatrixXd &T, MatrixXd &q, MatrixXd &qd, MatrixXd &qdd, double ti, double tf, double qi, double dqi, double ddqi, double qf, double dqf, double ddqf, double Ts)
{
    double deltaT = tf-ti;
    MatrixXd H(6, 6);
    H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0,
        1, deltaT, pow(deltaT, 2), pow(deltaT, 3), pow(deltaT, 4), pow(deltaT, 5),
        0, 1, 2 * deltaT, 3 * pow(deltaT, 2), 4 * pow(deltaT, 3), 5 * pow(deltaT, 4),
        0, 0, 2, 6 * deltaT, 12 * pow(deltaT, 2), 20 * pow(deltaT, 3);

    MatrixXd Q(6, 1);
    Q << qi,
        dqi,
        ddqi,
        qf,
        dqf,
        ddqf;

    MatrixXd a = H.inverse() * (Q);

    MatrixXd a0 = a.row(0);
    MatrixXd a1 = a.row(1);
    MatrixXd a2 = a.row(2);
    MatrixXd a3 = a.row(3);
    MatrixXd a4 = a.row(4);
    MatrixXd a5 = a.row(5);

    // int length = (int)floor((tf - ti) / Ts);
    // int count = 0;
    // for (double i = ti; i < length; i += Ts)
    // {
    //     if (count >= length)
    //     {
    //         break;
    //     }
    //     T(0, count) = i;
    //     count++;
    // }

    // std::cout << "A coeff fifth polynomials: " << a0.coeff(0, 0) << " " << a1.coeff(0, 0) << " " << a2.coeff(0, 0) << " " << a3.coeff(0, 0) << " " << a4.coeff(0, 0) << " " << a5.coeff(0, 0) << "\n"
    // << std::endl;
    for (int i = 0; i < length; i++)
    {
        double t = i*Ts;
        q(0, i) = a5.coeff(0, 0) * pow(t, 5) + a4.coeff(0, 0) * pow(t, 4) + a3.coeff(0, 0) * pow(t, 3) + a2.coeff(0, 0) * pow(t, 2) + a1.coeff(0, 0) * pow(t, 1) + a0.coeff(0, 0);
        qd(0, i) = 5 * a5.coeff(0, 0) * pow(t, 4) + 4 * a4.coeff(0, 0) * pow(t, 3) + 3 * a3.coeff(0, 0) * pow(t, 2) + 2 * a2.coeff(0, 0) * pow(t, 1) + a1.coeff(0, 0);
        qdd(0, i) = 20 * a5.coeff(0, 0) * pow(t, 3) + 12 * a4.coeff(0, 0) * pow(t, 2) + 6 * a3.coeff(0, 0) * pow(t, 1) + 2 * a2.coeff(0, 0);
    }
}

void CartesianTrajectory::EE_orientation(MatrixXd &T, MatrixXd &PHI_i, MatrixXd &PHI_f, MatrixXd &o_tilde, MatrixXd &do_tilde, MatrixXd &ddo_tilde, MatrixXd &pi, MatrixXd &pf, double ti, double tf, double Ts)
{

    // @todo understand how to deal with nan orientations, (consider them as infinity or use the PHI_i)
    double qi = 0;
    double dqi = 0;
    double ddqi = 0;
    MatrixXd support = PHI_f - PHI_i;
    double qf = support.norm();
    double dqf = 0;
    double ddqf = 0;

    // int length = (int)floor((tf - ti) / Ts);
    MatrixXd s(1, length);
    MatrixXd sd(1, length);
    MatrixXd sdd(1, length);

    CartesianTrajectory::fifth_polinomials(T, s, sd, sdd, ti, tf, qi, dqi, ddqi, qf, dqf, ddqf, Ts);

    MatrixXd l =  support.norm() == 0 ? support : support / support.norm();
    for (int i = 0; i < length; i++)
    {
        o_tilde.col(i) = PHI_i + l * s.col(i);
        do_tilde.col(i) = l * sd.col(i);
        ddo_tilde.col(i) = l * sdd.col(i);
    }
}

//all methods below are not used

double CartesianTrajectory::sign_func(double x)
{
    if (x > 0)
    {
        return +1.0;
    }
    else if (x == 0)
    {
        return 0.0;
    }
    else
    {
        return -1.0;
    }
}

double CartesianTrajectory::vecangle(Vector3d &v1, Vector3d &v2, Vector3d &normal)
{
    double toRad = 2 * 3.14 / 360;
    Vector3d xprod = v1.cross(v2);

    double a = xprod.dot(normal);
    double s = sign_func(a);
    double c = s * (xprod.norm());
    return atan2(c, v1.dot(v2));

    // Conversion radian to degrees
    // radians = atan(y/x)
    // degrees = radians * (180.0/3.141592653589793238463)
}

int CartesianTrajectory::circular_length(MatrixXd &pi, MatrixXd &pf, double Ts, MatrixXd &c)
{
    MatrixXd n = (pi - c);
    MatrixXd rho(1, 1);
    rho(0, 0) = n.norm();
    Vector3d x = c - pf;
    Vector3d y = c - pi;
    Vector3d r = x.cross(y);
    double arc_angle = vecangle(x, y, r);
    double p_length = rho.coeff(0, 0) * arc_angle;

    int length = (int)floor(p_length / Ts);

    return length;
}

void CartesianTrajectory::circular_motion(MatrixXd &T, MatrixXd &p, MatrixXd &dp, MatrixXd &ddp, MatrixXd &pi, MatrixXd &pf, double Ts, MatrixXd &c, int length)
{

    MatrixXd n = (pi - c);

    MatrixXd rho(1, 1);
    rho(0, 0) = n.norm();
    Vector3d x = c - pf;
    Vector3d y = c - pi;
    Vector3d r = x.cross(y);

    MatrixXd s(1, length);
    int count = 0;
    for (double i = 0; i < length; i++)
    {
        if (count >= length)
        {
            break;
        }
        s(0, count) = i * Ts;
        count++;
    }

    // std::cout << "s "<< s << std::endl;

    MatrixXd p_prime(3, length);
    // p_prime = [rho*cos((s/rho));rho*sin((s/rho));zeros(1,length(s))];

    MatrixXd zero(1, 1);
    zero << 0;
    p_prime.row(0) = (rho.coeff(0, 0) * ((s / rho.coeff(0, 0)).array().cos()));
    p_prime.row(1) = (rho.coeff(0, 0) * ((s / rho.coeff(0, 0)).array().sin()));
    for (int i = 0; i < length; i++)
    {
        p_prime(2, i) = zero.coeff(0, 0);
    }

    // std::cout << "p_prime:" << p_prime << std::endl;
    MatrixXd R(3, 3);

    R.col(0) = (pi - c) / rho.coeff(0, 0);
    R.col(2) = r / (r.norm());
    Vector3d x1 = R.col(0);
    Vector3d z = R.col(2);
    R.col(1) = x1.cross(z);

    MatrixXd tmp(3, length);
    MatrixXd tmp1(3, length);
    MatrixXd tmp2(3, length);

    tmp.row(0) = -((s / rho.coeff(0, 0)).array().sin());
    tmp.row(1) = ((s / rho.coeff(0, 0)).array().cos());
    tmp.row(2) = p_prime.row(2);

    tmp1.row(0) = -((1 / rho.coeff(0, 0)) * (s / rho.coeff(0, 0)).array().cos());
    tmp1.row(1) = -((1 / rho.coeff(0, 0)) * (s / rho.coeff(0, 0)).array().sin());
    tmp1.row(2) = p_prime.row(2);

    // tmp.row(0)  = ((s/rho.coeff(0,0)).array().sin())/pow(rho.coeff(0,0),2);
    // tmp.row(1) = ((s/rho.coeff(0,0)).array().cos())/pow(rho.coeff(0,0),2);
    // tmp.row(2) = p_prime.row(2);

    for (int i = 0; i < length; i++)
    {
        p.col(i) = c + R * p_prime.col(i);
        dp.col(i) = R * tmp.col(i);
        ddp.col(i) = R * tmp1.col(i);
    }
}

void CartesianTrajectory::linear_motion(MatrixXd &T, MatrixXd &p, MatrixXd &dp, MatrixXd &ddp, MatrixXd &s, MatrixXd &pi, MatrixXd &pf, double Ts, int length1)
{
    MatrixXd support = pf - pi;
    double end = support.norm();

    MatrixXd zero(3, 1);
    zero << 0,
        0,
        0;

    for (int i = 0; i < length1; i++)
    {
        p.col(i) = pi + s.coeff(0, i) * (support / end);
        dp.col(i) = support / end;
        ddp.col(i) = zero;
    }
}

void CartesianTrajectory::circular_tilde(MatrixXd &T, MatrixXd &p_tilde, MatrixXd &dp_tilde, MatrixXd &ddp_tilde, MatrixXd &pi, MatrixXd &pf, MatrixXd &c, double ti, double tf, double Ts, int length_qf)
{

    MatrixXd n = (pi - c);

    MatrixXd rho(1, 1);
    rho(0, 0) = n.norm();
    Vector3d x = c - pf;
    Vector3d y = c - pi;
    Vector3d r = x.cross(y);

    double qi = 0;
    double dqi = 0;
    double ddqi = 0;
    double qf = (double)length_qf;
    double dqf = 0;
    double ddqf = 0;

    // int length = (int)floor((tf - ti) / Ts);
    MatrixXd s(1, length);
    MatrixXd sd(1, length);
    MatrixXd sdd(1, length);

    CartesianTrajectory::fifth_polinomials(T, s, sd, sdd, ti, tf, qi, dqi, ddqi, qf, dqf, ddqf, Ts);

    MatrixXd p_prime(3, length);

    MatrixXd zero(1, 1);
    zero << 0;
    p_prime.row(0) = (rho.coeff(0, 0) * ((s / rho.coeff(0, 0)).array().cos()));
    p_prime.row(1) = (rho.coeff(0, 0) * ((s / rho.coeff(0, 0)).array().sin()));
    for (int i = 0; i < length; i++)
    {
        p_prime(2, i) = zero.coeff(0, 0);
    }

    MatrixXd R(3, 3);
    R.col(0) = (pi - c) / rho.coeff(0, 0);
    R.col(2) = r / (r.norm());
    Vector3d x1 = R.col(0);
    Vector3d z = R.col(2);
    R.col(1) = x1.cross(z);

    MatrixXd tmp(3, length);
    MatrixXd tmp1(3, length);
    //MatrixXd tmp2(3,length);

    MatrixXd tmp_1 = ((s / rho.coeff(0, 0)).array().sin());
    MatrixXd tmp_2 = ((s / rho.coeff(0, 0)).array().cos());

    for (int i = 0; i < length; i++)
    {

        tmp(0, i) = -tmp_1.coeff(0, i) * sd.coeff(0, i);
        tmp(1, i) = tmp_2.coeff(0, i) * sd.coeff(0, i);

        // tmp1.row(0)  = -((1/rho.coeff(0,0))*(s/rho.coeff(0,0)).array().cos());
        // tmp1.row(1) = -((1/rho.coeff(0,0))*(s/rho.coeff(0,0)).array().sin());
    }

    tmp.row(2) = p_prime.row(2);
    tmp1.row(2) = p_prime.row(2);

    // tmp.row(0)  = ((s/rho.coeff(0,0)).array().sin())/pow(rho.coeff(0,0),2);
    // tmp.row(1) = ((s/rho.coeff(0,0)).array().cos())/pow(rho.coeff(0,0),2);
    // tmp.row(2) = p_prime.row(2);

    for (int i = 0; i < length; i++)
    {
        p_tilde.col(i) = c + R * p_prime.col(i);
        dp_tilde.col(i) = R * tmp.col(i);
        ddp_tilde.col(i) = R * tmp1.col(i);
    }
}

void CartesianTrajectory::frenet_frame(MatrixXd &p, MatrixXd &dp, MatrixXd &ddp, MatrixXd &o_EE_t, MatrixXd &o_EE_n, MatrixXd &o_EE_b, MatrixXd &PHI_i, MatrixXd &PHI_f, int length)
{

    for (int column = 0; column < 2; column++)
    {

        int index = 0;
        if (column == 1)
        {
            index = length - 1;
        }

        MatrixXd n = (ddp.col(index)) / (ddp.col(index).norm());
        o_EE_t.col(column) = dp.col(index);
        o_EE_n.col(column) = n.col(0);

        Vector3d x = o_EE_t.col(column);
        Vector3d y = o_EE_n.col(column);
        MatrixXd b = x.cross(y);

        o_EE_b.col(column) = b.col(0);
    }

    MatrixXd R(3, 3);

    R.col(0) = o_EE_t.col(0);
    R.col(1) = o_EE_b.col(0);
    R.col(2) = o_EE_n.col(0);

    // std::cout << "R" << R << std::endl;

    PHI_i(0, 0) = atan2(sqrt(pow(R.coeff(0, 2), 2) + pow(R.coeff(1, 2), 2)), R.coeff(2, 2));
    PHI_i(1, 0) = atan2(R.coeff(1, 2), R.coeff(0, 2));
    PHI_i(2, 0) = atan2(R.coeff(2, 1), -R.coeff(2, 0));

    R.col(0) = o_EE_t.col(1);
    R.col(1) = o_EE_b.col(1);
    R.col(2) = o_EE_n.col(1);

    PHI_f(0, 0) = atan2(sqrt(pow(R.coeff(0, 2), 2) + pow(R.coeff(1, 2), 2)), R.coeff(2, 2));
    PHI_f(1, 0) = atan2(R.coeff(1, 2), R.coeff(0, 2));
    PHI_f(2, 0) = atan2(R.coeff(2, 1), -R.coeff(2, 0));
}
