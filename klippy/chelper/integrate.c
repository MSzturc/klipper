// Helpers to integrate the smoothing weight function
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020-2023  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "compiler.h" // unlikely
#include "integrate.h"
#include "trapq.h" // struct move

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/****************************************************************
 * Generic smoother integration
 ****************************************************************/

static double coeffs[] = {
    1./1., 1./2., 1./3., 1./4., 1./5., 1./6., 1./7., 1./8., 1./9., 1./10.,
    1./11., 1./12., 1./13., 1./14., 1./15., 1./16., 1./17., 1./18., 1./19.,
};

inline smoother_antiderivatives
calc_antiderivatives(const struct smoother* sm, double t)
{
    int n = sm->n, i;
    double it0 = (sm->c0[0] * t + sm->c0[1]) * t;
    double it1 = (sm->c1[0] * t + sm->c1[1]) * t;
    double it2 = (sm->c2[0] * t + sm->c2[1]) * t;
    for (i = 2; i < n; ++i) {
        it0 = (it0 + sm->c0[i]) * t;
        it1 = (it1 + sm->c1[i]) * t;
        it2 = (it2 + sm->c2[i]) * t;
    }
    it1 *= t;
    it2 *= t * t;
    return (smoother_antiderivatives) {
        .it0 = it0, .it1 = it1, .it2 = it2 };
}

inline smoother_antiderivatives
diff_antiderivatives(const smoother_antiderivatives* ad1
                     , const smoother_antiderivatives* ad2)
{
    return (smoother_antiderivatives) {
        .it0 = ad2->it0 - ad1->it0,
        .it1 = ad2->it1 - ad1->it1,
        .it2 = ad2->it2 - ad1->it2 };
}

inline double
integrate_move(const struct move* m, int axis, double base, double t0
               , const smoother_antiderivatives* s
               , double* smooth_velocity)
{
    double axis_r = m->axes_r.axis[axis - 'x'];
    double start_v = m->start_v * axis_r;
    double half_accel = m->half_accel * axis_r;
    // Substitute the integration variable tnew = t0 - t to simplify integrals
    double accel = 2. * half_accel;
    base += (half_accel * t0 + start_v) * t0;
    start_v += accel * t0;
    double smooth_pos = base * s->it0 - start_v * s->it1 + half_accel * s->it2;
    if (smooth_velocity)
        *smooth_velocity = start_v * s->it0 - accel * s->it1;
    return smooth_pos;
}

/****************************************************************
 * Smoother initialization
 ****************************************************************/

int
init_smoother(int n, const double a[], double t_sm, struct smoother* sm)
{
    if ((t_sm && n < 2) || n > ARRAY_SIZE(sm->c0))
        return -1;
    memset(sm, 0, sizeof(*sm));
    sm->n = n;
    sm->hst = 0.5 * t_sm;
    if (!t_sm) return 0;
    double inv_t_sm = 1. / t_sm;
    double inv_t_sm_n = inv_t_sm;
    int i, symm = n & 1;
    for (i = 0; i < n; ++i) {
        if ((i & 1) && a[i]) symm = 0;
        double c = a[i] * inv_t_sm_n;
        sm->c0[n-1-i] = c * coeffs[i];
        sm->c1[n-1-i] = c * coeffs[i+1];
        sm->c2[n-1-i] = c * coeffs[i+2];
        inv_t_sm_n *= inv_t_sm;
    }
    sm->symm = symm;
    double inv_norm = 1. / (calc_antiderivatives(sm, sm->hst).it0
                            - calc_antiderivatives(sm, -sm->hst).it0);
    for (i = 0; i < n; ++i) {
        sm->c0[i] *= inv_norm;
        sm->c1[i] *= inv_norm;
        sm->c2[i] *= inv_norm;
    }
    sm->p_hst = calc_antiderivatives(sm, sm->hst);
    sm->m_hst = calc_antiderivatives(sm, -sm->hst);
    sm->pm_diff = diff_antiderivatives(&sm->m_hst, &sm->p_hst);
    sm->t_offs = sm->pm_diff.it1;
    return 0;
}

/****************************************************************
 * Barycentric interpolation on Chebyshev nodes
 ****************************************************************/

// Chebyshev nodes of the second kind, mapped onto [a, b]:
//   x_k = (a+b)/2 + (b-a)/2 * cos(k*pi/(npts-1)),  k = 0..npts-1
// Requires npts >= 2; for npts < 2 the denominator (npts-1) is zero or negative,
// making the node spacing undefined. All callers guarantee npts >= 8.
void
bary_nodes(double a, double b, int npts, double x[])
{
    double mid = 0.5 * (a + b), half = 0.5 * (b - a);
    int last = npts - 1;
    for (int k = 0; k <= last; ++k)
        x[k] = mid + half * cos((double)k * M_PI / (double)last);
}

// Barycentric weights for Chebyshev-2nd-kind nodes are known in closed form:
//   w_k = (-1)^k, halved at the two endpoints. A common positive factor
//   cancels in bary_eval, so sign and endpoint halving are all that matter.
void
bary_weights(int npts, double w[])
{
    int last = npts - 1;
    for (int k = 0; k <= last; ++k) {
        double s = (k & 1) ? -1.0 : 1.0;
        w[k] = (k == 0 || k == last) ? 0.5 * s : s;
    }
}

// Evaluate the barycentric interpolant through (x[k], f[k]) at xq.
double
bary_eval(int npts, const double x[], const double w[]
          , const double f[], double xq)
{
    double num = 0., den = 0.;
    for (int k = 0; k < npts; ++k) {
        double d = xq - x[k];
        if (d == 0.)
            return f[k];                 // xq hits a node exactly
        double q = w[k] / d;
        num += q * f[k];
        den += q;
    }
    return num / den;
}
