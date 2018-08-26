from math import exp, sin, cos, asin, pi, sqrt
from scipy.optimize import minimize_scalar

# muscle specific parameters
f_m_o, l_m_o, l_t_sl, alpha_opt = 0, 0, 0, 0

# all muslces share these parameters
eps_t_o = 0.033
eps_m_o = 0.6
k_pe = 4.0
gamma = 0.5
dot_l_m_max_tilde = 10.
f_m_len_tilde = 1.8
A_f = 0.3

# Force-Length Relationship of Tendon
f_t_toe_tilde = 0.33
k_toe = 3.0
k_lin = 1.712 / eps_t_o
eps_t_toe = 0.609 * eps_t_o

def g_t_tilde(l_t):
    eps_t = l_t / l_t_sl - 1.
    if eps_t <= eps_t_toe:
        return f_t_toe_tilde * (exp(k_toe * eps_t / eps_t_toe - 1.) - 1.) \
                / \
                (exp(k_toe) - 1.)
    else:
        return k_lin * (eps_t - eps_t_toe) + f_t_toe_tilde


# Passive Force-Length Relationship of Muscle
def g_pl_tilde(l_m):
    l_m_tilde = l_m / l_m_o
    if l_m_tilde <= 1:
        return 0
    else:
        return (exp(k_pe * (l_m_tilde - 1)/eps_m_o) - 1) \
               / \
               (exp(k_pe) - 1)


# Active Force-Length Relationship of Muscle
def g_al_tilde(l_m):
    l_m_tilde = l_m / l_m_o
    return exp(-(l_m_tilde-1)*(l_m_tilde-1)/gamma)


# Force-Velocity Relationship of Muscle
def g_vl_tilde(dot_l_m):
    dot_l_m_tilde = dot_l_m / l_m_o
    if dot_l_m_tilde <= 0:
        return (dot_l_m_tilde + dot_l_m_max_tilde) \
               / \
               (dot_l_m_max_tilde - dot_l_m_tilde/A_f)
    else:
        _a = dot_l_m_tilde * (2. + 2./A_f)
        _b = dot_l_m_max_tilde * (f_m_len_tilde - 1.)
        return (f_m_len_tilde * _a + _b) / (_a + _b)


def compute_activation_deriv_scalar(u, a, tau_act, tau_deact):
    tau_total = 0.
    if u < a:
        tau_total = tau_deact / (0.5 + 1.5*a)
    else:
        tau_total = tau_act * (0.5 + 1.5*a)
    dadt = (u-a) / tau_total
    return dadt


def compute_cos_pennation_scalar(l_m, l_m_opt, pa_opt):
    pa = 0.

    if l_m < 0.:
        l_m = 0.

    if l_m < 1e-6:
        pa = asin(1.)
    else:
        pa = asin( l_m_opt * sin(pa_opt) / l_m )

    if pa > pi/4.:
        pa = pi/4.

    return cos(pa)


def compute_norm_tendon_force_scalar(eps_t, eps_t_o):
    f_t_norm = 0.
    if eps_t > eps_t_toe:
        f_t_norm = k_lin * (eps_t - eps_t_toe) + f_t_toe_tilde
    elif eps_t > 0.:
        f_t_norm = (f_t_toe_tilde / (exp(k_toe)-1.)) * (exp(k_toe * eps_t / eps_t_toe) - 1.)
    else:
        f_t_norm = 0.

    return f_t_norm

def compute_norm_passive_fiber_force_by_length_scalar(l_m_norm, eps_m_o, k_pe):
    f_p_norm = 0.
    if l_m_norm > 1.:
        f_p_norm = (exp(k_pe * (l_m_norm - 1.) / eps_m_o) - 1.) / (exp(k_pe) - 1.)
    else:
        f_p_norm = 0
    return f_p_norm


def compute_norm_active_fiber_force_by_length_scalar(l_m_norm, gamma):
    return exp(-(l_m_norm-1.)*(l_m_norm-1.) / gamma)


def compute_norm_active_fiber_force_by_velocity_scalar(dl_mdt_norm, a_f, f_m_len, v_m_max):
    gv_norm = 0.
    if dl_mdt_norm <= 0.:
        gv_norm = (dl_mdt_norm + v_m_max) / (v_m_max - dl_mdt_norm/a_f)
    else:
        lm_term = dl_mdt_norm*(2.+2./a_f)
        lmax_term = v_m_max*(f_m_len-1.)
        gv_norm = (f_m_len*lm_term + lmax_term) / (lm_term + lmax_term)
    return gv_norm


def compute_norm_fiber_length_deriv_scalar(f_m_norm, a, f_l, a_f, f_m_len, damping, v_m_max, option=None):
    a_f_l = a * f_l
    if damping > 0.:
        d = damping
        k = 1.

        if f_m_norm <= a_f_l:
            _a = d/a_f
            _b = -(a_f_l + f_m_norm/a_f + k*d)
            _c = k*(f_m_norm - a_f_l)
        else:
            _a = -(2.+2./a_f) * d / f_m_len
            _b = -((2.+2./a_f) * (a_f_l*f_m_len - f_m_norm)/(f_m_len-1.) + k*d)
            _c = k*(f_m_norm - a_f_l)

        det = _b*_b - 4*_a*_c
        dl_mdt_unit = (-_b-sqrt(det))/(2.*_a)
    else:
        if f_m_norm <= a_f_l:
            _b = a_f_l + (f_m_norm / a_f)
        else:
            _b = ( (2. + 2. /a_f) * (a_f_l * f_m_len - f_m_norm ) ) / (f_m_len - 1.)

        if _b > 0.:
            dl_mdt_unit = (f_m_norm - a_f_l) / _b
        else:
            dl_mdt_unit = 0.

    return v_m_max * dl_mdt_unit


def get_fiber_length_deriv_scalar(a, l_m, l_mt, l_m_opt, pa_opt, l_t_sl, eps_t_o, eps_m_o,
                                  k_pe, gamma, a_f, f_m_len, damping, v_m_max, option=None):
    cos_pa = compute_cos_pennation_scalar(l_m, l_m_opt, pa_opt)

    l_t = l_mt - l_m * cos_pa
    eps_t = (l_t - l_t_sl) / l_t_sl
    f_t_norm = compute_norm_tendon_force_scalar(eps_t, eps_t_o)

    l_m_norm = l_m / l_m_opt
    f_p_norm = compute_norm_passive_fiber_force_by_length_scalar(l_m_norm, eps_m_o, k_pe)
    f_l = compute_norm_active_fiber_force_by_length_scalar(l_m_norm, gamma)

    f_m_norm = f_t_norm / cos_pa - f_p_norm
    dl_mdt_norm = compute_norm_fiber_length_deriv_scalar(f_m_norm, a, f_l, a_f, f_m_len, damping, v_m_max, option)

    dl_mdt = l_m_opt * dl_mdt_norm

    return dl_mdt


def get_isometric_fiber_length(a, l_m, l_mt, l_m_opt, pa_opt, l_t_sl, eps_t_o, eps_m_o,
                               k_pe, gamma, a_f, f_m_len, damping, v_m_max, option=None):
    def obj_dl_m(_l_m):
        dl_mdt = get_fiber_length_deriv_scalar(a, _l_m, l_mt, l_m_opt, pa_opt, l_t_sl, eps_t_o, eps_m_o,
                                  k_pe, gamma, a_f, f_m_len, damping, v_m_max, 'modified_damping')
        return dl_mdt * dl_mdt

    ub = max(0., l_mt - l_t_sl)

    result = minimize_scalar(obj_dl_m, bounds=(0., ub), method='bounded')
    return result.x


class Muscle(object):
    def __init__(self):
        pass