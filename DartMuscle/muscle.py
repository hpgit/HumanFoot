from math import exp

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




class Muscle(object):
    def __init__(self):
        pass