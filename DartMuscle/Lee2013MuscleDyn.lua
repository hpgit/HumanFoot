function computeActivationDeriv_scalar(u, a, tau_act, tau_deact)
	local tau_total;
	if u<a then
		tau_total = tau_deact/(0.5+1.5*a);
	else
		tau_total = tau_act*(0.5+1.5*a);
	end
	local dadt = (u-a) / tau_total;
	return dadt
end

function computeActivationDeriv(u, a, tau_act, tau_deact)
	local dadt = {}
	for i=1,#u do
		dadt[i] = computeActivationDeriv_scalar(u[i], a[i], tau_act[i], tau_deact[i])
	end
    return dadt
end

function computeCosPennation(l_m, l_m_opt, pa_opt)
	local cos_pa = {}
	for i=1,#l_m do
		cos_pa[i] = computeCosPennation_scalar(l_m[i], l_m_opt[i], pa_opt[i])
	end
    return cos_pa
end

function computeCosPennation_scalar(l_m, l_m_opt, pa_opt)
	local sin = math.sin
	local cos = math.cos
	local rad = math.rad
	local pa

	if l_m < 0. then
		l_m = 0.
	end

	if l_m==0. then
		pa = ASIN(1.)
	else
    	pa = ASIN( l_m_opt * sin(pa_opt) / l_m)
	end

	if pa > rad(45) then
		pa = rad(45)
	end

    local cos_pa = cos(pa)
	return cos_pa
end

function computeNormTendonForce(eps_t, eps_t_o)
	local f_t_norm = {}
	for i=1,#eps_t do
		f_t_norm[i] = computeNormTendonForce_scalar(eps_t[i], eps_t_o[i])
	end
    return f_t_norm
end

function computeNormTendonForce_scalar(eps_t, eps_t_o)
    local k_toe = 3.0
    local F_T_toe = 0.33
    local k_lin = 1.712 / eps_t_o
    local eps_t_toe = 0.609 * eps_t_o

	local f_t_norm;
    if eps_t > eps_t_toe then
        f_t_norm = k_lin * (eps_t - eps_t_toe) + F_T_toe
    elseif eps_t > 0 then
        f_t_norm = (F_T_toe / (math.exp(k_toe)-1.)) * (math.exp(k_toe * eps_t / eps_t_toe) - 1.)
    else
        f_t_norm = 0;
	end

    return f_t_norm
end

function computeNormPassiveFiberForceByLength_scalar(l_m_norm, eps_m_o, k_pe)
	local f_p_norm;
    if l_m_norm > 1. then
        f_p_norm = (math.exp(k_pe * (l_m_norm - 1.) / eps_m_o) - 1.) / (math.exp(k_pe) - 1.)
    else
        f_p_norm = 0
	end
    return f_p_norm
end

function computeNormActiveFiberForceByLength_scalar(l_m_norm, gamma)
	local exp = math.exp
    local f_l = exp(-(l_m_norm-1.)*(l_m_norm-1.) / gamma)
    return f_l
end

function computeNormActiveFiberForceByVelocity_scalar(dl_mdt_norm, a_f, f_m_len, v_m_max)
	local gv_norm
    if dl_mdt_norm <= 0. then
        gv_norm = (dl_mdt_norm + v_m_max) / (v_m_max - dl_mdt_norm/a_f)
    else
        local lm_term = dl_mdt_norm*(2.+2./a_f)
        local lmax_term = v_m_max*(f_m_len-1.)
        gv_norm = (f_m_len*lm_term + lmax_term) / (lm_term + lmax_term)
	end
    return gv_norm
end

function computeNormFiberLengthDeriv_scalar(f_m_norm, a, f_l, a_f, f_m_len, damping, v_m_max, option)
    local a_f_l = a * f_l

	if damping~=0 then
		local d = damping
		local k = 1
		local a,b,c;

		if f_m_norm <= a_f_l then
			a = d/a_f
			b = -(a_f_l + f_m_norm/a_f + k*d)
			c = k*(f_m_norm - a_f_l)
		else
			a = -(2+2/a_f)*d/f_m_len
			b = -((2+2/a_f)*(a_f_l*f_m_len - f_m_norm)/(f_m_len-1) + k*d)
			c = k*(f_m_norm - a_f_l)
		end

		local det = b*b-4*a*c
		local dl_mdt_unit = (-b-math.sqrt(det))/(2*a)
		--return dl_mdt_unit
		return dl_mdt_unit*v_m_max
	else
        if f_m_norm <= a_f_l then
            b = a_f_l + (f_m_norm / a_f)
        else
            b = ( (2. + 2. /a_f) * (a_f_l * f_m_len - f_m_norm ) ) / (f_m_len - 1.)
		end

        if b==0 then
            dl_mdt_unit = 0.
        else
            dl_mdt_unit = (f_m_norm - a_f_l) / b
		end

		----clamping
		--if f_m_norm < 0. then
			--dl_mdt_unit = -1.
		--elseif f_m_norm > a_f_l*f_m_len or (f_m_norm > 0. and dl_mdt_unit > 1.) then
			--dl_mdt_unit = 1.
		--end

		return dl_mdt_unit*v_m_max
	end
end

function getFiberLengthDeriv_scalar(a, l_m, l_mt, l_m_opt, pa_opt, l_t_sl, eps_t_o, eps_m_o,
						k_pe, gamma, a_f, f_m_len, damping, v_m_max, option)

	local cos_pa = computeCosPennation_scalar(l_m, l_m_opt, pa_opt)

	local l_t = l_mt - l_m * cos_pa
	local eps_t = (l_t - l_t_sl) / l_t_sl
	local f_t_norm = computeNormTendonForce_scalar(eps_t, eps_t_o)

	local l_m_norm = l_m / l_m_opt
	local f_p_norm = computeNormPassiveFiberForceByLength_scalar(l_m_norm, eps_m_o, k_pe)
	local f_l = computeNormActiveFiberForceByLength_scalar(l_m_norm, gamma)

	local f_m_norm = f_t_norm / cos_pa - f_p_norm
	local dl_mdt_norm = computeNormFiberLengthDeriv_scalar(f_m_norm, a, f_l, a_f, f_m_len, damping, v_m_max, option)

	local dl_mdt = l_m_opt * dl_mdt_norm

	return dl_mdt
end

function getFiberLengthDeriv(a, l_m, l_mt, l_m_opt, pa_opt, l_t_sl, eps_t_o, eps_m_o,
						k_pe, gamma, a_f, f_m_len, damping, v_m_max, option)
	local dl_mdt = {}
	for i=1,#l_m do
		dl_mdt[i] = getFiberLengthDeriv_scalar(a[i], l_m[i], l_mt[i], l_m_opt[i], pa_opt[i], l_t_sl[i], eps_t_o[i], eps_m_o[i],
						k_pe[i], gamma[i], a_f[i], f_m_len[i], damping[i], v_m_max[i], option)

	end
	return dl_mdt
end

function getIsometricFiberLength(a, l_mt, l_m_opt, pa_opt, l_t_sl, eps_t_o, eps_m_o,
                        k_pe, gamma, a_f, f_m_len, damping, v_m_max)
	local iso_l_m = {}
	for m=1,#a do

		local function obj_dl_m(l_m)
			local dl_mdt = getFiberLengthDeriv_scalar(a[m], l_m, l_mt[m], l_m_opt[m],
											   pa_opt[m], l_t_sl[m], eps_t_o[m], eps_m_o[m],
											   k_pe[m], gamma[m], a_f[m], f_m_len[m],
											   damping[m], v_m_max[m], 'modified_damping')
			--print(l_m, dl_mdt*dl_mdt)
			return dl_mdt*dl_mdt
		end

		--local function obj_dl_m(l_m)
			--local cos_pa = computeCosPennation_scalar(l_m, l_m_opt[m], pa_opt[m])
			--local l_t = l_mt[m] - l_m * cos_pa
			--local eps_t = (l_t - l_t_sl[m]) / l_t_sl[m]

			--local gt = computeNormTendonForce_scalar(eps_t, eps_t_o[m])
			--local gal = computeNormActiveFiberForceByLength_scalar(l_m/l_m_opt[m], gamma[m])
			--local gv = computeNormActiveFiberForceByVelocity_scalar(0, a_f[m], f_m_len[m], v_m_max[m])
			--local gpl = computeNormPassiveFiberForceByLength_scalar(l_m/l_m_opt[m], eps_m_o[m], k_pe[m])

			--local diff = gt/cos_pa - (a[m]*gal*gv+gpl)
			--return diff*diff
		--end

		local ub;
		--ub = l_mt[m]
		if l_mt[m]-l_t_sl[m]>0 then
			ub = l_mt[m]-l_t_sl[m]
		else
			ub = 0.
		end
		local x0 = ub
		--ub = ub + .2

		--print'-------------------'
		--print(m)
		--print(l_mt[m]-l_t_sl[m])
		--print('x0 ', x0)

		local fret;
		fret, iso_l_m[m] = min_brent(obj_dl_m, x0, .0, ub)
	end
	return iso_l_m
end

function getMetabolicEnergyRate_muscle(mass, u, a, l_m, l_m_opt, dl_m, f_mtu, f_ce)
	local function fA(u)
		local sin = math.sin
		local cos = math.cos
		local hpi = math.pi/2.
		return 40*0.5*sin(hpi*u) + 133*0.5*(1-cos(hpi*u))
	end
	local dA = mass*fA(u)

	local function g(l_ce)
		if l_ce <= 0.5 then
			return 0.5
		elseif l_ce <= 1.0 then
			return l_ce
		elseif l_ce <= 1.5 then
			return -2*l_ce + 3
		else
			return 0
		end
	end
	local function fM(a)
		local sin = math.sin
		local cos = math.cos
		local hpi = math.pi/2.
		return 74*0.5*sin(hpi*a) + 111*0.5*(1-cos(hpi*a))
	end
	local dM = mass*g(l_m/l_m_opt)*fM(a)

	local dS
	if dl_m < 0 then
		dS = 0.25*f_mtu*-dl_m
	else
		dS = 0
	end

	local dW
	if dl_m < 0 then
		dW = f_ce*-dl_m
	else
		dW = 0
	end

	return dA + dM + dS + dW
end


--unit : 1 J/s = (1/4.184) cal/s)
function getMetabolicEnergyRate(totalMass, mass, u, a, l_m, l_m_opt, dl_m, f_mtu, f_ce)
	local dE = 0
	for i=1,#mass do
		dE = dE + getMetabolicEnergyRate_muscle(mass[i], u[i], a[i], l_m[i], l_m_opt[i], dl_m[i], f_mtu[i], f_ce[i])
	end
	local dB = 1.51 * totalMass		--basal metabolic energy rate
	return dE + dB
end


--yuri
function getApproximatedMetabolicEnergyRate_b_muscle(mass, u, a, l_m, l_m_opt, dl_m, f_mtu, f_ce)

	local dS
	if dl_m < 0 then
		dS = 0.25*f_mtu*-dl_m
	else
		dS = 0
	end

	local dW
	if dl_m < 0 then
		dW = f_ce*-dl_m
	else
		dW = 0
	end

	return dS + dW
end

function getApproximatedMetabolicEnergyRate_a_muscle(mass, u, a, l_m, l_m_opt, dl_m, f_mtu, f_ce)

	local dA = mass* 78

	local function g(l_ce)
		if l_ce <= 0.5 then
			return 0.5
		elseif l_ce <= 1.0 then
			return l_ce
		elseif l_ce <= 1.5 then
			return -2*l_ce + 3
		else
			return 1.0e-10		--for avoiding singular matrix
			--return 0
		end
	end

	local dM = mass*g(l_m/l_m_opt)*90

	return dA + dM
end

--yuri
--unit : 1 J/s = (1/4.184) cal/s)
function getApproximatedMetabolicEnergyRate(totalMass, mass, u, a, l_m, l_m_opt, dl_m, f_mtu, f_ce)
	--local a = vectorn(#mass)
	--local b = vectorn(#mass)
	--local km = matrixn()
	--local dE = 0

	local a = {}
	local b = {}
	for i=1,#mass do
		b[i] = getApproximatedMetabolicEnergyRate_b_muscle(mass[i], u[i], a[i], l_m[i], l_m_opt[i], dl_m[i], f_mtu[i], f_ce[i])
		a[i] = getApproximatedMetabolicEnergyRate_a_muscle(mass[i], u[i], a[i], l_m[i], l_m_opt[i], dl_m[i], f_mtu[i], f_ce[i])
	end

--km:setsize(b:size(),2)
--km:column(0):assign(a)
--km:column(1):assign(b)

	--local dB = 1.51 * totalMass		--basal metabolic energy rate
	--return dE + dB
--return km
	return a, b
end