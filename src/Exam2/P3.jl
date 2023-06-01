include("definitions.jl")
include("utils.jl")
include("matlab_utils.jl")
using DifferentialEquations
using Plots

function FlightPathAnglesFromState(aircraft_state)

    euler_angles = EulerAngles(aircraft_state[4:6])
    wind_angles = AirRelativeVelocityVectorToWindAngles(aircraft_state[7:9])
    Vg = TransformFromBodyToInertial(aircraft_state[7:9], euler_angles)
    #=
    We know γ_a = γ when there is no wind.
    Thus, γ = γ_a = θ - α
    =#
    # γ = aircraft_state.θ - wind_angles.α
    γ = atan(Vg[3], sqrt(Vg[1]^2 + Vg[2]^2))
    χ = atan(Vg[2], Vg[1])

    return (Vg,χ,γ)
end

function AeroForcesAndMomentsBodyStateWindCoeffs(aircraft_state::AircraftState, aircraft_surfaces::AircraftControl, wind_inertial, density, aircraft_parameters::AircraftParameters)

    ρ = density #stdatmo(-aircraft_state.z)
    euler_angles = EulerAngles(aircraft_state.ϕ, aircraft_state.θ, aircraft_state.ψ)
    aircraft_velocity = [aircraft_state.u,aircraft_state.v,aircraft_state.w]   #In body frame
    wind_body_frame = TransformFromInertialToBody(wind_inertial, euler_angles)
    air_relative_speed = aircraft_velocity - wind_body_frame   #Va vector in body frame
    wind_angles = AirRelativeVelocityVectorToWindAngles(air_relative_speed)
    Va = wind_angles.Va
    Q = 0.5*ρ*Va*Va
    S = aircraft_parameters.S
    b = aircraft_parameters.b
    c = aircraft_parameters.c
    CoefficientsDict = GetCoefficients(aircraft_state, aircraft_surfaces, wind_angles, aircraft_parameters)

    #Calculate force along body X
    C_X = CoefficientsDict["CX"]
    C_T = CoefficientsDict["CT"]
    T = S*Q*C_T
    X = S*Q*C_X + T

    #Calculate force along body Y
    C_Y = CoefficientsDict["CY"]
    Y = S*Q*C_Y

    #Calculate force along body Z
    C_Z = CoefficientsDict["CZ"]
    Z = S*Q*C_Z

    #Calculate moment along body X
    C_l = CoefficientsDict["Cl"]
    G = 0.0  #As mentioned by Dr.Frew on Slack
    L = b*S*Q*C_l + G

    #Calculate moment along body Y
    C_m = CoefficientsDict["Cm"]
    M = c*S*Q*C_m

    #Calculate moment along body Z
    C_n = CoefficientsDict["Cn"]
    N = b*S*Q*C_n

    aero_force = [X,Y,Z]
    aero_moment = [L,M,N]
    return aero_force,aero_moment
end

function GravityForcesBodyState(aircraft_state::AircraftState, aircraft_parameters::AircraftParameters)

    m = aircraft_parameters.m
    g = aircraft_parameters.g
    roll = aircraft_state.ϕ #phi
    pitch = aircraft_state.θ #theta

    #Calculate force along body X
    X = -m*g*sin(pitch)
    #Calculate force along body Y
    Y = m*g*cos(pitch)*sin(roll)
    #Calculate force along body Z
    Z = m*g*cos(pitch)*cos(roll)

    gravity_force = [X,Y,Z]
    return gravity_force
end

function AircraftForcesAndMoments(aircraft_state::AircraftState, aircraft_surfaces::AircraftControl, wind_inertial, density, aircraft_parameters::AircraftParameters)
    aero_force,aero_moment = AeroForcesAndMomentsBodyStateWindCoeffs(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters)
    gravity_force = GravityForcesBodyState(aircraft_state, aircraft_parameters)
    total_force = aero_force+gravity_force
    total_moment = aero_moment
    return total_force, total_moment
end

function AircraftEOM(t::Float64,aircraft_state::AircraftState,aircraft_surfaces::AircraftControl,wind_inertial::Array{Float64,1},aircraft_parameters::AircraftParameters)

    ρ = stdatmo(-aircraft_state.z)
    m = aircraft_parameters.m
    euler_angles = EulerAngles(aircraft_state.ϕ, aircraft_state.θ, aircraft_state.ψ)
    total_force, total_moment = AircraftForcesAndMoments(aircraft_state,aircraft_surfaces,wind_inertial,ρ,aircraft_parameters)

    #Position derivatives
    velocity_vector = SVector{3,Float64}(aircraft_state.u, aircraft_state.v, aircraft_state.w)
    position_dot = TransformFromBodyToInertial(velocity_vector, euler_angles)

    #Euler Angle derivatives
    multiplication_matrix = GetRotationalKinematicsMatrix(euler_angles)
    roll_rate_matrix = [aircraft_state.p, aircraft_state.q, aircraft_state.r]
    euler_angles_dot = multiplication_matrix*roll_rate_matrix

    #Velocity derivatives
    u = aircraft_state.u
    v = aircraft_state.v
    w = aircraft_state.w
    u_dot = (aircraft_state.r*v) - (aircraft_state.q*w) + (total_force[1]/m)
    v_dot = (aircraft_state.p*w) - (aircraft_state.r*u) + (total_force[2]/m)
    w_dot = (aircraft_state.q*u) - (aircraft_state.p*v) + (total_force[3]/m)
    velocity_dot = [u_dot,v_dot,w_dot]

    #Rate of rotation derivatives
    Gamma = GetGammaValues(aircraft_parameters)
    p_dot = (Gamma[1]*aircraft_state.p*aircraft_state.q) - (Gamma[2]*aircraft_state.q*aircraft_state.r) +
                (Gamma[3]*total_moment[1]) + (Gamma[4]*total_moment[3])
    q_dot = (Gamma[5]*aircraft_state.p*aircraft_state.r) - (Gamma[6]*(aircraft_state.p^2 - aircraft_state.r^2)) +
                (total_moment[2]/aircraft_parameters.Iy)
    r_dot = (Gamma[7]*aircraft_state.p*aircraft_state.q) - (Gamma[1]*aircraft_state.q*aircraft_state.r) +
                (Gamma[4]*total_moment[1]) + (Gamma[8]*total_moment[3])
    rotation_rate_dot = [p_dot,q_dot,r_dot]

    x_dot = vcat(position_dot,euler_angles_dot,velocity_dot,rotation_rate_dot)
    return x_dot
end

function aircraft_dynamics!(du,u,p,t)

    # ϕ = wrap_between_negative_pi_to_pi(u[3])
    # θ = wrap_between_negative_pi_to_pi(u[4])
    # ψ = wrap_between_negative_pi_to_pi(u[5])
    # aircraft_state = AircraftState(vcat(u[1:3],[ϕ,θ,ψ],u[7:12]))
    aircraft_state = AircraftState(u...)
    control_inputs = AircraftControl(p[1]...)
    wind_inertial = p[2]
    aircraft_parameters = p[3]
    x_dot = AircraftEOM(t,aircraft_state,control_inputs,wind_inertial,aircraft_parameters)
    for i in 1:length(u)
        du[i] = x_dot[i]
    end
end

function simulate(dynamics::Function, initial_state::Array{Float64,1}, time_interval::Array{Float64,1}, extra_parameters, save_at_value=1.0)
    prob = ODEProblem(dynamics,initial_state,time_interval,extra_parameters)
    sol = DifferentialEquations.solve(prob,saveat=save_at_value)
    aircraft_states = AircraftState[]
    for i in 1:length(sol.u)
        push!(aircraft_states,AircraftState(sol.u[i]...))
    end
    return aircraft_states
end


function guidance_model_dynamics!(du,u,p,t)

    #=
    u = [ pn, pe, Xdot, hdot, Va]
    =#
    pn = u[1]
    pe = u[2]
    Xdot = u[3]
    hdot = u[4]
    Va = u[5]

    Vg,X,γ = FlightPathAnglesFromState(p[6])
    ψ = X - asin( (1*Va) * (we*cos(X) - wn*sin(X)) )

    #=Tunable Parameters=#
    bXdot = 11.5
    bX = 1.5
    ahdot = 0.5
    bhdot = 0.75
    bh = 2.5
    bVa = 0.0

    hc = p[1]
    Xc = p[2]
    Vac = p[3]
    wn = p[4]
    we = p[5]
    h = p[6].z
    Xcdot = 0.0
    hcdot = 0.0

    pndot = Va*cos(ψ) + wn
    pedot = Va*sin(ψ) + we
    Xdoubledot = bXdot*(Xcdot - Xc) + bX*(Xc - X)
    hdoublefot = -ahdot*hcdot - bhdot*hdot + bh*(hc-h)
    Vadot = bVa*(Vac - Va)

    du[1] = pndot
    du[2] = pedot
    du[3] = Xdoubledot
    du[4] = hdoublefot
    du[5] = Vadot
end

function simulate(dynamics::Function, initial_state, time_interval, extra_parameters, save_at_value=1.0)
    prob = ODEProblem(dynamics,initial_state,time_interval,extra_parameters)
    sol = DifferentialEquations.solve(prob,saveat=save_at_value)
    guidance_model_data = []
    for i in 1:length(sol.u)
        push!(guidance_model_data,sol.u[i]...)
    end
    return guidance_model_data
end


function compare_plots( guidance_model_data, actual_dynamics_data, plot_location, save_plots=true)

    gm_pn_values = [state[1] for state in guidance_model_data]
    gm_pe_values = [state[2] for state in guidance_model_data]
    gm_ψ_values = [state[3] for state in guidance_model_data]
    gm_hdot_values = [state[4] for state in guidance_model_data]
    gm_Va_values = [state[5] for state in guidance_model_data]

    ad_pn_values = [state[1] for state in actual_dynamics_data]
    ad_pe_values = [state[2] for state in actual_dynamics_data]
    ad_ψ_values = [state[3] for state in actual_dynamics_data]
    ad_hdot_values = [state[4] for state in actual_dynamics_data]
    ad_Va_values = [state[5] for state in actual_dynamics_data]

    #Pn plots
    p1 = plot( time, gm_pn_values, xlabel = "Time (in s)", ylabel="pn (in m)", label = "Guidance Model")
    p2 = plot( time, ad_pn_values, xlabel = "Time (in s)", ylabel="pn (in m)", label = "Actual Aircraft dynamics")
    p_pos = plot(p1, p2, layout=(2,1))
    if(save_plots)
        savefig(plots_location*"/pn.png")
    end

    #Pe plots
    p1 = plot( time, gm_pe_values, xlabel = "Time (in s)", ylabel="pe (in m)", label = "Guidance Model")
    p2 = plot( time, ad_pe_values, xlabel = "Time (in s)", ylabel="pe (in m)", label = "Actual Aircraft dynamics")
    p_pos = plot(p1, p2, layout=(2,1))
    if(save_plots)
        savefig(plots_location*"/pe.png")
    end

    #ψ plots
    p1 = plot( time, gm_ψ_values, xlabel = "Time (in s)", ylabel="ψ", label = "Guidance Model")
    p2 = plot( time, ad_ψ_values, xlabel = "Time (in s)", ylabel="ψ", label = "Actual Aircraft dynamics")
    p_pos = plot(p1, p2, layout=(2,1))
    if(save_plots)
        savefig(plots_location*"/psi.png")
    end

    #hdot plots
    p1 = plot( time, gm_hdot_values, xlabel = "Time (in s)", ylabel="hdot (in m/s)", label = "Guidance Model")
    p2 = plot( time, ad_hdot_values, xlabel = "Time (in s)", ylabel="hdot (in m/s)", label = "Actual Aircraft dynamics")
    p_pos = plot(p1, p2, layout=(2,1))
    if(save_plots)
        savefig(plots_location*"/hdot.png")
    end

    #Va plots
    p1 = plot( time, gm_Va_values, xlabel = "Time (in s)", ylabel="Va (in m/s)", label = "Guidance Model")
    p2 = plot( time, ad_Va_values, xlabel = "Time (in s)", ylabel="Vs (in m/s)", label = "Actual Aircraft dynamics")
    p_pos = plot(p1, p2, layout=(2,1))
    if(save_plots)
        savefig(plots_location*"/Va.png")
    end

end
