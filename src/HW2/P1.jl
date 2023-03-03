include("../definitions.jl")
include("../utils.jl")
include("../matlab_utils.jl")
using Plots

function AeroForcesAndMomentsBodyStateWindCoeffs(aircraft_state::AircraftState, aircraft_surfaces::AircraftControl, wind_inertial, density, aircraft_parameters::AircraftParameters)

    rho = density #stdatmo(-aircraft_state.z)
    euler_angles = EulerAngles(aircraft_state.ϕ, aircraft_state.θ, aircraft_state.ψ)
    aircraft_velocity = [aircraft_state.u,aircraft_state.v,aircraft_state.w]   #In body frame
    wind_body_frame = TransformFromInertialToBody(wind_inertial, euler_angles)
    VelocityBody = aircraft_velocity - wind_body_frame   #Va vector in body frame
    wind_angles = AirRelativeVelocityVectorToWindAngles(VelocityBody)
    Va = wind_angles.Va
    Q = 0.5*rho*Va*Va
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

function AircraftEOM(aircraft_state,aircraft_surfaces,wind_inertial,aircraft_parameters)

    rho = stdatmo(-aircraft_state.z)
    m = aircraft_parameters.m
    euler_angles = EulerAngles(aircraft_state.ϕ, aircraft_state.θ, aircraft_state.ψ)
    total_force, total_moment = AircraftForcesAndMoments(aircraft_state,aircraft_surfaces,wind_inertial,rho,aircraft_parameters)

    #Position derivatives
    velocity_vector = [aircraft_state.u, aircraft_state.v, aircraft_state.w]
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

function PlotSimulation(time, aircraft_state_array, control_input_array, location, save_plots=false)

    statefields = fieldnames(AircraftState)
    controlfields = fieldnames(AircraftControl)
    plots_location = pwd()*location
    println(plots_location)

    #Extract State values
    x_pos_values = [getfield(state,:x) for state in aircraft_state_array]
    y_pos_values = [getfield(state,:y) for state in aircraft_state_array]
    z_pos_values = [getfield(state,:z) for state in aircraft_state_array]
    roll_values = [getfield(state,:ϕ) for state in aircraft_state_array]
    pitch_values = [getfield(state,:θ) for state in aircraft_state_array]
    yaw_values = [getfield(state,:ψ) for state in aircraft_state_array]
    u_values = [getfield(state,:u) for state in aircraft_state_array]
    v_values = [getfield(state,:v) for state in aircraft_state_array]
    w_values = [getfield(state,:w) for state in aircraft_state_array]
    p_values = [getfield(state,:p) for state in aircraft_state_array]
    q_values = [getfield(state,:q) for state in aircraft_state_array]
    r_values = [getfield(state,:r) for state in aircraft_state_array]

    #Extract Control values
    da_values = [getfield(control,:δa) for control in control_input_array]
    de_values = [getfield(control,:δe) for control in control_input_array]
    dr_values = [getfield(control,:δr) for control in control_input_array]
    dt_values = [getfield(control,:δt) for control in control_input_array]

    #Position plots
    px = plot( time, x_pos_values, xlabel = "Time (in s)", ylabel="x (in m)"  )
    py = plot( time, y_pos_values, xlabel = "Time (in s)", ylabel="y (in m)" )
    pz = plot( time, z_pos_values, xlabel = "Time (in s)", ylabel="z (in m)" )
    p_pos = plot(px, py, pz, layout=(3,1))
    if(save_plots)
        savefig(plots_location*"/position.png")
    end

    #Orientation plots
    proll = plot( time, roll_values, xlabel = "Time (in s)", ylabel="Roll (in radians)", legends=false )
    ppitch = plot( time, pitch_values, xlabel = "Time (in s)", ylabel="Pitch (in radians)", legends=false )
    pyaw = plot( time, yaw_values, xlabel = "Time (in s)", ylabel="Yaw (in radians)", legends=false )
    p_orientation = plot(proll, ppitch, pyaw, layout=(3,1))
    if(save_plots)
        savefig(plots_location*"/orientation.png")
    end

    #Velocity plots
    pu = plot( time, u_values, xlabel = "Time (in s)", ylabel="u (in m/s)", legends=false  )
    pv = plot( time, v_values, xlabel = "Time (in s)", ylabel="v (in m/s)", legends=false  )
    pw = plot( time, w_values, xlabel = "Time (in s)", ylabel="w (in m/s)", legends=false  )
    p_velocity = plot(pu, pv, pw, layout=(3,1))
    if(save_plots)
        savefig(plots_location*"/velocity.png")
    end

    #Roll rate plots
    pp = plot( time, p_values, xlabel = "Time (in s)", ylabel="p (in radians/s)", legends=false  )
    pq = plot( time, q_values, xlabel = "Time (in s)", ylabel="q (in radians/s)", legends=false  )
    pr = plot( time, r_values, xlabel = "Time (in s)", ylabel="r (in radians/s)", legends=false  )
    p_rollrates = plot(pp, pq, pr, layout=(3,1))
    if(save_plots)
        savefig(plots_location*"/angular_velocity.png")
    end

    #Control plots
    pde = plot( time, de_values, xlabel = "Time (in s)", ylabel="delta_e (in radians)", legends=false  )
    pda = plot( time, da_values, xlabel = "Time (in s)", ylabel="delta_a (in radians)", legends=false  )
    pdr = plot( time, dr_values, xlabel = "Time (in s)", ylabel="delta_r (in radians)", legends=false  )
    pdt = plot( time, dt_values, xlabel = "Time (in s)", ylabel="delta_t", legends=false )
    # pdt = plot( time, dt_values, xlabel = "Time (in s)", ylabel="delta_t", legends=false, ylim=(0.0, 0.25)  )
    p_control = plot(pde, pda, pdr, pdt, layout=(2,2))
    if(save_plots)
        savefig(plots_location*"/controls.png")
    end

    #Plot Aircraft trajectory
    p_trajectory = plot3d([x_pos_values], [y_pos_values], [-z_pos_values], line=(:blue, 2), xlabel="x (in meters)", ylabel="y (in meters)", zlabel="z (in meters)", legend=false)
    scatter!([x_pos_values[1]],[y_pos_values[1]],[-z_pos_values[1]], color="green")
    scatter!([x_pos_values[end]],[y_pos_values[end]],[-z_pos_values[end]], color="red")
    if(save_plots)
        savefig(plots_location*"/trajectory.png")
    end

    println("All the plots are generated and saved in the " * plots_location * " folder.")
end
