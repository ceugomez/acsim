## Ben Kraske, ASEN 5519-2, HW7
## 4/4/2023
using UAS5519
using DifferentialEquations
using LinearAlgebra
using MAT

f_gains = matread("./HW7/ttwistor_gains_feed.mat")
s_gains = matread("./HW7/ttwistor_gains_slc.mat")


function OrbitGuidance(aircraft_state, orbit_speed, orbit_radius, orbit_center, orbit_flag, orbit_gains)

    Vg,χ,γ = FlightPathAnglesFromState(aircraft_state)


function runHW7(;CONTROL_FLAG=:SLC)
    #Import Aircraft
    twst = get_ttwistor_params()

    ##Function for tracking a circle at specified height
    orbit_speed = 18.0
    orbit_radius = 200.0
    orbit_center = [1000.0;1000.0;-1805];
    orbit_flag = 1
    orbit_gains = [.01, .001]

    trim_definition = [300;300;-1805] #####################UPDATE ME
    trim_variables = best_slf_trim(trim_definition,twst)
    aircraft_state_trim, control_input_trim = trim_xu_slf(trim_definition,trim_variables)
    aircraft_state0 = copy(aircraft_state_trim);
    control_input0 = copy(control_input_trim);
    aircraft_state0[3] = -1655

    wind_inertial = [0.0;0.0;0.0];

    Ts = .1;
    Tfinal = 450;


    # # # iterate at control sample time
    n_ind = Int(Tfinal/Ts);

    aircraft_array = Vector{Float64}[]
    control_array = Vector{Float64}[]
    time_iter = Float64[]
    wind_array = Vector{Float64}[]
    wind_angles = Vector{Float64}[]
    x_command = Vector{Float64}[]

    push!(aircraft_array,aircraft_state0)
    push!(control_array,control_input0)
    push!(time_iter,0)


    pst = [Integrators() for _ in 1:6]
    statemachine = StateMachines()

    # control_gain_struct, linear_terms = CalculateControlGainsSimpleSLC_Nondim_Ttwistor(twst, trim_definition, trim_variables)
    # control_gain_struct.u_trim = control_input_trim;
    # @show control_gain_struct

    if (CONTROL_FLAG==:FEED)
        control_gain_struct = controlgainsF_given(f_gains["control_gain_struct"])
    else
        control_gain_struct = controlgains_given(s_gains["control_gain_struct"])
    end
    control_gain_struct.Ts=Ts;

    for i=1:n_ind

        tspan = Ts*[i-1 i];

        push!(wind_array, wind_inertial)

        wind_body = TransformFromInertialToBody(wind_inertial, aircraft_array[i][4:6]);
        air_rel_vel_body = aircraft_array[i][7:9] - wind_body;
        push!(wind_angles,AirRelativeVelocityVectorToWindAngles(air_rel_vel_body))

        control_objectives = OrbitGuidance(aircraft_array[i], orbit_speed,
                                        orbit_radius, orbit_center, orbit_flag, orbit_gains)
        # control_objectives = [100; 0; 45*pi/180; 0; trim_definition[1]]
        # control_objectives = [1805; 0; 45*pi/180; 0; 18.0]
        # @show control_objectives

        if (CONTROL_FLAG==:FEED)
            control_slc, x_c_slc = SLCWithFeedForwardAutopilot(Ts*(i-1), aircraft_array[i], wind_angles[i], control_objectives, control_gain_struct)
        else
            control_slc, x_c_slc = SimpleSLCAutopilot(Ts*(i-1), aircraft_array[i], wind_angles[i],
                                    control_objectives, control_gain_struct, pst..., statemachine)
        end

        push!(control_array,control_slc)
        push!(x_command,x_c_slc);
        x_command[i][5] = trim_variables[1];

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
        # # # Aircraft dynamics
        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
        # @show aircraft_array[i]
        # @show control_array[i]
        sol = simulate_eom(aircraft_array[i],control_array[i],wind_inertial,twst,Tuple(tspan))
        push!(aircraft_array,sol.u[end])
        push!(time_iter,sol.t[end])
        push!(wind_array,wind_inertial)
    end



    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
    # # # Plotting
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
    #Mine:
    PlotSimulationControls(time_iter, aircraft_array,control_array,x_command,wind_array;save=true) #Does this need wind and reference inputs
    return control_gain_struct
end
