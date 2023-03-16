#=
Adapted from Ben Kraske's conversion of Matlab code to Julia
=#

#Ben Kraske
#benjamin.kraske@colorado.edu
#3/8/2023
#Adapted from
# Eric W. Frew
# ASEN 5519
# RunHW6.m
# Created: 3/2/23
#
# This is a helper that students can use to complete HW 6.

include("../definitions.jl")
include("../utils.jl")
include("../matlab_utils.jl")
include("../aircraft_eom.jl")
include("../trim.jl")
include("controls_functions.jl")
include("P2.jl")


### Aircraft parameter file
filename = "ttwistor.mat"
aircraft_parameters = AircraftParameters(filename)
location = "HW6/plots"
save_plots = true


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # Determine trim state and control inputs
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

V_trim = 18;
h_trim = 1805;
gamma_trim = 0;
trim_definition = TrimDefinitionSL(V_trim, gamma_trim, h_trim);


# # # STUDENTS REPLACE THESE TWO FUNCTIONS WITH YOUR VERSIONS FROM HW3/4

aircraft_state_trim, control_input_trim, results = GetTrimConditions(trim_definition, aircraft_parameters)
trim_variables =  TrimVariablesSL(results.minimizer);
# aircraft_state_trim, control_input_trim = trim_xu_slf(trim_definition, trim_variables);


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # Determine control gains
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # STUDENTS MUST COMPLETE THIS FUNCTION. A SKELETON OF THE FUNCTION IS PROVIDED

control_gain_struct, linear_terms = CalculateControlGainsSimpleSLC_Nondim_Ttwistor(aircraft_parameters, trim_definition, trim_variables)
control_gain_struct.u_trim = control_input_trim;



# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # Set input commands for autopilot.
# # #
# # # Note, STUDENTS may need to change these while tuning the autopilot.
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

h_c         = h_trim;   # commanded altitude (m)
h_dot_c     = 0;   # commanded altitude rate (m)
chi_c       = 40*pi/180;   # commanded course (rad)
chi_dot_ff  = 0;   # commanded course rate (rad)
Va_c        = V_trim;   # commanded airspeed (m/s)



# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # Set aircraft and simulation initial conditions
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
aircraft_state0 = AircraftState(
                        aircraft_state_trim.x,
                        aircraft_state_trim.y,
                        -1655.0,        #<------- CLIMB mode starts when aircraft reaches h = 1675
                        0*pi/180,       #180*pi/180;  #<------- WHEN CONFIDENT INITIALIZE UPSIDE DOWN!!
                        aircraft_state_trim.θ,
                        aircraft_state_trim.ψ,
                        aircraft_state_trim.u,
                        aircraft_state_trim.v,
                        aircraft_state_trim.w,
                        aircraft_state_trim.p,
                        aircraft_state_trim.q,
                        aircraft_state_trim.r
                        )

control_input0 = control_input_trim;
wind_inertial = [0.0,0.0,0.0];


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # Set simulation and control parameters
# # #
# # # Note, the simulation runs on two separate times scales. The variable Ts
# # # specifies the "sample time" of the control system. The control law
# # # calculates a new control input every Ts seconds, and then holds that
# # # control input constant for a short simulation of duration Ts. Then, a
# # # new control input is calculated and a new simulation is run using the
# # # output of the previous iteration as initial condition of the next
# # # iteration. The end result of each short simulation is stored as the
# # # state and control output. Hence, the final result is a simulation with
# # # state and control input at every Ts seconds.
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

Ts = .1;
Tfinal = 500;
control_gain_struct.Ts=Ts;

# # # iterate at control sample time
n_ind = Int(Tfinal/Ts);

aircraft_array = Vector{AircraftState}()
control_array = Vector{AircraftControl}()
time_iter = Float64[]
wind_array = Vector{Vector{Float64}}()
wind_angles = Vector{Vector{Float64}}()
x_command = Vector{Vector{Float64}}()

push!(aircraft_array,aircraft_state0)
push!(control_array,control_input0)
push!(time_iter,0)

pst = [Integrators() for _ in 1:6]
statemachine = StateMachines()

for i in 1:n_ind

    tspan = Ts*[i-1,i];

    push!(wind_array, wind_inertial)

    wind_body = TransformFromInertialToBody(wind_inertial, EulerAngles(aircraft_array[i][4:6]));
    air_rel_vel_body = aircraft_array[i][7:9] - wind_body;
    wind_angle = AirRelativeVelocityVectorToWindAngles(air_rel_vel_body)
    push!(wind_angles,collect(values(wind_angle)))


    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
    # # # Guidance level commands
    # # #
    # # # Note, the format is to allow flexibility for future assignments
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
    control_objectives = zeros(5)
    control_objectives[1] = h_c;
    control_objectives[2] = h_dot_c;
    control_objectives[3] = chi_c;
    control_objectives[4] = chi_dot_ff;
    control_objectives[5] = Va_c;


    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
    # Autopilot
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

    control_slc, x_c_slc = SimpleSLCAutopilot(Ts*(i-1), aircraft_array[i], wind_angles[i],
                                control_objectives, control_gain_struct, pst..., statemachine);

    # long_control_slc = [control_slc[1],0.0,0.0,control_slc[4]]
    # control_slc = long_control_slc
    push!(control_array,control_slc)
    push!(x_command,x_c_slc);
    x_command[i][5] = trim_variables[1];
    #x_est(:,i) = zeros(16,1);

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
    # # # Aircraft dynamics
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
    # [TOUT2,YOUT2] = ode45(@(t,y) AircraftEOM(t,y,control_array[i],wind_inertial, aircraft_parameters),TSPAN,aircraft_array[i],[]);
    # sol = simulate_eom(aircraft_array[i],control_array[i],wind_inertial,aircraft_parameters,Tuple(tspan))
    time_interval = tspan
    extra_params = [control_array[i], wind_inertial, aircraft_parameters]
    trajectory_states = simulate(aircraft_dynamics!, collect(values(aircraft_array[i])), time_interval, extra_params)

    push!(aircraft_array,trajectory_states[end])
    push!(time_iter,tspan[end])
    push!(wind_array,wind_inertial)
    # control_array(:,i+1) = control_array(:,i); #Not needed?
    # x_command(:,i+1) = x_command(:,i); #Not needed?
end



# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # Plotting
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
time_values = [i*Ts for i in 0:n_ind]
PlotSimulation(time_values,aircraft_array,control_array,location,save_plots)


#PlotSimulation(time_iter, aircraft_array,control_array) #Does this need wind and reference inputs
# PlotSimulationWithCommands(time_iter,aircraft_array,control_array, wind_array, x_command, 'b')
# PlotSimulation(time_iter,aircraft_array,control_array, wind_array,'b')
