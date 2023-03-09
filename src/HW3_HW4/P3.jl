include("P1.jl")
include("P2.jl")

filename = "ttwistor.mat"
aircraft_parameters = AircraftParameters(filename)
location = "HW3_HW4/plots"
save_plots = true
case_num = 3


#Part 1
if(case_num == 1)
    trim_definition = TrimDefinitionSL(18.0,0.0,1655)
    state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
    initial_state = collect(values(state))
    control_input = collect(values(control))
    wind_inertial = [0.0,0.0,0.0]
end

#Part 2
if(case_num == 2)
    trim_definition = TrimDefinitionSL(18.0,0.0,1655)
    state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
    euler_angles = EulerAngles(state.ϕ, state.θ, state.ψ)
    initial_state = collect(values(state))
    wind_inertial = [10.0,10.0,0.0]
    wind_body = TransformFromInertialToBody(wind_inertial,euler_angles)
    initial_state[7] += wind_body[1]
    initial_state[8] += wind_body[2]
    initial_state[9] += wind_body[3]
    control_input = collect(values(control))
end

#Part 3
if(case_num == 3)
    trim_definition = TrimDefinitionSL(18.0,pi/18,1655)
    state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
    initial_state = collect(values(state))
    control_input = collect(values(control))
    wind_inertial = [0.0,0.0,0.0]
end

#Part 4
if(case_num == 4)
    trim_definition = TrimDefinitionCT(20.0,0.0,200,500.0)
    state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
    initial_state = collect(values(state))
    control_input = collect(values(control))
    wind_inertial = [0.0,0.0,0.0]
end

#HW 4 CT conditions
if(case_num == 5)
    trim_definition = TrimDefinitionCT(18.0,0.0,1655,500.0)
    state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
    initial_state = collect(values(state))
    control_input = collect(values(control))
    wind_inertial = [0.0,0.0,0.0]
end

time_interval = [0.0,500.0]
time_values = [i for i in 0:time_interval[2]]
extra_params = [control_input, wind_inertial, aircraft_parameters]
trajectory_states = simulate(aircraft_dynamics!, initial_state, time_interval, extra_params)
control_array = [AircraftControl(control_input...) for i in 1:length(trajectory_states)]
PlotSimulation(time_values, trajectory_states, control_array , location, save_plots)
