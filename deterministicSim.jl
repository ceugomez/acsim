# generate time series of deterministic aircraft dynamics in the presence of winds
# cgf cego6160@colorado.edu 1.22.25    
    using Optim,LinearAlgebra
    include("./src/aircraft_eom.jl")
    include("./src/trim.jl")
    include("noisyMeasurements.jl")
    include("plottingUtils.jl")
# use ttwistor because it's there 
    filename = "./src/ttwistor.mat"
    aircraft_parameters = AircraftParameters(filename)
    location = "./temp/"
    save_plots = false
# trim @ va
    trim_definition = TrimDefinitionSL(18.0,0.0,1655)
    state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
    initial_state = collect(values(state))
    control_input = collect(values(control))
# winds 
    wind_inertial = [0.0,0.0,0.0] # [N E D] (probably :P)
# run sim
    println("starting deterministic simulation...");
    time_interval = [0.0,500.0]
    time_values = [i for i in 0:time_interval[2]]
    extra_params = [control_input, wind_inertial, aircraft_parameters]
    trajectory_states = simulate(aircraft_dynamics!, initial_state, time_interval, extra_params);
    control_array = [AircraftControl(control_input...) for i in 1:length(trajectory_states)];
    #PlotSimulation(time_values, trajectory_states, control_array , location, save_plots)
    println("done!");
    Q = diagm([1, deg2rad(2), deg2rad(2)]) # [va α β ]
    result::Vector{WindAngles} =  getNoisyWindAngles(trajectory_states, Q)
# show noisy measurement results 
    plot_wind_angles(result, time_values)
