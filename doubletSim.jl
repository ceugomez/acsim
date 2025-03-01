using Optim,LinearAlgebra
include("./src/aircraft_eom.jl")
include("./src/trim.jl")
include("noisyMeasurements.jl")
include("plottingUtils.jl")
function doubletSim()
# generate time series of deterministic aircraft dynamics, return state and control time histories
# cgf cego6160 
#use ttwistor because it's there 
    filename = "./src/ttwistor.mat"
    aircraft_parameters = AircraftParameters(filename)
    location = "./temp/"
    save_plots = false
# winds 
    wind_inertial = [0.0,0.0,0.0]
#trim @ va
    trim_definition = TrimDefinitionSL(18.0+wind_inertial[1],0.0,1655)
    trim_definition = TrimDefinitionSL(21.0,-0.18,1655)
    state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
# run sim
    println("starting deterministic simulation...");
    time_interval = [0.0,500.0]
    time_values = [i for i in 0:time_interval[2]]
    extra_params = [control_input, wind_inertial, aircraft_parameters]
    trajectory_states = simulate(aircraft_dynamics!, initial_state, time_interval, extra_params);
    control_array = [AircraftControl(control_input...) for i in 1:length(trajectory_states)];
    return {trajectory_states, control_array}
    # could get measurements here, but will leave this for later
end