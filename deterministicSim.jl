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
    trim_definition = TrimDefinitionSL(21.0,-0.18,1655)
    state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
# set throttle to zero,
    mod_ctrl = AircraftControl(control[1],control[2],control[3],0.0)
    initial_state = collect(values(state))
    control_input = collect(values(control))
# winds 
    wind_inertial = [0.0,0.0,1.0] # [N E D] (probably :P)
# run sim
    println("starting deterministic simulation...");
    time_interval = [0.0,500.0]
    time_values = [i for i in 0:time_interval[2]]
    extra_params = [control_input, wind_inertial, aircraft_parameters]
    trajectory_states = simulate(aircraft_dynamics!, initial_state, time_interval, extra_params);
    control_array = [AircraftControl(control_input...) for i in 1:length(trajectory_states)];
    PlotSimulation(time_values, trajectory_states, control_array , location, save_plots)
# Get measurements and plot
    Q_probe = diagm([0.1^2, deg2rad(2)^2,deg2rad(2)^2])
    Q_gps = diagm([0.1^2,0.1^2,0.1^2,0.1^2,0.1^2,0.1^2])
    Q_eangs = diagm([deg2rad(0.05)^2,deg2rad(0.05)^2,deg2rad(0.005)^2])
    plot_wind_angles(getProbe(trajectory_states, wind_inertial, Q_probe), time_values)
    # perfect wind estimate - wind triangle
    plot_wind_velocities(getWinds(getProbe(trajectory_states, wind_inertial, Q_probe),getGPS(trajectory_states,Q_gps),getEAngs(trajectory_states,Q_eangs)), time_values)
    ww = getEnergyRateVWind(trajectory_states,control_array,aircraft_parameters,getGPS(trajectory_states,Q_gps),getProbe(trajectory_states, wind_inertial, Q_probe), wind_inertial)
    plot_vertical_wind_difference(ww, getWinds(getProbe(trajectory_states, wind_inertial, Q_probe),getGPS(trajectory_states,Q_gps),getEAngs(trajectory_states,Q_eangs)), time_values)
    println("done!");

