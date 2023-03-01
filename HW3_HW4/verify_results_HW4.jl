include("P1_P2.jl")

filename = "ttwistor.mat"
aircraft_parameters = AircraftParameters(filename)

#Part 1 - Verifying HW2
initial_state = [100, 200, 1000, 0.05, −0.02, 2.0, 15, −2, 3, .01, .02, .03]
s = AircraftState(initial_state...)
control_inputs =  [−.4, .01, .02, .3]
c = AircraftControl(control_inputs...)
wind_inertial = wind_inertial = [−1, −2, −3]
rho = 1.1

af, am = AeroForcesAndMomentsBodyStateWindCoeffs(s,c, wind_inertial, rho, aircraft_parameters)
println("Aero Forces : ", af)
println("Aero Moments : ", am)
tf, tm = AircraftForcesAndMoments(s,c, wind_inertial, rho, aircraft_parameters)
println("Aircraft Forces : ", af)
println("Aircraft Moments : ", am)
xd = AircraftEOM(s, c, wind_inertial,aircraft_parameters)
println("xdot : ", xd)

#Part 2 - Verifying HW3
trim_definition = TrimDefinitionSL(18.0,0.0,1655)
state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
println("SLUF: Trim Variables: ", results.minimizer)
trim_definition = TrimDefinitionCT(18.0,0.0,1655,500.0)
state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
println("Coordinated Turn: Trim Variables: ", results.minimizer)
