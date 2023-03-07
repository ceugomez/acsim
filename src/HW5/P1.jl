include("../linear_model.jl")
include("../trim.jl")

trim_definition = TrimDefinitionSL(18.0,0.0,1800.0)
state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
wind_inertial = [0.0,0.0,0.0]
Alat,Blat = GetLinearizedModel(LateralAircraftState,state,control,aircraft_parameters)
# Alat,Blat = @enter GetLinearizedModel(LateralAircraftState,state,control,aircraft_parameters)
Along,Blong = GetLinearizedModel(LongitudinalAircraftState,state,control,aircraft_parameters)
# Alat,Blat = @enter GetLinearizedModel(LateralAircraftState,state,control,aircraft_parameters)

println("A_lat")
display(Alat)
println("B_lat")
display(Blat)
println("A_long")
display(Along)
println("B_long")
display(Blong)


#=
Problem 1 - Part 2)
=#

reduced_Along = Along[1:4,1:4]
long_eigenvals, long_eigenvectors = eigen(reduced_Along)

reduced_Alat = Alat[1:4,1:4]
lat_eigenvals, lat_eigenvectors = eigen(reduced_Alat)

#Short Period Mode
short_natural_frequency = get_natural_frequency(long_eigenvals[1])
short_damping_ratio = get_damping_ratio(long_eigenvals[1])
#Phugoid Mode
phugoid_natural_frequency = get_natural_frequency(long_eigenvals[3])
phugoid_damping_ratio = get_damping_ratio(long_eigenvals[3])
#Roll Mode
roll_natural_frequency = get_natural_frequency(lat_eigenvals[1])
roll_damping_ratio = get_damping_ratio(lat_eigenvals[1])
#Dutch Roll Mode
dutch_roll_natural_frequency = get_natural_frequency(lat_eigenvals[2])
dutch_roll_damping_ratio = get_damping_ratio(lat_eigenvals[2])
#Spiral Mode
spiral_natural_frequency = get_natural_frequency(lat_eigenvals[4])
spiral_damping_ratio = get_damping_ratio(lat_eigenvals[4])


println("")
println("For Short Period Mode : ")
println("Damping Ratio : ", short_damping_ratio)
println("Natural Frequency : ", short_natural_frequency)
println("")
println("For Phugoid Mode : ")
println("Damping Ratio : ", phugoid_damping_ratio)
println("Natural Frequency : ", phugoid_natural_frequency)
println("")
println("For Dutch Roll Mode : ")
println("Damping Ratio : ", dutch_roll_damping_ratio)
println("Natural Frequency : ", dutch_roll_natural_frequency)


#=
Problem 1 - Part 2)
=#

println("")
println("For Roll Mode : ")
println("Damping Ratio : ", roll_damping_ratio)
println("Natural Frequency : ", roll_natural_frequency)
println("Time Constant : ", 1/roll_natural_frequency)
println("The roll mode is stable.")
println("")
println("For Spiral Mode : ")
println("Damping Ratio : ", spiral_damping_ratio)
println("Natural Frequency : ", spiral_natural_frequency)
println("Time Constant : ", 1/spiral_natural_frequency)
println("The spiral mode is unstable.")
