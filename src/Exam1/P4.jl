include("../aircraft_eom.jl")
include("../trim.jl")
include("../utils.jl")
include("../matlab_utils.jl")
include("P4utils.jl")


#Definitions
filename = "./Exam1/aerosonde.mat"
aircraft_parameters = CustomAircraftParameters(filename)
location = "Exam1/plots"
save_plots = true
trim_Va = 25.0
trim_γ = 0.0
trim_h = 200.0

#Linear Models
lm_filename = "./Exam1/AerosondeLinearModel.mat"
data = matread(lm_filename)
Alat = data["Alat"]
Blat = data["Blat"]
Along = data["Alon"]
Blong = data["Blon"]
trim_state_vector = data["aircraft_state0"]
trim_control_vector = data["control_surfaces0"]


#=
Part 1)
=#
reduced_Along = Along#[1:4,1:4]
long_eigenvals, long_eigenvectors = eigen(reduced_Along)

reduced_Alat = Alat#[1:4,1:4]
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

#=
Part 2)
=#
short_period_vector = long_eigenvectors[:,1]
phugoid_vector = long_eigenvectors[:,3]
println("For Short Period Mode : ")
println("Eigenvalue : ", long_eigenvals[1])
println("Eigenvector : ", short_period_vector)
println("")
println("For Phugoid Mode : ")
println("Eigenvalue : ", long_eigenvals[3])
println("Eigenvector : ", phugoid_vector)
println("")


#=
Part 3)
=#
initial_perturbation_vector = [x.re for x in phugoid_vector]
curr_pitch_per = initial_perturbation_vector[4]
initial_vector = initial_perturbation_vector*(3*pi/curr_pitch_per/180)

TI = [0.0,25000.0]
extra_params = [Along]
lm_state_values = P3ODEsimulate(P3dynamics!, initial_vector, TI, extra_params)
plotP3(lm_state_values, location)


#=
Part 4)
=#
start_state = vec(trim_state_vector)
wind_angles = AirRelativeVelocityVectorToWindAngles(start_state[7:9])
α = wind_angles.α
perturbation_vec = initial_vector
udev = perturbation_vec[1]
wdev = trim_Va*cos(α)*perturbation_vec[2]
qdev = perturbation_vec[3]
θdev = perturbation_vec[4]
hdev = -perturbation_vec[5]

perturbed_state = start_state + [0.0,0.0,hdev,0.0,θdev,0.0,udev,0.0,wdev,0.0,qdev,0.0]
control_input = vec(trim_control_vector)
wind_inertial = [0.0,0.0,0.0]
time_values = [i for i in 0:TI[2]]
extra_params = [control_input, wind_inertial, aircraft_parameters]
trajectory_states = simulate(aircraft_dynamics!, perturbed_state, TI, extra_params)
nl_state_values = calculate_xlon_states(trajectory_states, AircraftState(start_state...))
plotP4(lm_state_values,nl_state_values,location,"p4")

#=
Part 5)
=#
start_state = vec(trim_state_vector)
wind_angles = AirRelativeVelocityVectorToWindAngles(start_state[7:9])
α = wind_angles.α
perturbation_vec = initial_vector*20/3
udev = perturbation_vec[1]
wdev = trim_Va*cos(α)*perturbation_vec[2]
qdev = perturbation_vec[3]
θdev = perturbation_vec[4]
hdev = perturbation_vec[5]

perturbed_state = start_state + [0.0,0.0,hdev,0.0,θdev,0.0,udev,0.0,wdev,0.0,qdev,0.0]
control_input = vec(trim_control_vector)
wind_inertial = [0.0,0.0,0.0]
time_values = [i for i in 0:TI[2]]
extra_params = [control_input, wind_inertial, aircraft_parameters]
trajectory_states = simulate(aircraft_dynamics!, perturbed_state, TI, extra_params)
nl_state_values = calculate_xlon_states(trajectory_states, AircraftState(start_state...))
plotP4(lm_state_values,nl_state_values,location,"p5")
control_array = [AircraftControl(control_input...) for i in 1:length(trajectory_states)]
PlotSimulation(time_values, trajectory_states, control_array , location, save_plots)
