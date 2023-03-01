include("P1.jl")
using DifferentialEquations

function aircraft_dynamics!(du,u,p,t)
    aircraft_state = AircraftState(u...)
    control_inputs = AircraftControl(p[1]...)
    wind_inertial = p[2]
    aircraft_parameters = p[3]
    x_dot = AircraftEOM(aircraft_state,control_inputs,wind_inertial,aircraft_parameters)
    for i in 1:length(u)
        du[i] = x_dot[i]
    end
end

function simulate(initial_state, time_interval, controls, wind_inertial, aircraft_parameters, save_at_value=1.0)
    extra_parameters = [controls, wind_inertial, aircraft_parameters]
    prob = ODEProblem(aircraft_dynamics!,initial_state,time_interval,extra_parameters)
    sol = DifferentialEquations.solve(prob,saveat=save_at_value)
    aircraft_states = []
    for i in 1:length(sol.u)
        push!(aircraft_states,AircraftState(sol.u[i]...))
    end
    return aircraft_states
end

filename = "ttwistor.mat"
aircraft_parameters = AircraftParameters(filename)
case_num = 3
save_plots=true

#Part 1
if(case_num == 1)
    initial_state = [0.0,0.0,-1655.0,0.0,0.0,0.0,18.0,0.0,0.0,0.0,0.0,0.0]
    control_inputs = [0.0,0.0,0.0,0.0]
    wind_inertial = [0.0,0.0,0.0]
end

#Part 2
if(case_num == 2)
    initial_state = [0.0,0.0,-1655.0,0.0,0.0,0.0,18.0,0.0,0.0,0.0,0.0,0.0]
    control_inputs = [0.0,0.0,0.0,0.0]
    wind_inertial = [10.0,10.0,0.0]
end

#Part 3
if(case_num==3)
    initial_state = [0.0, 0.0, -1800.0, 15*pi/180.0, -12*pi/180.0, 270*pi/180.0, 19.0, 3.0, -2.0, pi*0.08/180.0, -0.2*pi/180.0, 0.0]
    control_inputs = [pi/180*5.0,pi/180*2.0,-pi/180*13.0,0.3]
    wind_inertial = [0.0,0.0,0.0]
end


time_interval = (0,150)
time_values = [i for i in 0:time_interval[2]]
trajectory_states = simulate(initial_state, time_interval, control_inputs, wind_inertial, aircraft_parameters)
control_array = [AircraftControl(control_inputs...) for i in 1:length(trajectory_states)]
PlotSimulation(time_values, trajectory_states, control_array , 'd', save_plots)
