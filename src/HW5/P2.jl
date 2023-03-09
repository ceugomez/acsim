include("../aircraft_eom.jl")
include("../trim.jl")


function PulseControl(t::Float64,true_control::Union{AircraftControl,Array{Float64,1}},pulse_control::Array{Float64,1},pulse_time::Array{Float64,1})

    if(t==0.0)
        return true_control
    end
    if( length(pulse_time) == 1 )
        if(t<=pulse_time[1])
            control = true_control + pulse_control
        else
            control = true_control
        end
    elseif( length(pulse_time) == 2)
        if(t<=pulse_time[1])
            control = true_control + pulse_control
        elseif(t>pulse_time[1] && t<=pulse_time[2])
            control = true_control - pulse_control
        else
            control = true_control
        end
    end

    return control
end

function AircraftEOMPulsed(t::Float64,aircraft_state::AircraftState,controls::AircraftControl,control_function::Function,
            pulse_control::Array{Float64,1},pulse_time::Array{Float64,1},wind_inertial::Array{Float64,1},aircraft_parameters::AircraftParameters)

    aircraft_surfaces = control_function(t,controls,pulse_control,pulse_time)
    return AircraftEOM(t,aircraft_state,aircraft_surfaces,wind_inertial,aircraft_parameters)
end

function aircraft_dynamics_pulsed!(du,u,p,t)
    aircraft_state = AircraftState(u...)
    control_function = p[1]
    control_inputs = AircraftControl(p[2]...)
    pulse_control = p[3]
    pulse_time = p[4]
    wind_inertial = p[5]
    aircraft_parameters = p[6]
    x_dot = AircraftEOMPulsed(t,aircraft_state,control_inputs,control_function,pulse_control,pulse_time,wind_inertial,aircraft_parameters)
    for i in 1:length(u)
        du[i] = x_dot[i]
    end
end

filename = "ttwistor.mat"
aircraft_parameters = AircraftParameters(filename)
location = "HW5/plots"
save_plots = true
case_num = 1


trim_definition = TrimDefinitionSL(18.0,0.0,1800.0)
state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
initial_state = collect(values(state))
control_input = collect(values(control))
wind_inertial = [0.0,0.0,0.0]

if(case_num == 1)
    pulse_control = [pi/10,0.0,0.0,0.0]
    pulse_time = [1.0]
end

if(case_num == 2)
    pulse_control = [0.0,pi/10,0.0,0.0]
    pulse_time = [1.0,2.0]
end

if(case_num == 3)
    pulse_control = [0.0,0.0,pi/10,0.0]
    pulse_time = [1.0,2.0]
end


time_interval = [0.0,200.0]
extra_parameters = [PulseControl,control_input,pulse_control,pulse_time,wind_inertial,aircraft_parameters]
trajectory_states = simulate(aircraft_dynamics_pulsed!,initial_state,time_interval,extra_parameters,1.0)
time_values = [i for i in 0:length(trajectory_states)-1]
control_array = [AircraftControl(PulseControl(1.0*(i-1),control_input,pulse_control,pulse_time)...) for i in 1:length(trajectory_states)]
PlotSimulation(time_values, trajectory_states, control_array , location, save_plots)
