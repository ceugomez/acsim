include("../definitions.jl")
include("../utils.jl")
include("../matlab_utils.jl")
include("../aircraft_eom.jl")

using LinearAlgebra
using Optim

#=
*******************************************************************************************************
Functions for Part 1
=#

function GetStateAndControl(trim_definition::TrimDefinitionSL,trim_variables::TrimVariablesSL)

    #Parameters that don't matter
    x = y = ψ = 0.0
    #Parameters that are zero
    ϕ = v = p = q = r = 0.0
    #=
    Since there is no wind, γ = γ_a.
    Thus, pitch θ = flight path angle γ + angle of attack α
    Also, this is constant altitude flight. So, flight path angle, γ=0.
    =#
    θ = trim_definition.γ + trim_variables.α
    z = -trim_definition.h
    #For straight, wings-level, there is no side slip β
    β = 0.0
    wind_angles = WindAngles(trim_definition.Va,β,trim_variables.α)
    Va_vector = WindAnglesToAirRelativeVelocityVector(wind_angles)
    u = Va_vector[1]
    w = Va_vector[3]
    state = AircraftState(x,y,z,ϕ,θ,ψ,u,v,w,p,q,r)

    de = trim_variables.δe
    da = dr = 0.0
    dt = trim_variables.δt
    control = AircraftControl(de,da,dr,dt)

    return state,control
end

function GetCost(trim_definition::TrimDefinitionSL,trim_variables::TrimVariablesSL,aircraft_parameters::AircraftParameters)

    state,control = GetStateAndControl(trim_definition,trim_variables)
    wind_inertial = [0.0,0.0,0.0]
    rho = stdatmo(-state.z)
    force, moment = AircraftForcesAndMoments(state, control, wind_inertial, rho, aircraft_parameters)
    cost = norm(force,2)^2 + norm(moment,2)^2
    return cost
end

function OptimizerCostFunction(params::Vector{Float64},trim_definition::TrimDefinitionSL,aircraft_parameters::AircraftParameters)
    trim_variables = TrimVariablesSL(params...)
    cost = GetCost(trim_definition,trim_variables,aircraft_parameters)
    return cost
end

function GetTrimConditions(trim_definition::TrimDefinitionSL,aircraft_parameters::AircraftParameters)
    lower = [-pi/4,-pi/4,0.0]
    upper = [pi/4,pi/4,1.0]
    initial_tv = [0.5, 0.5, 0.5]
    results = optimize(x->OptimizerCostFunction(x,trim_definition,aircraft_parameters), lower, upper, initial_tv)
    trim_variables_list = results.minimizer
    trim_variables = TrimVariablesSL(trim_variables_list...)
    state, control = GetStateAndControl(trim_definition, trim_variables)
    return state, control, results
end
