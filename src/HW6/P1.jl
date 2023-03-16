include("../utils.jl")

function FlightPathAnglesFromState(aircraft_state)

    euler_angles = EulerAngles(aircraft_state[4:6])
    wind_angles = AirRelativeVelocityVectorToWindAngles(aircraft_state[7:9])
    Vg = TransformFromBodyToInertial(aircraft_state[7:9], euler_angles)
    #=
    We know γ_a = γ when there is no wind.
    Thus, γ = γ_a = θ - α
    =#
    # γ = aircraft_state.θ - wind_angles.α
    γ = atan(Vg[3], sqrt(Vg[1]^2 + Vg[2]^2))
    χ = atan(Vg[2], Vg[1])

    return (Vg,χ,γ)
end


#=
filename = "ttwistor.mat"
aircraft_parameters = AircraftParameters(filename)

trim_definition = TrimDefinitionCT(18.0,0.0,1655,500.0)
state, control, results = GetTrimConditions(trim_definition, aircraft_parameters)
trim_variables = TrimVariablesCT(results.minimizer)
Vg, χ, γ = FlightPathAnglesFromState(state)
=#
