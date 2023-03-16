using StaticArrays

struct AircraftState <: FieldVector{12,Float64}
    x::Float64
    y::Float64
    z::Float64
    ϕ::Float64
    θ::Float64
    ψ::Float64
    u::Float64
    v::Float64
    w::Float64
    p::Float64
    q::Float64
    r::Float64
end

struct AircraftControl <: FieldVector{4,Float64}
    δe::Float64
    δa::Float64
    δr::Float64
    δt::Float64
end

struct EulerAngles <: FieldVector{3,Float64}
    ϕ::Float64
    θ::Float64
    ψ::Float64
end

struct WindAngles <: FieldVector{3,Float64}
    Va::Float64
    β::Float64
    α::Float64
end

struct TrimDefinitionSL <: FieldVector{3,Float64}
    Va::Float64
    γ::Float64
    h::Float64
end

struct TrimVariablesSL <: FieldVector{3,Float64}
    α::Float64
    δe::Float64
    δt::Float64
end

struct TrimDefinitionCT <: FieldVector{4,Float64}
    Va::Float64
    γ::Float64
    h::Float64
    R::Float64
end

struct TrimVariablesCT <: FieldVector{7,Float64}
    α::Float64
    δe::Float64
    δt::Float64
    ϕ::Float64
    β::Float64
    δa::Float64
    δr::Float64
end

struct LateralAircraftState <: FieldVector{5,Float64}
    Δβ::Float64
    Δp::Float64
    Δr::Float64
    Δϕ::Float64
    Δψ::Float64
end

struct LateralAircraftControl <: FieldVector{2,Float64}
    Δδe::Float64
    Δδt::Float64
end

struct LongitudinalAircraftState <: FieldVector{5,Float64}
    Δu::Float64
    Δα::Float64
    Δq::Float64
    Δθ::Float64
    Δh::Float64
end

struct LongitudinalAircraftControl <: FieldVector{2,Float64}
    Δδa::Float64
    Δδr::Float64
end

struct AircraftParameters
    g
    S
    b
    c
    AR
    m
    W
    Ix
    Iy
    Iz
    Ixz
    CDmin
    CLmin
    K
    e
    CD0
    K1
    CDpa
    Sprop
    Cprop
    kmotor
    CL0
    Cm0
    CY0
    Cl0
    Cn0
    CLalpha
    Cmalpha
    CLq
    Cmq
    CLalphadot
    Cmalphadot
    CYbeta
    Clbeta
    Cnbeta
    CYp
    Clp
    Cnp
    Clr
    Cnr
    CYr
    CLde
    Cmde
    CYda
    Clda
    Cnda
    CYdr
    Cldr
    Cndr
end

Base.@kwdef mutable struct ControlGains
    g = typemax(Float64)
    max_roll = typemax(Float64)
    max_roll_rate = typemax(Float64)
    max_pitch = typemax(Float64)
    max_da = typemax(Float64)
    max_dr = typemax(Float64)
    max_de = typemax(Float64)
    Kp_roll = typemax(Float64)
    Kd_roll = typemax(Float64)
    Ki_roll = typemax(Float64)
    Kp_course = typemax(Float64)
    Ki_course = typemax(Float64)
    Kp_beta = typemax(Float64)
    Ki_beta = typemax(Float64)
    Kd_beta = typemax(Float64)
    Kp_pitch = typemax(Float64)
    Kd_pitch = typemax(Float64)
    Kp_height = typemax(Float64)
    Ki_height = typemax(Float64)
    Kpitch_DC = typemax(Float64)
    takeoff_height = typemax(Float64)
    takeoff_pitch = typemax(Float64)
    height_hold_limit = typemax(Float64)
    climb_throttle = typemax(Float64)
    Kp_speed_pitch = typemax(Float64)
    Ki_speed_pitch = typemax(Float64)
    Kp_speed_throttle = typemax(Float64)
    Ki_speed_throttle = typemax(Float64)
    u_trim = typemax(Float64)
    Ts = typemax(Float64)
end

Base.@kwdef mutable struct LinearTerms
    a_phi1 = typemax(Float64)
    a_phi2 = typemax(Float64)
    a_beta1 = typemax(Float64)
    a_bet2 = typemax(Float64)
    a_theta1 = typemax(Float64)
    a_theta2 = typemax(Float64)
    a_theta3 = typemax(Float64)
    a_v1 = typemax(Float64)
    a_v2 = typemax(Float64)
end
