using StaticArrays

struct AircraftState <: FieldVector{12,Float64}
    x::Float64
    y::Float64
    z::Float64
    roll::Float64
    pitch::Float64
    yaw::Float64
    u::Float64
    v::Float64
    w::Float64
    p::Float64
    q::Float64
    r::Float64
end

struct AircraftControl <: FieldVector{4,Float64}
    de::Float64
    da::Float64
    dr::Float64
    dt::Float64
end

struct EulerAngles <: FieldVector{3,Float64}
    roll::Float64
    pitch::Float64
    yaw::Float64
end

struct WindAngles <: FieldVector{3,Float64}
    Va::Float64
    beta::Float64
    alpha::Float64
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
