function GetLinearizedModel(::Type{LateralAircraftState},ts::AircraftState,tc::AircraftControl,ap::AircraftParameters)

    rho = stdatmo(-ts.z)
    S = ap.S
    b = ap.b
    m = ap.m
    AirSpeedVector = [ts.v, ts,u, ts.w]  #because it has been given that wind speed is zero.
    wind_angles = AirRelativeVelocityVectorToWindAngles(AirSpeedVector)
    Va = wind_angles.Va
    α = wind_angles.alpha
    β = wind_angles.beta
    af_term = Va*cos(β)
    Q = 0.25*rho*Va*S*b/m
    Γ = GetGammaValues(ap)
    lc = GetLateralEOMCoefficients(Γ,ap)

    A11 = 0.25*rho*S*b*ts.v*( (ap.CYp*ts.p) + (ap.CYr*ts.r) )/(m*Va)
    A11 += rho*S*ts.v*(ap.CY0 + ap.CYbeta*β + ap.CYda*tc.da + ap.CYdr*dr)/(m)
    A11 += 0.5*rho*S*ap.CYbeta*sqrt(ts.u^2 + ts.w^2)/m
    A12 = ts.w + Q*ap.CYp
    A12 = A12/af_term
    A13 = -ts.u + Q*ap.CYr
    A13 = A13/af_term
    A14 = ap.g*cos(ts.θ)*cos(ts.ϕ)
    A14 = A14/af_term
    A15 = 0.0

    A21 = 0.25*rho*S*b*b*ts.v*( (lc["Cpp"]*ts.p) + (lc["Cpr"]*ts.r) )/(Va)
    A21 += rho*S*b*ts.v*(lc["Cp0"] + lc["Cpβ"]*β + lc["Cpδa"]*tc.da + lc["Cpδr"]*dr)
    A21 += 0.5*rho*S*b*lc["Cpβ"]*sqrt(ts.u^2 + ts.w^2)
    A21 = A21*af_term
    A22 = (Γ[1]*ts.q) + (Q*b*m*lc["Cpp"])
    A23 = (-Γ[2]*ts.q) + (Q*b*m*lc["Cpr"])
    A24 = 0.0
    A25 = 0.0

    A31 = 0.25*rho*S*b*b*ts.v*( (lc["Crp"]*ts.p) + (lc["Crr"]*ts.r) )/(Va)
    A31 += rho*S*b*ts.v*(lc["Cr0"] + lc["Crβ"]*β + lc["Crδa"]*tc.da + lc["Crδr"]*dr)
    A31 += 0.5*rho*S*b*lc["Crβ"]*sqrt(ts.u^2 + ts.w^2)
    A31 = A31*af_term
    A32 = (Γ[7]*ts.q) + (Q*b*m*lc["Crp"])
    A33 = (-Γ[1]*ts.q) + (Q*b*m*lc["Crr"])
    A34 = 0.0
    A35 = 0.0

    A41 = 0.0
    A42 = 1.0
    A43 = cos(ts.ϕ)*tan(ts.θ)
    A44 = (ts.q*cos(ts.ϕ)*tan(ts.θ)) - (ts.r*sin(ts.ϕ)*tan(ts.θ))
    A45 = 0.0

    A51 = 0.0
    A52 = 0.0
    A53 = cos(ts.ϕ)*sec(ts.θ)
    A54 = (ts.p*cos(ts.ϕ)*sec(ts.θ)) - (ts.r*sin(ts.ϕ)*sec(ts.θ))
    A55 = 0.0

    A = [A11 A12 A13 A14 A15
        A21 A22 A23 A24 A25
        A31 A32 A33 A34 A35
        A41 A42 A43 A44 A45
        A51 A52 A53 A54 A55
        ]

    B11 = (1/b)*2*Va*Q*ap.CYda
    B11 = B11/af_term
    B12 = (1/b)*2*Va*Q*ap.CYdr
    B12 = B12/af_term

    B21 = 2*Va*m*Q*lc["Cpδa"]
    B22 = 2*Va*m*Q*lc["Cpr"]

    B31 = 2*Va*m*Q*lc["Crδa"]
    B32 = 2*Va*m*Q*lc["Crδr"]

    B41 = 0.0
    B42 = 0.0

    B51 = 0.0
    B52 = 0.0

    B = [B11 B12
        B21 B22
        B31 B32
        B41 B42
        B51 B52
        ]

    return A,B
end

function GetLinearizedModel(::Type{LongitudinalAircraftState},trim_state::AircraftState,trim_control::AircraftControl,aircraft_parameters::AircraftParameters)

    rho = stdatmo(-ts.z)
    S = ap.S
    b = ap.b
    m = ap.m
    AirSpeedVector = [ts.v, ts,u, ts.w]  #because it has been given that wind speed is zero.
    wind_angles = AirRelativeVelocityVectorToWindAngles(AirSpeedVector)
    Va = wind_angles.Va
    α = wind_angles.alpha
    β = wind_angles.beta
    af_term = Va*cos(β)
    Q = 0.25*rho*Va*S*b/m
    Γ = GetGammaValues(ap)
    lc = GetLateralEOMCoefficients(Γ,ap)

    A11 = 0.25*rho*S*b*ts.v*( (ap.CYp*ts.p) + (ap.CYr*ts.r) )/(m*Va)
    A11 += rho*S*ts.v*(ap.CY0 + ap.CYbeta*β + ap.CYda*tc.da + ap.CYdr*dr)/(m)
    A11 += 0.5*rho*S*ap.CYbeta*sqrt(ts.u^2 + ts.w^2)/m
    A12 = ts.w + Q*ap.CYp
    A12 = A12/af_term
    A13 = -ts.u + Q*ap.CYr
    A13 = A13/af_term
    A14 = ap.g*cos(ts.θ)*cos(ts.ϕ)
    A14 = A14/af_term
    A15 = 0.0

    A21 = 0.25*rho*S*b*b*ts.v*( (lc["Cpp"]*ts.p) + (lc["Cpr"]*ts.r) )/(Va)
    A21 += rho*S*b*ts.v*(lc["Cp0"] + lc["Cpβ"]*β + lc["Cpδa"]*tc.da + lc["Cpδr"]*dr)
    A21 += 0.5*rho*S*b*lc["Cpβ"]*sqrt(ts.u^2 + ts.w^2)
    A21 = A21*af_term
    A22 = (Γ[1]*ts.q) + (Q*b*m*lc["Cpp"])
    A23 = (-Γ[2]*ts.q) + (Q*b*m*lc["Cpr"])
    A24 = 0.0
    A25 = 0.0

    A31 = 0.25*rho*S*b*b*ts.v*( (lc["Crp"]*ts.p) + (lc["Crr"]*ts.r) )/(Va)
    A31 += rho*S*b*ts.v*(lc["Cr0"] + lc["Crβ"]*β + lc["Crδa"]*tc.da + lc["Crδr"]*dr)
    A31 += 0.5*rho*S*b*lc["Crβ"]*sqrt(ts.u^2 + ts.w^2)
    A31 = A31*af_term
    A32 = (Γ[7]*ts.q) + (Q*b*m*lc["Crp"])
    A33 = (-Γ[1]*ts.q) + (Q*b*m*lc["Crr"])
    A34 = 0.0
    A35 = 0.0

    A41 = 0.0
    A42 = 0.0
    A43 = 1.0
    A44 = 0.0
    A45 = 0.0

    A51 = sin(ts.θ)
    A52 = -cos(ts.θ)
    A53 = 0.0
    A54 = (ts.u*cos(ts.θ)) + (ts.w*sin(ts.θ))
    A55 = 0.0

    A = [A11 A12 A13 A14 A15
        A21 A22 A23 A24 A25
        A31 A32 A33 A34 A35
        A41 A42 A43 A44 A45
        A51 A52 A53 A54 A55
        ]

    B11 = (1/b)*2*Va*Q*ap.CYda
    B11 = B11/af_term
    B12 = (1/b)*2*Va*Q*ap.CYdr
    B12 = B12/af_term

    B21 = 2*Va*m*Q*lc["Cpδa"]
    B22 = 0.0

    B31 = 2*Va*m*Q*lc["Crδa"]
    B32 = 0.0

    B41 = 0.0
    B42 = 0.0

    B51 = 0.0
    B52 = 0.0

    B = [B11 B12
        B21 B22
        B31 B32
        B41 B42
        B51 B52
        ]

    return A,B
end
