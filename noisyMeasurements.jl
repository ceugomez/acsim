# generate noisy measurements (both direct and indirect) of desired quantities from timeseries of aircraft state data
# cgf cego6160@colorado.edu 1.22.25    
# get wind angles

using Distributions
struct mGPS
    pos::Vector{Float64}
    vel::Vector{Float64}
end
struct mGyro
    p::Float64
    q::Float64
    r::Float64
end
struct mAccel
    ax::Float64
    ay::Float64
    az::Float64
end
# get simulated GPS measurements from aircraft state
# - Returns position and inertial velocity
# Q_gps is 6x6 matrix of noise covariances present in the direct measurements
# Q_gps = diagm([σ_x^2, σ_y^2, σ_z^2, σ_u^2, σ_v^2, σ_w^2])
function getGPS(x::Vector{AircraftState},Q::Matrix{Float64})
    gps = Vector{mGPS}(undef,length(x))
    for i in 1:length(x)
        pos = x[i][1:3] + rand(MvNormal(zeros(3), Q[1:3,1:3]))                                                          
        vel = TransformFromBodyToInertial(x[i][7:9], EulerAngles(x[i].ϕ,x[i].θ,x[i].ψ)) + rand(MvNormal(zeros(3), Q[4:6,4:6]))
        gps[i] = mGPS(pos,vel)
    end
    return gps
end

# gets simulated wind angle measurents from deterministic aircraft state vector
# Q_probe is 3x3 vector of noise covariances present in the direct measurements 
# Q_probe = diagm([σ_Va^2, σ_β^2, σ_α^2])
function getProbe(x::Vector{AircraftState}, wind::Vector{Float64}, Q::Matrix{Float64})
    probe = Vector{WindAngles}(undef,length(x))
    for i in 1:length(x)
        Vinertial = TransformFromBodyToInertial(x[i][7:9], EulerAngles(x[i].ϕ,x[i].θ,x[i].ψ))   # inertial in inertial
        Vairrel = Vinertial - wind                                                              # air-relative in inertial
        VairrelBody = TransformFromInertialToBody(Vairrel, EulerAngles(x[i].ϕ,x[i].θ,x[i].ψ))   # air-relative in body
        u, v, w = VairrelBody                                                                   # wind angles
        Va = norm(VairrelBody) + rand(Normal(0.0,Q[1,1]))                                      # airspeed, additive gaussion zero-mean noise (questionable assumption)
        β = asin(v/Va) + rand(Normal(0.0,Q[2,2]))                                              # sideslip
        α = atan(w/u) + rand(Normal(0.0,Q[3,3]))                                               # angle of attack                                       
        probe[i] = WindAngles(Va,β,α)
    end
    return probe
end
# gets simulated gyro measurements from deterministic aircraft state vector
# Q is 3x3 matrix of noise covariances present in the direct measurements
# Q = diagm([σ_p^2, σ_q^2, σ_r^2])
function getGyros(x::Vector{AircraftState}, Q::Matrix{Float64})
    gyro = Vector{mGyro}(undef,length(x))
    for i in 1:length(x)
        rates = x[i][10:12] + rand(MvNormal(0.0,Q))
        gyro[i] = mGyro(rates[1],rates[2],rates[3])
    end
    return gyro
end
function getEAngs(x::Vector{AircraftState}, Q::Matrix{Float64})
    eang = Vector{EulerAngles}(undef,length(x))
    for i in 1:length(x)
        eang[i] = EulerAngles(x[i].ϕ + rand(Normal(0.0,Q[1,1])),x[i].θ + rand(Normal(0.0,Q[2,2])),x[i].ψ + rand(Normal(0.0,Q[3,3])))
    end
    return eang
end


# gets winds from simulated measurement using wind triangle relation
# uncertainties are defined in inputs and not considered here
function getWinds(windAngles::Vector{WindAngles}, GPS::Vector{mGPS}, eang::Vector{EulerAngles})
    W = Vector{Vector{Float64}}(undef, length(windAngles))
    for i in 1:length(GPS)
        VbW = WindAnglesToAirRelativeVelocityVector(windAngles[i])                    # air-relative velocity in body coordinates
        Vairrel = TransformFromBodyToInertial(VbW, eang[i])                           # transform to inertial frame !!!(THIS NEEDS UNCERTAINTIES)
        W[i] = GPS[i].vel - Vairrel                                                   # get winds from triangle relation
    end
    return W
end


function getEnergyRateVWind(x::Vector{AircraftState}, c::Vector{AircraftControl}, param::AircraftParameters, gps::Vector{mGPS}, wa::Vector{WindAngles})
    ww  = Vector{Float64}(undef,length(gps))    # vertical wind estimate
    g = param.g
    for i in 1:length(gps)
        aero_force, aero_moments = AeroForcesAndMomentsBodyStateWindCoeffs(x, c, wind, density, param)
        Fd = aero_force[1]
        ww[i] = (1/g)*(norm(gps[i].vel)*norm(accel)+g*gps[i].vel[3] + Fd/m*Va) 
    end
    return ww
end