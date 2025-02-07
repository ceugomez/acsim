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
struct mAccel <: FieldVector{3,Float64}
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
        pos = x[i][1:3] #+ rand(MvNormal(zeros(3), Q[1:3,1:3]))                                                          
        vel = TransformFromBodyToInertial(x[i][7:9], EulerAngles(x[i].ϕ,x[i].θ,x[i].ψ)) #+ rand(MvNormal(zeros(3), Q[4:6,4:6]))
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
        Va = norm(VairrelBody) #+ rand(Normal(0.0,Q[1,1]))                                      # airspeed, additive gaussion zero-mean noise (questionable assumption)
        β = asin(v/Va) #+ rand(Normal(0.0,Q[2,2]))                                              # sideslip
        α = atan(w/u) #+ rand(Normal(0.0,Q[3,3]))                                               # angle of attack                                       
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
        rates = x[i][10:12] #+ rand(MvNormal(0.0,Q))
        gyro[i] = mGyro(rates[1],rates[2],rates[3])
    end
    return gyro
end

# gets simulated accelerometer measurements from deterministic aircraft state vector
# Q is 3x3 matrix of noise covariances present in the direct measurements
# Q = diagm([σ_ax^2, σ_ay^2, σ_az^2])
function getAccels(x::Vector{AircraftState},c::Vector{AircraftControl}, param::AircraftParameters,wind::Vector{Float64}, Q::Matrix{Float64})
    accel = Vector{mAccel}(undef,length(x))
    for i in 1:length(x)
        # determine forces for acceleration
        ρ = stdatmo(-x[i].z)
        forces,moments = AircraftForcesAndMoments(x[i],c[i],wind,ρ,param)
        acc = forces/param.m # could use noise implementation
        accel[i] = mAccel(acc[1],acc[2],acc[3]) # struct it! woo
    end
    return accel
end

# gets simulated euler angle measurements - this is techincally a derivative of the aircraft EKF, so approximating it this way is definitely wrong
# Q is 3x3 matrix of noise covariances present in the direct measurements, additive gaussian zero-mean noise (wrong!)
# Q = diagm([σ_ϕ^2, σ_θ^2, σ_ψ^2])
function getEAngs(x::Vector{AircraftState}, Q::Matrix{Float64})
    eang = Vector{EulerAngles}(undef,length(x))
    for i in 1:length(x)
        #eang[i] = EulerAngles(x[i].ϕ + rand(Normal(0.0,Q[1,1])),x[i].θ + rand(Normal(0.0,Q[2,2])),x[i].ψ + rand(Normal(0.0,Q[3,3])))
        eang[i] = EulerAngles(x[i].ϕ,x[i].θ,x[i].ψ)
    end
    return eang
end

# gets winds from simulated measurement using wind triangle relation
# uncertainties are defined in inputs and not considered here
function getWinds(windAngles::Vector{WindAngles}, GPS::Vector{mGPS}, eang::Vector{EulerAngles})
    W = Vector{Vector{Float64}}(undef, length(windAngles))
    for i in 1:length(GPS)                             # get acceleration
        VbW = WindAnglesToAirRelativeVelocityVector(windAngles[i])                    # air-relative velocity in body coordinates
        Vairrel = TransformFromBodyToInertial(VbW, eang[i])                           # transform to inertial frame !!!(THIS NEEDS UNCERTAINTIES)
        W[i] = GPS[i].vel - Vairrel                                                   # get winds from triangle relation
    end
    return W
end

# My way of getting vertical wind data
function getEnergyRateVWind(x::Vector{AircraftState}, c::Vector{AircraftControl}, param::AircraftParameters, gps::Vector{mGPS}, wa::Vector{WindAngles}, wind::Vector{Float64})
    ww = Vector{Float64}(undef,length(gps))
    TE = Vector{Float64}(undef,length(gps))
    accel = getAccels(x,c,param, wind, diagm([0.01, 0.01, 0.01]))
    eang = getEAngs(x, diagm([deg2rad(0.05)^2,deg2rad(0.05)^2,deg2rad(0.005)^2]))
    g = param.g
    
    for i in 1:length(gps)
        # Get states and transforms        
        vel_inertial = gps[i].vel  # [N,E,D]
        accel_inertial = TransformFromBodyToInertial(accel[i], eang[i])
        
        # Get velocities in different frames
        V_body = [x[i].u, x[i].v, x[i].w]  # Body frame velocity
        V_wind_i = wind  # Wind in inertial frame [N,E,D]
        
        # Calculate air-relative velocity in inertial frame
        V_air_i = vel_inertial - V_wind_i  # Air-relative in inertial frame
        Va = wa[i].Va  # True airspeed magnitude
        
        # Get drag force aligned with air-relative velocity
        ρ = stdatmo(-x[i].z)
        aero_force_body, _ = AeroForcesAndMomentsBodyStateWindCoeffs(x[i], c[i], wind, ρ, param)
        aero_force_i = TransformFromBodyToInertial(aero_force_body, eang[i])
        
        # Project drag along air-relative velocity vector
        drag_direction = normalize(V_air_i)
        Fd = -dot(aero_force_i, drag_direction)
        
        # Energy rate terms 
        kinematic_term = norm(vel_inertial) * norm(accel_inertial)
        potential_term = g * -vel_inertial[3]  # Using D component (positive down)
        drag_term = (Fd/param.m) * Va
        
        # Vertical wind calculation (note: wind[3] is positive down)
        raw_estimate = (1/g) * (kinematic_term + potential_term + drag_term)
        ww[i] = -raw_estimate
        
        # Debug output
        println("Step $i:")
        println("  Flight path angle (deg): $(rad2deg(asin(vel_inertial[3]/norm(vel_inertial))))")
        println("  Total energy: $(0.5*norm(vel_inertial)^2 - g*x[i].z)")
        println("  V_inertial: $(vel_inertial)")
        println("  V_air_i: $(V_air_i)")
        println("  Wind: $(wind)")
        println("  Raw kinematic: $(kinematic_term/g)")
        println("  Raw potential: $(potential_term/g)")
        println("  Raw drag: $(drag_term/g)")
        println("  Net estimate: $(ww[i])")
        
    end
    ww[1]=0.0
    return ww
end