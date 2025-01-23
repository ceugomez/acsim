# generate noisy measurements (both direct and indirect) of desired quantities from timeseries of aircraft state data
# cgf cego6160@colorado.edu 1.22.25    
# get wind angles
"""
    getNoisyWindAngles(stateHist::Array{AircraftState})
    calculates α,β, & V_a from input aircraft state time history and a covariance matrix of known sensor uncertainties. assumes AWGN.
"""
# we assume here that vehicle is equipped with α/β/Va direct sensors, i.e. vanes and a pitot-static probe, with some characterized uncertainties Q
function getNoisyWindAngles(x::Array{AircraftState}, Q::Matrix{Float64})
    vec = Vector{WindAngles}()
    for i in 1:length(x)
        va = sqrt(x[i].u^2+x[i].v^2+x[i].w^2) + randn()*Q[1,1]*0.33;   # let's call it 1 stddev for now, σ(randn()) = 1 by fn def
        α = atan(x[i].w, x[i].u) + randn()*Q[2,2]*0.33                 # obviously assuming AWGN
        β = asin(x[i].v/va) + randn()*Q[3,3]*0.33                      # ditto
        push!(vec, WindAngles(va,β,α));
    end
    return vec;
end
# Get overall wind estimates by wind triangle method
"""
    getNoisyWinds(stateHist::Array{AircraftState})
    calculates W_E from input aircraft state time history and a covariance matrix of known sensor uncertainties. assumes AWGN.
"""
# direct measurements here are: 
# - 
function getNoisyWinds(x::Array{AircraftState}, t::Vector{Float64},Q::Matrix{Float64})
    W = Vector{Vector{Float64}}() # inertial wind velocities
    for i in 1:(length(x)-1)
        windAngles = getNoisyWindAngles(x, Q)   
        VbW = WindAnglesToAirRelativeVelocityVector(windAngles[1]);                                # air relative velocity in body coords
        Vairrel = TransformFromBodyToInertial(VbW,EulerAngles(x[i].ϕ,x[i].θ, x[i].ψ));             # air relative velocity in inertial coords
        Vinertial = ((x[i+1][1:3] - x[i][1:3]) ./ (t[i+1] - t[i])) #+ randn(3)*0.01                # lets just interpolate velocity based on interval between successive points
        We = Vinertial-Vairrel
        push!(W, We)
    end
    push!(W, W[end]);
    return W
end

 