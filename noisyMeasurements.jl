# generate noisy measurements (both direct and indirect) of desired quantities from timeseries of aircraft state data
# cgf cego6160@colorado.edu 1.22.25    
# get wind angles
"""
    getNoisyWindAngles(stateHist::Array{AircraftState})
    calculates α,β, & V_a from input aircraft state time history and a covariance matrix of known sensor uncertainties. assumes AWGN.
"""
# we assume here that vehicle is equipped with α/β/Va direct sensors, i.e. vanes.
function getNoisyWindAngles(x::Array{AircraftState}, Q::Matrix{Float64})
    vec = Vector{WindAngles}()
    for i in 1:length(x)
        va = sqrt(x[i].u^2+x[i].v^2+x[i].w^2) + randn()*Q[1,1]*0.33; 
        α = atan(x[i].w, x[i].u) + randn()*Q[2,2]*0.33
        β = asin(x[i].v/va) + randn()*Q[3,3]*0.33
        push!(vec, WindAngles(va,β,α));
    end
    return vec;
end
function getNoisyHorizontalWinds()
    Vi = Vector{Vector{Float64}}() # inertial velocities, direct 
    for i in 1:(length(x)-1)
        Rbe = V
        Vn = 
    end


end

 