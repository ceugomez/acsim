include("../DMDc/identify.jl");
include("doubletSim.jl");
using FileIO,JLD2
genDataFlag = true;
# function to convert array of AircraftState to a matrix
function states_to_matrix(states::Vector{AircraftState})
    n_states = length(states)
    state_matrix = zeros(12, n_states)
    
    for i in 1:n_states
        state_matrix[:, i] = [
            states[i].x, states[i].y, states[i].z,
            states[i].ϕ, states[i].θ, states[i].ψ,
            states[i].u, states[i].v, states[i].w,
            states[i].p, states[i].q, states[i].r
        ]
    end
    
    return state_matrix
end
# Function to convert array of AircraftControl to a matrix
function controls_to_matrix(controls::Vector{AircraftControl})
    n_controls = length(controls)
    # Create a matrix with each column being a control vector
    control_matrix = zeros(4, n_controls)
    
    for i in 1:n_controls
        control_matrix[:, i] = [
            controls[i].δe, controls[i].δa, 
            controls[i].δr, controls[i].δt
        ]
    end
    
    return control_matrix
end
genDataFlag = true;
if genDataFlag
    # gen simulation data
    state_data, control_data = doubletSim();
    state_matrix = states_to_matrix(state_data)
    control_matrix = controls_to_matrix(control_data)
    jldsave("cruise.jld2"; data=state_matrix, control=control_matrix)
else
    # load saved matrices using JLD2 
    saved_data = load("cruise.jld2")
    state_matrix = saved_data["data"]
    control_matrix = saved_data["control"]
end
# I do not care about positions
A, B = DMDcOnData(state_matrix[4:end, :], control_matrix);

println("-------- Linear System Identification ---------------------------------");
println("cego6160")
println("A: ", A);
println("B: ", B);
println("-------------------------------------------------------------------------");