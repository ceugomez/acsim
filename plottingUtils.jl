# plotting utilities for wind estimation simulation
# cgf cego6160@colorado.edu 1.22.25

function plot_wind_angles(result::Vector{WindAngles}, time::Vector{Float64})
    if length(result) != length(time)
        throw(ArgumentError("The length of `result` and `time` must be the same."))
    end

    # Extract the individual components of WindAngles
    Va_values = [r.Va for r in result]
    β_values = rad2deg.([r.β for r in result])  # Convert β from radians to degrees
    α_values = rad2deg.([r.α for r in result])  # Convert α from radians to degrees

    # Create a 2-row layout for subplots
    p = plot(
        layout = @layout([a; b]),  # Two vertically stacked subplots
        size = (800, 600)          # Specify window size
    )

    # Plot Va (airspeed) in the first subplot
    plot!(time, Va_values, label="Va (Airspeed)", color=:blue, lw=2, xlabel="Time (s)", ylabel="Va (m/s)", subplot=1)

    # Plot β and α (angles in degrees) in the second subplot
    plot!(time, β_values, label="β (Sideslip, °)", color=:red, lw=2, xlabel="Time (s)", ylabel="Angle (°)", subplot=2)
    plot!(time, α_values, label="α (Angle of Attack, °)", color=:green, lw=2, subplot=2)
    display(p)
    wait_for_key("press any key to continue")
end

# wait for key fn
wait_for_key(prompt) = (print(stdout, prompt); read(stdin, 1); nothing)
