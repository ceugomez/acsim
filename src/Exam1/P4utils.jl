function CustomAircraftParameters(filename)
    file_location = joinpath(pwd(),filename)
    matlab_data = matread(file_location)
    matlab_params = matlab_data["aircraft_parameters"]
    julia_params = []
    for k in fieldnames(AircraftParameters)
        if(string(k)=="K1")
            push!(julia_params, 0.0)
        else
            push!(julia_params, matlab_params[string(k)])
        end
    end
    return AircraftParameters(julia_params...)
end

function P3dynamics!(du,u,p,t)
    Amatrix = p[1]
    x_dot = Amatrix*u
    for i in 1:length(u)
        du[i] = x_dot[i]
    end
end

function P3ODEsimulate(dynamics, initial_state, time_interval, extra_parameters, save_at_value=1.0)
    prob = ODEProblem(dynamics,initial_state,time_interval,extra_parameters)
    sol = DifferentialEquations.solve(prob,saveat=save_at_value)
    lm_ubar, lm_αbar, lm_qbar, lm_θbar, lm_hbar = [],[],[],[],[]
    for i in 1:length(sol.u)
        push!(lm_ubar,sol.u[i][1])
        push!(lm_αbar,sol.u[i][2])
        push!(lm_qbar,sol.u[i][3])
        push!(lm_θbar,sol.u[i][4])
        push!(lm_hbar,sol.u[i][5])
    end
    return [lm_ubar, lm_αbar, lm_qbar, lm_θbar, lm_hbar]
end

function plotP3(obs,location,save_plots=true)
    plots_location = joinpath(pwd(),location)
    lm_ubar, lm_αbar, lm_qbar, lm_θbar, lm_hbar = obs[1], obs[2], obs[3], obs[4], obs[5]
    t = 1:length(lm_ubar)

    p1 = plot(t, lm_ubar, xlabel="Time(s)", ylabel="ubar (m/s)", label="Linear")
    if(save_plots)
        savefig(plots_location*"/p3_1.png")
    end

    p2 = plot(t, lm_αbar, xlabel="Time(s)", ylabel="αbar (radians)", label="Linear")
    if(save_plots)
        savefig(plots_location*"/p3_2.png")
    end

    p3 = plot(t, lm_qbar, xlabel="Time(s)", ylabel="qbar (radians/s)", label="Linear")
    if(save_plots)
        savefig(plots_location*"/p3_3.png")
    end

    p4 = plot(t, lm_θbar, xlabel="Time(s)", ylabel="θbar (radians)", label="Linear")
    if(save_plots)
        savefig(plots_location*"/p3_4.png")
    end

    p5 = plot(t, lm_hbar, xlabel="Time(s)", ylabel="hbar (m)", label="Linear")
    if(save_plots)
        savefig(plots_location*"/p3_5.png")
    end
end

function calculate_xlon_states(aircraft_states, trim_state)
    ubar, αbar, qbar, θbar, hbar = [],[],[],[],[]
    utrim = trim_state.u
    qtrim = trim_state.q
    θtrim = trim_state.θ
    htrim = trim_state.z
    trim_wind_angles = AirRelativeVelocityVectorToWindAngles(trim_state[7:9])
    αtrim = trim_wind_angles.α
    for i in 1:length(aircraft_states)
        udev = aircraft_states[i].u - utrim
        push!(ubar,udev)
        wind_angles = AirRelativeVelocityVectorToWindAngles(aircraft_states[i][7:9])
        αdev = wind_angles.α - αtrim
        push!(αbar, αdev)
        qdev = aircraft_states[i].q - qtrim
        push!(qbar,qdev)
        θdev = aircraft_states[i].θ - θtrim
        θdev = rem(θdev, 2*pi)
        # θdev =
        push!(θbar,θdev)
        hdev = aircraft_states[i].z - htrim
        push!(hbar,hdev)
    end
    return ubar, αbar, qbar, θbar, hbar
end

function plotP4(lm_obs,nl_obs,location,problem_num,save_plots = true)
    plots_location = joinpath(pwd(),location)
    lm_ubar, lm_αbar, lm_qbar, lm_θbar, lm_hbar = lm_obs[1], lm_obs[2], lm_obs[3], lm_obs[4], lm_obs[5]
    nl_ubar, nl_αbar, nl_qbar, nl_θbar, nl_hbar = nl_obs[1], nl_obs[2], nl_obs[3], nl_obs[4], nl_obs[5]
    t = 1:length(lm_ubar)

    p1 = plot(t, lm_ubar, xlabel="Time(s)", ylabel="ubar (m/s)", label="Linear")
    plot!(t,nl_ubar, label="Non Linear")
    if(save_plots)
        savefig(plots_location*"/"*string(problem_num)*"_1.png")
    end

    p2 = plot(t, lm_αbar, xlabel="Time(s)", ylabel="αbar (radians)", label="Linear")
    plot!(t,nl_αbar,label="Non Linear")
    if(save_plots)
        savefig(plots_location*"/"*string(problem_num)*"_2.png")
    end

    p3 = plot(t, lm_qbar, xlabel="Time(s)", ylabel="qbar (radians/s)", label="Linear")
    plot!(t,nl_qbar,label="Non Linear")
    if(save_plots)
        savefig(plots_location*"/"*string(problem_num)*"_3.png")
    end

    p4 = plot(t, lm_θbar, xlabel="Time(s)", ylabel="θbar (radians)", label="Linear")
    plot!(t,nl_θbar,label="Non Linear")
    if(save_plots)
        savefig(plots_location*"/"*string(problem_num)*"_4.png")
    end

    p5 = plot(t, lm_hbar, xlabel="Time(s)", ylabel="hbar (m)", label="Linear")
    plot!(t,nl_hbar,label="Non Linear")
    if(save_plots)
        savefig(plots_location*"/"*string(problem_num)*"_5.png")
    end
end
