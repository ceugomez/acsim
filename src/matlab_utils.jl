using MAT

function AircraftParameters(filename)
    file_location = joinpath(pwd(),filename)
    matlab_data = matread(file_location)
    matlab_params = matlab_data["aircraft_parameters"]
    julia_params = [ matlab_params[string(k)] for k in fieldnames(AircraftParameters)]
    return AircraftParameters(julia_params...)
end

function stdatmo(h)
    Toffset = 0
    if h < 11000
        TonTi = 1 - 2.255769564462953e-005 * h
        press = 101325 * TonTi .^ (5.255879812716677)
        temp = TonTi * 288.15 + Toffset
        rho = press ./ temp / 287.05287
        return rho
    end
end
