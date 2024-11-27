# This module focuses on opening a communication stream between the simulation and
# external programs, such as a python script. These programs can be used to control
# which parameters to execute next.

include("main.jl")

using Sockets

config = get_config()
port = config["port"]

function parse_message(s)::Tuple{Dict{Symbol,Real},Vector{String},Vector{Symbol},Vector{Symbol}}
    "Returns dictionary of params, n-steps, and a-data and m-data to collect. "
    j = JSON.parse(s)
    params = Dict()
    mdata = []
    adata = []
    units = []

    # Convert keys from strings to symbols
    for (parameter, (value, unit)) in zip(keys(j["params"]), values(j["params"]))
        value = parse_unit(value, unit)
        push!(params, Symbol(parameter) => value)
        push!(units, unit)
    end

    if "adata" ∈ keys(j)
        for var in j["adata"]
            push!(adata, Symbol(var))
        end
    end

    if "mdata" ∈ keys(j)
        for var in j["mdata"]
            push!(mdata, Symbol(var))
        end
    end

    return params, units, adata, mdata
end

function connect_as_client(logical_scenario::Function, parameters::Vector{Symbol}, low::Vector{<:Real}, high::Vector{<:Real}; client::Optional{TCPSocket}=nothing, constants::Dict{Symbol,<:Real}=Dict{Symbol,<:Real}())
    socket = isnothing(client) ? connect(port) : client

    println("Establishing configuration...")
    sim_config = JSON.json(Dict(:num_params => length(parameters)))
    print(socket, sim_config * "\n")
    ack = readline(socket)

    if ack != "OK"
        close(socket)
        println("Failed due to invalid number of parameters! Expected: $ack params.")
        return
    end

    println("Connection established.")

    running = true

    while running
        s = readline(socket)
        running = s != "end"

        if running
            values = JSON.parse(s)["p"]
            # denormalize
            values = values .* (high - low) + low

            params = Dict(
                key => val for (key, val) in zip(parameters, values)
            )

            adf, mdf = logical_scenario(;
                adata=[:scenario],
                mdata=[:contactLevel],
                params...,
                constants...)
            # adf = adf[adf.role.==drone, :]
            is_violation = mdf[end, :contactLevel] != none
            data = Dict(
                :cls => is_violation
            )

            # Return classification
            results = JSON.json(data)
            print(socket, results * "\n")
        end
    end

    close(socket)
end

function open_server(logical_scenario::Function; server=nothing)
    # This only transmits the last entry of the adata and mdata
    println("Waiting for connection...")
    server = isnothing(server) ? listen(port) : server
    socket = accept(server)
    println("Connection established!")

    serving = true

    while serving
        s = readline(socket)
        serving = s != "end"

        if serving
            params, units, adata, mdata = parse_message(s)
            adf, mdf = logical_scenario(;
                adata=adata,
                mdata=mdata,
                params...
            )
            adf = adf[adf.role.==drone, :]
            adf = adf[end:end, :]
            mdf = mdf[end:end, :]

            df = innerjoin(adf, mdf, on=:step)
            data = Dict([
                name => v for (name, v) in zip(names(df), df[1, :])
            ])
            results = JSON.json(data)

            print(socket, results * "\n")
        end
    end
end

function open_client_safe(logical_scenario::Function, parameters::Vector{Symbol}, low::Vector{<:Real}, high::Vector{<:Real}; constants::Dict{Symbol,<:Real}=Dict{Symbol,<:Real}())
    client = connect(port)
    try
        connect_as_client(logical_scenario, parameters, low, high; client=client, constants=constants)
    finally
        # println("Closing client...")
        close(client)
    end
end

function open_server_safe(logical_scenario::Function)
    server = listen(port)
    try
        open_server(logical_scenario; server=server)
    finally
        # println("Closing server...")
        close(server)
    end
end