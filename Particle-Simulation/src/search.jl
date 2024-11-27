"""
This module is responsible for exploring the simulation parameter space.
Parameters are expected to be in a csv file.

"""


using DataFrames
using CSV
# Collection of parameters 

function convert_unit_to_std(external_val::Real, unit)::Real
    return external_val * units_to_std[unit]
end

function convert_unit_from_std(std_value::Real, unit)::Real
    return std_value * units_from_std[unit]
end

function parse_unit(external_val::Real, key)::Real
    units = split(key, "/")
    val = convert_unit_to_std(external_val, units[1])

    for unit in units[2:end]
        val /= convert_unit_to_std(1.0, unit)
    end
    return val
end

function unparse_unit(std_val::Real, key)::Real
    units = split(key, "/")
    val = convert_unit_from_std(std_val, units[1])

    for unit in units[2:end]
        val /= convert_unit_from_std(1.0, unit)
    end
    return val
end

function ncomb(params::Dict)::Int
    return prod(map(
        (v) -> length(v) > 0 ? length(v) : 1,
        values(params)))
    # prod(map(
    #     (v) -> length(v) > 0 ? length(v) : 1,
    #     values(manifest)))
end



function to_radians(deg::Real)::Real
    return deg * pi / 180
end

function to_fps(mph::Real)::Real
    return 1.46667 * mph
end
function to_mph(fps::Real)::Real
    return fps / 1.46667
end

function array_subset(A, B)
    return issubset(Set(A), Set(B))
end

function measure_nsims(; include_adverse=false)
    ncombs = []
    for fn in readdir("params/")[2:end]
        if !include_adverse && contains(fn, "adverse")
            continue
        end

        push!(ncombs, ncomb(expand_parameters(read_manifest("params/$fn")[1])))
    end
    return ncombs
end

function read_parameters(
    path::String
)::Tuple{Dict{Symbol,Vector{<:Real}},Vector{String}}
    """
    Returns both the unique parameter values for each
    dimension as a dictionary of lists. Converts the 
    units to standard units as defined by units_to_std 
    dictionary.  
    """
    df = CSV.File(
        path;
        ignoreemptyrows=true,
        silencewarnings=true,
    ) |> DataFrame

    # Initialize the dictionary to store parameters
    params_dict = Dict{Symbol,Vector{<:Real}}()
    units = Vector{String}()

    # Loop through each row of the dataframe
    for row in eachrow(df)
        param_name = Symbol(row[1])
        unit = row[2]
        push!(units, unit)

        @assert array_subset(split(unit, "/"), supported_units) "Invalid unit $unit found in manifest? Supported units are $supported_units."

        param_values = parse_unit.(
            [v for v in row[3:end] if !ismissing(v)],
            unit
        )

        params_dict[param_name] = param_values
    end

    return params_dict, units
end

function read_manifest(path::String)::Tuple{DataFrame,Dict{Symbol,String}}
    """
    Converts the units to standard units as defined by units_to_std 
    dictionary. Return both the experiment manifest and the input 
    units array (each element corresponds to each row in the mani-
    fest.)
    """
    df = CSV.File(path; ignoreemptyrows=true) |> DataFrame
    input_units = Dict{Symbol,String}()

    # for row in eachrow(df)
    for (i, row) in enumerate(eachrow(df))
        param_name = row[:param_name]
        unit = row[:unit]
        start = row[:start]
        stop = row[:stop]

        @assert array_subset(split(unit, "/"), supported_units) "Invalid unit $unit found in manifest? Supported units are $supported_units."

        df[i, :start] = parse_unit(start, unit)
        df[i, :stop] = parse_unit(stop, unit)

        if row[:steps] > 0
            push!(input_units, Symbol(param_name) => unit)
        end
    end

    select!(df, Not(:unit))

    return df, input_units
end

function expand_parameters(manifest::DataFrame)::Dict{Symbol,Union{Vector{<:Real},Real}}
    lower = manifest.start
    upper = manifest.stop
    n_steps = manifest.steps
    dif = (upper - lower)
    steps = dif ./ n_steps
    # n_steps = [ceil(s != 0 ? d / s : 1) for (d, s) in zip(dif, step)]
    param_values = Dict{Symbol,Union{Vector{<:Real},Real}}()

    # get lists of unique values for each dimension 
    for (name, low, step, n) in zip(manifest.param_name, lower, steps, n_steps)

        if isnan(step)
            values = low
        else
            values = [(low + step * i) for i in 0:n]
        end

        push!(param_values, Symbol(name) => values)
    end

    return param_values
end

function expand_parameters(path::String)::Dict{Symbol,Real}
    return expand_parameters(read_manifest(path))
end

function calc_sim_length(manifest::DataFrame)::Int
    bounds_x = BOUNDS_X
    bounds_y = BOUNDS_Y

    if "bounds_x" ∈ manifest[:, "param_name"]
        bounds_x = manifest[manifest.param_name.=="bounds_x", "stop"][1]
    end

    if "bounds_y" ∈ manifest[:, "param_name"]
        bounds_y = manifest[manifest.param_name.=="bounds_y", "stop"][1]
    end

    max_dist = max(bounds_x, bounds_y)

    min_drone_speed = manifest[manifest.param_name.=="drone_speed", "start"][1]
    min_heli_speed = manifest[manifest.param_name.=="heli_speed", "start"][1]
    speed = max(min_drone_speed, min_heli_speed)

    return ceil(max_dist / speed / dt)
end


function save_results(model_df::DataFrame, dir::String; prefix::Optional{String}=nothing, units::Optional{Dict}=nothing)
    """
    If units is defined, you either need to remove units of params 
    that are constant, or ensure that model_df contains constant 
    params as well.
    """
    title = !isnothing(prefix) ? "$prefix-violations.csv" : "violations.csv"

    if !isnothing(units)
        # convert units back into parameter units
        offset = length(units) - 1
        # println("units: $units, \nSaved params: $(names(model_df)[end-offset:end])")
        for col in names(model_df)[end-offset:end]
            model_df[:, col] = unparse_unit.(model_df[:, col], units[col])
        end
    end

    # println("Writing output to $dir/$title...")
    model_df = model_df[model_df.step.==findmax(model_df.step)[1], :]
    # println("Number of entries: $(nrow(model_df))")

    CSV.write("$dir/$title", model_df)
end

function convert_df_units_from_std(df::DataFrame, units::Dict{Symbol,String})::DataFrame
    for (param, unit) in units
        df[:, param] = unparse_unit.(df[:, param], unit)
    end

    return df
end

function generate_param_combinations(params)
    combs = Iterators.product(values(params)...)
    param_list = []
    names = keys(params)
    for p in combs
        push!(
            param_list,
            Dict(
                k => v
                for (k, v) in zip(names, p)
            )
        )
    end
    return param_list
end