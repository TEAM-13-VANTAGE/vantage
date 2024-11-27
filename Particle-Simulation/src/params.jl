# Just for some easy manual testing examples. Uses
# lower and upper bounds from the corresponding param CSV 

using CSV

function _get_param_dicts(path::String)
    df, _ = read_manifest(path)

    lower = Dict()
    upper = Dict()

    for (name, start, stop, _) in eachrow(df)
        push!(lower, Symbol(name) => Float64(start))
        push!(upper, Symbol(name) => Float64(stop))
    end

    return lower, upper
end

function copy_with(params::Dict, changes::Dict)::Dict
    Dict(
        params...,
        changes...
    )
end
p_headon_norm_h_low, p_headon_norm_h_high = _get_param_dicts(PATH_PARAMS_HEADON_N_H)
p_headon_norm_v_low, p_headon_norm_v_high = _get_param_dicts(PATH_PARAMS_HEADON_N_V)
p_headon_adverse_h_low, p_headon_adverse_h_high = _get_param_dicts(PATH_PARAMS_HEADON_A_H)
p_converging_norm_h_low, p_converging_norm_h_high = _get_param_dicts(PATH_PARAMS_CONVERGING_N_H)
p_converging_adverse_h_low, p_converging_adverse_h_high = _get_param_dicts(PATH_PARAMS_CONVERGING_A_H)
p_overtaking_norm_h_low, p_overtaking_norm_h_high = _get_param_dicts(PATH_PARAMS_OVERTAKING_N_H)
p_overtaking_adverse_h_low, p_overtaking_adverse_h_high = _get_param_dicts(PATH_PARAMS_OVERTAKING_A_H)
# p_headon_h_low, p_headon_h_high = _get_param_dicts(PATH_PARAMS_HEADON_H)
# p_headon_v_low, p_headon_v_high = _get_param_dicts(PATH_PARAMS_HEADON_V)
# p_converging_h_low, p_converging_h_high = _get_param_dicts(PATH_PARAMS_CONVERGING_H)
# p_converging_v_low, p_converging_v_high = _get_param_dicts(PATH_PARAMS_CONVERGING_V)
# p_overtaking_h_low, p_overtaking_h_high = _get_param_dicts(PATH_PARAMS_OVERTAKING_H)
# p_overtaking_v_low, p_overtaking_v_high = _get_param_dicts(PATH_PARAMS_OVERTAKING_V)
