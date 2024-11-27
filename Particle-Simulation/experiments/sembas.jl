using Distributed
using ProgressMeter
using CSV

@everywhere begin
    using Agents
    include("../src/main.jl")
    include("logical_scenarios/ls-round1.jl")
end

N = nworkers()

println("Starting simulation for sembas")

manifest, units = read_manifest("params/sembas/sembas-manifest.csv")
params = expand_parameters(manifest)
total_runs = ncomb(params)
println("Simulation Count: $total_runs")

param_list = generate_param_combinations(params)#[1:2]
progress = ProgressMeter.Progress(total_runs; enabled=true)
all_data = ProgressMeter.progress_map(
    param_list;
    mapfun=pmap,
    progress=progress
) do comb
    adf, mdf = round1_logical_scenario(; force_right_turn=true, comb...)
    adf = adf[adf.role.==drone, :] # remove heli data
    vars = adf[end, :vars]
    leftjoin(
        mdf[end:end, :],
        DataFrame(Dict(
            :min_dist => "min_dist" ∈ keys(vars) ? vars["min_dist"] : missing,
            :step => mdf[end, :step],
            comb...,
        )),
        on=:step
    )
end
results = vcat(all_data...)
convert_df_units_from_std(results, units)
CSV.write("results/sembas/results-row.csv", results)
next!(progress)
