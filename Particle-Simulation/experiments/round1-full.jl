using Distributed
using ProgressMeter
using CSV

@everywhere begin
    using Agents
    include("../src/main.jl")
    include("logical_scenarios/ls-round1.jl")
    # include("../test.jl")
end

N = nworkers()

all_titles = [
    "$typ-$maneuver"
    for typ in [
        "converging-norm",
        "converging-adverse",
        "headon",
        "overtaking",
    ]
    for maneuver in [
        "row",
        "vertical",
        "horizontal"
    ]
]
# julia --project=. -p 11 Particle-Simulation/experiments/round1-full.jl
for title in all_titles
    println("Starting simulation for $title")

    manifest, units = read_manifest("/Users/jpamplona/Coding Projects/UAV-TestingSim/Particle-Simulation/params/round1/$title-manifest.csv")
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
        adf, mdf = round1_logical_scenario(; comb...)
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
    CSV.write("results/round1/$title-results.csv", results)
    next!(progress)
end
