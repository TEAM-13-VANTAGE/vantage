using Distributed
using ProgressMeter
using CSV
using Base.Filesystem

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
# cd Particle-Simulation
# julia --project=. -p 11 /experiments/round1-full.jl
for title in all_titles
    println("Starting simulation for $title")

    manifest, units = read_manifest("./params/round1/$title-manifest.csv")
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

    filename = "./results/round1/$title-results.csv"
    if !isfile(filename)
        touch(filename)
        println("File created: $filename")
    end

    CSV.write(filename, results)
    next!(progress)
end
