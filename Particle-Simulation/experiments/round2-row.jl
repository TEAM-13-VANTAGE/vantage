using Distributed
using CSV
using ProgressMeter

rect_w, rect_l, rect_h = 4600.0, 3000.0, 400.0
rect_rac_center = [
    20000.0 - 2000.0 - rect_w / 2,
    (2000.0 + rect_l / 2) + 2000.0,
    200.0
]
rect_rac_corner = corner_from_center(rect_rac_center, [rect_w, rect_l, rect_h])

narrow_w, narrow_l, narrow_h = 5200.0, 1700.0, 400.0
narrow_rac_center = [
    20000.0 - 2000.0 - narrow_w / 2,
    (2000.0 + narrow_l / 2) + 2000.0,
    200.0
]
narrow_rac_corner = corner_from_center(narrow_rac_center, [narrow_w, narrow_l, narrow_h])

narrow_rac = RAC_by_dimensions(
    narrow_rac_corner, [narrow_w, narrow_l, narrow_h],
    2000.0
)

@everywhere begin
    include("../src/main.jl")
    include("logical_scenarios/ls-round2-row.jl")

    rect_rac = RAC_by_dimensions(
        rect_rac_corner, [rect_w, rect_l, rect_h],
        2000.0
    )

    narrow_rac = RAC_by_dimensions(
        narrow_rac_corner, [narrow_w, narrow_l, narrow_h],
        2000.0
    )

    manifest, units = read_manifest("params/round2/round2-horizontal-manifest.csv")
    params = expand_parameters(manifest)
end

N = nworkers()
param_list = generate_param_combinations(params)#[1:2]
total_runs = ncomb(params)

rac_types = ["narrow", "rectangle"]
for rac_type in rac_types
    rac = rac_type == "narrow" ? narrow_rac : rectangle_rac

    progress = ProgressMeter.Progress(total_runs; enabled=true)
    all_data = ProgressMeter.progress_map(param_list; mapfun=pmap, progress=progress) do comb
        adf, mdf = test(; rac=rac, comb...)
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
    run_id = hash(params)
    CSV.write("results/round2/round2-$rac_type-row-$run_id.csv", results)
end

