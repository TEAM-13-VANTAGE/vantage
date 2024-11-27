import Pkg; 
Pkg.add("Meshes")

using Agents
include("src/main.jl")
include("src/graphics.jl")


function test(;
    drone_speed::Real,
    heli_speed::Real,
    drone_x_pos::Real,
    drone_y_pos::Real,
    drone_direction::Real,
    drone_response_distance::Real,
    force_right_turn::Union{Bool,<:Real}=false,
    drone_horizontal_turn_rate::Optional{<:Real}=nothing,
    drone_horizontal_turn_angle::Optional{<:Real}=nothing,
    drone_ascent_rate::Optional{<:Real}=nothing,
    n_override::Optional{<:Real}=nothing,
    bounds_x::Optional{<:Real}=nothing,
    bounds_y::Optional{<:Real}=nothing,
)
    force_right_turn = Bool(force_right_turn)

    scene_step! = create_standard_scene(;
        response_distance=drone_response_distance,
        drone_turn_rate=drone_horizontal_turn_rate,
        drone_ascent_rate=drone_ascent_rate,
        force_right_turn=false,
    )

    model = init_base_model(
        ;
        drone_speed=drone_speed,
        heli_speed=heli_speed,
        drone_x_pos=drone_x_pos,
        drone_y_pos=drone_y_pos,
        drone_θ=drone_direction,
        drone_altitude=1000.0 / 2,
        heli_altitude=1000.0 / 2,
        bounds_altitude=1000.0,
        bounds_x=bounds_x,
        bounds_y=bounds_y,
    )

    n = ceil(Int, (isnothing(bounds_x) ? BOUNDS_X : bounds_x) / (heli_speed * dt))

    return interactable_animation(
        model,
        scene_step!,
        n;
        timescale=1.0,
        show_violation_region=true,
        response_distance=drone_response_distance,
    )

    # return run!(
    #     model, scene_step!, n;
    #     mdata=[:contactLevel, :contactEvents, :scenario],
    #     adata=[:pos_log, :vars, :role]
    # )
end

manifest, units = read_manifest("params.csv")
params = expand_parameters(manifest)
all_combinations = generate_param_combinations(params)
comb = all_combinations[353]

# adf, mdf = test(;
#     comb...
# )

c = [6000.0, 6000.0]
d = 3000.0

for i in 0:3
    θ = i * pi / 2
    x = c[1] + d * cos(θ + pi)
    y = c[2] + d * sin(θ + pi)

    println("Angle: $θ, Pos: ($x, $y)")

    p = Dict(
        comb...,
        :drone_direction => θ,
        :drone_x_pos => x,
        :drone_y_pos => y,
    )

    adf, mdf = test(;
        p...
    )
end