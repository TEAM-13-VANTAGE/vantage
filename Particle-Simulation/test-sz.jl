
function corner_from_center(loc::Vector{<:Real}, dimensions::Vector{<:Real})
    return [x - a / 2 for (x, a) in zip(loc, dimensions)]
end

function get_init_from_route(route::Vector{<:Vector{<:Real}})
    # gets initial direction and position in the form (θ, x, y) for centers of
    # route.
    inits = []

    n_nodes = length(route)
    for i in 1:n_nodes
        j = i % n_nodes + 1
        ni = route[i]
        nj = route[j]
        s = nj - ni
        p = s / 2 + ni
        θ = atan(s[2], s[1])
        push!(inits, (θ, p...))
    end

    return vcat(inits[end], inits[1:end-1])
end

function get_route_from_rac(rac::RAC, dfb::Real)::Vector{<:Vector{<:Real}}
    l = rac.lower[1:2] + [dfb for _ in 1:2]
    u = rac.upper[1:2] - [dfb for _ in 1:2]

    result = unique([
        [x...] for x in Iterators.product(zip(l, u)...)
    ])

    result[1], result[2] = result[2], result[1]

    return result
end

function get_heli_path_from_rac(
    rac::RAC;
    percent_shift::Real=0.0,
)::Vector{<:Vector{<:Real}}
    dimensions = rac.upper - rac.lower

    # Converging 45, intercepts y-axis 
    x1 = rac.lower[1]
    y1 = rac.lower[2] + dimensions[2] / 2 * (1 + percent_shift)

    # Converging 45, intercepts x-axis 
    x2 = rac.lower[1] + dimensions[1] / 2 * (1 + percent_shift)
    y2 = rac.upper[2]

    # Converging 90
    x3 = rac.lower[1] + dimensions[1] / 2 * (1 + percent_shift)
    y3 = rac.upper[2]

    # Headon/overtaking 0
    x4 = rac.lower[1]
    y4 = rac.lower[2] + dimensions[2] / 2 * (1 + percent_shift)


    v = [get_vel(-π / 4, 0.0, 1)[1:2]...]

    # linearly interpolate to a reasonable starting position:
    # paths = Dict(
    #     :y_intercept => [x1, y1] - 9900.0 * v,
    #     :x_intercept => [x2, y2] - 9900.0 * v
    # )
    paths = [
        [-π / 4, ([x1, y1] - 9900.0 * v)...],
        [-π / 4, ([x2, y2] - 9900.0 * v)...],
        [-π / 2, ([x3, y3 + 9900.0])...],
        [0.0, ([x4 - 9900, y4])...],
    ]
    return paths
end

function distance_from_path(p::Vector{<:Real}, p0::Vector{<:Real}, θ::Real)
    dif = p - p0
    return abs(dif[1] * sin(θ) - dif[2] * cos(θ))
end


function min_viable_volume(rac::RAC, separation::Real)::Tuple{Vector{<:Real},Vector{<:Real}}
    l = lowerbuffer(rac)[1:2] + [separation, separation]
    u = upperbuffer(rac)[1:2] - [separation, separation]

    return (l, u)
end


function test(;
    flight_index::Real,
    heli_path_index::Real,
    drone_speed::Real,
    heli_speed::Real,
    drone_response_distance::Real,
    drone_turn_rate::Real,
    heli_shift::Real=0.0,
    clockwise::Real=1.0,
    n_override::Optional{<:Real}=nothing
)
    flight_index = ceil(Int, flight_index)
    heli_path_index = ceil(Int, heli_path_index)

    cw = clockwise != 0.0


    w, l, h = 5200.0, 1700.0, 400.0
    rac_center = [20000.0 - 2000.0 - w / 2, 20000.0 / 4, 200.0] #[7150.0 + 5000.0, 6000.0 + 2000, 200.0]
    rac_corner = corner_from_center(rac_center, [w, l, h])

    rac = RAC_by_dimensions(
        rac_corner, [w, l, h],
        2000.0
    )

    heli_paths = get_heli_path_from_rac(rac; percent_shift=heli_shift)
    heli_path = heli_paths[heli_path_index]
    println("Heli path: $heli_path")

    separation = 2.1 * turn_rate_to_radius(20.0 * pi / 180, 44.0)
    minvol = min_viable_volume(rac, separation)
    safe_zones = get_sz_from_rac_and_path(
        minvol...,
        heli_path[2:3],
        heli_path[1],
        separation * 2 + VIOLATION_DISTANCE_H
    )

    route = get_route_from_rac(rac, 400.0)
    if !cw
        reverse!(route)
    end

    flights = get_init_from_route(route)
    flight = flights[flight_index]
    println("drone flight: $flight")

    scene_step! = create_test_sz_scene(
        route,
        safe_zones,
        rac;
        response_distance=drone_response_distance,
        drone_turn_rate=drone_turn_rate,
        init_poi_indx=flight_index
    )

    model = init_base_model(
        ;
        heli_θ=heli_path[1],
        heli_x_pos=heli_path[2],
        heli_y_pos=heli_path[3],
        heli_speed=heli_speed,
        drone_θ=flight[1],
        drone_speed=drone_speed,
        drone_x_pos=flight[2],
        drone_y_pos=flight[3],
        drone_altitude=h / 2,
        bounds_altitude=1000.0,
        heli_altitude=h / 2,
        bounds_x=20000.0,
        bounds_y=20000.0,
    )

    n = ceil(Int, 20000.0 / (heli_speed * dt))

    # return interactable_animation(
    #     model,
    #     scene_step!,
    #     isnothing(n_override) ? n : n_override;
    #     timescale=100.0,
    #     show_violation_region=true,
    #     response_distance=drone_response_distance,
    #     rac=rac,
    #     pois=safe_zones
    # )
    return run!(
        model, scene_step!, n;
        mdata=[:contactLevel, :contactEvents, :scenario],
        adata=[:pos_log, :vars, :role]
    )
end

params, units = read_parameters("params/validation/round2-horizontal-params.csv")
all_params = generate_param_combinations(params)

min_dists = []

for p in all_params
    adf, mdf = test(; p...)

    push!(
        min_dists,
        adf[adf.role.==drone, :].vars[end]["min_dist"]
    )
end

# adf, mdf = test(;
#     all_params[1]...,
# )

