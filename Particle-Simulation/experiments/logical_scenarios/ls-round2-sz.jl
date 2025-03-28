function test(;
    rac::RAC,
    flight_index::Real,
    heli_path_index::Real,
    drone_speed::Real,
    heli_speed::Real,
    drone_response_distance::Real,
    drone_turn_rate::Real,
    heli_shift::Real,
    clockwise::Real=1.0,
    n_override::Optional{<:Real}=nothing,
    mdata=[:contactEvents, :scenario],
    adata=[:vars, :role],
)
    flight_index = ceil(Int, flight_index)
    heli_path_index = ceil(Int, heli_path_index)

    cw = clockwise != 0.0

    heli_paths = get_heli_path_from_rac(rac; percent_shift=heli_shift)
    heli_path = heli_paths[heli_path_index]

    route = get_route_from_rac(rac, 400.0)
    if !cw
        reverse!(route)
    end

    flights = get_init_from_route(route)
    flight = flights[flight_index]

    separation = 2.1 * turn_rate_to_radius(20.0 * pi / 180, 44.0)
    minvol = min_viable_volume(rac, separation)
    safe_zones = get_sz_from_rac_and_path(
        minvol...,
        heli_path[2:3],
        heli_path[1],
        separation * 2 + VIOLATION_DISTANCE_H
    )

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

    return run!(
        model, scene_step!, n;
        mdata=mdata,
        adata=adata
    )
end