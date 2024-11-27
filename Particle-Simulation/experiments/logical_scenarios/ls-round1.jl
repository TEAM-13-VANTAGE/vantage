function round1_logical_scenario(;
    drone_speed::Real,
    heli_speed::Real,
    drone_x_pos::Real,
    drone_y_pos::Real,
    drone_direction::Real,
    drone_response_distance::Real,
    force_right_turn::Union{Bool,<:Real}=false,
    heli_x_pos::Optional{<:Real}=nothing,
    heli_y_pos::Optional{<:Real}=nothing,
    drone_horizontal_turn_rate::Optional{<:Real}=nothing,
    drone_horizontal_turn_angle::Optional{<:Real}=nothing,
    drone_ascent_rate::Optional{<:Real}=nothing,
    bounds_x::Optional{<:Real}=nothing,
    bounds_y::Optional{<:Real}=nothing,
    mdata=[:contactLevel, :scenario],
    adata=[:vars, :role],
)
    force_right_turn = Bool(force_right_turn)

    scene_step! = create_standard_scene(;
        response_distance=drone_response_distance,
        drone_turn_rate=drone_horizontal_turn_rate,
        drone_ascent_rate=drone_ascent_rate,
        force_right_turn=force_right_turn,
        drone_turn_angle=drone_horizontal_turn_angle,
    )

    model = init_base_model(
        ;
        heli_speed=heli_speed,
        drone_θ=drone_direction,
        drone_speed=drone_speed,
        drone_x_pos=drone_x_pos,
        drone_y_pos=drone_y_pos,
        heli_x_pos=heli_x_pos,
        heli_y_pos=heli_y_pos,
        drone_altitude=1000.0 / 2,
        heli_altitude=1000.0 / 2,
        bounds_altitude=1000.0,
        bounds_x=bounds_x,
        bounds_y=bounds_y,
    )

    n = ceil(Int, (isnothing(bounds_x) ? BOUNDS_X : bounds_x) / (heli_speed * dt))

    # return interactable_animation(
    #     model,
    #     scene_step!,
    #     n;
    #     timescale=100.0,
    #     show_violation_region=true,
    #     response_distance=drone_response_distance,
    # )

    return run!(
        model, scene_step!, n;
        mdata=mdata,
        adata=adata
    )

end