

function create_test_sz_scene(
    # params to init scene
    pois::Vector{<:Vector{<:Real}},
    safe_zones::Vector{<:Vector{<:Real}},
    rac::RAC;
    init_poi_indx=1,
    response_distance=3000.0,
    drone_turn_rate::Real=20.0 * π / 180,
    min_drone_speed::Real=44.0,
    max_drone_turn_rate::Real=20.0 * π / 180,
    dt::Real=dt,
    debug=false,
)
    function scene!(agent::Vehicle, model::StandardABM)
        agent_start_step!(agent, model)
        if agent.role == heli
            agent_end_step!(agent, model, Vector{Vehicle}())
            return
        end

        register_rac_pos(agent, model, rac)
        neighbors = [
            a
            for a in nearby_agents(
                agent, model, response_distance
            )
        ]
        register_scenario(agent, model, neighbors)
        do_respond = response_required(
            agent, model, neighbors;
            listening_state=Union{idling,transitingTo})

        if (
            do_respond &&
            "safe" ∉ keys(agent.vars)
        )
            self_pos = [agent.pos...]
            self_vel = [agent.vel...]
            other_pos = [neighbors[1].pos...]
            other_vel = [neighbors[1].vel...]
            r = turn_rate_to_radius(max_drone_turn_rate, norm(agent.vel))
            s = self_pos - other_pos
            side = angle_between_2(other_vel[1:2], s[1:2]) > 0 ? left : right

            approaching_zones = [
                sz[1:2]
                for sz in safe_zones
                if abs(angle_between_2(sz[1:2] - self_pos[1:2], self_vel[1:2])) < π / 2
            ]

            same_side_zones = [
                sz[1:2]
                for sz in safe_zones
                if (
                    angle_between_2(other_vel[1:2], sz[1:2] - other_pos[1:2]) > 0 ? left : right
                ) == side
            ]

            if length(same_side_zones) > 0
                s_ssz = [norm(sz - self_pos[1:2]) for sz in same_side_zones]
                min_ssz, i = findmin(s_ssz)

                s_az = [norm(sz - self_pos[1:2]) for sz in approaching_zones]
                min_az, j = isempty(s_az) ? (NaN, -1) : findmin(s_az)

                if min_ssz < min_az + 1.5pi * r || j == -1
                    # Prefer not crossing path by going to same-side zones
                    sz = same_side_zones[i]
                    push!(
                        agent.pos_log,
                        PositionLog(agent.id, agent.pos, model.step, "Navigating to same-side safe zone")
                    )
                else
                    # Go to zones ahead if too costly to turn around
                    sz = approaching_zones[j]
                    push!(
                        agent.pos_log,
                        PositionLog(agent.id, agent.pos, model.step, "Navigating to ahead safe zone")
                    )
                end
            elseif length(approaching_zones) > 0
                sz_indx = nearest_point(self_pos[1:2], approaching_zones)
                sz = safe_zones[sz_indx]
                push!(
                    agent.pos_log,
                    PositionLog(agent.id, agent.pos, model.step, "Navigating to ahead safe zone")
                )
            else
                # edge case
                sz_indx = nearest_point(self_pos[1:2], [sz[1:2] for sz in safe_zones])
                sz = safe_zones[sz_indx]
                push!(
                    agent.pos_log,
                    PositionLog(agent.id, agent.pos, model.step, "Navigating to nearest safe zone")
                )
            end

            agent.state = create_transit_to_loc(agent, sz, max_drone_turn_rate)

            agent.vars["safe"] = false
        end

        # Logic
        if "poi_index" ∉ keys(agent.vars) && agent.role == drone
            indx = agent.vars["poi_index"] = init_poi_indx
            # println("Beginning route to poi $indx: $(pois[indx])")
            push!(
                agent.pos_log,
                PositionLog(agent.id, agent.pos, model.step, "Beginning route to POI $indx")
            )

            agent.state = create_transit_to_loc(
                agent,
                pois[indx],
                drone_turn_rate
            )
        end

        # Transitions and behaviors
        if isa(agent.state, transitingTo) && "safe" ∉ keys(agent.vars)
            agent.state = goto!(agent, agent.state, dt)
            if isa(agent.state, idling)

                indx = agent.vars["poi_index"] % length(pois) + 1
                # println("Going to next POI: $indx, $(pois[indx])")
                agent.vars["poi_index"] = indx
                agent.state = create_transit_to_loc(
                    agent,
                    pois[indx],
                    drone_turn_rate
                )
                # PositionLog(agent.id, agent.pos, model.step, "Going to POI $indx")
                # # println("Turn rate: $(agent.state.turn.turn_rate), angle: $(agent.state.turn.turn_angle)")
            end

        elseif isa(agent.state, transitingTo)
            agent.state = goto!(agent, agent.state, dt)
            if isa(agent.state, idling)
                # println("Left danger, now orbiting")
                agent.state = circling(max_drone_turn_rate)
                agent.vel = Tuple([agent.vel...] / norm(agent.vel) * min_drone_speed)
                push!(
                    agent.pos_log,
                    PositionLog(agent.id, agent.pos, model.step, "Left danger, now orbiting")
                )
            end

        elseif isa(agent.state, circling)
            agent.state = orbit!(agent, agent.state, dt)
        end

        agent_end_step!(agent, model, neighbors)
    end

    return scene!
end


function create_sz_scene(
    # params to init scene
    safe_zones::Vector{<:Vector{<:Real}},
    rac::RAC;
    sz_turn_rate::Real=0.34907,
    response_distance::Real=3000.0,
    drone_turn_rate::Optional{Float64}=nothing,
    dt::Real=dt,
)
    function scene!(agent::Vehicle, model::StandardABM)
        agent_start_step!(agent, model)

        neighbors = [
            a
            for a in nearby_agents(
                agent, model, response_distance
            )
        ]
        register_scenario(agent, model, neighbors)
        do_respond = response_required(agent, model, neighbors)

        # Logic
        if (
            do_respond &&
            "safe" ∉ keys(agent.vars)
        )
            self_pos = [agent.pos...]
            self_vel = [agent.vel...]
            approaching_zones = [
                sz[1:2]
                for sz in safe_zones
                if abs(angle_between(sz[1:2] - self_pos[1:2], self_vel[1:2])) < π / 2
            ]

            if length(approaching_zones) > 0
                sz_indx = nearest_point(self_pos[1:2], approaching_zones)
            else
                # edge case
                sz_indx = nearest_point(self_pos[1:2], [sz[1:2] for sz in safe_zones])
            end
            sz = safe_zones[sz_indx]

            # cw = angle_between(sz - self_pos[1:2], self_vel[1:2]) >= 0
            turn = create_turn_to_tangent(
                agent, sz_turn_rate, sz_turn_rate, sz
            )

            agent.state = transitingTo(
                Tuple(sz),
                turn_rate_to_radius(sz_turn_rate, norm(agent.vel)),
                turn
            )
        end

        # Transitions and behaviors
        if isa(agent.state, transitingTo)
            agent.state = goto!(agent, agent.state, dt)

            if isa(agent.state, idling)
                # println("Orbiting safe zone")
                agent.vars["safe"] = true
                agent.state = circling(sz_turn_rate)
            end

        elseif isa(agent.state, circling)
            orbit!(agent, agent.state, dt)
        end

        agent_end_step!(agent, model, neighbors)
    end

    return scene!
end


function create_rac_hv_scene(
    # params to init scene
    rac::RAC;
    response_distance=3000.0,
    drone_turn_rate::Optional{Float64}=nothing,
    drone_ascent_rate::Optional{Float64}=nothing,
    drone_turn_angle::Real=1.0π / 2,
    ascend_relative_distance::Real=VIOLATION_DISTANCE_V,
    dt::Real=dt,
)
    # note that you need to specify drone's initial state if missioned

    function scene!(agent::Vehicle, model::StandardABM)
        agent_start_step!(agent, model)

        register_rac_pos(agent, rac)

        neighbors = [
            a
            for a in nearby_agents(
                agent, model, response_distance
            )
        ]

        register_scenario(agent, model, neighbors)
        do_respond = response_required(agent, model, neighbors)

        # Transitions
        if (
            do_respond &&
            !isnothing(drone_ascent_rate)
        )
            target_alt = neighbors[1].pos[3] + ascend_relative_distance
            agent.state = ascending(target_alt, drone_ascent_rate)
            push!(
                agent.pos_log,
                PositionLog(agent.id, agent.pos, model.step, MSG_ASCENT_BEGAN)
            )
        elseif (
            do_respond &&
            !isnothing(drone_turn_rate)
        )
            other = neighbors[1]
            other_v = [other.vel...]
            self_v = [agent.vel...]
            s = [other.pos...] - [agent.pos...]

            is_towards = abs(angle_between(self_v, s)) < π / 2
            # Drone needs to avoid
            # agent.state = turning(drone_turn_rate, drone_turn_angle)


            if (
                model.scenario ∈ [headOn, overtaking] ||
                (model.scenario == converging && !is_towards)
            )
                # println("Point away 90")
                side = angle_between(other_v, -s) < 0 ? right : left
                relative_θ = side == left ? π / 2 : -π / 2
                u = [get_vel(relative_θ, 0.0, 1.0)...]
                θ = angle_between(self_v, u)
                cw = θ < 0
                agent.state = turning((-1)^cw * abs(drone_turn_rate), abs(θ))

                # elseif model.scenario == overtaking
                #     # println("Point right 90")
                #     θ = 1.0π / 2 - relative_θ
                #     agent.state = turning(drone_turn_rate, θ)
                #     # 90 degrees from heli, always right.
            elseif is_towards
                # converging scenario
                # println("Point away 180")
                agent.state = get_turn_against_vector(
                    agent,
                    [neighbors[1].vel...],
                    drone_turn_rate,
                    0.0 # turn so we face against heli
                )
            end

            push!(
                agent.pos_log,
                PositionLog(agent.id, agent.pos, model.step, MSG_TURN_BEGAN)
            )
        end

        # Behavior
        if isa(agent.state, turning)
            agent.state = turn!(agent, agent.state, dt)
            if isa(agent.state, idling)
                # println("Leaving")
                agent.state = LEAVE
                push!(
                    agent.pos_log,
                    PositionLog(agent.id, agent.pos, model.step, MSG_TURN_END)
                )
            end

        elseif isa(agent.state, circling)
            orbit!(agent, agent.state, dt)

        elseif isa(agent.state, ascending)
            agent.state = ascend_to!(agent, agent.state, dt)

            if isa(agent.state, idling) && !isnothing(drone_turn_rate)
                # if both are specified
                agent.state = turning(drone_turn_rate, drone_turn_angle)
            end
            if isa(agent.state, idling)
                push!(
                    agent.pos_log,
                    PositionLog(agent.id, agent.pos, model.step, MSG_ASCENT_END)
                )
            end
        end

        agent_end_step!(agent, model, neighbors)
    end

    return scene!
end

