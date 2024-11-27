

function create_rac_row(
    # params to init scene
    pois::Vector{<:Vector{<:Real}},
    rac::RAC;
    init_poi_indx=1,
    response_distance=3000.0,
    drone_turn_rate::Optional{Float64}=nothing,
    drone_turn_angle::Real=1.0π / 2,
    dt::Real=dt,
    min_drone_speed::Real=44.0,
    max_drone_turn_rate::Real=20.0 * π / 180,
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

        if "poi_index" ∉ keys(agent.vars) && agent.role == drone
            indx = agent.vars["poi_index"] = init_poi_indx
            # println("Beginning route to poi $indx: $(pois[indx])")
            PositionLog(agent.id, agent.pos, model.step, "Beginning route to POI $indx")

            agent.state = create_transit_to_loc(
                agent,
                pois[indx],
                drone_turn_rate
            )
        end

        if (
            do_respond
        )
            agent.state = turning(-abs(drone_turn_rate), drone_turn_angle)

            push!(
                agent.pos_log,
                PositionLog(agent.id, agent.pos, model.step, "Beginning $(agent.state.direction) turn")
            )
        end

        # Transitions and behaviors
        if isa(agent.state, transitingTo)
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
                PositionLog(agent.id, agent.pos, model.step, "Going to POI $indx")
                # # println("Turn rate: $(agent.state.turn.turn_rate), angle: $(agent.state.turn.turn_angle)")
            end

        elseif isa(agent.state, turning)
            direction = agent.state.direction
            agent.state = turn!(agent, agent.state, dt)
            if isa(agent.state, idling)
                r = turn_rate_to_radius(max_drone_turn_rate, min_drone_speed) #norm([agent.vel...]))
                l = lowerbuffer(rac) + [[r * 2.1 for i in 1:2]..., 0.0]
                u = upperbuffer(rac) - [[r * 2.1 for i in 1:2]..., 0.0]
                # println("Leaving volume with \nl = $l\nu = $u\nTurn radius of $r")
                agent.state = create_transit_to_leave_volume(l, u)
                # println("Transiting to leave danger")
                push!(
                    agent.pos_log,
                    PositionLog(agent.id, agent.pos, model.step, "End of $direction turn")
                )
            end

        elseif isa(agent.state, transitingUntil)
            agent.state = go_until!(agent, agent.state, dt)
            if isa(agent.state, idling)
                # println("Left danger, now orbiting")
                agent.state = circling(max_drone_turn_rate)
                agent.vel = Tuple([agent.vel...] / norm(agent.vel) * min_drone_speed)
                PositionLog(agent.id, agent.pos, model.step, "Left danger, now orbiting")
            end

        elseif isa(agent.state, circling)
            agent.state = orbit!(agent, agent.state, dt)
        end

        agent_end_step!(agent, model, neighbors)
    end

    return scene!
end


function create_test_scene(
    # params to init scene
    pois::Vector{<:Vector{<:Real}},
    rac::RAC;
    init_poi_indx=1,
    response_distance=3000.0,
    drone_turn_rate::Optional{Float64}=nothing,
    drone_ascent_rate::Optional{Float64}=nothing,
    drone_turn_angle::Real=1.0π / 2,
    ascend_relative_distance::Real=VIOLATION_DISTANCE_V,
    dt::Real=dt,
    min_drone_speed::Real=44.0,
    max_drone_turn_rate::Real=20.0 * π / 180,
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
            do_respond
        )
            other = neighbors[1]
            other_v = [other.vel...]
            self_v = [agent.vel...]
            s = [other.pos...] - [agent.pos...]

            is_towards = abs(angle_between_2(self_v[1:2], s[1:2])) < π / 2
            # println(println()angle_between_2(self_v[1:2], s[1:2]))
            # Drone needs to avoid

            if (
                model.scenario ∈ [headOn, overtaking] ||
                (model.scenario == converging && !is_towards)
            )
                # println("Point away 90")
                side = angle_between_2(other_v[1:2], -s[1:2]) < 0 ? right : left
                relative_θ = side == left ? π / 2 : -π / 2
                heading = atan(other_v[2], other_v[1])

                u = [get_vel(heading + relative_θ, 0.0, 1.0)...]
                θ = angle_between_2(self_v[1:2], u[1:2])
                cw = θ < 0
                agent.state = turning((-1)^cw * abs(drone_turn_rate), abs(θ))

            elseif is_towards
                # converging scenario
                # println("Converging - Point away 180 (turn against)")
                agent.state = get_turn_against_vector(
                    agent,
                    [neighbors[1].vel...],
                    drone_turn_rate,
                    0.0 # turn so we face against heli
                )
            end


            # println("Scenario: $(model.scenario)")
            if agent.state.direction == right
                # println("conducting right hand turn")
            else
                # println("conducting left hand turn")
            end

            push!(
                agent.pos_log,
                PositionLog(agent.id, agent.pos, model.step, "Beginning $(agent.state.direction) turn")
            )
        end

        # Logic
        if "poi_index" ∉ keys(agent.vars) && agent.role == drone
            indx = agent.vars["poi_index"] = init_poi_indx
            # println("Beginning route to poi $indx: $(pois[indx])")
            PositionLog(agent.id, agent.pos, model.step, "Beginning route to POI $indx")

            agent.state = create_transit_to_loc(
                agent,
                pois[indx],
                drone_turn_rate
            )
        end

        # Transitions and behaviors
        if isa(agent.state, transitingTo)
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
                PositionLog(agent.id, agent.pos, model.step, "Going to POI $indx")
                # # println("Turn rate: $(agent.state.turn.turn_rate), angle: $(agent.state.turn.turn_angle)")
            end

        elseif isa(agent.state, turning)
            direction = agent.state.direction
            agent.state = turn!(agent, agent.state, dt)
            if isa(agent.state, idling)
                r = turn_rate_to_radius(max_drone_turn_rate, min_drone_speed) #norm([agent.vel...]))
                l = lowerbuffer(rac) + [[r * 2.1 for i in 1:2]..., 0.0]
                u = upperbuffer(rac) - [[r * 2.1 for i in 1:2]..., 0.0]
                # println("Leaving volume with \nl = $l\nu = $u\nTurn radius of $r")
                agent.state = create_transit_to_leave_volume(l, u)
                # println("Transiting to leave danger")
                push!(
                    agent.pos_log,
                    PositionLog(agent.id, agent.pos, model.step, "End of $direction turn")
                )
            end

        elseif isa(agent.state, transitingUntil)
            agent.state = go_until!(agent, agent.state, dt)
            if isa(agent.state, idling)
                # println("Left danger, now orbiting")
                agent.state = circling(max_drone_turn_rate)
                agent.vel = Tuple([agent.vel...] / norm(agent.vel) * min_drone_speed)
                PositionLog(agent.id, agent.pos, model.step, "Left danger, now orbiting")
            end
        elseif isa(agent.state, circling)
            agent.state = orbit!(agent, agent.state, dt)
        end

        agent_end_step!(agent, model, neighbors)
    end

    return scene!
end
