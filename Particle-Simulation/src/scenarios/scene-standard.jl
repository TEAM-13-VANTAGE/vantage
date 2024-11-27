
# Scenarios
function create_standard_scene(
    # params to init scene
    ;
    response_distance=3000.0,
    drone_turn_rate::Optional{<:Real}=nothing,
    drone_ascent_rate::Optional{<:Real}=nothing,
    drone_turn_angle::Optional{<:Real}=nothing,
    ascend_relative_distance::Real=VIOLATION_DISTANCE_V,
    force_right_turn::Bool=false,
    dt::Real=dt,
    debug=false
)
    """
    A standard agent update method for UAV vs. Helicopter encounters.
    The drone will continue along its current path until it encounters the
    Helicopter, in which case it will begin a maneuver based on the scenario type
    (head-on, overtaking, converging). If the drone is already moving away from the
    helicopter's path, it will not respond. After executing any needed maneuver, the
    drone will enter a "leaving" state, and will continue along it's new trajectory
    until the end of the simulation.

    # Arguments:
    * response_distance: How far away the drone must be before it begins its
      maneuver.
    * drone_turn_rate: If specified, the drone will conduct a horizontal turn at the
      given turn rate.
    * drone_turn_angle: How far to turn the drone to avoid a violation for an ROW
      maneuver. Only relevant if both a drone_turn_rate and force_right_turn are
      specified (i.e. an standard ROW maneuver).
    * ascend_relative_distance: How far vertically above/below the helicopter to
      transit to during an encounter. (Having both a horizontal and vertical maneuver
      is a WIP, not sure if it works.)
    * force_right_turn: Always turn right by @drone_turn_angle radians (90 degrees
      for standard ROW maneuver).
    """
    drone_turn_angle = !isnothing(drone_turn_rate) && isnothing(drone_turn_angle) ? pi / 2 : drone_turn_angle
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
            !isnothing(drone_ascent_rate)
        )
            target_alt = neighbors[1].pos[3] + ascend_relative_distance
            agent.state = ascending(target_alt, drone_ascent_rate, norm(agent.vel))

        elseif (
            do_respond &&
            !isnothing(drone_turn_rate) &&
            force_right_turn
        )
            other = neighbors[1]
            self_v = [agent.vel[1:2]...]
            s = [other.pos[1:2]...] - [agent.pos[1:2]...]

            is_towards = abs(angle_between_2(self_v, s)) < π / 2

            if (
                model.scenario ∈ [headOn, overtaking] ||
                (model.scenario == converging && is_towards)
            )
                agent.state = turning(-drone_turn_rate, drone_turn_angle)
            end

        elseif (
            do_respond &&
            !isnothing(drone_turn_rate)
        )
            # println("Starting response")
            other = neighbors[1]
            other_v = [other.vel[1:2]...]
            self_v = [agent.vel[1:2]...]
            s = [other.pos[1:2]...] - [agent.pos[1:2]...]

            is_towards = abs(angle_between_2(self_v, s)) < π / 2
            # println("Is towards: $is_towards")

            if (
                model.scenario ∈ [headOn, overtaking] ||
                (model.scenario == converging && !is_towards)
            )
                # println("Point away 90")
                side = angle_between_2(other_v, -s) < 0 ? right : left
                relative_θ = side == left ? π / 2 : -π / 2
                u = [get_vel(relative_θ, 0.0, 1.0)[1:2]...]
                θ = angle_between_2(self_v, u)
                cw = θ < 0

                # cw = θ < 0 || force_right_turn
                # if force_right_turn && θ > 0
                #     θ += π
                # end

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

        # Transitions and behaviors
        if isa(agent.state, turning)
            agent.state = turn!(agent, agent.state, dt)

            if isa(agent.state, idling)
                # println("Leaving")
                agent.state = LEAVE
            end

        elseif isa(agent.state, ascending)
            agent.state = ascend_to!(agent, agent.state, dt)

            if isa(agent.state, idling) && !isnothing(drone_turn_rate)
                # if both are specified
                agent.state = turning(drone_turn_rate, drone_turn_angle)
            elseif isa(agent.state, idling)
                agent.state = LEAVE
            end
        end

        agent_end_step!(agent, model, neighbors)
    end

    return scene!
end
