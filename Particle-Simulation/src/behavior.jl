using Agents
using LinearAlgebra
using Setfield

# if !isdefined(Base, :__init__) || Base.function_module(__init__) != MyModule
#     include("structs.jl")
#     include("utils.jl")
#     include("model.jl")
#     include("search.jl")
#     include("structs.jl")
#     include("states.jl")
# end

"""
verticle angle could change over time, but currently just
snaps to the set angle... I don't think it's relevant enough 
to implement.
"""

function rotate_vehicle!(vehicle::Vehicle, θ::Real, φ::Real)
    # Rotate velocity vector horizontally (θ) and/or vertically (φ)
    vehicle.vel = Tuple(rotation_matrix(θ, φ) * [vehicle.vel...])
end

function evaluate_contact(agent::Vehicle, neighbor::Vehicle, model::StandardABM)
    # Determines if contact has escalated (from none to violation, or violation to
    # collision). When a new contact is detected, the event is logged and the model's
    # contact level will be updated.
    ct = get_contact_type(agent, neighbor, model)

    if ct > model.contactLevel
        append!(
            model.contactEvents,
            [ContactEvent(
                (agent, neighbor),
                ct,
                dt * model.step
            )]
        )
        model.contactLevel = ct
    end
end

function turn!(agent::Vehicle, state::turning, dt::Real)::State
    # Turns the @agent and updates @state         
    Δθ = state.turn_rate * dt

    # Snap to desired angle if overshoot
    Δθ = (
        (state.θ + Δθ) <= state.turn_angle ?
        Δθ :
        state.turn_angle - state.θ
    )

    # Updated state to track progress of turn
    state = @set state.θ += abs(Δθ)

    agent.vel = rotate_vehicle!(agent, Δθ, 0.0)

    # Return IDLE if turn complete
    return state.θ >= state.turn_angle ? IDLE : state
end

function turnTowards!(agent::Vehicle, state::turningTowards, dt::Real)::State
    # Turns the @agent towards a target destination and updates @state         
    pos = [agent.pos[1:2]...]
    vel = [agent.vel[1:2]...]
    s = state.destination - pos
    turn_angle = angle_between_2(vel, s)
    cw = turn_angle > 0
    Δθ = (-1)^(!cw) * abs(state.turn_rate) * dt
    # Ensure the final angle is always equal to target angle.

    # # println("Δθ = $Δθ, angle from: $turn_angle")
    Δθ = (
        2 * abs(Δθ) < abs(turn_angle) ?
        Δθ :
        turn_angle
    )

    agent.vel = rotate_vehicle!(agent, Δθ, 0.0)

    return Δθ == turn_angle ? IDLE : state
end

function ascend_to!(agent::Vehicle, state::ascending, dt::Real)::State
    # Ascend @agent towards a target altitude and update @state
    speed = norm(agent.vel)

    if agent.pos[3] >= state.target_alt
        # level out
        agent.vel = Tuple(speed * [agent.vel[1:2]..., 0] / norm(agent.vel[1:2]))
        state = IDLE # set state to IDLE when complete
    elseif !state.is_ascending
        θ = atan(agent.vel[2], agent.vel[1])
        agent.vel = get_vel(θ, state.ϕ, speed)
        state = @set state.is_ascending = true # Ascent has begun, continue until completion
    end

    return state
end

function orbit!(agent::Vehicle, state::circling, dt::Real)::State
    # Orbit @agent around a center of rotation at some distance. No termination condition.
    dθ = state.turn_rate * dt
    rotate_vehicle!(agent, dθ, 0.0)
    return state
end

function goto!(agent::Vehicle, state::transitingTo, dt::Real)::State
    # Travel @agent to a target destination. Updates @state. Terminates at arival.
    # Target destinations are horizontal only.
    if !isnothing(state.turn)
        if isa(state.turn, turning)
            t = turn!(agent, state.turn, dt)
        elseif isa(state.turn, turningTowards)
            t = turnTowards!(agent, state.turn, dt)
        end
        state = @set state.turn = isa(t, idling) ? nothing : t
    else
        # Close the distance to target
        dist = norm([state.destination...] - [agent.pos[1:2]...])
        state = dist <= state.radius + (norm(agent.vel) * dt) ? IDLE : state
    end

    return state
end

function go_until!(agent::Vehicle, state::transitingUntil, dt::Real)::State
    # Transit horizontally until a condition is met.
    return state.condition(agent) ? IDLE : state
end

function turn_until!(agent::Vehicle, state::turningUntil, dt::Real)::State
    # Turn orizontally until a condition is met.
    Δθ = state.turn_rate * dt
    rotate_vehicle!(agent, Δθ, 0.0)
    return state.condition(agent) ? IDLE : state
end