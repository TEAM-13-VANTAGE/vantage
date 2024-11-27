# Defines the valid states of the drone. States are used with behaviors to execute
# manuevers. 

using LinearAlgebra

abstract type TurnState <: State end
abstract type TransitState <: State end

# The default state
struct idling <: State end

# Similar to idling, but used to indicate that the drone is no longer responding to
# stimuli.
struct leaving <: State end

# Const states, since they have no data.
const IDLE = idling()
const LEAVE = leaving()


# Turn the vehicle horizontally by some number of radians.
struct turning <: TurnState
    turn_rate::Real # radians/sec
    turn_angle::Real
    direction::Direction
    θ::Real
    turning(turn_rate, turn_angle) = new(
        turn_rate, turn_angle, turn_rate < 0 ? left : right, 0.0
    )
    turning(turn_rate, turn_angle, direction, cur) = new(
        turn_rate, turn_angle, direction, cur
    )
end

# Turn the vehicle horizontally towards a destination.
struct turningTowards <: TurnState
    turn_rate::Real # radians/sec
    destination::Vector{<:Real}
end

# Turn the vehicle until a condition is met. (Function takes in the agent and returns
# a bool)
struct turningUntil <: TurnState
    turn_rate::Real # radians/sec
    condition::Function
end

# Ascend to a target altitude.
struct ascending <: State
    target_alt::Real
    # ascent_rat::Real
    ϕ::Real
    is_ascending::Bool
    ascending(target_alt::Real, ascent_rate::Real, speed::Real) = new(
        target_alt,
        asin(ascent_rate / speed),
        false
    )
    ascending(target_alt::Real, ϕ::Real) = new(
        target_alt,
        ϕ,
        false
    )
    ascending(target_alt::Real, ϕ::Real, is_ascending::Bool) = new(
        target_alt,
        ϕ,
        is_ascending
    )
end

# Turn towards and transit to a target destination.
struct transitingTo <: TransitState
    destination::Vector{<:Real} # Only considers XY plane
    radius::Real
    turn::Optional{TurnState}
    transitingTo(destination::Vector{<:Real},
        radius::Real,
        turn::Optional{TurnState}
    ) = new(
        destination, radius, turn
    )
end

# Transit until a condition is met. (Function takes in the agent and returns a bool)
struct transitingUntil <: TransitState
    condition::Function
end

# Continuously rotate a vehicle by some radians per second.
struct circling <: State
    turn_rate::Real
    circling(radius::Real, speed::Real, clockwise::Bool) = new(
        (-1)^clockwise * speed / radius
    )
    circling(turn_rate) = new(turn_rate)
end

# Constructors
## Used for more easily creating states.
function create_turn_to_tangent(
    agent::Vehicle,
    turn_rate::Real,
    orbit_turn_rate::Real,
    orbit_loc::Vector{<:Real}
)::Tuple{turning,Bool}
    # Returns the turning state and whether or not you need to orbit Clockwise (T/F)
    pos = [agent.pos[1:2]...]
    dest = orbit_loc[1:2]
    speed = norm(agent.vel)

    r1 = turn_rate_to_radius(turn_rate, speed)
    r2 = turn_rate_to_radius(orbit_turn_rate, speed)

    cw = turn_rate < 0
    # vector from pos to center of rotation
    u = [get_vel(atan(agent.vel[2], agent.vel[1]) + (-1)^cw * π / 2, 0.0, 1.0)[1:2]...]
    # center of rotation
    c = pos + r1 * u
    s = dest - c

    ϕ = acos(clamp((r1 - r2) / norm(s), -1, 1))

    ψ = angle_between(-u, s) # abs()?

    θ = ψ - ϕ
    if θ < 0
        ϕ = acos(clamp((r1 + r2) / norm(s), -1, 1)) # second contact point
        θ = ψ - ϕ
        orbit_cw = !cw
    else
        orbit_cw = cw
    end

    θ += θ < 0 ? 2.0pi : 0.0

    println("Turning by $θ at turn rate of $(turn_rate * 180 / pi) degrees/s")

    return turning(turn_rate, θ), orbit_cw
end

function create_transit_to_loc(agent::Vehicle, loc::Vector{<:Real}, turn_rate::Real)
    # Creates a transitingTo state, given a location and turn rate.
    turn = turningTowards(turn_rate, loc)
    return transitingTo(loc, 100.0, turn)
end

function create_transit_to_leave_volume(
    lower::Vector{<:Real},
    upper::Vector{<:Real}
)::transitingUntil
    # Lower and upper are 3D vectors and are the bounds to the volume we are leaving.
    # Assumes we are facing the direction we want to go. Keeps going until volume is 
    # exited 

    return transitingUntil(
        (a) -> !is_in_volume([a.pos...], lower, upper)
    )
end

function get_pos_for_orbit(
    center::Point2D,
    radius::Real,
    θ::Real,
)::Point2D
    # Gets the position and direction of vehicle along an orbit
    # θ : The angle from the x-axis where the pos it placed along the circumference
    pos = Tuple([center...] + radius * [cos(θ), sin(θ)])
    return pos
end

function get_turn_against_vector(
    agent::Vehicle,
    v::Vector{<:Real},
    turn_rate::Real,
    relative_angle::Real
)::turning
    # Creates a turning state the will maneuver the agent to turn the opposite
    # direction of a vector. 
    ϕ = angle_between_2([agent.vel[1:2]...], v[1:2])
    cw = ϕ > 0
    θ = π - abs(ϕ) - relative_angle

    return turning((-1)^cw * abs(turn_rate), θ)
end