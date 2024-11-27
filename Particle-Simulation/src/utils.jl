
using JSON

function get_vel(θ::Real, ϕ::Real, v::Real)::Tuple{Real,Real,Real}
    return (v * cos(ϕ) * cos(θ), v * cos(ϕ) * sin(θ), v * sin(ϕ))
end

function get_config()::Dict
    f = open(PATH_CONFIG, "r")
    d = JSON.parse(f)
    close(f)

    return d
end

function lowerbuffer(rac::RAC)::Vector{<:Real}
    return rac.lower - [rac.buffer_x, rac.buffer_y, rac.buffer_z]
end

function upperbuffer(rac::RAC)::Vector{<:Real}
    return rac.upper + [rac.buffer_x, rac.buffer_y, rac.buffer_z]
end

function turn_rate_to_radius(turn_rate::Real, speed::Real)::Real
    return speed / turn_rate
end

function is_in_volume(p::Vector{<:Real}, lower::Vector{<:Real}, upper::Vector{<:Real})
    return all([l <= v <= h for (v, l, h) in zip(p, lower, upper)])
end

# function is_in_volume(p::Tuple{<:Real}, lower::Vector{<:Real}, upper::Vector{<:Real})
#     return is_in_volume([p...], lower, upper)
# end

function get_rac_region(p::Vector{<:Real}, rac::RAC)::Region
    if is_in_volume(p, rac.lower, rac.upper)
        region = inside
    elseif is_in_volume(p, lowerbuffer(rac), upperbuffer(rac))
        region = buffer
    else
        region = outside
    end

    return region
end
# function get_rac_region(p::Tuple{<:Real,3}, rac::RAC)::Region
#     return get_rac_region([p...], rac)
# end

function rac_msg_from_region(region::Region)::String
    if region == inside
        return MSG_RAC_ENTER
    elseif region == buffer
        return MSG_RACBUFFER_ENTER
    else
        return MSG_RACBUFFER_EXIT
    end
end



function nearest_point(p::Vector{<:Real}, cloud::Vector{<:Vector{<:Real}})
    min_d = norm(cloud[1] - p)
    i = 1
    for (j, p_i) in enumerate(cloud[2:end])
        tmp = norm(p_i - p)
        if tmp < min_d
            min_d = tmp
            i = j + 1
        end
    end

    return i
end


function angle_between_2(
    from::Vector{<:Real},
    to::Vector{<:Real}
)
    if length(from) != 2 || length(to) != 2
        throw(ArgumentError("Both vectors must be 2D vectors"))
    end
    θ1 = atan(from[2], from[1])
    θ2 = atan(to[2], to[1])
    Δθ = θ2 - θ1
    if Δθ > π
        Δθ -= 2π
    elseif Δθ < -π
        Δθ += 2π
    end
    return Δθ
end

function angle_between(
    v1::Vector{<:Real},
    v2::Vector{<:Real}
)
    l1, l2 = norm(v1), norm(v2)
    v1 /= l1
    v2 /= l2
    return v1 == v2 ? 0.0 : acos(clamp(dot(v1, v2), -1, 1))
end

function get_relative_travel_direction(v1::Vector{<:Real}, v2::Vector{<:Real})::Direction
    angle = angle_between_2(v1[1:2], v2[1:2])

    if angle >= -pi / 4 && angle < pi / 4
        direction = prograde
    elseif angle >= pi / 4 && angle < 3 * pi / 4
        direction = left
    elseif abs(angle) >= 3 * pi / 4 && angle <= pi
        direction = retrograde
    else
        direction = right
    end

    return direction
end

function rotation_matrix(θ::Real, φ::Real)
    # Define rotation matrices
    Rθ = [cos(θ) -sin(θ) 0;
        sin(θ) cos(θ) 0;
        0 0 1]

    Rφ = [cos(φ) 0 sin(φ);
        0 1 0;
        -sin(φ) 0 cos(φ)]

    return Rθ * Rφ
end

function determine_scenario(self::Vehicle, other::Vehicle)::Scenario
    """
    Determines which scenario @self is in relative to @other. 
    Does not account for distance, only what type of scenario is occurring if 
    they were in range of a necessary procedure.
    """
    direction = get_relative_travel_direction([self.vel...], [other.vel...])
    if direction == left
        return converging
    elseif direction == prograde && self.vel > other.vel
        return overtaking
    elseif direction == retrograde
        return headOn
    else
        return ignoring
    end
end

function get_contact_type(self::Vehicle, other::Vehicle, model)::ContactType
    distance = euclidean_distance(self, other, model)
    p1 = [self.pos...]
    p2 = [other.pos...]
    v_dist = abs(p2[3] - p1[3])
    h_dist = norm(p2[1:2] - p1[1:2])


    if distance <= COLLISION_DISTANCE
        return collision
    elseif h_dist <= VIOLATION_DISTANCE_H && v_dist <= VIOLATION_DISTANCE_V
        return violation
    else
        return none
    end
end

function position_log_to_dict(l::PositionLog)::Dict{<:Symbol}
    return Dict(
        :agent_id => l.agent_id,
        :pos => l.pos,
        :step => l.step,
        :title => l.title,
    )
end

function get_sz_from_rac_and_path(
    lower::Vector{<:Real},
    upper::Vector{<:Real},
    p0::Vector{<:Real},
    θ::Real,
    min_distance::Real
)::Vector{<:Vector{<:Real}}
    # find opposite corners 
    potential = Iterators.product(zip(lower, upper)...)

    midpoint = (upper - lower) / 2 + lower

    safe_zones = [
        [sz...] for sz in potential
        if distance_from_path([sz[1:2]...], p0[1:2], θ) > min_distance
    ]

    append!(
        safe_zones,
        [
            sz for sz in [
                [midpoint[1], lower[2]],
                [midpoint[1], upper[2]],
                [lower[1], midpoint[2]],
                [upper[1], midpoint[2]],
            ] if distance_from_path([sz[1:2]...], p0[1:2], θ) > min_distance
        ]
    )

    return safe_zones
end


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