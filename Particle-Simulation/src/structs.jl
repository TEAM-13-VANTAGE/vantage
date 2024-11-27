
using Agents

const Optional{T} = Union{T,Nothing}
const Point3D = Tuple{Real,Real,Real}
const Point2D = Tuple{Real,Real}

const VIOLATION_DISTANCE_H = 2000.0 # feet
const VIOLATION_DISTANCE_V = 250.0 # feet
const COLLISION_DISTANCE = 36.0 # feet

const dt = 0.1 # seconds
const NDIM = 3

# space
const BOUNDS_X = 12000.0 # feet
const BOUNDS_Y = 12000.0 # feet
const BOUNDS_ALTITUDE = 3000.0 # feet
const BOUNDS = (BOUNDS_X, BOUNDS_Y, BOUNDS_ALTITUDE)

# directory

const PATH_CONFIG = "config.json"
const PATH_PARAMS_HEADON_N_H = "params/headon-norm-horizontal-params.csv"
const PATH_PARAMS_HEADON_A_H = "params/headon-adverse-horizontal-params.csv"
const PATH_PARAMS_HEADON_N_V = "params/headon-norm-vertical-params.csv"
const PATH_PARAMS_HEADON_A_V = "params/headon-adverse-vertical-params.csv"
const PATH_PARAMS_CONVERGING_N_H = "params/converging-norm-horizontal-params.csv"
const PATH_PARAMS_CONVERGING_A_H = "params/converging-adverse-horizontal-params.csv"
const PATH_PARAMS_CONVERGING_N_V = "params/converging-norm-vertical-params.csv"
const PATH_PARAMS_CONVERGING_A_V = "params/converging-adverse-vertical-params.csv"
const PATH_PARAMS_OVERTAKING_N_H = "params/overtaking-norm-horizontal-params.csv"
const PATH_PARAMS_OVERTAKING_A_H = "params/overtaking-adverse-horizontal-params.csv"
const PATH_PARAMS_OVERTAKING_N_V = "params/overtaking-norm-vertical-params.csv"
const PATH_PARAMS_OVERTAKING_A_V = "params/overtaking-adverse-vertical-params.csv"

# messages
## behaviors
const MSG_ASCENT_BEGAN = "ascent/descent began"
const MSG_TURN_BEGAN = "turn began"
const MSG_ASCENT_END = "ascent/descent ended"
const MSG_TURN_END = "turn ended"
## events
const MSG_RAC_ENTER = "RAC interior entered"
const MSG_RACBUFFER_ENTER = "RAC buffer entered"
const MSG_RACBUFFER_EXIT = "RAC exited"

# vars
const VAR_RAC_REGION = "rac_region"
const VAR_MIN_DIST = "min_dist"
const VAR_MIN_DISPL = "min_displ"

# types
abstract type State end

@enum Direction prograde retrograde left right
@enum Scenario headOn converging overtaking ignoring noContact
@enum ContactType none violation collision
@enum Role heli drone
@enum Region outside buffer margin inside

struct RAC
    lower::Vector{<:Real}
    upper::Vector{<:Real}
    buffer_x::Real
    buffer_y::Real
    buffer_z::Real
    RAC(
        lower::Vector{<:Real},
        upper::Vector{<:Real},
        buffer_x::Real,
        buffer_y::Real,
        buffer_z::Real,
    ) = new(lower, upper, buffer_x, buffer_y, buffer_z)
    RAC(
        lower::Vector{<:Real},
        upper::Vector{<:Real},
        buffer_h::Real,
        buffer_v::Real
    ) = new(lower, upper, buffer_h, buffer_h, buffer_v)
    RAC(
        lower::Vector{<:Real}, upper::Vector{<:Real}, buffer::Real
    ) = new(lower, upper, buffer, buffer, buffer)
    RAC(
        lower::Vector{<:Real}, upper::Vector{<:Real}
    ) = new(lower, upper, 0.0, 0.0, 0.0)
end

function RAC_by_dimensions(
    origin::Vector{<:Real}, dimensions::Vector{<:Real};
)::RAC
    return RAC(origin, origin + dimensions)
end

function RAC_by_dimensions(
    origin::Vector{<:Real}, dimensions::Vector{<:Real}, buffer::Real
)::RAC
    return RAC(origin, origin + dimensions, buffer, 0.0)
end

# An event that occurred for a given agent at a position and step (time).
struct PositionLog
    agent_id::Int
    pos::Point3D
    step::Int
    title::String
end

@agent Vehicle ContinuousAgent{NDIM} begin
    role::Role
    state::State
    scenario::Scenario
    pos_log::Vector{PositionLog}
    vars::Dict{String,Any} # Used by logical scenarios to track data for logging and behavior.
end

# When two vehicles enounter eachother (e.g. collision), these events are logged.
struct ContactEvent
    vehicles::Tuple{Vehicle,Vehicle}
    type::ContactType
    time::Real
end

# The properties to the model, i.e. model-wide state.
mutable struct Properties
    contactEvents::Array{ContactEvent}
    step::Int
    contactLevel::ContactType
    scenario::Scenario#Optional{Scenario}
    Properties() = new([], 0, none, noContact)
end

# Used to convert external units to internally used units.
# For each category (time, distance, angle), the internal 
#   unit is placed at the top (assigned a scale of 1.0).
units_to_std = Dict(
    # time
    "seconds" => 1.0,
    "s" => 1.0,
    "minutes" => 60.0,
    "min" => 60.0,
    "hours" => 60.0 * 60.0,
    "h" => 60.0 * 60.0,
    # distance
    "feet" => 1.0,
    "foot" => 1.0,
    "ft" => 1.0,
    "miles" => 5280,
    "mi" => 5280,
    "meters" => 3.28084,
    "m" => 3.28084,
    "kilometers" => 3280.84,
    "km" => 3280.84,
    # angle
    "radians" => 1.0,
    "degrees" => pi / 180,
    # rates 
    "fps" => 1.0,
    "mph" => 1.46667,
    # Unitless 
    "count" => 1.0,
    "index" => 1.0,
    "%" => 1.0,
    "flag" => 1,
    "bool" => 1,
    "boolean" => 1,
)

units_from_std = Dict(
    # time
    "seconds" => 1.0,
    "s" => 1.0,
    "minutes" => 1 / 60.0,
    "min" => 1 / 60.0,
    "hours" => 1 / (60.0 * 60.0),
    "h" => 1 / (60.0 * 60.0),
    # distance
    "feet" => 1.0,
    "ft" => 1.0,
    "miles" => 1 / 5280,
    "mi" => 1 / 5280,
    "meters" => 1 / 3.28084,
    "m" => 1 / 3.28084,
    "kilometers" => 1 / 3280.84,
    "km" => 1 / 3280.84,
    # angle
    "radians" => 1.0,
    "degrees" => 180 / pi,
    # rate
    "fps" => 1.0,
    "mph" => 0.681818,
    "count" => 1.0,
    "index" => 1.0,
    "N/A" => 1.0,
    "" => 1.0,
    "unitless" => 1.0,
    "%" => 1.0,
    "flag" => 1,
    "bool" => 1,
    "boolean" => 1,
)

supported_units = keys(units_to_std)