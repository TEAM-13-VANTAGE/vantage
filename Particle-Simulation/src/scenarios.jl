# Scenarios are state machines.
# A state machine is just a function that takes in a state and executes a behavior
# on the agent, returning a state.

using Agents

function register_nearest_distance(agent::Vehicle, other::Vehicle)
    s = [other.pos...] - [agent.pos...]
    d = norm(s)
    exists = VAR_MIN_DIST ∈ keys(agent.vars)

    if !exists || exists && d < agent.vars[VAR_MIN_DIST]
        agent.vars[VAR_MIN_DIST] = d
        agent.vars[VAR_MIN_DISPL] = s
    end
end

function agent_start_step!(agent::Vehicle, model::StandardABM)
    agent.pos_log = []
end
function agent_end_step!(agent::Vehicle, model::StandardABM, neighbors::Vector{Vehicle})
    if agent.role == drone && length(neighbors) > 0
        # Only logging contacts from the perspective of the drone
        neighbor = first(neighbors)
        evaluate_contact(agent, neighbor, model)
        register_nearest_distance(agent, neighbor)
    end

    move_agent!(agent, model, dt)
    model.step += 1
end

# Tools
function response_required(
    agent::Vehicle, model::StandardABM, neighbors::Vector{Vehicle};
    listening_state=Union{idling,circling}
)::Bool
    return (
        !isa(agent.state, leaving) && # not necessary?
        agent.role == drone &&
        isa(agent.state, listening_state) &&
        !isnothing(model.scenario) &&
        length(neighbors) > 0
    )
end

function register_scenario(agent::Vehicle, model::StandardABM, neighbors)
    if model.scenario == noContact && length(neighbors) > 0
        scenario = determine_scenario(agent, neighbors[1])
        other_scenario = determine_scenario(neighbors[1], agent)

        # Scenario is from the perspective of the drone.
        # The drone's scenario gives RoW to heli 
        model.scenario = (
            scenario != ignoring ? scenario : other_scenario
        )
    end
end

function register_rac_pos(agent::Vehicle, model::StandardABM, rac::RAC)
    rac_region = get_rac_region([agent.pos...], rac)
    has_crossed = (
        VAR_RAC_REGION ∈ keys(agent.vars) ?
        rac_region != agent.vars[VAR_RAC_REGION] :
        false
    )

    if has_crossed
        push!(
            agent.pos_log,
            PositionLog(
                agent.id,
                agent.pos,
                model.step,
                rac_msg_from_region(rac_region)
            )
        )
    end

    agent.vars[VAR_RAC_REGION] = rac_region
end


include("scenarios/scene-archive.jl")
include("scenarios/scene-standard.jl")
include("scenarios/scene-sz.jl")
include("scenarios/scene-rac-row.jl")