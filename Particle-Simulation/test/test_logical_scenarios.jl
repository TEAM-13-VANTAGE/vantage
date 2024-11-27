using Test

include("../src/main.jl")

function horizontal_test_wrapper(
    logical_scenario::Function,
    evaluator::Function,
    fixed_params::Dict{Symbol,Real}=Dict{Symbol,Real}()
)
    """
    Explores the four cardinal directions for a scenario. Returns true if passed.
    Tests:
        Headon (left)
        Converging (left)
        Overtaking (right)
        Converging (right)
        Headon (right)
        Overtaking (left)
    Evaluator takes in the following 
        scenario::Scenario,
        side::Direction,
            Which side of the helicopter the drone is on.
        initial_state::Dict{Symbol,Real}
            drone/heli_pos, drone/heli_vel
        final_state::Dict{Symbol,Real}
            drone/heli_pos, drone/heli_vel
    """
    response_distance = 10000.0
    d = 3000.0

    drone_speed = 30.0

    heli_pos = [6000.0, 6000.0, 200.0]
    heli_direcion = 0.0
    heli_speed = 100.0

    @testset "Scenario $(Symbol(logical_scenario)) Horizontal Tests" begin
        for (i, scenario) in zip(0:3, [headOn, converging, overtaking])
            for side in [left, right]
                θ = π / 2 * i * (side == left ? 1.0 : -1.0)
                drone_direction = θ + pi
                drone_pos = heli_pos + [
                    d * cos(θ),
                    d * sin(θ) + (side == left ? 50.0 : -50.0),
                    0.0
                ]

                adf, mdf = logical_scenario(
                    drone_response_distance=response_distance,
                    drone_speed=drone_speed,
                    heli_speed=heli_speed,
                    drone_x_pos=drone_pos[1],
                    drone_y_pos=drone_pos[2],
                    drone_direction=drone_direction,
                    drone_horizontal_turn_rate=pi / 8,
                    heli_x_pos=heli_pos[1],
                    heli_y_pos=heli_pos[2],
                    adata=[:pos, :vel, :role],
                    mdata=[:scenario]
                )

                heli_df = adf[adf.role.==heli, :]
                drone_df = adf[adf.role.==drone, :]

                initial_state = Dict(
                    :heli_pos => heli_df[1, :pos],
                    :heli_vel => heli_df[1, :vel],
                    :drone_pos => drone_df[1, :pos],
                    :drone_vel => drone_df[1, :vel],
                )
                end_state = Dict(
                    :heli_pos => heli_df[end, :pos],
                    :heli_vel => heli_df[end, :vel],
                    :drone_pos => drone_df[end, :pos],
                    :drone_vel => drone_df[end, :vel],
                )

                @test evaluator(scenario, side, initial_state, end_state)
            end
        end
    end

end

function standard_scene_horizontal_nonrow_evaluator(
    scenario::Scenario,
    side::Direction,
    initial_state::Dict,
    end_state::Dict,
)::Bool
    drone_vel = end_state[:drone_vel]

    if scenario ∈ [headOn, overtaking]
        target_θ = pi / 2 * (
            side == left ? 1 : -1
        )
    else
        target_θ = pi
    end

    θ = atan(drone_vel[2], drone_vel[1])
    println("($scenario, $side): Expected: $target_θ, Got: $θ")
    return ≈(θ, target_θ; atol=1e-15)
end

function test_round1_horizontal()
    horizontal_test_wrapper(round1_logical_scenario, standard_scene_horizontal_nonrow_evaluator)
end