# This module provides tools for visualizing the simulation state.
using Makie
using GLMakie
using MeshViz
using Meshes
using Agents
using DataFrames

function get_cyl(
    center;
    height=VIOLATION_DISTANCE_V * 2.0,
    radius=VIOLATION_DISTANCE_H,
)
    # Can be used to visualize view-distances.
    bottom = [center[1:2]..., center[3] - height / 2]
    top = [center[1:2]..., center[3] + height / 2]
    bottom, top = Tuple(x for x in bottom), Tuple(x for x in top)

    return Cylinder(
        bottom,
        top,
        radius
    )
end

function rac_to_rect(rac::RAC)
    # Used to visualize rectangular reserved airspaces.
    bounds = zip(rac.lower, rac.upper)
    vertices = Iterators.product(bounds...)
    return collect(zip(vertices...))
end

function viz_rac!(ax::Axis3, rac::RAC)
    l, h = [rac.lower...], [rac.upper...]
    viz!(
        ax,
        CartesianGrid(Tuple(l), Tuple(h)),
        alpha=0.25,
        color=:black
    )
    # Display a RAC
    if rac.buffer_x != 0.0 || rac.buffer_y != 0.0 || rac.buffer_z != 0.0
        l[1] -= rac.buffer_x
        l[2] -= rac.buffer_y
        l[3] -= rac.buffer_z
        h[1] += rac.buffer_x
        h[2] += rac.buffer_y
        h[3] += rac.buffer_z
        viz!(
            ax,
            CartesianGrid(Tuple(l), Tuple(h)),
            alpha=0.25,
            color=:gray
        )
    end
end

function interactable_animation(
    model::StandardABM,
    agent_step!,
    n::Int;
    timescale::Real=1.0,
    show_violation_region=false,
    rac::Optional{RAC}=nothing,
    response_distance::Optional{Float64}=nothing,
    pois=[]
)
    """
    Shows a graphics window of the transpiring simulation.
    # Arguments:
        model: The model to run the simulation with.
        agent_step!: The logical scenario to run.
        timescale: How fast to run the simulation. 1 means real-time.
        show_violation_region: Display a red cylinder around the Helicopter.
        rac: Display a RAC in the space.
        response_distance: Display the distance at which the drone begins its response.
        pois: Points of Interests, can be anything, such as drone path markers or safe zones.
    """
    GLMakie.activate!(inline=false)
    adf, mdf = run!(
        model,
        agent_step!,
        n;
        # adata=[:pos, :vel, :role]
        mdata=[:contactLevel, :contactEvents, :scenario],
        adata=[:pos_log, :vars, :pos, :vel, :role]
    )

    bounds_x, bounds_y, bounds_altitude = model.space.extent
    drone_points = Observable(adf[(adf.step.==1).&(adf.role.==drone), :pos])
    heli_points = Observable(adf[(adf.step.==1).&(adf.role.==heli), :pos])

    fig = Figure()
    ax = Axis3(fig[1, 1])
    scatter!(ax, drone_points, markersize=10)
    scatter!(ax, heli_points, markersize=20)

    if length(pois) > 0
        scatter!(ax, [Tuple(p) for p in pois], markersize=10, color=:black)
    end

    if show_violation_region
        viol_region = Observable(get_cyl(
            adf[(adf.step.==1).&(adf.role.==heli), :pos][1]
        ))
        viz!(ax, viol_region, shading=Makie.automatic, alpha=0.25, color=:red)
    end
    if !isnothing(response_distance)
        resp_region = Observable(get_cyl(
            adf[(adf.step.==1).&(adf.role.==drone), :pos][1];
            radius=response_distance,
            height=VIOLATION_DISTANCE_V
        ))
        viz!(ax, resp_region, shading=Makie.automatic, alpha=0.25, color=:cyan)
    end

    if !isnothing(rac)
        viz_rac!(ax, rac)
    end

    display(fig)
    limits!(ax, 0, bounds_x, 0, bounds_y, 0, bounds_altitude)
    for step_i in 1:n
        drone_points[] = adf[(adf.step.==step_i).&(adf.role.==drone), :pos]
        heli_points[] = adf[(adf.step.==step_i).&(adf.role.==heli), :pos]
        viol_region[] = get_cyl(adf[(adf.step.==step_i).&(adf.role.==heli), :pos][1])
        resp_region[] = get_cyl(
            adf[(adf.step.==step_i).&(adf.role.==drone), :pos][1];
            radius=response_distance,
            height=VIOLATION_DISTANCE_V * 0.9
        )
        sleep(dt / timescale)
    end

    return adf, mdf
end
