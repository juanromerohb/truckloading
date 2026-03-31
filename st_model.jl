if !@isdefined(LOADED_ST_MODEL)
    const LOADED_ST_MODEL = true
end

if !@isdefined(LOADED_ST_UTILS) || (@isdefined(RELOAD) && RELOAD)
    include("st_utils.jl")
end

function get_st_model(
    stacks::Vector{Stack},
    truck::Truck;
    principalBounds::Bool = true,
    symmetryBreaking::Bool = true,
    secondaryBounds::Bool = false,
    xDiscret::Bool = false,
    yDiscret::Bool = false,
    baseModel::Bool = false,
)
    datapack = get_st_datapack(stacks, truck)

    model = initialize_model()

    add_st_decision_variables!(model, datapack)
    add_st_principal_constraints!(model, datapack)
    add_st_support_constraints!(model, datapack)
    add_st_order_constraints!(model, datapack)
    add_st_weight_constraints!(model, datapack)
    add_st_objective!(model, datapack)

    if !baseModel
        if principalBounds
            add_st_principal_bounds!(model, datapack)
        end

        if symmetryBreaking
            add_st_symmetry_break!(model, datapack)
        end

        if secondaryBounds
            add_st_secondary_bounds!(model, datapack)
        end

        if xDiscret
            add_st_X_discret!(model, datapack)
        end

        if yDiscret
            add_st_Y_discret!(model, datapack)
        end
    end

    return model
end

function solve_st(
    items::Vector{Item},
    truck::Truck;
    timeLimit::Int = 60,
    principalBounds::Bool = true,
    symmetryBreaking::Bool = true,
    secondaryBounds::Bool = false,
    xDiscret::Bool = false,
    yDiscret::Bool = false,
    baseModel::Bool = false,
    onlyFirst::Bool = true,
    onlyStackHeuristic::Bool = false,
)
    time_0 = time_ns()
    stacks = stack_items(items, truck; onlyStackHeuristic=onlyStackHeuristic)
    model = get_st_model(
        stacks,
        truck;
        principalBounds = principalBounds,
        symmetryBreaking = symmetryBreaking,
        secondaryBounds = secondaryBounds,
        xDiscret = xDiscret,
        yDiscret = yDiscret,
        baseModel = baseModel
    )
    time_1 = time_ns()
    elapsed_1 = (time_1 - time_0) / 1e9
    set_optimizer_attribute(model, "TimeLimit", float(timeLimit) - elapsed_1)
    optimize!(model)
    time_2 = time_ns()
    elapsed_2 = (time_2 - time_0) / 1e9
    loadedTruck = extract_loaded_truck_from_st_model(model, stacks, truck)
    unloadedStacks = get_unloaded_stacks(stacks, loadedTruck)
    loadedTrucks = [loadedTruck]
    truck_volume = truck.length * truck.width * truck.height
    vol_fill_rate = sum(ls.stack.length * ls.stack.width * ls.stack.height for ls in loadedTruck.loadedStacks) /
                    truck_volume
    best_bound = objective_bound(model)
    best_bound_volume_fill_rate = best_bound / truck_volume
    runData = [Dict("termination_status" => string(termination_status(model)),
                   "primal_status" => string(primal_status(model)),
                   "objective_value" => objective_value(model),
                   "gap" => MOI.get(model, MOI.RelativeGap()),
                   "solve_time" => elapsed_2,
                   "best_bound" => best_bound,
                   "best_bound_volume_fill_rate" => best_bound_volume_fill_rate,
                   "volume_fill_rate" => vol_fill_rate)]

    i = 1
    while !isempty(unloadedStacks) && !isempty(loadedTrucks[end].loadedStacks) && !onlyFirst
        extraTruck = get_extra_truck(truck, i)
        time_0 = time_ns()
        modelExtra = get_st_model(
            unloadedStacks,
            extraTruck;
            principalBounds = principalBounds,
            symmetryBreaking = symmetryBreaking,
            secondaryBounds = secondaryBounds,
            xDiscret = xDiscret,
            yDiscret = yDiscret,
            baseModel = baseModel
        )
        time_1 = time_ns()
        elapsed_1 = (time_1 - time_0) / 1e9
        set_optimizer_attribute(modelExtra, "TimeLimit", float(timeLimit) - elapsed_1)
        optimize!(modelExtra)
        time_2 = time_ns()
        elapsed_2 = (time_2 - time_0) / 1e9
        loadedTruckExtra = extract_loaded_truck_from_st_model(modelExtra, unloadedStacks, extraTruck)
        push!(loadedTrucks, loadedTruckExtra)
        vol_fill_rate = sum(ls.stack.length * ls.stack.width * ls.stack.height for ls in loadedTruckExtra.loadedStacks) /
                    truck_volume
        best_bound_extra = objective_bound(modelExtra)
        best_bound_volume_fill_rate_extra = (isfinite(best_bound_extra) && truck_volume > 0) ? best_bound_extra / truck_volume : nothing
        push!(runData, Dict("termination_status" => string(termination_status(modelExtra)),
                            "primal_status" => string(primal_status(modelExtra)),
                            "objective_value" => objective_value(modelExtra),
                            "gap" => MOI.get(modelExtra, MOI.RelativeGap()),
                            "solve_time" => elapsed_2,
                            "best_bound" => best_bound_extra,
                            "best_bound_volume_fill_rate" => best_bound_volume_fill_rate_extra,
                            "volume_fill_rate" => vol_fill_rate))
        unloadedStacks = get_unloaded_stacks(unloadedStacks, loadedTruckExtra)
        i += 1
    end

    return loadedTrucks, runData
end