if !@isdefined(LOADED_IT_MODEL)
    const LOADED_IT_MODEL = true
end

if !@isdefined(LOADED_IT_UTILS) || (@isdefined(RELOAD) && RELOAD)
    include("it_utils.jl")
end

function get_it_model(
    items::Vector{Item},
    truck::Truck;
    principalBounds::Bool = true,
    symmetryBreaking::Bool = true,
    secondaryBounds::Bool = false,
    xDiscret::Bool = false,
    baseModel::Bool = false,
)

    datapack = get_it_datapack(items, truck)

    model = initialize_model()

    add_it_decision_variables!(model, datapack)
    add_it_principal_constraints!(model, datapack)
    add_it_support_constraints!(model, datapack)
    add_it_order_constraints!(model, datapack)
    add_it_weight_constraints!(model, datapack)
    add_it_objective!(model, datapack)

    if !baseModel
        if principalBounds
            add_it_principal_bounds!(model, datapack)
        end

        if symmetryBreaking
            add_it_symmetry_break!(model, datapack)
        end

        if secondaryBounds
            add_it_secondary_bounds!(model, datapack)
        end

        if xDiscret
            add_it_X_discret!(model, datapack)
        end
    end
    
    return model
end

function solve_it(
    items::Vector{Item},
    truck::Truck;
    timeLimit::Int = 60,
    principalBounds::Bool = false,
    symmetryBreaking::Bool = false,
    secondaryBounds::Bool = false,
    xDiscret::Bool = false,
    baseModel::Bool = false,
    onlyFirst::Bool = true,
    warmStartLoadedTrucks::Union{Nothing, Vector{LoadedTruck}} = nothing,
)
    time_0 = time_ns()

    model = get_it_model(
        items,
        truck;
        principalBounds = principalBounds,
        symmetryBreaking = symmetryBreaking,
        secondaryBounds = secondaryBounds,
        xDiscret = xDiscret,
        baseModel = baseModel
    )
    if !isnothing(warmStartLoadedTrucks) && !isempty(warmStartLoadedTrucks)
        apply_it_warm_start_from_loaded_truck!(model, items, truck, warmStartLoadedTrucks[1])
    end

    time_1 = time_ns()
    elapsed_1 = (time_1 - time_0) / 1e9
    set_optimizer_attribute(model, "TimeLimit", float(timeLimit) - elapsed_1)
    optimize!(model)
    time_2 = time_ns()
    elapsed_2 = (time_2 - time_0) / 1e9
    loadedTruck = extract_loaded_truck_from_it_model(model, items, truck)
    unloadedItems = get_unloaded_items(items, loadedTruck)
    loadedTrucks = [loadedTruck]
    truck_volume = truck.length * truck.width * truck.height
    if isempty(loadedTruck.loadedStacks)
        vol_fill_rate = 0.0
    else
        vol_fill_rate = sum(ls.stack.length * ls.stack.width * ls.stack.height for ls in loadedTruck.loadedStacks) / truck_volume
    end
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

    while !isempty(unloadedItems) && !isempty(loadedTrucks[end].loadedStacks) && !onlyFirst
        extraTruck = get_extra_truck(truck, i)
        time_0 = time_ns()
        modelExtra = get_it_model(
            unloadedItems,
            extraTruck;
            principalBounds = principalBounds,
            symmetryBreaking = symmetryBreaking,
            secondaryBounds = secondaryBounds,
            xDiscret = xDiscret,
            baseModel = baseModel
        )
        if !isnothing(warmStartLoadedTrucks) && (length(warmStartLoadedTrucks) >= i + 1)
            apply_it_warm_start_from_loaded_truck!(modelExtra, unloadedItems, extraTruck, warmStartLoadedTrucks[i + 1])
        end
        time_1 = time_ns()
        elapsed_1 = (time_1 - time_0) / 1e9
        set_optimizer_attribute(modelExtra, "TimeLimit", float(timeLimit) - elapsed_1)
        optimize!(modelExtra)
        time_2 = time_ns()
        elapsed_2 = (time_2 - time_0) / 1e9
        loadedTruckExtra = extract_loaded_truck_from_it_model(modelExtra, unloadedItems, extraTruck)
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
        unloadedItems = get_unloaded_items(unloadedItems, loadedTruckExtra)
        i += 1
    end

    return loadedTrucks, runData
end
