using CSV
using DataFrames
using SHA
using JuMP
using Gurobi

if !@isdefined(LOADED_STACK_MAKING)
    const LOADED_STACK_MAKING = true
end

if !@isdefined(LOADED_STRUCTS) || (@isdefined(RELOAD) && RELOAD)
    include("structs.jl")
end

if !@isdefined(LOADED_UTILS) || (@isdefined(RELOAD) && RELOAD)
    include("utils.jl")
end

function initialize_model(; timeLimit::Union{Int, Nothing} = nothing, threads::Int = 1, memLimit::Float64 = 16.0)
    model = Model(Gurobi.Optimizer; with_cache_type = Float64)
    set_optimizer_attribute(model, "Threads", threads)
    set_optimizer_attribute(model, "SoftMemLimit", memLimit)
    set_optimizer_attribute(model, "NumericFocus", 1)
    if !isnothing(timeLimit)
        set_optimizer_attribute(model, "TimeLimit", timeLimit)
    end
    return model
end

function group_items(items::Vector{Item})
    groups = Vector{Vector{Item}}(undef, 0)

    for item in items
        index = findfirst(group -> 
            (item.stackableGroup === group[1].stackableGroup) && 
            (item.supplier === group[1].supplier) &&
            (item.plant === group[1].plant) && 
            (item.plantDock === group[1].plantDock) &&
            (item.supplierDock === group[1].supplierDock), groups)

        if index !== nothing
            push!(groups[index], item)
        else
            push!(groups, [item])
        end
    end

    return groups
end

function get_incompatibles(items::Vector{Item})
    incompatibles = Vector{Tuple{Item, Item}}(undef, 0)

    for i in eachindex(items)
        if items[i].forcedOrientation == NoneForced
            continue
        end

        for j in i+1:length(items)
            if (items[j].forcedOrientation !== NoneForced) &&
               (items[i].forcedOrientation !== items[j].forcedOrientation)
                
                push!(incompatibles, (items[i], items[j]))
            end
        end
    end

    return incompatibles
end

function get_possible_items_heur(
    remaining::Vector{Int},
    stack::Vector{Int},
    Inc::Vector{Tuple{Item, Item}},
    E::Vector{Int},
    hatE::Vector{Int},
    H::Vector{Int},
    hatH::Vector{Int},
    N::Vector{Int},
    Hmax::Int,
    Emax::Int,
    )
    
    if isempty(stack)
        return remaining
    else
        possible = [
            i
            for i in remaining
            if (sum(E[stack]) + E[i] <= Emax) &&
                (length(stack) == 1 ? (H[stack[1]] + H[i] - hatH[i] <= Hmax) : (H[stack[1]] + sum(H[stack[2:end]] - hatH[stack[2:end]]) + H[i] - hatH[i] <= Hmax)) &&
                (length(stack) + 1 <= min(N[i], minimum(N[stack]))) &&
                (sum(E[stack[2:end]]) + E[i] <= hatE[stack[1]]) &&
                !any([((i, j) in Inc) || ((j, i) in Inc) for j in stack])
        ]

        return possible
    end
end

function get_best_item_heur(
        possible::Vector{Int},
        remaining::Vector{Int},
        stack::Vector{Int},
        E::Vector{Int},
        hatE::Vector{Int},
        H::Vector{Int},
        hatH::Vector{Int},
        N::Vector{Int},
        Hmax::Int)

    E_rem  = sum(E[remaining])
    E_mean = E_rem / length(remaining)
    F      = [0 for _ in 1:length(E)]
    for i in remaining
        F[i] = min(hatE[i], floor((N[i]-1) * E_mean), E_rem)
    end

    if isempty(stack)
        maxF       = maximum(F[possible])
        candidates = [i for i in possible if F[i] == maxF]
        return length(candidates) == 1 ? candidates[1] : candidates[argmax(H[candidates])]
    end

    stackHeight = length(stack) == 1 ?
        H[stack[1]] :
        H[stack[1]] + sum(H[stack[2:end]] .- hatH[stack[2:end]])
    H_rem = Hmax - stackHeight

    ratio = ceil(Int, sum(H[i] - hatH[i] for i in possible) / H_rem)
    Nadj  = i -> min(N[i], ratio)

    N_min = minimum(Nadj(j) for j in stack)

    I3 = [i for i in possible if Nadj(i) >= N_min]

    select_from(cands) = begin
        minF   = minimum(F[cands])
        tied   = [i for i in cands if F[i] == minF]
        length(tied) == 1 ? tied[1] :
            tied[argmax(H[tied] .- hatH[tied])]
    end

    if !isempty(I3)
        return select_from(I3)
    else
        maxN   = maximum(Nadj(i) for i in possible)
        cands1 = [i for i in possible if Nadj(i) == maxN]
        return select_from(cands1)
    end
end

function stack_heuristic(
        n::Int,
        Inc::Vector{Tuple{Item, Item}},
        E::Vector{Int},
        hatE::Vector{Int},
        H::Vector{Int},
        hatH::Vector{Int},
        N::Vector{Int},
        Hmax::Int,
        Emax::Int)

    stacks    = [Vector{Int}(undef, 0)]
    remaining = collect(1:n)

    while !isempty(remaining)
        possible = get_possible_items_heur(
            remaining, stacks[end], Inc,
            E, hatE, H, hatH, N, Hmax, Emax)

        if isempty(possible)
            push!(stacks, Vector{Int}(undef, 0))
            continue
        end

        best = get_best_item_heur(
            possible, remaining, stacks[end],
            E, hatE, H, hatH, N, Hmax)

        deleteat!(remaining, findfirst(==(best), remaining))
        push!(stacks[end], best)
    end

    return stacks
end


function stacks_lower_bound(
    n::Int,
    E::Vector{Int},
    hatE::Vector{Int},
    H::Vector{Int},
    N::Vector{Int},
    Hmax::Int,
    Emax::Int,
    )
    
    return max(ceil(sum(E) / min(Emax, maximum(E + hatE))), ceil(sum(H) / Hmax), ceil(n / maximum(N)))
end

function get_first_variables(
    n::Int,
    Inc::Vector{Tuple{Item, Item}},
    E::Vector{Int},
    hatE::Vector{Int},
    H::Vector{Int},
    hatH::Vector{Int},
    N::Vector{Int},
    Hmax::Int,
    Emax::Int,
    )

    stacks = stack_heuristic(n, Inc, E, hatE, H, hatH, N, Hmax, Emax)
    lowerS = stacks_lower_bound(n, E, hatE, H - hatH, N, Hmax, Emax)
    upperS = length(stacks)
    uSol = ones(Int, upperS)
    aSol = [Int(i in stacks[s]) for i in 1:n, s in 1:upperS]
    bSol = [Int(i == stacks[s][1]) for i in 1:n, s in 1:upperS]

    return uSol, aSol, bSol, upperS, lowerS
end

function build_stack_from_items(stackItems::Vector{Item})
    stackLength = stackItems[1].length
    width = stackItems[1].width
    height = sum(item.height for item in stackItems)

    if length(stackItems) == 1
        realHeight = stackItems[1].height
    else
        realHeight = stackItems[1].height + sum(item.height - item.nestingHeight for item in stackItems[2:end])
    end


    weight = sum(item.weight for item in stackItems)

    forcedOrientation = NoneForced
    for item in stackItems
        if item.forcedOrientation != NoneForced
            forcedOrientation = item.forcedOrientation
            break
        end
    end

    supplier = stackItems[1].supplier
    supplierDock = stackItems[1].supplierDock
    plant = stackItems[1].plant
    plantDock = stackItems[1].plantDock
    earliestArrivalTime = maximum(item.earliestArrivalTime for item in stackItems)
    latestArrivalTime = minimum(item.latestArrivalTime for item in stackItems)

    return Stack(
        stackItems,
        stackLength,
        width,
        height,
        realHeight,
        weight,
        forcedOrientation,
        supplier,
        supplierDock,
        plant,
        plantDock,
        earliestArrivalTime,
        latestArrivalTime,
    )
end

function variables_to_stacks(items::Vector{Item}, u::Vector{Int}, a::Matrix{Int}, b::Matrix{Int})
    stacks = Vector{Stack}(undef, 0)

    for s in 1:length(u)
        if u[s] == 0
            continue
        end

        stackItems = Vector{Item}(undef, 0)

        for i in 1:length(items)
            if b[i, s] == 1
                push!(stackItems, items[i])
                break
            end
        end

        for i in 1:length(items)
            if (b[i, s] == 0) && (a[i, s] == 1)
                push!(stackItems, items[i])
            end
        end

        if isempty(stackItems)
            continue
        end

        push!(stacks, build_stack_from_items(stackItems))
    end
    
    return stacks
end

function get_stacks(items::Vector{Item}, truck::Truck; timeLimit::Int = 5, onlyStackHeuristic::Bool = false)
    I = 1:length(items)
    n = length(I)
    Inc = get_incompatibles(items)
    E = [items[i].weight for i in I]
    hatE = get_items_max_weight_above_grams(items, truck)
    H = [items[i].height for i in I]
    N = [items[i].maxStackability for i in I]
    hatH = [items[i].nestingHeight for i in I]
    Hmax = truck.height
    Emax = floor(Int, min(truck.maxWeight * 1000, truck.maxDensity * items[1].length * items[1].width / 1000))

    uSol, aSol, bSol, upperS, lowerS = get_first_variables(n, Inc, E, hatE, H, hatH, N, Hmax, Emax)
    S = 1:upperS

    if (upperS == lowerS) || onlyStackHeuristic
        return variables_to_stacks(items, uSol, aSol, bSol)
    end

    model = initialize_model(; timeLimit=timeLimit)
    
    @variable(model, u[S], Bin)
    @variable(model, a[I, S], Bin)
    @variable(model, b[I, S], Bin)

    @objective(model, Min, sum(u[s] for s in S))

    @constraint(model, oneAssigned[i in I], sum(a[i, s] for s in S) == 1)
    @constraint(model, bottomIfUsed[s in S], sum(b[i, s] for i in I) == u[s])
    @constraint(model, assignedIfBottom[i in I, s in S], b[i, s] <= a[i, s])
    @constraint(
        model,
        height[s in S],
        sum((H[i] - hatH[i]) * a[i, s] for i in I) + sum(hatH[i] * b[i, s] for i in I) <= Hmax * u[s]
    )
    @constraint(model, weight[s in S], sum(E[i] * a[i, s] for i in I) <= Emax * u[s])
    @constraint(
        model,
        weightAbove[s in S],
        sum(E[i] * a[i, s] for i in I) <= sum((hatE[i] + E[i]) * b[i, s] for i in I)
    )
    @constraint(model, stackability[i in I, s in S], sum(a[j, s] for j in I) <= N[i] + (n - N[i]) * (1 - a[i, s]))
    @constraint(model, incompatibles[i in I, j in I, s in S; (i, j) in Inc], a[i, s] + a[j, s] <= 1)

    for s in S
        set_start_value(u[s], uSol[s])
        for i in I
            set_start_value(a[i, s], aSol[i, s])
            set_start_value(b[i, s], bSol[i, s])
        end
    end

    optimize!(model)

    newUSol = [value(u[s]) > 0.5 ? 1 : 0 for s in S]

    if sum(newUSol) == upperS
        return variables_to_stacks(items, uSol, aSol, bSol)
    end

    newASol = [value(a[i, s]) > 0.5 ? 1 : 0 for i in I, s in S]
    newBSol = [value(b[i, s]) > 0.5 ? 1 : 0 for i in I, s in S]

    return variables_to_stacks(items, newUSol, newASol, newBSol)
end

function stack_items(items::Vector{Item}, truck::Truck; timeLimit::Int = 5, onlyStackHeuristic::Bool = false)
    itemGroups = group_items(items)
    stacks = Vector{Stack}(undef, 0)

    for itemGroup in itemGroups
        append!(stacks, get_stacks(itemGroup, truck; timeLimit = timeLimit, onlyStackHeuristic=onlyStackHeuristic))
    end

    return stacks
end
