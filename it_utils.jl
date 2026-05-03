if !@isdefined(LOADED_IT_UTILS)
    const LOADED_IT_UTILS = true
end

if !@isdefined(LOADED_MILP_UTILS) || (@isdefined(RELOAD) && RELOAD)
    include("milp_utils.jl")
end

function get_items_loading_order(items::Vector{Item}, truck::Truck)
    loadingOrder = Dict{Item, Int}()

    supplierGroups = Dict{Supplier, Vector{Item}}()

    for item in items
        if !haskey(supplierGroups, item.supplier)
            supplierGroups[item.supplier] = Vector{Item}()
        end

        push!(supplierGroups[item.supplier], item)
    end
    
    supplierOrderMap = Dict{Supplier, Int}()

    for (i, supplierData) in enumerate(truck.orderedTruckSuppliersData)
        supplierOrderMap[supplierData.supplier] = i
    end
    
    sortedSuppliers = sort(collect(keys(supplierGroups)), 
                           by=supplier -> get(supplierOrderMap, supplier, typemax(Int)))
    
    currentOrder = 1
    
    for supplier in sortedSuppliers
        supplierItems = supplierGroups[supplier]
        supplierData = nothing

        for sd in truck.orderedTruckSuppliersData
            if sd.supplier == supplier
                supplierData = sd
                break
            end
        end

        dockGroups = Dict{SupplierDock, Vector{Item}}()

        for item in supplierItems
            if !haskey(dockGroups, item.supplierDock)
                dockGroups[item.supplierDock] = Vector{Item}()
            end
            push!(dockGroups[item.supplierDock], item)
        end
        
        dockOrderMap = Dict{SupplierDock, Int}()

        if supplierData !== nothing
            for (i, dock) in enumerate(supplierData.orderedSupplierDocks)
                dockOrderMap[dock] = i
            end
        end
        
        sortedDocks = sort(collect(keys(dockGroups)), 
                          by=dock -> get(dockOrderMap, dock, typemax(Int)))
        
        for dock in sortedDocks
            dockItems = dockGroups[dock]
            plantDockGroups = Dict{PlantDock, Vector{Item}}()

            for item in dockItems
                if !haskey(plantDockGroups, item.plantDock)
                    plantDockGroups[item.plantDock] = Vector{Item}()
                end
                push!(plantDockGroups[item.plantDock], item)
            end

            plantDockOrderMap = Dict{PlantDock, Int}()

            for (i, plantDock) in enumerate(truck.truckPlantData.orderedPlantDocks)
                plantDockOrderMap[plantDock] = i
            end
            
            sortedPlantDocks = sort(collect(keys(plantDockGroups)), 
                                   by=plantDock -> get(plantDockOrderMap, plantDock, typemax(Int)))
            
            for plantDock in sortedPlantDocks
                plantDockItems = plantDockGroups[plantDock]
                
                for item in plantDockItems
                    loadingOrder[item] = currentOrder
                end
                
                currentOrder += 1
            end
        end
    end
    
    return loadingOrder
end

function get_compatibles(items::Vector{Item}, truck::Truck)
    Omega = Vector{Vector{Int}}(undef, length(items))
    overq = get_items_max_weight_above_grams(items, truck)

    for i in eachindex(items)
        Omega[i] = Vector{Int}(undef, 0)

        for j in eachindex(items)
            if i == j
                continue
            end

            it = items[i]
            jt = items[j]

            if ((it.forcedOrientation == NoneForced) || (jt.forcedOrientation == NoneForced) ||
                (it.forcedOrientation == jt.forcedOrientation)) && (it.stackableGroup == jt.stackableGroup) &&
                (it.supplier == jt.supplier) && (it.plant == jt.plant) && (it.plantDock == jt.plantDock) &&
                (it.supplierDock == jt.supplierDock) && (it.height + jt.height - jt.nestingHeight <= truck.height) &&
                (it.weight + jt.weight <= min(truck.maxWeight*1000, truck.maxDensity*it.length*it.width/1000)) && 
                (jt.weight <= overq[i]) && (it.maxStackability >= 2) && (jt.maxStackability >= 2)

                push!(Omega[i], j)
            end
        end
    end

    return Omega
end

function get_unloaded_items(items::Vector{Item}, loadedTruck::LoadedTruck)
    stacks = [ls.stack for ls in loadedTruck.loadedStacks]
    loadedItems = vcat([s.items for s in stacks]...)

    unloadedItems = Vector{Item}(undef, 0)

    for item in items
        if item in loadedItems
            deleteat!(loadedItems, findfirst(x -> x === item, loadedItems))
            continue
        end

        push!(unloadedItems, item)
    end

    return unloadedItems
end

function get_same_item_groups(
    l::Vector{Int},
    w::Vector{Int},
    h::Vector{Int},
    hatH::Vector{Int},
    q::Vector{Int},
    hatq::Vector{Int},
    n::Vector{Int},
    Omega::Vector{Vector{Int}},
    o::Vector{Int},
    fl::Vector{Int},
    fw::Vector{Int},
)

    I = 1:length(l)

    sameItemGroups = Vector{Vector{Int}}()

    for i in I
        foundGroup = false

        for group in sameItemGroups
            j = group[1]

            if (l[i] == l[j]) && (w[i] == w[j]) && (h[i] == h[j]) &&
               (hatH[i] == hatH[j]) && (q[i] == q[j]) && (hatq[i] == hatq[j]) &&
               (n[i] == n[j]) && (o[i] == o[j]) &&
               (i in Omega[j]) && (j in Omega[i]) &&
               ((i in fl) == (j in fl)) && ((i in fw) == (j in fw))

                push!(group, i)
                foundGroup = true
                break
            end
        end

        if !foundGroup
            push!(sameItemGroups, [i])
        end
    end

    return sameItemGroups
end

function get_it_KV(l::Vector{Int}, w::Vector{Int}, hh::Vector{Int}, a::Vector{Int}, A::Int; timeLimit::Int = 600)
    I = 1:length(l)

    model = initialize_model(; timeLimit=timeLimit)
    set_silent(model)

    @variable(model, u[I], Bin)

    @objective(model, Max, sum(l[i] * w[i] * hh[i] * u[i] for i in I))

    @constraint(model, sum(a[i] * u[i] for i in I) <= A)

    optimize!(model)

    if !is_solved_and_feasible(model)
        @error "it KV model not solved to optimality"
        return 0
    end

    return objective_value(model)
end

function get_KHi(
    i::Int,
    Omega_i::Vector{Int},
    h::Vector{Int},
    hatH::Vector{Int},
    H::Int,
    q::Vector{Int},
    hatq::Vector{Int},
    n::Vector{Int};
    timeLimit::Int = 600,
)

    model = initialize_model(; timeLimit=timeLimit)
    set_silent(model)

    @variable(model, s[j in Omega_i], Bin)

    @objective(model, Max, h[i] + sum((h[j] - hatH[j]) * s[j] for j in Omega_i))

    @constraint(model, height_limit, h[i] + sum((h[j] - hatH[j]) * s[j] for j in Omega_i) <= H)
    @constraint(model, weight_above_limit, sum(q[j] * s[j] for j in Omega_i) <= hatq[i])
    @constraint(model, stackability_limit[j in Omega_i], 1 + sum(s[k] for k in Omega_i) <= min(n[i], n[j]) * s[j] + n[i] * (1 - s[j]))

    optimize!(model)

    if !is_solved_and_feasible(model)
        @error "KHi model not solved to optimality"
        return 0
    end

    return objective_value(model)
end

function get_hatKHi(
    i::Int,
    Omega_i::Vector{Int},
    h::Vector{Int},
    hatH::Vector{Int},
    H::Int,
    q::Vector{Int},
    hatq::Vector{Int},
    n::Vector{Int};
    timeLimit::Int = 600,
)

    model = initialize_model(; timeLimit=timeLimit)
    set_silent(model)

    @variable(model, s[j in Omega_i], Bin)

    @objective(model, Max, h[i] + sum(h[j] * s[j] for j in Omega_i))

    @constraint(model, height_limit, h[i] + sum((h[j] - hatH[j]) * s[j] for j in Omega_i) <= H)
    @constraint(model, weight_above_limit, sum(q[j] * s[j] for j in Omega_i) <= hatq[i])
    @constraint(model, stackability_limit[j in Omega_i], 1 + sum(s[k] for k in Omega_i) <= min(n[i], n[j]) * s[j] + n[i] * (1 - s[j]))

    optimize!(model)

    if !is_solved_and_feasible(model)
        @error "hatKHi model not solved to optimality"
        return 0
    end

    return objective_value(model)
end

function get_KQi(
    i::Int,
    Omega_i::Vector{Int},
    h::Vector{Int},
    hatH::Vector{Int},
    H::Int,
    q::Vector{Int},
    hatq::Vector{Int},
    n::Vector{Int};
    timeLimit::Int = 600,
)

    model = initialize_model(; timeLimit=timeLimit)
    set_silent(model)

    @variable(model, s[j in Omega_i], Bin)

    @objective(model, Max, sum(q[j] * s[j] for j in Omega_i))

    @constraint(model, height_limit, h[i] + sum((h[j] - hatH[j]) * s[j] for j in Omega_i) <= H)
    @constraint(model, weight_above_limit, sum(q[j] * s[j] for j in Omega_i) <= hatq[i])
    @constraint(model, stackability_limit[j in Omega_i], 1 + sum(s[k] for k in Omega_i) <= min(n[i], n[j]) * s[j] + n[i] * (1 - s[j]))

    optimize!(model)

    if !is_solved_and_feasible(model)
        @error "KQi model not solved to optimality"
        return 0
    end

    return objective_value(model)
end

function get_it_datapack(items::Vector{Item}, truck::Truck)
    loadingOrder = get_items_loading_order(items, truck)
    I = 1:length(items)
    L, W, H = truck.length, truck.width, truck.height
    Q = truck.maxWeight * 1000  # g
    D = truck.maxDensity / 1000  # g/mm^2
    l, w, h = [item.length for item in items], [item.width for item in items], [item.height for item in items]
    hatH = [item.nestingHeight for item in items]
    q = [item.weight for item in items]
    overq = get_items_max_weight_above_grams(items, truck)
    d = [D * l[i] * w[i] for i in I]
    hatq = [round(Int, min(overq[i], d[i] - q[i])) for i in I]
    n = [item.maxStackability for item in items]
    Omega = get_compatibles(items, truck)
    o = [loadingOrder[item] for item in items]
    O = 1:maximum(o)

    CJfh, EJeh, const_1, const_2, const_3 = get_consts(truck)

    minx, maxx, miny, maxy = get_min_max(items)

    fl, fw = get_fl_fw(items, truck)

    sameItemGroups = get_same_item_groups(l, w, h, hatH, q, hatq, n, Omega, o, fl, fw)

    KQ = round(Int, get_KQ(q, Q))

    KH = [round(Int, get_KHi(i, Omega[i], h, hatH, H, q, hatq, n)) for i in I]
    KQi = [round(Int, get_KQi(i, Omega[i], h, hatH, H, q, hatq, n)) for i in I]

    KL = round(Int, get_KL(l, w, fl, fw, L))
    KW = round(Int, get_KW(l, w, fl, fw, W))
    
    e = get_e(l, w, fl, fw, W)
    e_star = get_e(l, w, fl, fw, KW)

    a_star = zeros(Int, length(I))

    for i in I
        if i in fl
            a_star[i] = l[i] * e_star[i][1]
            continue
        elseif i in fw
            a_star[i] = w[i] * e_star[i][2]
            continue
        end

        a_star[i] = min(l[i] * e_star[i][1], w[i] * e_star[i][2])
    end

    KA = round(Int, get_KA(a_star, KL * KW))
    hatKH = [round(Int, get_hatKHi(i, Omega[i], h, hatH, H, q, hatq, n)) for i in I]
    KV = round(Int, get_it_KV(l, w, hatKH, a_star, KA))

    fit = [min(length(gr), floor(Int, KA / a_star[gr[1]])) for gr in sameItemGroups]

    KF = get_KF(miny, W)
    KS = [get_KS(i, o, miny, maxy) for i in I]
    KD = [get_KD(i, miny, minx, maxx, W) for i in I]

    caL = [get_caL(i, l, w, fl, fw) for i in I]

    datapack = Dict(
        :I => I,
        :L => L,
        :W => W,
        :H => H,
        :Q => Q,
        :l => l,
        :w => w,
        :h => h,
        :hatH => hatH,
        :q => q,
        :hatq => hatq,
        :n => n,
        :Omega => Omega,
        :o => o,
        :O => O,
        :CJfh => CJfh,
        :EJeh => EJeh,
        :const_1 => const_1,
        :const_2 => const_2,
        :const_3 => const_3,
        :minx => minx,
        :miny => miny,
        :fl => fl,
        :fw => fw,
        :sameItemGroups => sameItemGroups,
        :KQ => KQ,
        :KH => KH,
        :KQi => KQi,
        :KL => KL,
        :KW => KW,
        :e => e,
        :a_star => a_star,
        :KA => KA,
        :KV => KV,
        :fit => fit,
        :KF => KF,
        :KS => KS,
        :KD => KD,
        :caL => caL,
    )

    return datapack
end

function add_it_decision_variables!(model::Model, datapack)
    I = datapack[:I]
    Omega = datapack[:Omega]
    L, W = datapack[:L], datapack[:W]
    fl, fw = datapack[:fl], datapack[:fw]

    @variable(
        model,
        0 <= x[i in I] <= L
    )

    @variable(
        model,
        0 <= y[i in I] <= W
    )

    @variable(
        model,
        u[I],
        Bin
    )

    @constraint(
        model,
        x_u[i in I],
        x[i] <= L * u[i]
    )

    @constraint(
        model,
        y_u[i in I],
        y[i] <= W * u[i]
    )

    @variable(
        model,
        ul[I],
        Bin
    )

    @variable(
        model,
        uw[I],
        Bin
    )

    for i in fw
        fix(ul[i], 0; force = true)
    end

    for i in fl
        fix(uw[i], 0; force = true)
    end

    @constraint(
        model,
        u_ul_uw[i in I],
        u[i] == ul[i] + uw[i]
    )

    @variable(
        model,
        b[I],
        Bin
    )

    @variable(
        model,
        s[i in I, j in I; j in Omega[i]],
        Bin
    )

    return nothing
end

function add_it_principal_constraints!(model::Model, datapack)
    I = datapack[:I]
    l, w, h = datapack[:l], datapack[:w], datapack[:h]
    hatH = datapack[:hatH]
    q, hatq = datapack[:q], datapack[:hatq]
    n = datapack[:n]
    Omega = datapack[:Omega]
    L, W, H = datapack[:L], datapack[:W], datapack[:H]
    o = datapack[:o]
    x, y = model[:x], model[:y]
    u = model[:u]
    ul, uw = model[:ul], model[:uw]
    b, s = model[:b], model[:s]

    @constraint(
        model,
        s_b[i in I, j in I; j in Omega[i]],
        s[i, j] <= b[i]
    )

    @constraint(
        model,
        u_b_s[i in I],
        u[i] == b[i] + sum(s[j, i] for j in I if i in Omega[j])
    )

    @constraint(
        model,
        st_height[i in I],
        sum((h[j] - hatH[j]) * s[i, j] for j in Omega[i]) + h[i] * b[i] <= H * b[i]
    )

    @constraint(
        model,
        st_weight[i in I],
        sum(q[j] * s[i, j] for j in Omega[i]) <= hatq[i] * b[i]
    )

    @constraint(
        model,
        st_num[i in I, j in I; j in Omega[i]],
        b[i] + sum(s[i, k] for k in Omega[i]) <= min(n[i], n[j]) * s[i, j] + n[i] * (1 - s[i, j])
    )

    @constraint(
        model,
        truck_limits_x[i in I],
        x[i] + l[i]ul[i] + w[i]uw[i] <= L
    )

    @constraint(
        model,
        truck_limits_y[i in I],
        y[i] + w[i]ul[i] + l[i]uw[i] <= W
    )

    @variable(
        model,
        left[i in I, j in I; i != j],
        Bin
    )

    @variable(
        model,
        down[i in I, j in I; i != j],
        Bin
    )

    for i in I
        for j in I
            if o[i] > o[j]
                fix(left[i, j], 0; force = true)
            end
        end
    end

    @constraint(
        model,
        left_bi[i in I, j in I; i != j],
        left[i, j] <= b[i]
    )

    @constraint(
        model,
        left_bj[i in I, j in I; i != j],
        left[i, j] <= b[j]
    )

    @constraint(
        model,
        down_bi[i in I, j in I; i != j],
        down[i, j] <= b[i]
    )

    @constraint(
        model,
        down_bj[i in I, j in I; i != j],
        down[i, j] <= b[j]
    )

    @constraint(
        model,
        rel_pos[i in I, j in I; i < j],
        left[i, j] + left[j, i] + down[i, j] + down[j, i] >= b[i] + b[j] - 1
    )

    @constraint(
        model,
        x_left[i in I, j in I; i != j],
        x[i] + l[i] * ul[i] + w[i] * uw[i] <= x[j] + L * (1 - left[i, j])
    )

    @constraint(
        model,
        y_down[i in I, j in I; i != j],
        y[i] + w[i] * ul[i] + l[i] * uw[i] <= y[j] + W * (1 - down[i, j])
    )

    @constraint(
        model,
        same_x_1[i in I, j in I; j in Omega[i]],
        x[i] - x[j] <= L * (1 - s[i, j])
    )

    @constraint(
        model,
        same_x_2[i in I, j in I; j in Omega[i]],
        x[j] - x[i] <= L * (1 - s[i, j])
    )

    @constraint(
        model,
        same_or_l[i in I, j in I; j in Omega[i]],
        ul[i] <= ul[j] + (1 - s[i, j])
    )

    @constraint(
        model,
        same_or_w[i in I, j in I; j in Omega[i]],
        uw[i] <= uw[j] + (1 - s[i, j])
    )

    return nothing
end

function add_it_support_constraints!(model::Model, datapack)
    I = datapack[:I]
    l, w = datapack[:l], datapack[:w]
    L, W = datapack[:L], datapack[:W]
    o = datapack[:o]
    x, y = model[:x], model[:y]
    ul, uw = model[:ul], model[:uw]
    b = model[:b]
    left = model[:left]

    @variable(
        model,
        sup[i in I, j in I; i != j],
        Bin
    )

    for i in I
        for j in I
            if o[i] > o[j]
                fix(sup[i, j], 0; force = true)
            end
        end
    end

    @constraint(
        model,
        sup_bi[i in I, j in I; i != j],
        sup[i, j] <= b[i]
    )

    @constraint(
        model,
        sup_bj[i in I, j in I; i != j],
        sup[i, j] <= b[j]
    )

    @constraint(
        model,
        sup_left[i in I, j in I; i != j],
        sup[i, j] <= left[i, j]
    )

    @constraint(
        model,
        x_sup[i in I, j in I; i != j],
        x[i] + l[i] * ul[i] + w[i] * uw[i] + L * (1 - sup[i, j]) >= x[j]
    )

    @constraint(
        model,
        y_sup_1[i in I, j in I; i != j],
        y[i] <= y[j] + w[j] * ul[j] + l[j] * uw[j] + W * (1 - sup[i, j])
    )

    @constraint(
        model,
        y_sup_2[i in I, j in I; i != j],
        y[j] <= y[i] + w[i] * ul[i] + l[i] * uw[i] + W * (1 - sup[i, j])
    )

    @variable(
        model,
        front[I],
        Bin
    )

    @constraint(
        model,
        front_b[i in I],
        front[i] <= b[i]
    )

    @constraint(
        model,
        x_front[i in I],
        x[i] <= L * (1 - front[i])
    )

    @constraint(
        model,
        b_front_sup[i in I],
        b[i] <= front[i] + sum(sup[j, i] for j in I if i != j)
    )

    return nothing
end

function add_it_order_constraints!(model::Model, datapack)
    I = datapack[:I]
    o = datapack[:o]
    L = datapack[:L]
    x, u = model[:x], model[:u]

    @constraint(
        model,
        st_order[i in I, j in I; o[i] < o[j]],
        x[i] <= x[j] + L * (1 - u[j])
    )

    return nothing
end

function add_it_weight_constraints!(model::Model, datapack)
    I = datapack[:I]
    o = datapack[:o]
    O = datapack[:O]
    q = datapack[:q]
    Q = datapack[:Q]
    CJfh, EJeh = datapack[:CJfh], datapack[:EJeh]
    const_1, const_2, const_3 = datapack[:const_1], datapack[:const_2], datapack[:const_3]
    l, w = datapack[:l], datapack[:w]
    x, u = model[:x], model[:u]
    ul, uw = model[:ul], model[:uw]

    @constraint(
        model,
        truck_weight,
        sum(q[i] * u[i] for i in I) <= Q
    )

    @constraint(
        model,
        axis_1[ord in O],
        const_1 * sum(q[i] * u[i] for i in I if o[i] <= ord) - CJfh * sum(q[i] * (x[i] + (l[i] / 2) * ul[i] + (w[i] / 2) * uw[i]) for i in I if o[i] <= ord) <= const_2
    )

    @constraint(
        model,
        axis_2[ord in O],
        sum(q[i] * (x[i] + (l[i] / 2) * ul[i] + (w[i] / 2) * uw[i]) for i in I if o[i] <= ord) - EJeh * sum(q[i]u[i] for i in I if o[i] <= ord) <= const_3
    )

    return nothing
end

function add_it_objective!(model::Model, datapack)
    I = datapack[:I]
    l, w, h = datapack[:l], datapack[:w], datapack[:h]
    u = model[:u]

    @objective(
        model,
        Max,
        sum(l[i] * w[i] * h[i] * u[i] for i in I)
    )

    return nothing
end

function add_it_position_bounds!(model::Model, datapack)
    I = datapack[:I]
    Omega = datapack[:Omega]
    L, W = datapack[:L], datapack[:W]
    o = datapack[:o]
    minx, miny = datapack[:minx], datapack[:miny]
    x, y = model[:x], model[:y]
    u, s = model[:u], model[:s]
    front = model[:front]
    x_u, y_u = model[:x_u], model[:y_u]
    same_x_1, same_x_2 = model[:same_x_1], model[:same_x_2]
    x_front = model[:x_front]
    st_order = model[:st_order]

    for i in I
        set_upper_bound(x[i], L - minx[i])
        set_upper_bound(y[i], W - miny[i])
    end

    delete.(model, x_u)

    @constraint(
        model,
        x_u_min[i in I],
        x[i] <= (L - minx[i]) * u[i]
    )

    delete.(model, y_u)

    @constraint(
        model,
        y_u_min[i in I],
        y[i] <= (W - miny[i]) * u[i]
    )

    delete.(model, same_x_1)

    @constraint(
        model,
        same_x_1_min[i in I, j in I; j in Omega[i]],
        x[i] - x[j] <= (L - minx[i]) * (1 - s[i, j])
    )

    delete.(model, same_x_2)

    @constraint(
        model,
        same_x_2_min[i in I, j in I; j in Omega[i]],
        x[j] - x[i] <= (L - minx[j]) * (1 - s[i, j])
    )

    delete.(model, x_front)

    @constraint(
        model,
        x_front_min[i in I],
        x[i] <= (L - minx[i]) * (1 - front[i])
    )

    delete.(model, st_order)

    @constraint(
        model,
        st_order_min[i in I, j in I; o[i] < o[j]],
        x[i] <= x[j] + (L - minx[j]) * (1 - u[j])
    )

    return nothing
end

function add_it_khi_kqi_bounds!(model::Model, datapack)
    I = datapack[:I]
    Omega = datapack[:Omega]
    q, h, hatH = datapack[:q], datapack[:h], datapack[:hatH]
    KH, KQi = datapack[:KH], datapack[:KQi]
    b, s = model[:b], model[:s]
    st_height, st_weight = model[:st_height], model[:st_weight]

    delete.(model, st_height)

    @constraint(
        model,
        st_height_kh[i in I],
        sum((h[j] - hatH[j]) * s[i, j] for j in Omega[i]) + h[i] * b[i] <= KH[i] * b[i]
    )

    delete.(model, st_weight)

    @constraint(
        model,
        st_weight_kq[i in I],
        sum(q[j] * s[i, j] for j in Omega[i]) <= KQi[i] * b[i]
    )

    return nothing
end

function add_it_kl_bounds!(model::Model, datapack)
    I = datapack[:I]
    Omega = datapack[:Omega]
    l, w = datapack[:l], datapack[:w]
    o = datapack[:o]
    KL = datapack[:KL]
    minx = datapack[:minx]
    x, u = model[:x], model[:u]
    ul, uw = model[:ul], model[:uw]
    left = model[:left]
    s = model[:s]
    front = model[:front]
    sup = model[:sup]
    x_u_min = model[:x_u_min]
    truck_limits_x = model[:truck_limits_x]
    x_left = model[:x_left]
    same_x_1_min, same_x_2_min = model[:same_x_1_min], model[:same_x_2_min]
    x_sup = model[:x_sup]
    x_front_min = model[:x_front_min]
    st_order_min = model[:st_order_min]
    
    for i in I
        set_upper_bound(x[i], KL - minx[i])
    end

    delete.(model, x_u_min)

    @constraint(
        model,
        x_u_min_kl[i in I],
        x[i] <= (KL - minx[i]) * u[i]
    )

    delete.(model, truck_limits_x)

    @constraint(
        model,
        truck_limits_x_kl[i in I],
        x[i] + l[i] * ul[i] + w[i] * uw[i] <= KL
    )

    delete.(model, x_left)

    @constraint(
        model,
        x_left_kl[i in I, j in I; i != j],
        x[i] + l[i] * ul[i] + w[i] * uw[i] <= x[j] + KL * (1 - left[i, j])
    )

    delete.(model, same_x_1_min)

    @constraint(
        model,
        same_x_1_min_kl[i in I, j in I; j in Omega[i]],
        x[i] - x[j] <= (KL - minx[i]) * (1 - s[i, j])
    )

    delete.(model, same_x_2_min)

    @constraint(
        model,
        same_x_2_min_kl[i in I, j in I; j in Omega[i]],
        x[j] - x[i] <= (KL - minx[j]) * (1 - s[i, j])
    )

    delete.(model, x_sup)

    @constraint(
        model,
        x_sup_kl[i in I, j in I; i != j],
        x[i] + l[i] * ul[i] + w[i] * uw[i] + KL * (1 - sup[i, j]) >= x[j]
    )

    delete.(model, x_front_min)

    @constraint(
        model,
        x_front_min_kl[i in I],
        x[i] <= (KL - minx[i]) * (1 - front[i])
    )

    delete.(model, st_order_min)

    @constraint(
        model,
        st_order_min_kl[i in I, j in I; o[i] < o[j]],
        x[i] <= x[j] + (KL - minx[j]) * (1 - u[j])
    )

    return nothing
end

function add_it_e_bounds!(model::Model, datapack)
    I = datapack[:I]
    W = datapack[:W]
    e = datapack[:e]
    y = model[:y]
    ul, uw = model[:ul], model[:uw]
    down = model[:down]
    truck_limits_y = model[:truck_limits_y]
    y_down = model[:y_down]

    delete.(model, truck_limits_y)

    @constraint(
        model,
        truck_limits_y_e[i in I],
        y[i] + e[i][1] * ul[i] + e[i][2] * uw[i] <= W
    )
    
    delete.(model, y_down)

    @constraint(
        model,
        y_down_e[i in I, j in I; i != j],
        y[i] + e[i][1] * ul[i] + e[i][2] * uw[i] <= y[j] + W * (1 - down[i, j])
    )

    return nothing
end

function add_it_kq_bound!(model::Model, datapack)
    I = datapack[:I]
    q = datapack[:q]
    KQ = datapack[:KQ]
    u = model[:u]
    truck_weight = model[:truck_weight]

    delete.(model, truck_weight)

    @constraint(
        model,
        truck_weight_kq,
        sum(q[i] * u[i] for i in I) <= KQ
    )

    return nothing
end

function add_it_ka_bound!(model::Model, datapack)
    I = datapack[:I]
    a_star = datapack[:a_star]
    KA = datapack[:KA]
    b = model[:b]

    @constraint(
        model,
        truck_area_ka,
        sum(a_star[i] * b[i] for i in I) <= KA
    )

    return nothing
end

function add_it_kv_bound!(model::Model, datapack)
    I = datapack[:I]
    l, w, h = datapack[:l], datapack[:w], datapack[:h]
    KV = datapack[:KV]
    u = model[:u]

    @constraint(
        model,
        truck_volume_kv,
        sum(l[i] * w[i] * h[i] * u[i] for i in I) <= KV
    )

    return nothing
end

function add_it_principal_bounds!(model::Model, datapack)
    add_it_position_bounds!(model, datapack)
    add_it_khi_kqi_bounds!(model, datapack)
    add_it_kl_bounds!(model, datapack)
    add_it_e_bounds!(model, datapack)
    add_it_kq_bound!(model, datapack)
    add_it_ka_bound!(model, datapack)
    add_it_kv_bound!(model, datapack)

    return nothing
end

function it_crop_variables!(model::Model, datapack)
    Omega = datapack[:Omega]
    fit = datapack[:fit]
    b, s = model[:b], model[:s]
    sameItemGroups = datapack[:sameItemGroups]

    for g in eachindex(sameItemGroups)
        gr = sameItemGroups[g]

        for i in 2:fit[g]
            for j in 1:(i - 1)
                fix(s[gr[i], gr[j]], 0; force = true)
            end
        end

        for i in (fit[g] + 1):length(gr)
            fix(b[gr[i]], 0; force = true)

            for j in Omega[gr[i]]
                fix(s[gr[i], j], 0; force = true)
            end
        end
    end

    return nothing
end

function add_it_symmetry_break!(model::Model, datapack)
    sameItemGroups = datapack[:sameItemGroups]
    u, b = model[:u], model[:b]

    @constraint(
        model,
        sym_1[group in sameItemGroups, k in 1:length(group)-1],
        u[group[k+1]] <= u[group[k]]
    )

    @constraint(
        model,
        sym_2[group in sameItemGroups, k in 1:length(group)-1],
        b[group[k+1]] <= b[group[k]]
    )

    it_crop_variables!(model, datapack)

    return nothing
end

function add_it_kf_bound!(model::Model, datapack)
    I = datapack[:I]
    KF = datapack[:KF]
    front = model[:front]

    @constraint(
        model,
        kf,
        sum(front[i] for i in I) <= KF
    )

    return nothing
end

function add_it_ks_bounds!(model::Model, datapack)
    I = datapack[:I]
    KS = datapack[:KS]
    sup = model[:sup]

    @constraint(
        model,
        ks[i in I],
        sum(sup[i, j] for j in I if j != i) <= KS[i]
    )

    return nothing
end

function add_it_kd_bounds!(model::Model, datapack)
    I = datapack[:I]
    KD = datapack[:KD]
    down = model[:down]

    @constraint(
        model,
        kd_1[i in I],
        sum(down[i, j] for j in I if j != i) <= KD[i]
    )

    @constraint(
        model,
        kd_2[i in I],
        sum(down[j, i] for j in I if j != i) <= KD[i]
    )

    return nothing
end

function add_it_secondary_bounds!(model::Model, datapack)
    add_it_kf_bound!(model, datapack)
    add_it_ks_bounds!(model, datapack)
    add_it_kd_bounds!(model, datapack)

    return nothing
end

function add_it_X_discret!(model::Model, datapack)
    I = datapack[:I]
    caL = datapack[:caL]
    x = model[:x]

    @variable(
        model,
        f[i in I, d in caL[i]],
        Bin
    )

    @constraint(
        model,
        x_discret[i in I],
        sum(d * f[i, d] for d in caL[i]) == x[i]
    )

    return nothing
end

function _set_start_value_respecting_fix!(var::VariableRef, value::Real)
    if is_fixed(var)
        set_start_value(var, JuMP.fix_value(var))
    else
        set_start_value(var, Float64(value))
    end
    return nothing
end

function apply_it_warm_start_from_loaded_truck!(
    model::Model,
    items::Vector{Item},
    truck::Truck,
    loadedTruck::LoadedTruck,
)
    I = collect(eachindex(items))
    Omega = get_compatibles(items, truck)
    x, y = model[:x], model[:y]
    u = model[:u]
    ul, uw = model[:ul], model[:uw]
    b, s = model[:b], model[:s]

    for i in I
        _set_start_value_respecting_fix!(x[i], 0.0)
        _set_start_value_respecting_fix!(y[i], 0.0)
        _set_start_value_respecting_fix!(u[i], 0.0)
        _set_start_value_respecting_fix!(ul[i], 0.0)
        _set_start_value_respecting_fix!(uw[i], 0.0)
        _set_start_value_respecting_fix!(b[i], 0.0)
    end

    for i in I
        for j in Omega[i]
            _set_start_value_respecting_fix!(s[i, j], 0.0)
        end
    end

    code_to_indices = Dict{String, Vector{Int}}()
    for i in I
        code = items[i].code
        if !haskey(code_to_indices, code)
            code_to_indices[code] = Int[]
        end
        push!(code_to_indices[code], i)
    end

    used = falses(length(items))

    can_be_one(var::VariableRef) = !is_fixed(var) || JuMP.fix_value(var) > 0.5
    start_or_zero(var::VariableRef) = begin
        v = start_value(var)
        isnothing(v) ? 0.0 : Float64(v)
    end

    loadingOrder = get_items_loading_order(items, truck)
    l = [item.length for item in items]
    w = [item.width for item in items]
    h = [item.height for item in items]
    hatH = [item.nestingHeight for item in items]
    q = [item.weight for item in items]
    overq = get_items_max_weight_above_grams(items, truck)
    d = [truck.maxDensity * l[i] * w[i] for i in I]
    hatq = [round(Int, min(overq[i], d[i] - q[i])) for i in I]
    n = [item.maxStackability for item in items]
    o = [loadingOrder[item] for item in items]
    fl, fw = get_fl_fw(items, truck)
    sameItemGroups = get_same_item_groups(l, w, h, hatH, q, hatq, n, Omega, o, fl, fw)

    idx_to_group = zeros(Int, length(items))
    group_to_indices = Dict{Int, Vector{Int}}()
    code_to_groups = Dict{String, Vector{Int}}()
    for (g, gr) in enumerate(sameItemGroups)
        group_to_indices[g] = copy(gr)
        for idx in gr
            idx_to_group[idx] = g
            code = items[idx].code
            if !haskey(code_to_groups, code)
                code_to_groups[code] = Int[]
            end
            if !(g in code_to_groups[code])
                push!(code_to_groups[code], g)
            end
        end
    end

    function stack_candidate(
        base_idx::Int,
        upper_group_options::Vector{Vector{Int}},
        base_need_after_current::Dict{Int, Int},
    )
        can_be_one(b[base_idx]) || return nothing
        local_used = Set{Int}([base_idx])
        upper_idxs = Int[]

        function remaining_base_capable(group_id::Int, extra_used::Set{Int})
            cnt = 0
            for idx in get(group_to_indices, group_id, Int[])
                if used[idx] || (idx in extra_used)
                    continue
                end
                if can_be_one(b[idx])
                    cnt += 1
                end
            end
            return cnt
        end

        order = collect(eachindex(upper_group_options))
        sort!(order, by = pos -> begin
            cnt = 0
            seen = Set{Int}()
            for group_id in upper_group_options[pos]
                for idx in get(group_to_indices, group_id, Int[])
                    if idx in seen
                        continue
                    end
                    push!(seen, idx)
                    if used[idx] || (idx in local_used)
                        continue
                    end
                    if (idx in Omega[base_idx]) && can_be_one(s[base_idx, idx])
                        cnt += 1
                    end
                end
            end
            cnt
        end)

        for pos in order
            options = Int[]
            seen = Set{Int}()
            for group_id in upper_group_options[pos]
                for idx in get(group_to_indices, group_id, Int[])
                    if idx in seen
                        continue
                    end
                    push!(seen, idx)
                    if used[idx] || (idx in local_used)
                        continue
                    end
                    if (idx in Omega[base_idx]) && can_be_one(s[base_idx, idx])
                        push!(options, idx)
                    end
                end
            end
            isempty(options) && return nothing

            sort!(options, by = idx -> (can_be_one(b[idx]) ? 1 : 0, -idx))

            chosen = nothing
            for idx in options
                if can_be_one(b[idx])
                    group_id = idx_to_group[idx]
                    extra_used = Set(local_used)
                    push!(extra_used, idx)
                    base_cap_left = remaining_base_capable(group_id, extra_used)
                    required = get(base_need_after_current, group_id, 0)
                    if base_cap_left < required
                        continue
                    end
                end
                chosen = idx
                break
            end

            chosen === nothing && return nothing
            push!(upper_idxs, chosen)
            push!(local_used, chosen)
        end

        return upper_idxs
    end

    stack_specs = Vector{NamedTuple{(:sid, :base_code, :base_group_options, :upper_codes, :upper_group_options, :is_lengthwise, :xo, :yo), Tuple{Int, String, Vector{Int}, Vector{String}, Vector{Vector{Int}}, Bool, Float64, Float64}}}()
    for (sid, loadedStack) in enumerate(loadedTruck.loadedStacks)
        stack_codes = [item.code for item in loadedStack.stack.items]
        isempty(stack_codes) && continue
        if !haskey(code_to_groups, stack_codes[1])
            @warn "Warm-start stack skipped: base code not present in model groups" stack_id=sid base_code=stack_codes[1]
            continue
        end

        upper_group_options = Vector{Vector{Int}}()
        missing_upper_codes = String[]
        for code in stack_codes[2:end]
            if haskey(code_to_groups, code)
                push!(upper_group_options, code_to_groups[code])
            else
                push!(missing_upper_codes, code)
            end
        end
        if !isempty(missing_upper_codes)
            @warn "Warm-start stack skipped: upper code not present in model groups" stack_id=sid base_code=stack_codes[1] missing_upper_codes=missing_upper_codes
            continue
        end

        push!(
            stack_specs,
            (
                sid = sid,
                base_code = stack_codes[1],
                base_group_options = code_to_groups[stack_codes[1]],
                upper_codes = stack_codes[2:end],
                upper_group_options = upper_group_options,
                is_lengthwise = loadedStack.orientation == LengthWise,
                xo = Float64(loadedStack.xo),
                yo = Float64(loadedStack.yo),
            )
        )
    end

    ordered_specs = sort(stack_specs, by = spec -> (spec.xo, spec.yo, spec.sid))
    pending_base_need = Dict{Int, Int}()
    for spec in ordered_specs
        if length(spec.base_group_options) == 1
            group_id = spec.base_group_options[1]
            pending_base_need[group_id] = get(pending_base_need, group_id, 0) + 1
        end
    end

    for spec in ordered_specs
        base_need_after_current = copy(pending_base_need)
        if length(spec.base_group_options) == 1
            group_id = spec.base_group_options[1]
            base_need_after_current[group_id] = get(base_need_after_current, group_id, 0) - 1
        end

        candidates = Tuple{Int, Vector{Int}}[]
        base_candidates = Int[]
        seen_bases = Set{Int}()
        for group_id in spec.base_group_options
            for base_idx in get(group_to_indices, group_id, Int[])
                if base_idx in seen_bases
                    continue
                end
                push!(seen_bases, base_idx)
                push!(base_candidates, base_idx)
            end
        end

        for base_idx in base_candidates
            if used[base_idx]
                continue
            end
            upper_idxs = stack_candidate(base_idx, spec.upper_group_options, base_need_after_current)
            upper_idxs === nothing && continue
            push!(candidates, (base_idx, upper_idxs))
        end

        pending_base_need = base_need_after_current

        if isempty(candidates)
            @warn "Warm-start stack skipped: no feasible mapping after fixed-variable filtering" stack_id=spec.sid base_code=spec.base_code base_group_options=spec.base_group_options upper_codes=spec.upper_codes x=spec.xo y=spec.yo
            continue
        end

        sort!(candidates, by = c -> c[1])
        base_idx, upper_idxs = candidates[1]

        used[base_idx] = true
        _set_start_value_respecting_fix!(b[base_idx], 1.0)

        active_indices = Int[base_idx]
        for idx in upper_idxs
            used[idx] = true
            _set_start_value_respecting_fix!(s[base_idx, idx], 1.0)
            push!(active_indices, idx)
        end

        for idx in active_indices
            _set_start_value_respecting_fix!(u[idx], 1.0)
            _set_start_value_respecting_fix!(x[idx], spec.xo)
            _set_start_value_respecting_fix!(y[idx], spec.yo)
            _set_start_value_respecting_fix!(ul[idx], spec.is_lengthwise ? 1.0 : 0.0)
            _set_start_value_respecting_fix!(uw[idx], spec.is_lengthwise ? 0.0 : 1.0)
        end
    end

    function recompute_u_from_bs!()
        for i in I
            rhs = start_or_zero(b[i])
            for j in I
                if i in Omega[j]
                    rhs += start_or_zero(s[j, i])
                end
            end
            _set_start_value_respecting_fix!(u[i], rhs > 0.5 ? 1.0 : 0.0)
        end
        return nothing
    end

    function canonicalize_group_starts!(group::Vector{Int})
        group_set = Set(group)
        used_in_group = [i for i in group if start_or_zero(u[i]) > 0.5]
        isempty(used_in_group) && return

        base_in_group = [i for i in used_in_group if start_or_zero(b[i]) > 0.5]
        nonbase_in_group = [i for i in used_in_group if start_or_zero(b[i]) <= 0.5]
        unused_in_group = [i for i in group if start_or_zero(u[i]) <= 0.5]

        src_order = vcat(base_in_group, nonbase_in_group, unused_in_group)
        if length(src_order) != length(group)
            @warn "Warm-start canonicalization skipped: inconsistent group cardinality" group=group
            return
        end

        src_to_dst = Dict(src_order[k] => group[k] for k in eachindex(group))
        dst_to_src = Dict(group[k] => src_order[k] for k in eachindex(group))

        old_x = Dict(i => start_or_zero(x[i]) for i in group)
        old_y = Dict(i => start_or_zero(y[i]) for i in group)
        old_u = Dict(i => start_or_zero(u[i]) for i in group)
        old_ul = Dict(i => start_or_zero(ul[i]) for i in group)
        old_uw = Dict(i => start_or_zero(uw[i]) for i in group)
        old_b = Dict(i => start_or_zero(b[i]) for i in group)

        old_s = Dict{Tuple{Int, Int}, Float64}()
        for i in I
            for j in Omega[i]
                if (i in group_set) || (j in group_set)
                    old_s[(i, j)] = start_or_zero(s[i, j])
                end
            end
        end

        for ((i, j), _) in old_s
            _set_start_value_respecting_fix!(s[i, j], 0.0)
        end

        for dst in group
            src = dst_to_src[dst]
            _set_start_value_respecting_fix!(x[dst], old_x[src])
            _set_start_value_respecting_fix!(y[dst], old_y[src])
            _set_start_value_respecting_fix!(u[dst], old_u[src])
            _set_start_value_respecting_fix!(ul[dst], old_ul[src])
            _set_start_value_respecting_fix!(uw[dst], old_uw[src])
            _set_start_value_respecting_fix!(b[dst], old_b[src])
        end

        for ((i, j), v) in old_s
            if v <= 0.5
                continue
            end
            i2 = haskey(src_to_dst, i) ? src_to_dst[i] : i
            j2 = haskey(src_to_dst, j) ? src_to_dst[j] : j
            if j2 in Omega[i2]
                _set_start_value_respecting_fix!(s[i2, j2], v)
            else
                @warn "Warm-start canonicalization dropped incompatible arc after permutation" from=(i, j) to=(i2, j2)
            end
        end

        return nothing
    end

    for group in sameItemGroups
        canonicalize_group_starts!(group)
    end

    recompute_u_from_bs!()

    left = model[:left]
    down = model[:down]
    sup = model[:sup]
    front = model[:front]
    L = truck.length

    function span_x(i::Int)
        return l[i] * start_or_zero(ul[i]) + w[i] * start_or_zero(uw[i])
    end

    function span_y(i::Int)
        return w[i] * start_or_zero(ul[i]) + l[i] * start_or_zero(uw[i])
    end

    function can_activate(var::VariableRef)
        return !is_fixed(var) || JuMP.fix_value(var) > 0.5
    end

    tol = 1e-6

    function try_activate_rel!(var::VariableRef, condition::Bool)
        if condition && can_activate(var)
            _set_start_value_respecting_fix!(var, 1.0)
            return true
        end
        return false
    end

    function build_aux_starts!()
        for i in I
            _set_start_value_respecting_fix!(front[i], 0.0)
            for j in I
                if i == j
                    continue
                end
                _set_start_value_respecting_fix!(left[i, j], 0.0)
                _set_start_value_respecting_fix!(down[i, j], 0.0)
                _set_start_value_respecting_fix!(sup[i, j], 0.0)
            end
        end
    
        loaded = [i for i in I if start_or_zero(b[i]) > 0.5]

        for i_pos in 1:length(loaded)-1
            i = loaded[i_pos]
            xi = start_or_zero(x[i])
            yi = start_or_zero(y[i])
            dxi = span_x(i)
            dyi = span_y(i)

            for j_pos in i_pos+1:length(loaded)
                j = loaded[j_pos]
                xj = start_or_zero(x[j])
                yj = start_or_zero(y[j])
                dxj = span_x(j)
                dyj = span_y(j)

                sep_x_ij = xi + dxi <= xj + tol
                sep_x_ji = xj + dxj <= xi + tol
                sep_y_ij = yi + dyi <= yj + tol
                sep_y_ji = yj + dyj <= yi + tol

                ok = false
                ok |= try_activate_rel!(left[i, j], sep_x_ij)
                ok |= try_activate_rel!(left[j, i], !ok && sep_x_ji)
                ok |= try_activate_rel!(down[i, j], !ok && sep_y_ij)
                ok |= try_activate_rel!(down[j, i], !ok && sep_y_ji)

                if !ok
                    ok |= try_activate_rel!(left[i, j], xi <= xj + tol)
                    ok |= try_activate_rel!(left[j, i], !ok)
                    ok |= try_activate_rel!(down[i, j], !ok && yi <= yj + tol)
                    ok |= try_activate_rel!(down[j, i], !ok)
                end
            end
        end

        for i in loaded
            xi = start_or_zero(x[i])
            yi = start_or_zero(y[i])
            dyi = span_y(i)

            if (xi <= tol) && can_activate(front[i])
                _set_start_value_respecting_fix!(front[i], 1.0)
                continue
            end

            supporter = nothing
            best_x = -Inf
            for j in loaded
                i == j && continue
                if start_or_zero(left[j, i]) < 0.5
                    continue
                end
                can_activate(sup[j, i]) || continue

                xj = start_or_zero(x[j])
                dxj = span_x(j)
                yj = start_or_zero(y[j])
                dyj = span_y(j)

                touch_x = abs((xj + dxj) - xi) <= tol
                overlap_y = (yi <= yj + dyj + tol) && (yj <= yi + dyi + tol)
                if touch_x && overlap_y && xj > best_x
                    best_x = xj
                    supporter = j
                end
            end

            if supporter !== nothing
                _set_start_value_respecting_fix!(sup[supporter, i], 1.0)
            elseif can_activate(front[i]) && (xi <= L * 1e-4)
                _set_start_value_respecting_fix!(front[i], 1.0)
            end
        end

        return loaded
    end

    loaded = build_aux_starts!()
    unsupported = Int[]
    for i in loaded
        support = start_or_zero(front[i]) + sum(start_or_zero(sup[j, i]) for j in I if j != i)
        if support < 0.5
            push!(unsupported, i)
        end
    end
    if !isempty(unsupported)
        sample = unsupported[1:min(end, 10)]
        @warn "Warm-start support graph incomplete after mapping" unsupported_bases=length(unsupported) sample_indices=sample
    end

    for i in I
        ui = start_or_zero(u[i])
        uli = start_or_zero(ul[i])
        uwi = start_or_zero(uw[i])
        if ui < 0.5
            _set_start_value_respecting_fix!(x[i], 0.0)
            _set_start_value_respecting_fix!(y[i], 0.0)
            _set_start_value_respecting_fix!(ul[i], 0.0)
            _set_start_value_respecting_fix!(uw[i], 0.0)
        else
            su = uli + uwi
            if su < 0.5
                if can_activate(ul[i]) && !(is_fixed(uw[i]) && JuMP.fix_value(uw[i]) > 0.5)
                    _set_start_value_respecting_fix!(ul[i], 1.0)
                    _set_start_value_respecting_fix!(uw[i], 0.0)
                else
                    _set_start_value_respecting_fix!(ul[i], 0.0)
                    _set_start_value_respecting_fix!(uw[i], 1.0)
                end
            elseif su > 1.5
                if can_activate(ul[i]) && !(is_fixed(uw[i]) && JuMP.fix_value(uw[i]) > 0.5)
                    _set_start_value_respecting_fix!(ul[i], 1.0)
                    _set_start_value_respecting_fix!(uw[i], 0.0)
                else
                    _set_start_value_respecting_fix!(ul[i], 0.0)
                    _set_start_value_respecting_fix!(uw[i], 1.0)
                end
            end
        end
    end

    return nothing
end

function extract_loaded_truck_from_it_model(model::Model, items::Vector{Item}, truck::Truck)
    Omega = get_compatibles(items, truck)
    x_vals = value.(model[:x])
    y_vals = value.(model[:y])
    ul_vals = value.(model[:ul])
    uw_vals = value.(model[:uw])
    b_vals = value.(model[:b])
    s_vals = value.(model[:s])

    loadedStacks = Vector{LoadedStack}()

    for i in eachindex(items)
        if b_vals[i] > 0.5
            stackItems = [items[i]]
            length = items[i].length
            width = items[i].width
            height = items[i].height
            realHeight = items[i].height
            weight = items[i].weight
            forcedOrientation = items[i].forcedOrientation
            supplier = items[i].supplier
            supplierDock = items[i].supplierDock
            plant = items[i].plant
            plantDock = items[i].plantDock
            earliestArrivalTime = items[i].earliestArrivalTime
            latestArrivalTime = items[i].latestArrivalTime

            for j in Omega[i]
                if s_vals[i, j] > 0.5
                    push!(stackItems, items[j])
                    height += items[j].height
                    realHeight += items[j].height - items[j].nestingHeight
                    weight += items[j].weight

                    if items[j].forcedOrientation !== NoneForced
                        forcedOrientation = items[j].forcedOrientation
                    end

                    if items[j].earliestArrivalTime > earliestArrivalTime
                        earliestArrivalTime = items[j].earliestArrivalTime
                    end

                    if items[j].latestArrivalTime < latestArrivalTime
                        latestArrivalTime = items[j].latestArrivalTime
                    end
                end
            end

            stack = Stack(
                stackItems,
                length,
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
                latestArrivalTime
            )

            xo = x_vals[i]
            yo = y_vals[i]
            xe = x_vals[i] + length * (ul_vals[i] > 0.5 ? 1 : 0) + width * (uw_vals[i] > 0.5 ? 1 : 0)
            ye = y_vals[i] + width * (ul_vals[i] > 0.5 ? 1 : 0) + length * (uw_vals[i] > 0.5 ? 1 : 0)
            orientation = (ul_vals[i] > 0.5) ? LengthWise : WidthWise

            loadedStack = LoadedStack(
                stack,
                round(Int, xo),
                round(Int, yo),
                round(Int, xe),
                round(Int, ye),
                orientation
            )

            push!(loadedStacks, loadedStack)
        end
    end

    return LoadedTruck(truck, loadedStacks)
end
