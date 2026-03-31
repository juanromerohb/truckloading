if !@isdefined(LOADED_ST_UTILS)
    const LOADED_ST_UTILS = true
end

if !@isdefined(LOADED_MILP_UTILS) || (@isdefined(RELOAD) && RELOAD)
    include("milp_utils.jl")
end

if !@isdefined(LOADED_STACK_MAKING) || (@isdefined(RELOAD) && RELOAD)
    include("stack_making.jl")
end

function get_unloaded_stacks(stacks::Vector{Stack}, loadedTruck::LoadedTruck)
    loadedStacks = [ls.stack for ls in loadedTruck.loadedStacks]
    unloadedStacks = Vector{Stack}(undef, 0)

    for stack in stacks
        if stack in loadedStacks
            deleteat!(loadedStacks, findfirst(x -> x === stack, loadedStacks))
            continue
        end

        push!(unloadedStacks, stack)
    end

    return unloadedStacks
end

function get_stacks_loading_order(stacks::Vector{Stack}, truck::Truck)
    loadingOrder = Dict{Stack, Int}()
    
    supplierGroups = Dict{Supplier, Vector{Stack}}()
    for stack in stacks
        if !haskey(supplierGroups, stack.supplier)
            supplierGroups[stack.supplier] = Vector{Stack}()
        end

        push!(supplierGroups[stack.supplier], stack)
    end
    
    supplierOrderMap = Dict{Supplier, Int}()

    for (i, supplierData) in enumerate(truck.orderedTruckSuppliersData)
        supplierOrderMap[supplierData.supplier] = i
    end
    
    sortedSuppliers = sort(collect(keys(supplierGroups)), 
                           by=supplier -> get(supplierOrderMap, supplier, typemax(Int)))
    
    currentOrder = 1
    
    for supplier in sortedSuppliers
        supplierStacks = supplierGroups[supplier]
        
        supplierData = nothing
        for sd in truck.orderedTruckSuppliersData
            if sd.supplier == supplier
                supplierData = sd
                break
            end
        end
        
        dockGroups = Dict{SupplierDock, Vector{Stack}}()

        for stack in supplierStacks
            if !haskey(dockGroups, stack.supplierDock)
                dockGroups[stack.supplierDock] = Vector{Stack}()
            end
            push!(dockGroups[stack.supplierDock], stack)
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
            dockStacks = dockGroups[dock]
            plantDockGroups = Dict{PlantDock, Vector{Stack}}()

            for stack in dockStacks
                if !haskey(plantDockGroups, stack.plantDock)
                    plantDockGroups[stack.plantDock] = Vector{Stack}()
                end

                push!(plantDockGroups[stack.plantDock], stack)
            end
            
            plantDockOrderMap = Dict{PlantDock, Int}()

            for (i, plantDock) in enumerate(truck.truckPlantData.orderedPlantDocks)
                plantDockOrderMap[plantDock] = i
            end
            
            sortedPlantDocks = sort(collect(keys(plantDockGroups)), 
                                   by=plantDock -> get(plantDockOrderMap, plantDock, typemax(Int)))
            
            for plantDock in sortedPlantDocks

                plantDockStacks = plantDockGroups[plantDock]
                
                for stack in plantDockStacks
                    loadingOrder[stack] = currentOrder
                end
                
                currentOrder += 1
            end
        end
    end
    
    return loadingOrder
end

function get_same_stack_groups(l::Vector{Int}, w::Vector{Int}, h::Vector{Int}, fl::Vector{Int}, fw::Vector{Int}, q::Vector{Int}, o::Vector{Int})
    S = 1:length(l)

    sameStackGroups = Vector{Vector{Int}}()

    for s in S
        foundGroup = false
        
        for group in sameStackGroups
            r = group[1]

            if (l[s] == l[r]) && (w[s] == w[r]) && (h[s] == h[r]) && (q[s] == q[r]) &&
               ((s in fl) == (r in fl)) && ((s in fw) == (r in fw)) && (o[s] == o[r])

                push!(group, s)
                foundGroup = true
                break
            end
        end

        if !foundGroup
            push!(sameStackGroups, [s])
        end
    end

    return sameStackGroups
end

function get_st_KA(
    l::Vector{Int}, w::Vector{Int}, fl::Vector{Int}, fw::Vector{Int}, e_star::Vector{Tuple{Int, Int}},
    KL::Int, KW::Int;
    timeLimit::Int = 600
)
    I = 1:length(l)

    model = initialize_model(; timeLimit=timeLimit)
    set_silent(model)

    @variable(model, ul[I], Bin)
    @variable(model, uw[I], Bin)

    @objective(model, Max, sum(l[i] * e_star[i][1] * ul[i] + w[i] * e_star[i][2] * uw[i] for i in I))

    @constraint(model, u_lig_ul_uw[i in I], ul[i] + uw[i] <= 1)
    @constraint(model, forced_lengthwise[i in fl], uw[i] == 0)
    @constraint(model, forced_widthwise[i in fw], ul[i] == 0)
    @constraint(model, max_A, sum(l[i] * e_star[i][1] * ul[i] + w[i] * e_star[i][2] * uw[i] for i in I) <= KL * KW)

    optimize!(model)

    if !is_solved_and_feasible(model)
        @error "KA model not solved to optimality"
        return 0
    end

    return objective_value(model)
end

function get_st_KV(
    l::Vector{Int}, w::Vector{Int}, e_star::Vector{Tuple{Int, Int}}, h::Vector{Int},
    fl::Vector{Int}, fw::Vector{Int}, KL::Int, KW::Int;
    timeLimit::Int = 600
)
    I = 1:length(l)

    model = initialize_model(; timeLimit=timeLimit)
    set_silent(model)

    maxH = maximum(h)

    @variable(model, ul[I], Bin)
    @variable(model, uw[I], Bin)

    @objective(model, Max, sum(l[i] * e_star[i][1] * h[i] * ul[i] + w[i] * e_star[i][2] * h[i] * uw[i] for i in I))

    @constraint(model, u_lig_ul_uw[i in I], ul[i] + uw[i] <= 1)
    @constraint(model, forced_lengthwise[i in fl], uw[i] == 0)
    @constraint(model, forced_widthwise[i in fw], ul[i] == 0)
    @constraint(model, max_A, sum(l[i] * e_star[i][1] * ul[i] + w[i] * e_star[i][2] * uw[i] for i in I) <= KL * KW)
    @constraint(model, max_V, sum(l[i] * e_star[i][1] * h[i] * ul[i] + w[i] * e_star[i][2] * h[i] * uw[i] for i in I) <= KL * KW * maxH)

    optimize!(model)

    if !is_solved_and_feasible(model)
        @error "KV model not solved to optimality"
        return 0
    end

    return objective_value(model)
end

function get_st_datapack(stacks::Vector{Stack}, truck::Truck)
    loadingOrder = get_stacks_loading_order(stacks, truck)
    I = 1:length(stacks)
    L, W, H = truck.length, truck.width, truck.height
    Q = truck.maxWeight * 1000
    l, w, h = [stacks[i].length for i in I], [stacks[i].width for i in I], [stacks[i].height for i in I]
    q = [stacks[i].weight for i in I]
    o = [loadingOrder[stacks[i]] for i in I]
    O = 1:maximum(o)

    CJfh, EJeh, const_1, const_2, const_3 = get_consts(truck)

    minx, maxx, miny, maxy = get_min_max(stacks)

    fl, fw = get_fl_fw(stacks, truck)

    sameStackGroups = get_same_stack_groups(l, w, h, fl, fw, q, o)

    KQ = round(Int, get_KQ(q, Q))

    KL = round(Int, get_KL(l, w, fl, fw, L))
    KW = round(Int, get_KW(l, w, fl, fw, W))

    e_star = get_e(l, w, fl, fw, KW)

    a_star = zeros(Int, length(l))

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

    KA = round(Int, get_st_KA(l, w, fl, fw, e_star, KL, KW))
    KV = round(Int, get_st_KV(l, w, e_star, h, fl, fw, KL, KW))

    fit = [min(length(gr), floor(Int, KA / a_star[gr[1]])) for gr in sameStackGroups]

    KF = get_KF(miny, W)
    KS = [get_KS(i, o, miny, maxy) for i in I]
    KD = [get_KD(i, miny, minx, maxx, W) for i in I]

    caL = [get_caL(i, l, w, fl, fw) for i in I]
    caW = [get_caW(i, l, w, fl, fw) for i in I]

    datapack = Dict(
        :I => I,
        :L => L,
        :W => W,
        :H => H,
        :Q => Q,
        :l => l,
        :w => w,
        :h => h,
        :q => q,
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
        :sameStackGroups => sameStackGroups,
        :KQ => KQ,
        :KL => KL,
        :KW => KW,
        :e_star => e_star,
        :a_star => a_star,
        :KA => KA,
        :KV => KV,
        :fit => fit,
        :KF => KF,
        :KS => KS,
        :KD => KD,
        :caL => caL,
        :caW => caW,
    )

    return datapack
end

function add_st_decision_variables!(model::Model, datapack)
    I = datapack[:I]
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

    for i in fl
        fix(uw[i], 0; force = true)
    end

    for i in fw
        fix(ul[i], 0; force = true)
    end

    @constraint(
        model,
        u_ul_uw[i in I],
        u[i] == ul[i] + uw[i]
    )

    return nothing
end

function add_st_principal_constraints!(model::Model, datapack)
    I = datapack[:I]
    l, w = datapack[:l], datapack[:w]
    L, W = datapack[:L], datapack[:W]
    o = datapack[:o]
    x, y = model[:x], model[:y]
    u = model[:u]
    ul, uw = model[:ul], model[:uw]

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
        left_ui[i in I, j in I; i != j],
        left[i, j] <= u[i]
    )

    @constraint(
        model,
        left_uj[i in I, j in I; i != j],
        left[i, j] <= u[j]
    )

    @constraint(
        model,
        down_ui[i in I, j in I; i != j],
        down[i, j] <= u[i]
    )

    @constraint(
        model,
        down_uj[i in I, j in I; i != j],
        down[i, j] <= u[j]
    )

    @constraint(
        model,
        rel_pos[i in I, j in I; i < j],
        left[i, j] + left[j, i] + down[i, j] + down[j, i] >= u[i] + u[j] - 1
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

    return nothing
end

function add_st_support_constraints!(model::Model, datapack)
    I = datapack[:I]
    l, w = datapack[:l], datapack[:w]
    L, W = datapack[:L], datapack[:W]
    o = datapack[:o]
    x, y = model[:x], model[:y]
    ul, uw = model[:ul], model[:uw]
    u = model[:u]
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
        sup_ui[i in I, j in I; i != j],
        sup[i, j] <= u[i]
    )

    @constraint(
        model,
        sup_uj[i in I, j in I; i != j],
        sup[i, j] <= u[j]
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
        front_u[i in I],
        front[i] <= u[i]
    )

    @constraint(
        model,
        x_front[i in I],
        x[i] <= L * (1 - front[i])
    )

    @constraint(
        model,
        u_front_sup[i in I],
        u[i] <= front[i] + sum(sup[j, i] for j in I if i != j)
    )

    return nothing
end

function add_st_order_constraints!(model::Model, datapack)
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

function add_st_weight_constraints!(model::Model, datapack)
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

function add_st_objective!(model::Model, datapack)
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

function add_st_position_bounds!(model::Model, datapack)
    I = datapack[:I]
    L, W = datapack[:L], datapack[:W]
    o = datapack[:o]
    minx, miny = datapack[:minx], datapack[:miny]
    x, y = model[:x], model[:y]
    u = model[:u]
    front = model[:front]
    x_u, y_u = model[:x_u], model[:y_u]
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

function add_st_kl_bounds!(model::Model, datapack)
    I = datapack[:I]
    l, w = datapack[:l], datapack[:w]
    o = datapack[:o]
    KL = datapack[:KL]
    minx = datapack[:minx]
    x, u = model[:x], model[:u]
    ul, uw = model[:ul], model[:uw]
    left = model[:left]
    front = model[:front]
    sup = model[:sup]
    x_u_min = model[:x_u_min]
    truck_limits_x = model[:truck_limits_x]
    x_left = model[:x_left]
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

function add_st_kw_bounds!(model::Model, datapack)
    I = datapack[:I]
    l, w = datapack[:l], datapack[:w]
    o = datapack[:o]
    KW = datapack[:KW]
    miny = datapack[:miny]
    y, u = model[:y], model[:u]
    ul, uw = model[:ul], model[:uw]
    down = model[:down]
    sup = model[:sup]
    y_u_min = model[:y_u_min]
    truck_limits_y = model[:truck_limits_y]
    y_down = model[:y_down]
    y_sup_1, y_sup_2 = model[:y_sup_1], model[:y_sup_2]
    
    for i in I
        set_upper_bound(y[i], KW - miny[i])
    end

    delete.(model, y_u_min)

    @constraint(
        model,
        y_u_min_kw[i in I],
        y[i] <= (KW - miny[i]) * u[i]
    )

    delete.(model, truck_limits_y)

    @constraint(
        model,
        truck_limits_y_kw[i in I],
        y[i] + l[i] * ul[i] + w[i] * uw[i] <= KW
    )

    delete.(model, y_down)

    @constraint(
        model,
        y_down_kw[i in I, j in I; i != j],
        y[i] + l[i] * ul[i] + w[i] * uw[i] <= y[j] + KW * (1 - down[i, j])
    )

    delete.(model, y_sup_1)

    @constraint(
        model,
        y_sup_1_kw[i in I, j in I; i != j],
        y[i] <= y[j] + w[j] * ul[j] + l[j] * uw[j] + KW * (1 - sup[i, j])
    )

    delete.(model, y_sup_2)

    @constraint(
        model,
        y_sup_2_kw[i in I, j in I; i != j],
        y[j] <= y[i] + w[i] * ul[i] + l[i] * uw[i] + KW * (1 - sup[i, j])
    )

    return nothing
end

function add_st_e_star_bounds!(model::Model, datapack)
    I = datapack[:I]
    KW = datapack[:KW]
    e_star = datapack[:e_star]
    y = model[:y]
    ul, uw = model[:ul], model[:uw]
    down = model[:down]
    truck_limits_y_kw = model[:truck_limits_y_kw]
    y_down_kw = model[:y_down_kw]

    delete.(model, truck_limits_y_kw)

    @constraint(
        model,
        truck_limits_y_kw_e[i in I],
        y[i] + e_star[i][1] * ul[i] + e_star[i][2] * uw[i] <= KW
    )

    delete.(model, y_down_kw)

    @constraint(
        model,
        y_down_kw_e[i in I, j in I; i != j],
        y[i] + e_star[i][1] * ul[i] + e_star[i][2] * uw[i] <= y[j] + KW * (1 - down[i, j])
    )

    return nothing
end

function add_st_kq_bound!(model::Model, datapack)
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

function add_st_ka_bound!(model::Model, datapack)
    I = datapack[:I]
    l, w, e_star = datapack[:l], datapack[:w], datapack[:e_star]
    KA = datapack[:KA]
    ul, uw = model[:ul], model[:uw]

    @constraint(
        model,
        truck_area_ka,
        sum((l[i] * e_star[i][1] * ul[i] + w[i] * e_star[i][2] * uw[i]) for i in I) <= KA
    )

    return nothing
end

function add_st_kv_bound!(model::Model, datapack)
    I = datapack[:I]
    l, w, e_star, h = datapack[:l], datapack[:w], datapack[:e_star], datapack[:h]
    KV = datapack[:KV]
    ul, uw = model[:ul], model[:uw]

    @constraint(
        model,
        truck_volume_kv,
        sum(l[i] * e_star[i][1] * h[i] * ul[i] + w[i] * e_star[i][2] * h[i] * uw[i] for i in I) <= KV
    )

    return nothing
end

function add_st_principal_bounds!(model::Model, datapack)
    add_st_position_bounds!(model, datapack)
    add_st_kl_bounds!(model, datapack)
    add_st_kw_bounds!(model, datapack)
    add_st_e_star_bounds!(model, datapack)
    add_st_kq_bound!(model, datapack)
    add_st_ka_bound!(model, datapack)
    add_st_kv_bound!(model, datapack)

    return nothing
end

function st_crop_variables!(model::Model, datapack)
    fit = datapack[:fit]
    u = model[:u]
    sameStackGroups = datapack[:sameStackGroups]

    for g in eachindex(sameStackGroups)
        gr = sameStackGroups[g]

        for i in (fit[g] + 1):length(gr)
            fix(u[gr[i]], 0; force = true)
        end
    end

    return nothing
end

function add_st_symmetry_break!(model::Model, datapack)
    sameStackGroups = datapack[:sameStackGroups]
    u, x = model[:u], model[:x]

    @constraint(
        model,
        sym_1[group in sameStackGroups, k in 1:length(group)-1],
        u[group[k+1]] <= u[group[k]]
    )

    @constraint(
        model,
        sym_2[group in sameStackGroups, k in 1:length(group)-1],
        x[group[k+1]] <= x[group[k]]
    )

    st_crop_variables!(model, datapack)

    return nothing
end

function add_st_kf_bound!(model::Model, datapack)
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

function add_st_ks_bounds!(model::Model, datapack)
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

function add_st_kd_bounds!(model::Model, datapack)
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

function add_st_secondary_bounds!(model::Model, datapack)
    add_st_kf_bound!(model, datapack)
    add_st_ks_bounds!(model, datapack)
    add_st_kd_bounds!(model, datapack)

    return nothing
end

function add_st_X_discret!(model::Model, datapack)
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

function add_st_Y_discret!(model::Model, datapack)
    I = datapack[:I]
    caW = datapack[:caW]
    y = model[:y]
    x = model[:x]

    @variable(
        model,
        g[i in I, d in caW[i]],
        Bin
    )

    @constraint(
        model,
        y_discret[i in I],
        sum(d * g[i, d] for d in caW[i]) == y[i]
    )

    return nothing
end

function extract_loaded_truck_from_st_model(model::Model, stacks::Vector{Stack}, truck::Truck)
    u_vals = value.(model[:u])
    x_vals, y_vals = value.(model[:x]), value.(model[:y])
    ul_vals = value.(model[:ul])

    loadedStacks = Vector{LoadedStack}()
    
    for s in 1:length(stacks)
        if u_vals[s] > 0.5
            x_pos = round(Int, x_vals[s])
            y_pos = round(Int, y_vals[s])
            
            orientation = ul_vals[s] > 0.5 ? LengthWise : WidthWise
            
            x_end = x_pos + (orientation == LengthWise ? stacks[s].length : stacks[s].width)
            y_end = y_pos + (orientation == LengthWise ? stacks[s].width : stacks[s].length)
            
            push!(loadedStacks, LoadedStack(stacks[s], x_pos, y_pos, x_end, y_end, orientation))
        end
    end
    
    return LoadedTruck(truck, loadedStacks)
end
