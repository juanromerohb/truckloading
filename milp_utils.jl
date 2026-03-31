using JuMP
using Gurobi

if !@isdefined(LOADED_MILP_UTILS)
    const LOADED_MILP_UTILS = true
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

function get_KA(a::Vector{Int}, A::Int; timeLimit::Int = 600)
    I = 1:length(a)

    model = initialize_model(; timeLimit=timeLimit)
    set_silent(model)

    @variable(model, u[I], Bin)

    @objective(model, Max, sum(a[i] * u[i] for i in I))

    @constraint(model, sum(a[i] * u[i] for i in I) <= A)

    optimize!(model)

    if !is_solved_and_feasible(model)
        @error "KA model not solved to optimality"
        return 0
    end

    return objective_value(model)
end

function get_KQ(q::Vector{Int}, Q::Int; timeLimit::Int = 600)
    I = 1:length(q)

    model = initialize_model(; timeLimit=timeLimit)
    set_silent(model)

    @variable(model, u[I], Bin)

    @objective(model, Max, sum(q[i] * u[i] for i in I))

    @constraint(model, sum(q[i] * u[i] for i in I) <= Q)

    optimize!(model)

    if !is_solved_and_feasible(model)
        @error "KQ model not solved to optimality"
        return 0
    end

    return objective_value(model)
end

function get_KL(
    l::Vector{Int}, w::Vector{Int}, fl::Vector{Int}, fw::Vector{Int},
    L::Int; timeLimit::Int = 600
)
    I = 1:length(l)

    model = initialize_model(; timeLimit=timeLimit)
    set_silent(model)

    @variable(model, ul[I], Bin)
    @variable(model, uw[I], Bin)

    @objective(model, Max, sum(l[i] * ul[i] + w[i] * uw[i] for i in I))

    @constraint(model, u_lig_ul_uw[i in I], ul[i] + uw[i] <= 1)
    @constraint(model, forced_lengthwise[i in fl], uw[i] == 0)
    @constraint(model, forced_widthwise[i in fw], ul[i] == 0)
    @constraint(model, max_L, sum(l[i] * ul[i] + w[i] * uw[i] for i in I) <= L)

    optimize!(model)

    if !is_solved_and_feasible(model)
        @error "KL model not solved to optimality"
        return 0
    end

    return objective_value(model)
end

function get_KW(
    l::Vector{Int}, w::Vector{Int}, fl::Vector{Int}, fw::Vector{Int},
    W::Int; timeLimit::Int = 600
)

    I = 1:length(l)

    model = initialize_model(; timeLimit=timeLimit)
    set_silent(model)

    @variable(model, ul[I], Bin)
    @variable(model, uw[I], Bin)

    @objective(model, Max, sum(w[i] * ul[i] + l[i] * uw[i] for i in I))

    @constraint(model, u_ul_uw[i in I], ul[i] + uw[i] <= 1)
    @constraint(model, forced_lengthwise[i in fl], uw[i] == 0)
    @constraint(model, forced_widthwise[i in fw], ul[i] == 0)
    @constraint(model, max_L, sum(w[i] * ul[i] for i in I) + sum(l[i] * uw[i] for i in I) <= W)

    optimize!(model)

    if !is_solved_and_feasible(model)
        @error "KW model not solved to optimality"
        return 0
    end

    return objective_value(model)
end