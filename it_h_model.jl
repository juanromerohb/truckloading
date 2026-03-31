if !@isdefined(LOADED_IT_H_MODEL)
    const LOADED_IT_H_MODEL = true
end

if !@isdefined(LOADED_IT_MODEL) || (@isdefined(RELOAD) && RELOAD)
    include("it_model.jl")
end

if !@isdefined(LOADED_READ) || (@isdefined(RELOAD) && RELOAD)
    include("read.jl")
end

const DEFAULT_IT_H_WARMSTARTS_DIR = abspath("camiones/julia/heur_warmstart_rand_iter100")

function _split_semicolon(line::AbstractString)
    return [strip(p) for p in split(chomp(line), ';'; keepempty=true)]
end

function _read_output_truck_codes(heur_instance_dir::AbstractString)
    out_trucks = joinpath(heur_instance_dir, "output_trucks.csv")
    isfile(out_trucks) || return String[]

    codes = String[]
    seen = Set{String}()

    open(out_trucks, "r") do io
        eof(io) && return
        header = _split_semicolon(readline(io))
        hmap = Dict(lowercase(h) => i for (i, h) in enumerate(header))
        idx_truck = get(hmap, "id truck", 0)
        idx_truck == 0 && return

        for line in eachline(io)
            isempty(strip(line)) && continue
            parts = _split_semicolon(line)
            length(parts) < idx_truck && continue
            code = strip(parts[idx_truck])
            isempty(code) && continue
            if !(code in seen)
                push!(codes, code)
                push!(seen, code)
            end
        end
    end

    return codes
end

function _clone_truck_with_code(truck::Truck, code::String)
    return Truck(
        code,
        truck.length,
        truck.width,
        truck.height,
        truck.maxWeight,
        truck.maxWeightAboveItem,
        truck.maxDensity,
        truck.EMmm,
        truck.EMmr,
        truck.CM,
        truck.CJfm,
        truck.CJfc,
        truck.CJfh,
        truck.EM,
        truck.EJhr,
        truck.EJcr,
        truck.EJeh,
        truck.productsData,
        truck.orderedTruckSuppliersData,
        truck.truckPlantData,
        truck.arrivalTime,
        truck.multipleDocks,
        truck.cost,
    )
end

function load_it_h_warmstart_trucks(
    items::Vector{Item},
    truck::Truck;
    instanceName::AbstractString,
    warmStartsDir::AbstractString = DEFAULT_IT_H_WARMSTARTS_DIR,
)
    heur_instance_dir = joinpath(warmStartsDir, instanceName)

    for f in ("output_items.csv", "output_stacks.csv", "output_trucks.csv")
        p = joinpath(heur_instance_dir, f)
        isfile(p) || error("Warm-start file not found: $(p)")
    end

    truck_codes = _read_output_truck_codes(heur_instance_dir)
    if isempty(truck_codes)
        truck_codes = [truck.code]
    end

    trucks = Truck[]
    seen_codes = Set{String}()
    for code in truck_codes
        if code in seen_codes
            continue
        end
        if code == truck.code
            push!(trucks, truck)
        else
            push!(trucks, _clone_truck_with_code(truck, code))
        end
        push!(seen_codes, code)
    end
    if !(truck.code in seen_codes)
        push!(trucks, truck)
    end

    dummy_instance = Instance(
        items,
        trucks,
        Supplier[],
        SupplierDock[],
        Plant[],
        PlantDock[],
        Product[],
        Parameters(0, 0, 0, 0),
    )

    loaded_trucks = read_loaded_trucks_from_csv(dummy_instance, heur_instance_dir)
    isempty(loaded_trucks) && error("No warm-start trucks reconstructed from: $(heur_instance_dir)")

    return loaded_trucks
end

function solve_it_h(
    items::Vector{Item},
    truck::Truck;
    instanceName::AbstractString,
    warmStartsDir::AbstractString = DEFAULT_IT_H_WARMSTARTS_DIR,
    timeLimit::Int = 60,
    principalBounds::Bool = true,
    symmetryBreaking::Bool = true,
    secondaryBounds::Bool = false,
    xDiscret::Bool = false,
    baseModel::Bool = false,
    onlyFirst::Bool = true,
)
    warmStartLoadedTrucks = load_it_h_warmstart_trucks(
        items,
        truck;
        instanceName = instanceName,
        warmStartsDir = warmStartsDir,
    )

    return solve_it(
        items,
        truck;
        timeLimit = timeLimit,
        principalBounds = principalBounds,
        symmetryBreaking = symmetryBreaking,
        secondaryBounds = secondaryBounds,
        xDiscret = xDiscret,
        baseModel = baseModel,
        onlyFirst = onlyFirst,
        warmStartLoadedTrucks = warmStartLoadedTrucks,
    )
end
