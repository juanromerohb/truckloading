include("read.jl")
include("write.jl")
include("it_model.jl")
include("it_h_model.jl")
include("st_model.jl")

# ENV["GRB_LICENSE_FILE"] = "gurobi.lic"

const DEFAULT_PERFORM_WARMSTARTS_DIR = joinpath("results", "heur_warmstart_rand_iter100")
const SUPPORTED_ALGS = ("it0", "it", "ith", "st")

function usage_error(msg::AbstractString)
    error(
        msg *
        "\nUsage: julia perform.jl <instance_tar> <alg> <max_cpu> <out_dir> [warmstarts_dir]" *
        "\nSupported alg values: " * join(SUPPORTED_ALGS, ", ")
    )
end

function normalize_alg(alg::AbstractString)
    normalized = lowercase(strip(alg))
    normalized in SUPPORTED_ALGS || usage_error("Unsupported alg: $alg")
    return normalized
end

function run_model(
    alg::AbstractString,
    items::Vector{Item},
    truck::Truck,
    instance_name::AbstractString,
    max_cpu::Int;
    warmstarts_dir::AbstractString = DEFAULT_PERFORM_WARMSTARTS_DIR,
)
    if alg == "it0"
        return solve_it(
            items,
            truck;
            timeLimit = max_cpu,
            baseModel = true,
        )
    elseif alg == "it"
        return solve_it(
            items,
            truck;
            timeLimit = max_cpu,
            principalBounds = true,
            symmetryBreaking = true,
        )
    elseif alg == "ith"
        return solve_it_h(
            items,
            truck;
            instanceName = instance_name,
            warmStartsDir = warmstarts_dir,
            timeLimit = max_cpu,
            principalBounds = true,
            symmetryBreaking = true,
        )
    elseif alg == "st"
        return solve_st(
            items,
            truck;
            timeLimit = max_cpu,
            principalBounds = true,
            symmetryBreaking = true,
        )
    end

    usage_error("Unsupported alg: $alg")
end

function write_run_data(runData, output_file::AbstractString)
    text_file_data = ""

    for i in eachindex(runData)
        text_file_data *= "$i\n"
        for (key, value) in pairs(runData[i])
            text_file_data *= "  $key: $value\n"
        end
        text_file_data *= "\n"
    end

    open(output_file, "w") do file
        write(file, text_file_data)
    end

    return nothing
end

function my_main()
    @show ARGS
    @show Base.PROGRAM_FILE
    @show DEPOT_PATH
    @show LOAD_PATH
    @show pwd()
    @show Base.active_project()
    @show Sys.BINDIR

    @show Base.JLOptions().opt_level
    @show Base.JLOptions().nthreads
    @show Base.JLOptions().check_bounds

    length(ARGS) >= 4 || usage_error("Expected at least 4 arguments, got $(length(ARGS))")

    instance_tar = ARGS[1]
    alg = normalize_alg(ARGS[2])
    max_cpu = parse(Int, ARGS[3])
    out_dir = ARGS[4]
    warmstarts_dir = (alg == "ith" && length(ARGS) >= 5) ? ARGS[5] : DEFAULT_PERFORM_WARMSTARTS_DIR

    instance = read_instance_tar(instance_tar; double = false)
    items, truck = instance.items, instance.trucks[1]

    instance_name = splitext(basename(instance_tar))[1]
    precompile_time = 180

    # First run to precompile the selected model family.
    _, _ = run_model(
        alg,
        items,
        truck,
        instance_name,
        precompile_time;
        warmstarts_dir = warmstarts_dir,
    )

    loadedTrucks, runData = run_model(
        alg,
        items,
        truck,
        instance_name,
        max_cpu;
        warmstarts_dir = warmstarts_dir,
    )

    out_path = "$out_dir/$instance_name-$alg"
    if !isdir(out_path)
        mkpath(out_path)
    end

    write_loaded_trucks(loadedTrucks, out_path)
    write_run_data(runData, "$out_dir/data-$(instance_name)-$alg.txt")

    return nothing
end

my_main()
