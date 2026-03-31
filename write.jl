using CSV
using DataFrames
using Logging

if !@isdefined(LOADED_WRITE)
    const LOADED_WRITE = true
end

if !@isdefined(LOADED_STRUCTS) || (@isdefined(RELOAD) && RELOAD)
    include("structs.jl")
end

function int_to_AA(n::Int)
    label = ""

    while n > 0
        n -= 1
        label = string(Char('A' + (n % 26))) * label
        n ÷= 26
    end

    return label
end

function write_trucks_file(loadedTrucks::Vector{LoadedTruck}, outputPath::String)
    df = DataFrame([
        "Id truck" => String[], 
        "Loaded length" => Int[],
        "Weight of loaded items" => Float64[],
        "Volume of loaded items" => Float64[],
        "emm" => Float64[],
        "emr" => Float64[],
    ])

    for loadedTruck in loadedTrucks
        truck = loadedTruck.truck
        loadedStacks = loadedTruck.loadedStacks
        CJfh, EJeh, EJhr, CM, CJfc, EM, EJcr, CJfm = truck.CJfh, truck.EJeh, truck.EJhr, truck.CM, truck.CJfc, truck.EM, truck.EJcr, truck.CJfm

        println("Truck code: ", truck.code)
        println("CJfh: ", CJfh)
        println("EJeh: ", EJeh)
        println("EJhr: ", EJhr)
        println("CM: ", CM)
        println("CJfc: ", CJfc)
        println("EM: ", EM)
        println("EJcr: ", EJcr)
        println("CJfm: ", CJfm)

        loadedLength = maximum([ls.xe for ls in loadedStacks])
        loadedWeight = sum([ls.stack.weight for ls in loadedStacks])
        loadedVolume = sum([ls.stack.length * ls.stack.width * ls.stack.realHeight for ls in loadedStacks])

        weightMoment = sum(ls.stack.weight * (ls.xo + (ls.orientation == LengthWise ? ls.stack.length : ls.stack.width) / 2) for ls in loadedStacks)

        middleAxleWeight = (CJfh * (EJeh + EJhr) * loadedWeight - CJfh * weightMoment + CM * CJfc * EJhr + CJfh * EM * EJcr) / (CJfm * EJhr)
        rearAxleWeight = (weightMoment - EJeh * loadedWeight + EM * (EJhr - EJcr)) / EJhr

        push!(df, (
            truck.code,
            round(Int, loadedLength / 1000),
            loadedWeight / 1000,
            loadedVolume / 1e9,
            middleAxleWeight / 1000,
            rearAxleWeight / 1000
        ))
    end

    CSV.write("$outputPath/output_trucks.csv", df; delim = ';', decimal = ',')

    return nothing
end

function write_stacks_and_items(loadedTrucks::Vector{LoadedTruck}, outputPath::String)
    dfStacks = DataFrame([
        "Id truck" => String[],
        "Id stack" => String[],
        "Stack code" => String[],
        "X origin" => Int[],
        "Y origin" => Int[],
        "Z origin" => Int[],
        "X extremity" => Int[],
        "Y extremity" => Int[],
        "Z extremity" => Int[],
    ])

    dfItems = DataFrame([
        "Item ident" => String[],
        "Id truck" => String[],
        "Id stack" => String[],
        "Item code" => String[],
        "X origin" => Int[],
        "Y origin" => Int[],
        "Z origin" => Int[],
        "X extremity" => Int[],
        "Y extremity" => Int[],
        "Z extremity" => Int[],
    ])

    for loadedTruck in loadedTrucks
        truck = loadedTruck.truck
        loadedStacks = loadedTruck.loadedStacks

        sortedLoadedStacks = sort(loadedStacks, by = ls -> (ls.xo, ls.yo))

        for i in eachindex(sortedLoadedStacks)
            loadedStack = sortedLoadedStacks[i]
            stack = loadedStack.stack
            stackCode = truck.code * "_$i"
            AAcode = int_to_AA(i)

            push!(dfStacks, (
                truck.code,
                stackCode,
                AAcode,
                loadedStack.xo,
                loadedStack.yo,
                0,
                loadedStack.xe,
                loadedStack.ye,
                stack.realHeight,
            ))

            currentHeight = 0

            for j in eachindex(stack.items)
                item = stack.items[j]
                itemHeight = ((j > 1) ? item.height - item.nestingHeight : item.height)

                push!(dfItems, (
                    item.code,
                    truck.code,
                    stackCode,
                    AAcode * "$j",
                    loadedStack.xo,
                    loadedStack.yo,
                    currentHeight,
                    loadedStack.xe,
                    loadedStack.ye,
                    currentHeight + itemHeight,
                ))

                currentHeight += itemHeight
            end
        end
    end

    CSV.write("$outputPath/output_stacks.csv", dfStacks; delim = ';', decimal = ',')
    CSV.write("$outputPath/output_items.csv", dfItems; delim = ';', decimal = ',')

    return nothing
end

function write_loaded_trucks(loadedTrucks::Union{Vector{LoadedTruck}, Nothing}, outputPath::String)
    
    if !isnothing(loadedTrucks)
        @info "FACTIBLE. Writing trucks, stacks, and items to output directory: $outputPath"
        write_trucks_file(loadedTrucks, outputPath)
        write_stacks_and_items(loadedTrucks, outputPath)
        @info "Writing completed."
    else
        @info "INFACTIBLE. No loaded trucks to write."
    end
    
    return nothing
end
