using CSV
using DataFrames
using Tar

if !@isdefined(LOADED_READ)
    const LOADED_READ = true
end

if !@isdefined(LOADED_STRUCTS) || (@isdefined(RELOAD) && RELOAD)
    include("structs.jl")
end

function get_expanded_items(items::Vector{Item})
    expandedItems = Vector{Item}(undef, 0)

    for item in items
        for _ in 1:item.quantity
            push!(expandedItems, item)
        end
    end

    return expandedItems
end

function read_instance(path::String; double::Bool = false)
    itemsDF = CSV.read(path * "/input_items.csv", DataFrame; delim=';')
    parametersDF = CSV.read(path * "/input_parameters.csv", DataFrame; delim=';')
    trucksDF = CSV.read(path * "/input_trucks.csv", DataFrame; delim=';')

    itemCodes = unique(string.(itemsDF[!, :"Item ident"]))
    truckCodes = unique(string.(trucksDF[!, :"Id truck"]))
    supplierCodes = unique(
        vcat(
            string.(itemsDF[!, :"Supplier code"]),
            string.(trucksDF[!, :"Supplier code"])
        )
    )
    supplierDockCodes = unique(vcat(
        [
            ismissing(d) ? "only_" * string(sc) : string(d)
            for (d, sc) in zip(itemsDF[!, :"Supplier dock"], itemsDF[!, :"Supplier code"])
        ],
        [
            ismissing(d) ? "only_" * string(sc) : string(d)
            for (d, sc) in zip(trucksDF[!, :"Supplier dock"], trucksDF[!, :"Supplier code"])
        ]
    ))
    plantCodes = unique(
        vcat(
            string.(itemsDF[!, :"Plant code"]),
            string.(trucksDF[!, :"Plant code"])
        )
    )
    plantDockCodes = unique(vcat(
        [
            ismissing(d) ? "only_" * string(pc) : String(d)
            for (d, pc) in zip(itemsDF[!, :"Plant dock"], itemsDF[!, :"Plant code"])
        ],
        [
            ismissing(d) ? "only_" * string(pc) : String(d)
            for (d, pc) in zip(trucksDF[!, :"Plant dock"], trucksDF[!, :"Plant code"])
        ]
    ))
    productCodes = unique(
        vcat(
            string.(itemsDF[!, :"Product code"]),
            string.(trucksDF[!, :"Product code"])
        )
    )
    packageCodes = unique(string.(itemsDF[!, :"Package code"]))
    stackabilityCodes = unique(string.(itemsDF[!, :"Stackability code"]))
    suppliers = [Supplier(code) for code in supplierCodes]
    plants = [Plant(code) for code in plantCodes]
    products = [Product(code) for code in productCodes]
    stackableGroups = [StackableGroup(code) for code in stackabilityCodes]
    packageGroups = [PackageGroup(code) for code in packageCodes]

    supplierDocks = SupplierDock[]
    for code in supplierDockCodes
        if startswith(code, "only_")
            supplier = suppliers[findfirst(s -> s.code == code[6:end], suppliers)]
            push!(supplierDocks, SupplierDock(code, supplier))
        else
            row = filter(row -> string(row[:"Supplier dock"]) == code, eachrow(itemsDF))[1]
            supplier = suppliers[findfirst(s -> s.code == string(row[:"Supplier code"]), suppliers)]
            push!(supplierDocks, SupplierDock(code, supplier))
        end
    end
    plantDocks = PlantDock[]
    for code in plantDockCodes
        if startswith(code, "only_")
            plant = plants[findfirst(p -> p.code == code[6:end], plants)]
            push!(plantDocks, PlantDock(code, plant))
        else
            row = filter(row -> string(row[:"Plant dock"]) == code, eachrow(itemsDF))[1]
            plant = plants[findfirst(p -> p.code == string(row[:"Plant code"]), plants)]
            push!(plantDocks, PlantDock(code, plant))
        end
    end

    items = Item[]
    for code in itemCodes
        row = filter(row -> string(row[:"Item ident"]) == code, eachrow(itemsDF))[1]
        quantity = Int(row[:"Number of items"])
        if double
            quantity *= 2
        end
        length = Int(row[:"Length"])
        width = Int(row[:"Width"])
        height = Int(row[:"Height"])
        nestingHeight = Int(row[:"Nesting height"])
        weight = parse(Int, replace(string(row[:"Weight"]), "," => ""))
        product = products[findfirst(p -> p.code == string(row[:"Product code"]), products)]
        maxStackability = Int(row[:"Max stackability"])
        stackableGroup = stackableGroups[findfirst(sg -> sg.code == string(row[:"Stackability code"]), stackableGroups)]
        packageGroup = packageGroups[findfirst(pg -> pg.code == string(row[:"Package code"]), packageGroups)]
        forcedOrientation = 
            row[:"Forced orientation"] == "none" ?
            NoneForced :
            (row[:"Forced orientation"] == "widthwise" ? ForcedWidthWise : ForcedLengthWise)
        supplier = suppliers[findfirst(s -> s.code == string(row[:"Supplier code"]), suppliers)]
        supplierDock = supplierDocks[findfirst(
            sd -> sd.code ==
            (row[:"Supplier dock"] === missing ? "only_" * string(row[:"Supplier code"]) : string(row[:"Supplier dock"])),
            supplierDocks
        )]
        plant = plants[findfirst(p -> p.code == string(row[:"Plant code"]), plants)]
        plantDock = plantDocks[findfirst(pd -> pd.code == string(row[:"Plant dock"]), plantDocks)]
        earliestArrivalTime = DateTime(
            parse(Int, string(row[:"Earliest arrival time"])[1:4]),
            parse(Int, string(row[:"Earliest arrival time"])[5:6]),
            parse(Int, string(row[:"Earliest arrival time"])[7:8]),
            parse(Int, string(row[:"Earliest arrival time"])[9:10]),
            parse(Int, string(row[:"Earliest arrival time"])[11:12])
        )
        latestArrivalTime = DateTime(
            parse(Int, string(row[:"Latest arrival time"])[1:4]),
            parse(Int, string(row[:"Latest arrival time"])[5:6]),
            parse(Int, string(row[:"Latest arrival time"])[7:8]),
            parse(Int, string(row[:"Latest arrival time"])[9:10]),
            parse(Int, string(row[:"Latest arrival time"])[11:12])
        )
        cost = parse(Int, string(row[:"Inventory cost"]))
        push!(items, Item(
            code,
            quantity,
            length,
            width,
            height,
            nestingHeight,
            weight,
            product,
            maxStackability,
            stackableGroup,
            packageGroup,
            forcedOrientation,
            supplier,
            supplierDock,
            plant,
            plantDock,
            earliestArrivalTime,
            latestArrivalTime,
            cost
        ))
    end

    trucks = Truck[]
    for code in truckCodes
        row = filter(row -> string(row[:"Id truck"]) == code, eachrow(trucksDF))[1]
        length = Int(row[:"Length"])
        width = Int(row[:"Width"])
        height = Int(row[:"Height"])
        maxWeight = Int(row[:"Max weight"])
        maxWeightAboveItem = maximum(
            Int(row2[:"Max weight on the bottom item in stacks"])
            for row2 in eachrow(trucksDF)
            if string(row2[:"Id truck"]) == code
        )
        maxDensity = Int(row[:"Max density"])
        EMmm = Int(row[:"EMmm"])
        EMmr = Int(row[:"EMmr"])
        CM = parse(Int, replace(string(row[:"CM"]), "," => ""))
        CJfm = Int(row[:"CJfm"])
        CJfc = Int(row[:"CJfc"])
        CJfh = Int(row[:"CJfh"])
        EM = parse(Int, replace(string(row[:"EM"]), "," => ""))
        EJhr = Int(row[:"EJhr"])
        EJcr = Int(row[:"EJcr"])
        EJeh = Int(row[:"EJeh"])

        truckProductsData = ProductData[]
        for row2 in eachrow(trucksDF)
            if string(row2[:"Id truck"]) == code
                included = false
                for productData in truckProductsData
                    if productData.product.code == string(row2[:"Product code"])
                        included = true
                        break
                    end
                end

                if !included
                    push!(truckProductsData, ProductData(
                        products[findfirst(p -> p.code == string(row2[:"Product code"]), products)],
                        Int(row2[:"Max weight on the bottom item in stacks"])
                    ))
                end
            end
        end

        truckSuppliersDataWithOrder = Tuple{TruckSupplierData, Int}[]
        for row2 in eachrow(trucksDF)
            if string(row2[:"Id truck"]) == code
                included = false
                for truckSupplierDataWithOrder in truckSuppliersDataWithOrder
                    if truckSupplierDataWithOrder[1].supplier.code == string(row2[:"Supplier code"])
                        included = true
                        break
                    end
                end

                if !included
                    supplier = suppliers[findfirst(s -> s.code == string(row2[:"Supplier code"]), suppliers)]

                    supplierDocksWithOrder = Tuple{SupplierDock, Int}[]
                    for row3 in eachrow(trucksDF)
                        included = false
                        if string(row3[:"Id truck"]) == code &&
                            string(row3[:"Supplier code"]) == string(row2[:"Supplier code"])

                            for supplierDockWithOrder in supplierDocksWithOrder
                                if supplierDockWithOrder[1].code == 
                                    (row3[:"Supplier dock"] === missing ?
                                    "only_" * string(row3[:"Supplier code"]) :
                                    string(row3[:"Supplier dock"]))

                                    included = true
                                    break
                                end
                            end

                            if !included
                                supplierDock = supplierDocks[findfirst(
                                    sd -> sd.code == (row3[:"Supplier dock"] === missing ?
                                        "only_" * string(row3[:"Supplier code"]) :
                                        string(row3[:"Supplier dock"])),
                                    supplierDocks
                                )]
                                push!(supplierDocksWithOrder, (supplierDock, Int(row3[:"Supplier dock loading order"])))
                            end
                        end
                    end

                    orderedSupplierDocks = [data for (data, _) in sort(supplierDocksWithOrder, by = x -> x[2])]

                    push!(truckSuppliersDataWithOrder, (
                        TruckSupplierData(supplier, orderedSupplierDocks),
                        Int(row2[:"Supplier loading order"])
                    ))
                end
            end
        end

        orderedTruckSuppliers = [data for (data, _) in sort(truckSuppliersDataWithOrder, by = x -> x[2])]

        plant = plants[findfirst(p -> p.code == string(row[:"Plant code"]), plants)]

        plantDocksWithOrder = Tuple{PlantDock, Int}[]
        for row2 in eachrow(trucksDF)
            if string(row2[:"Id truck"]) == code
                included = false
                for plantDockWithOrder in plantDocksWithOrder
                    if plantDockWithOrder[1].code ==
                        (row2[:"Plant dock"] === missing ?
                        "only_" * string(row2[:"Plant code"]) :
                        string(row2[:"Plant dock"]))
                        
                        included = true
                        break
                    end
                end

                if !included
                    plantDock = plantDocks[findfirst(
                        pd -> pd.code == (row2[:"Plant dock"] === missing ?
                            "only_" * string(row2[:"Plant code"]) :
                            string(row2[:"Plant dock"])),
                        plantDocks
                    )]
                    push!(plantDocksWithOrder, (plantDock, Int(row2[:"Plant dock loading order"])))
                end
            end
        end

        orderedPlantDocks = [data for (data, _) in sort(plantDocksWithOrder, by = x -> x[2])]
        truckPlantData = TruckPlantData(plant, orderedPlantDocks)

        arrivalTime = DateTime(
            parse(Int, string(row[:"Arrival time"])[1:4]),
            parse(Int, string(row[:"Arrival time"])[5:6]),
            parse(Int, string(row[:"Arrival time"])[7:8]),
            parse(Int, string(row[:"Arrival time"])[9:10]),
            parse(Int, string(row[:"Arrival time"])[11:12])
        )
        multipleDocks = (row[:"Stack with multiple docks"] == 1)
        cost = parse(Int, string(row[:"Cost"]))

        push!(trucks, Truck(
            code,
            length,
            width,
            height,
            maxWeight,
            maxWeightAboveItem,
            maxDensity,
            EMmm,
            EMmr,
            CM,
            CJfm,
            CJfc,
            CJfh,
            EM,
            EJhr,
            EJcr,
            EJeh,
            truckProductsData,
            orderedTruckSuppliers,
            truckPlantData,
            arrivalTime,
            multipleDocks,
            cost
        ))
    end

    parameters = Parameters(
        parametersDF[1, :"Coefficient transportation cost"],
        parametersDF[1, :"Coefficient inventory cost"],
        parse(Int, replace(string(parametersDF[1, :"Coefficient cost extra truck"]), "," => "")),
        parametersDF[1, :"timelimit (sec)"]
    )

    instance = Instance(
        get_expanded_items(items),
        trucks,
        suppliers,
        supplierDocks,
        plants,
        plantDocks, 
        products,
        parameters
    )

    return instance
end

function read_instance_tar(path::String; double::Bool = false)
    tmp = Tar.extract(path)
    itemsDF = CSV.read(tmp * "/input_items.csv", DataFrame; delim=';')
    parametersDF = CSV.read(tmp * "/input_parameters.csv", DataFrame; delim=';')
    trucksDF = CSV.read(tmp * "/input_trucks.csv", DataFrame; delim=';')

    itemCodes = unique(string.(itemsDF[!, :"Item ident"]))
    truckCodes = unique(string.(trucksDF[!, :"Id truck"]))
    supplierCodes = unique(
        vcat(
            string.(itemsDF[!, :"Supplier code"]),
            string.(trucksDF[!, :"Supplier code"])
        )
    )
    supplierDockCodes = unique(vcat(
        [
            ismissing(d) ? "only_" * string(sc) : string(d)
            for (d, sc) in zip(itemsDF[!, :"Supplier dock"], itemsDF[!, :"Supplier code"])
        ],
        [
            ismissing(d) ? "only_" * string(sc) : string(d)
            for (d, sc) in zip(trucksDF[!, :"Supplier dock"], trucksDF[!, :"Supplier code"])
        ]
    ))
    plantCodes = unique(
        vcat(
            string.(itemsDF[!, :"Plant code"]),
            string.(trucksDF[!, :"Plant code"])
        )
    )
    plantDockCodes = unique(vcat(
        [
            ismissing(d) ? "only_" * string(pc) : String(d)
            for (d, pc) in zip(itemsDF[!, :"Plant dock"], itemsDF[!, :"Plant code"])
        ],
        [
            ismissing(d) ? "only_" * string(pc) : String(d)
            for (d, pc) in zip(trucksDF[!, :"Plant dock"], trucksDF[!, :"Plant code"])
        ]
    ))
    productCodes = unique(
        vcat(
            string.(itemsDF[!, :"Product code"]),
            string.(trucksDF[!, :"Product code"])
        )
    )
    packageCodes = unique(string.(itemsDF[!, :"Package code"]))
    stackabilityCodes = unique(string.(itemsDF[!, :"Stackability code"]))
    suppliers = [Supplier(code) for code in supplierCodes]
    plants = [Plant(code) for code in plantCodes]
    products = [Product(code) for code in productCodes]
    stackableGroups = [StackableGroup(code) for code in stackabilityCodes]
    packageGroups = [PackageGroup(code) for code in packageCodes]

    supplierDocks = SupplierDock[]
    for code in supplierDockCodes
        if startswith(code, "only_")
            supplier = suppliers[findfirst(s -> s.code == code[6:end], suppliers)]
            push!(supplierDocks, SupplierDock(code, supplier))
        else
            row = filter(row -> string(row[:"Supplier dock"]) == code, eachrow(itemsDF))[1]
            supplier = suppliers[findfirst(s -> s.code == string(row[:"Supplier code"]), suppliers)]
            push!(supplierDocks, SupplierDock(code, supplier))
        end
    end
    plantDocks = PlantDock[]
    for code in plantDockCodes
        if startswith(code, "only_")
            plant = plants[findfirst(p -> p.code == code[6:end], plants)]
            push!(plantDocks, PlantDock(code, plant))
        else
            row = (length(filter(row -> string(row[:"Plant dock"]) == code, eachrow(itemsDF))) == 0) ?
                filter(row -> string(row[:"Plant dock"]) == code, eachrow(trucksDF))[1] :
                filter(row -> string(row[:"Plant dock"]) == code, eachrow(itemsDF))[1]
            plant = plants[findfirst(p -> p.code == string(row[:"Plant code"]), plants)]
            push!(plantDocks, PlantDock(code, plant))
        end
    end

    items = Item[]
    for code in itemCodes
        row = filter(row -> string(row[:"Item ident"]) == code, eachrow(itemsDF))[1]
        quantity = Int(row[:"Number of items"])
        if double
            quantity *= 2
        end
        length = Int(row[:"Length"])
        width = Int(row[:"Width"])
        height = Int(row[:"Height"])
        nestingHeight = Int(row[:"Nesting height"])
        weight = parse(Int, replace(string(row[:"Weight"]), "," => ""))
        product = products[findfirst(p -> p.code == string(row[:"Product code"]), products)]
        maxStackability = Int(row[:"Max stackability"])
        stackableGroup = stackableGroups[findfirst(sg -> sg.code == string(row[:"Stackability code"]), stackableGroups)]
        packageGroup = packageGroups[findfirst(pg -> pg.code == string(row[:"Package code"]), packageGroups)]
        forcedOrientation = 
            row[:"Forced orientation"] == "none" ?
            NoneForced :
            (row[:"Forced orientation"] == "widthwise" ? ForcedWidthWise : ForcedLengthWise)
        supplier = suppliers[findfirst(s -> s.code == string(row[:"Supplier code"]), suppliers)]
        supplierDock = supplierDocks[findfirst(
            sd -> sd.code ==
            (row[:"Supplier dock"] === missing ? "only_" * string(row[:"Supplier code"]) : string(row[:"Supplier dock"])),
            supplierDocks
        )]
        plant = plants[findfirst(p -> p.code == string(row[:"Plant code"]), plants)]
        plantDock = plantDocks[findfirst(pd -> pd.code == string(row[:"Plant dock"]), plantDocks)]
        earliestArrivalTime = DateTime(
            parse(Int, string(row[:"Earliest arrival time"])[1:4]),
            parse(Int, string(row[:"Earliest arrival time"])[5:6]),
            parse(Int, string(row[:"Earliest arrival time"])[7:8]),
            parse(Int, string(row[:"Earliest arrival time"])[9:10]),
            parse(Int, string(row[:"Earliest arrival time"])[11:12])
        )
        latestArrivalTime = DateTime(
            parse(Int, string(row[:"Latest arrival time"])[1:4]),
            parse(Int, string(row[:"Latest arrival time"])[5:6]),
            parse(Int, string(row[:"Latest arrival time"])[7:8]),
            parse(Int, string(row[:"Latest arrival time"])[9:10]),
            parse(Int, string(row[:"Latest arrival time"])[11:12])
        )
        cost = parse(Int, string(row[:"Inventory cost"]))

        push!(items, Item(
            code,
            quantity,
            length,
            width,
            height,
            nestingHeight,
            weight,
            product,
            maxStackability,
            stackableGroup,
            packageGroup,
            forcedOrientation,
            supplier,
            supplierDock,
            plant,
            plantDock,
            earliestArrivalTime,
            latestArrivalTime,
            cost
        ))
    end

    trucks = Truck[]
    for code in truckCodes
        row = filter(row -> string(row[:"Id truck"]) == code, eachrow(trucksDF))[1]
        length = Int(row[:"Length"])
        width = Int(row[:"Width"])
        height = Int(row[:"Height"])
        maxWeight = Int(row[:"Max weight"])
        maxWeightAboveItem = maximum(
            Int(row2[:"Max weight on the bottom item in stacks"])
            for row2 in eachrow(trucksDF)
            if string(row2[:"Id truck"]) == code
        )
        maxDensity = Int(row[:"Max density"])
        EMmm = Int(row[:"EMmm"])
        EMmr = Int(row[:"EMmr"])
        CM = parse(Int, replace(string(row[:"CM"]), "," => ""))
        CJfm = Int(row[:"CJfm"])
        CJfc = Int(row[:"CJfc"])
        CJfh = Int(row[:"CJfh"])
        EM = parse(Int, replace(string(row[:"EM"]), "," => ""))
        EJhr = Int(row[:"EJhr"])
        EJcr = Int(row[:"EJcr"])
        EJeh = Int(row[:"EJeh"])

        truckProductsData = ProductData[]
        for row2 in eachrow(trucksDF)
            if string(row2[:"Id truck"]) == code
                included = false
                for productData in truckProductsData
                    if productData.product.code == string(row2[:"Product code"])
                        included = true
                        break
                    end
                end

                if !included
                    push!(truckProductsData, ProductData(
                        products[findfirst(p -> p.code == string(row2[:"Product code"]), products)],
                        Int(row2[:"Max weight on the bottom item in stacks"])
                    ))
                end
            end
        end

        truckSuppliersDataWithOrder = Tuple{TruckSupplierData, Int}[]
        for row2 in eachrow(trucksDF)
            if string(row2[:"Id truck"]) == code
                included = false
                for truckSupplierDataWithOrder in truckSuppliersDataWithOrder
                    if truckSupplierDataWithOrder[1].supplier.code == string(row2[:"Supplier code"])
                        included = true
                        break
                    end
                end

                if !included
                    supplier = suppliers[findfirst(s -> s.code == string(row2[:"Supplier code"]), suppliers)]

                    supplierDocksWithOrder = Tuple{SupplierDock, Int}[]
                    for row3 in eachrow(trucksDF)
                        included = false
                        if string(row3[:"Id truck"]) == code &&
                            string(row3[:"Supplier code"]) == string(row2[:"Supplier code"])

                            for supplierDockWithOrder in supplierDocksWithOrder
                                if supplierDockWithOrder[1].code == 
                                    (row3[:"Supplier dock"] === missing ?
                                    "only_" * string(row3[:"Supplier code"]) :
                                    string(row3[:"Supplier dock"]))

                                    included = true
                                    break
                                end
                            end

                            if !included
                                supplierDock = supplierDocks[findfirst(
                                    sd -> sd.code == (row3[:"Supplier dock"] === missing ?
                                        "only_" * string(row3[:"Supplier code"]) :
                                        string(row3[:"Supplier dock"])),
                                    supplierDocks
                                )]
                                push!(supplierDocksWithOrder, (supplierDock, Int(row3[:"Supplier dock loading order"])))
                            end
                        end
                    end

                    orderedSupplierDocks = [data for (data, _) in sort(supplierDocksWithOrder, by = x -> x[2])]

                    push!(truckSuppliersDataWithOrder, (
                        TruckSupplierData(supplier, orderedSupplierDocks),
                        Int(row2[:"Supplier loading order"])
                    ))
                end
            end
        end

        orderedTruckSuppliers = [data for (data, _) in sort(truckSuppliersDataWithOrder, by = x -> x[2])]

        plant = plants[findfirst(p -> p.code == string(row[:"Plant code"]), plants)]

        plantDocksWithOrder = Tuple{PlantDock, Int}[]
        for row2 in eachrow(trucksDF)
            if string(row2[:"Id truck"]) == code
                included = false
                for plantDockWithOrder in plantDocksWithOrder
                    if plantDockWithOrder[1].code ==
                        (row2[:"Plant dock"] === missing ?
                        "only_" * string(row2[:"Plant code"]) :
                        string(row2[:"Plant dock"]))
                        
                        included = true
                        break
                    end
                end

                if !included
                    plantDock = plantDocks[findfirst(
                        pd -> pd.code == (row2[:"Plant dock"] === missing ?
                            "only_" * string(row2[:"Plant code"]) :
                            string(row2[:"Plant dock"])),
                        plantDocks
                    )]
                    push!(plantDocksWithOrder, (plantDock, Int(row2[:"Plant dock loading order"])))
                end
            end
        end

        orderedPlantDocks = [data for (data, _) in sort(plantDocksWithOrder, by = x -> x[2])]
        truckPlantData = TruckPlantData(plant, orderedPlantDocks)

        arrivalTime = DateTime(
            parse(Int, string(row[:"Arrival time"])[1:4]),
            parse(Int, string(row[:"Arrival time"])[5:6]),
            parse(Int, string(row[:"Arrival time"])[7:8]),
            parse(Int, string(row[:"Arrival time"])[9:10]),
            parse(Int, string(row[:"Arrival time"])[11:12])
        )
        multipleDocks = (row[:"Stack with multiple docks"] == 1)
        cost = parse(Int, string(row[:"Cost"]))

        push!(trucks, Truck(
            code,
            length,
            width,
            height,
            maxWeight,
            maxWeightAboveItem,
            maxDensity,
            EMmm,
            EMmr,
            CM,
            CJfm,
            CJfc,
            CJfh,
            EM,
            EJhr,
            EJcr,
            EJeh,
            truckProductsData,
            orderedTruckSuppliers,
            truckPlantData,
            arrivalTime,
            multipleDocks,
            cost
        ))
    end

    parameters = Parameters(
        parametersDF[1, :"Coefficient transportation cost"],
        parametersDF[1, :"Coefficient inventory cost"],
        parse(Int, replace(string(parametersDF[1, :"Coefficient cost extra truck"]), "," => "")),
        parametersDF[1, :"timelimit (sec)"]
    )

    instance = Instance(
        get_expanded_items(items),
        trucks,
        suppliers,
        supplierDocks,
        plants,
        plantDocks,
        products,
        parameters
    )
    return instance
end

function read_loaded_trucks_from_csv(instance::Instance, outputPath::String)
    trucksDF = CSV.read("$outputPath/output_trucks.csv", DataFrame; delim=';', decimal=',')
    stacksDF = CSV.read("$outputPath/output_stacks.csv", DataFrame; delim=';', decimal=',')
    itemsDF = CSV.read("$outputPath/output_items.csv", DataFrame; delim=';', decimal=',')
    
    loadedTrucks = LoadedTruck[]
    
    truck_map = Dict(truck.code => truck for truck in instance.trucks)
    item_map = Dict(item.code => item for item in instance.items)
    
    for truck_row in eachrow(trucksDF)
        truck_code = string(truck_row[:"Id truck"])
        
        if !haskey(truck_map, truck_code)
            @warn "Truck code '$truck_code' not found in instance"
            continue
        end
        
        truck = truck_map[truck_code]
        
        truck_stacks = filter(row -> string(row[:"Id truck"]) == truck_code, stacksDF)
        
        loadedStacks = LoadedStack[]
        
        for stack_row in eachrow(truck_stacks)
            stack_code = string(stack_row[:"Id stack"])
            
            stack_items_df = filter(row -> string(row[:"Id stack"]) == stack_code, itemsDF)
            
            stack_items_df = sort(stack_items_df, order(:"Z origin"))
            
            stack_items = Item[]
            for item_row in eachrow(stack_items_df)
                item_code = string(item_row[:"Item ident"])
                
                matching_items = [item for item in instance.items if item.code == item_code]
                if isempty(matching_items)
                    @warn "Item code '$item_code' not found in instance"
                    continue
                end
                
                push!(stack_items, matching_items[1])
            end
            
            if isempty(stack_items)
                continue
            end
            
            xo = Int(stack_row[:"X origin"])
            yo = Int(stack_row[:"Y origin"])
            xe = Int(stack_row[:"X extremity"])
            ye = Int(stack_row[:"Y extremity"])
            ze = Int(stack_row[:"Z extremity"])
            
            stack_length = xe - xo
            stack_width = ye - yo
            stack_real_height = ze

            stack_height = sum(item.height for item in stack_items)
            
            first_item = stack_items[1]
            stack_weight = sum(item.weight for item in stack_items)
            
            forced_orientation = first_item.forcedOrientation
            
            earliest_time = minimum(item.earliestArrivalTime for item in stack_items)
            latest_time = maximum(item.latestArrivalTime for item in stack_items)
            
            stack = Stack(
                stack_items,
                stack_length,
                stack_width,
                stack_height,
                stack_real_height,
                stack_weight,
                forced_orientation,
                first_item.supplier,
                first_item.supplierDock,
                first_item.plant,
                first_item.plantDock,
                earliest_time,
                latest_time
            )
            
            natural_length = first_item.length
            natural_width = first_item.width
            
            orientation = if stack_length == natural_length
                LengthWise
            else
                WidthWise
            end
            
            loaded_stack = LoadedStack(
                stack,
                xo,
                yo,
                xe,
                ye,
                orientation
            )
            
            push!(loadedStacks, loaded_stack)
        end
        
        loaded_truck = LoadedTruck(truck, loadedStacks)
        push!(loadedTrucks, loaded_truck)
    end
    
    return loadedTrucks
end
