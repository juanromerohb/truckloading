if !@isdefined(LOADED_UTILS)
    const LOADED_UTILS = true
end

if !@isdefined(LOADED_STRUCTS) || (@isdefined(RELOAD) && RELOAD)
    include("structs.jl")
end

function get_extra_truck(truck::Truck, num::Int)
    newCode = "Q" * truck.code[2:end] * "_$num"

    return Truck(
        newCode,
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

function get_product_max_weight_above(truck::Truck, product::Product)
    idx = findfirst(pd -> pd.product.code == product.code, truck.productsData)
    if idx === nothing
        return truck.maxWeightAboveItem
    end
    return truck.productsData[idx].maxWeightAbove
end

function get_item_max_weight_above_grams(item::Item, truck::Truck)
    return get_product_max_weight_above(truck, item.product) * 1000
end

function get_items_max_weight_above_grams(items::Vector{Item}, truck::Truck)
    return [get_item_max_weight_above_grams(item, truck) for item in items]
end

function get_consts(truck::Truck)
    CJfh = truck.CJfh
    EJeh = truck.EJeh
    EJhr = truck.EJhr
    EMmm = truck.EMmm * 1000
    CJfm = truck.CJfm
    CM = truck.CM
    CJfc = truck.CJfc
    EM = truck.EM
    EJcr = truck.EJcr
    EMmr = truck.EMmr * 1000

    const_1 = CJfh * (EJeh + EJhr)
    const_2 = EJhr * (EMmm * CJfm - CM * CJfc) - CJfh * EM * EJcr
    const_3 = EMmr * EJhr + EM * (EJcr - EJhr)

    return CJfh, EJeh, const_1, const_2, const_3
end

function get_min_max(boxes::Union{Vector{Item}, Vector{Stack}})
    l, w = [box.length for box in boxes], [box.width for box in boxes]

    minx =  [
        (boxes[i].forcedOrientation == NoneForced) ? min(l[i], w[i]) : 
            ((boxes[i].forcedOrientation == ForcedLengthWise) ? l[i] : w[i])
        for i in eachindex(boxes)
    ]

    maxx =  [
        (boxes[i].forcedOrientation == NoneForced) ? max(l[i], w[i]) : 
            ((boxes[i].forcedOrientation == ForcedLengthWise) ? l[i] : w[i])
        for i in eachindex(boxes)
    ]

    miny = [
        (boxes[i].forcedOrientation == NoneForced) ? min(l[i], w[i]) : 
            ((boxes[i].forcedOrientation == ForcedLengthWise) ? w[i] : l[i])
        for i in eachindex(boxes)
    ]

    maxy = [
        (boxes[i].forcedOrientation == NoneForced) ? max(l[i], w[i]) : 
            ((boxes[i].forcedOrientation == ForcedLengthWise) ? w[i] : l[i])
        for i in eachindex(boxes)
    ]

    return minx, maxx, miny, maxy
end

function get_fl_fw(boxes::Union{Vector{Item}, Vector{Stack}}, truck::Truck)
    l, w = [box.length for box in boxes], [box.width for box in boxes]
    L, W = truck.length, truck.width

    fl = [
        i
        for i in eachindex(boxes)
        if (boxes[i].forcedOrientation == ForcedLengthWise) || (l[i] > W) || (w[i] > L)
    ]

    fw = [
        i
        for i in eachindex(boxes)
        if (boxes[i].forcedOrientation == ForcedWidthWise) || (l[i] > L) || (w[i] > W)
    ]

    return fl, fw
end

function get_e(l::Vector{Int}, w::Vector{Int}, fl::Vector{Int}, fw::Vector{Int}, W::Int)
    n = length(l)
    I = 1:n

    e = Vector{Tuple{Int, Int}}(undef, n)

    fl_set = Set(fl)
    fw_set = Set(fw)

    for i in I
        wi = Int[]

        for j in I
            if j != i
                if !(j in fw_set)
                    push!(wi, w[j])
                end
                if !(j in fl_set)
                    push!(wi, l[j])
                end
            end
        end

        e1 = w[i]
        e2 = l[i]

        if !(i in fw_set)
            if w[i] <= W && all(w[i] + wid > W for wid in wi)
                e1 = W
            end
        end

        if !(i in fl_set)
            if l[i] <= W && all(l[i] + wid > W for wid in wi)
                e2 = W
            end
        end

        if i in fw_set
            e1 = w[i]
        end
        if i in fl_set
            e2 = l[i]
        end

        e[i] = (e1, e2)
    end

    return e
end

function get_KF(miny::Vector{Int}, W::Int)
    sortedMiny = sort(miny)
    KF = length(sortedMiny)
    sumMin = 0

    for K in eachindex(sortedMiny)
        sumMin += sortedMiny[K]

        if sumMin > W
            KF = K - 1
            break
        end
    end

    return KF
end

function get_KS(i::Int, o::Vector{Int}, miny::Vector{Int}, maxy::Vector{Int})
    maxyi = maxy[i]
    I = 1:length(o)
    filteredMiny = [miny[j] for j in I if (o[i] <= o[j]) && (i != j)]
    sortedMiny = sort(filteredMiny)
    KS = length(sortedMiny)
    sumMin = 0

    for K in eachindex(sortedMiny)
        sumMin += sortedMiny[K]

        if sumMin > maxyi
            KS = K - 1
            break
        end
    end

    KS += 2

    return KS
end

function get_KD(i::Int, miny::Vector{Int}, minx::Vector{Int}, maxx::Vector{Int}, W::Int)
    minyi = miny[i]
    maxxi = maxx[i]
    I = 1:length(miny)
    filteredMiny = [miny[j] for j in I if i != j]
    sortedMiny = sort(filteredMiny)
    filteredMinx = [minx[j] for j in I if i != j]
    sortedMinx = sort(filteredMinx)
    rowi = length(sortedMiny)
    sumMin = 0

    for K in eachindex(sortedMiny)
        sumMin += sortedMiny[K]

        if sumMin > W - minyi
            rowi = K - 1
            break
        end
    end

    kd = length(sortedMinx)
    sumMin = 0

    for K in eachindex(sortedMinx)
        sumMin += sortedMinx[K]

        if sumMin > maxxi
            kd = K - 1
            break
        end
    end

    kd += 2

    KD = rowi * kd

    return KD
end

function get_caL(i::Int, l::Vector{Int}, w::Vector{Int}, fl::Vector{Int}, fw::Vector{Int})
    I = 1:length(l)
    return unique!([
        [l[j] for j in I if j != i && !(j in fw)];
        [w[j] for j in I if j != i && !(j in fl)]
    ])
end

function get_caW(i::Int, l::Vector{Int}, w::Vector{Int}, fl::Vector{Int}, fw::Vector{Int})
    I = 1:length(l)
    return unique!([
        [w[j] for j in I if j != i && !(j in fw)];
        [l[j] for j in I if j != i && !(j in fl)]
    ])
end
