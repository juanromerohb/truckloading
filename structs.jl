using Dates

if !@isdefined(LOADED_STRUCTS)
    const LOADED_STRUCTS = true
end

@enum ForcedOrientation NoneForced ForcedLengthWise ForcedWidthWise
@enum Orientation LengthWise WidthWise

struct Supplier
    code::String
end

struct SupplierDock
    code::String
    supplier::Supplier
end

struct Plant
    code::String
end

struct PlantDock
    code::String
    plant::Plant
end

struct Product
    code::String
end

struct StackableGroup
    code::String
end

struct PackageGroup
    code::String
end

struct Item
    code::String
    quantity::Int
    length::Int             # mm
    width::Int              # mm
    height::Int             # mm
    nestingHeight::Int      # mm
    weight::Int             # g
    product::Product
    maxStackability::Int
    stackableGroup::StackableGroup
    packageGroup::PackageGroup
    forcedOrientation::ForcedOrientation
    supplier::Supplier
    supplierDock::SupplierDock
    plant::Plant
    plantDock::PlantDock
    earliestArrivalTime::DateTime
    latestArrivalTime::DateTime
    cost::Int               # EUR
end

struct Stack
    items::Vector{Item}
    length::Int             # mm
    width::Int              # mm
    height::Int             # mm
    realHeight::Int         # mm
    weight::Int             # g
    forcedOrientation::ForcedOrientation
    supplier::Supplier
    supplierDock::SupplierDock
    plant::Plant
    plantDock::PlantDock
    earliestArrivalTime::DateTime
    latestArrivalTime::DateTime
end

struct StackModelData
    stacks::Vector{Stack}
    KL_star::Int
end

struct TruckSupplierData
    supplier::Supplier
    orderedSupplierDocks::Vector{SupplierDock}
end

struct TruckPlantData
    plant::Plant
    orderedPlantDocks::Vector{PlantDock}
end

struct ProductData
    product::Product
    maxWeightAbove::Int     # kg
end

struct Truck
    code::String
    length::Int             # mm
    width::Int              # mm
    height::Int             # mm
    maxWeight::Int          # kg
    maxWeightAboveItem::Int # kg
    maxDensity::Int         # kg/m^2
    EMmm::Int               # kg
    EMmr::Int               # kg
    CM::Int                 # g
    CJfm::Int               # mm
    CJfc::Int               # mm
    CJfh::Int               # mm
    EM::Int                 # g
    EJhr::Int               # mm
    EJcr::Int               # mm
    EJeh::Int               # mm
    productsData::Vector{ProductData}
    orderedTruckSuppliersData::Vector{TruckSupplierData}
    truckPlantData::TruckPlantData
    arrivalTime::DateTime
    multipleDocks::Bool
    cost::Int               # EUR
end

struct LoadedStack
    stack::Stack
    xo::Int
    yo::Int
    xe::Int
    ye::Int
    orientation::Orientation
end

struct LoadedTruck
    truck::Truck
    loadedStacks::Vector{LoadedStack}
end

struct Parameters
    transportCostCoefficient::Int
    inventoryCostCoefficient::Int
    extraTruckCostCoefficient::Int # x 10
    timeLimit::Int  # s
end

struct Instance
    items::Vector{Item}
    trucks::Vector{Truck}
    suppliers::Vector{Supplier}
    supplierDocks::Vector{SupplierDock}
    plants::Vector{Plant}
    plantDocks::Vector{PlantDock}
    products::Vector{Product}
    parameters::Parameters
end