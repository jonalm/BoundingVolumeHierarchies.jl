# used to build the Bounding Volume Hierarchy

"""
BoxedCells wraps around a set of face indices of a mesh, and the AABBs (axis
aligned bounding boxes) of the corresponding faces, identified by min/max
coordinates.
"""
struct BoxedCells
    min::Matrix{FloatT}  #    size N x 3
    max::Matrix{FloatT}  #    size N x 3
    center::Matrix{FloatT} #
    indices::Vector{Int} #  lenght N
end
Base.length(b::BoxedCells) = length(b.indices)
center(bc::BoxedCells) = 0.5 * (bc.min + bc.max)

function Base.sort!(bc::BoxedCells,i::Int)
    perm          = sortperm(bc.center[:,i])
    bc.min[:]     = bc.min[perm,:]
    bc.max[:]     = bc.max[perm,:]
    bc.center[:]  = bc.center[perm,:]
    bc.indices[:] = bc.indices[perm]
end

"""
BoxedCells constructor from a triangular mesh defined by

    - vertices: vertex coordinates
    - faces: triplets of indices (which maps vertices to corners of the face triangle)
"""
function BoxedCells(vertices, faces)
    triplets = (vertices[f] for f in faces)
    min_    = [m[i] for m in (min.(t...) for t in triplets), i in 1:3]
    max_    = [m[i] for m in (max.(t...) for t in triplets), i in 1:3]
    center_ = 0.5 * (bc.min + bc.max)
    BoxedCells(min_, max_, center_, 1:length(faces))
end

valid_partition(p::BitVector) = 0 < sum(p) < length(p) # must contain true and false
partitions_unfiltered(m::Matrix) = (m[:,i] .< m[j,i] for i in 1:size(m)[2] for j in 1:size(m)[1])

"""
Returns an iterator over all potential partitions.

The partition are represented as bitarrays.

A partition is constructed by dividing the cells (defined by their center
coordinate) at a point along an axis (x, y or z).
"""
partitions(m::Matrix) = (p for p in partitions_unfiltered(m) if valid_partition(p))
partitions(bc::BoxedCells) = partitions(center(bc))

function subdivide(bc::BoxedCells, partition::BitVector)
    p, np = partition, .~partition
    bc1 = BoxedCells(bc.min[ p,:], bc.max[ p,:], bc.indices[ p])
    bc2 = BoxedCells(bc.min[np,:], bc.max[np,:], bc.indices[np])
    bc1, bc2
end

function area(bc::BoxedCells, partition)
    max_ = @view bc.max[partition,:]
    min_ = @view bc.min[partition,:]
    diag = maximum(max_, dims=1) - minimum(min_, dims=1)
    _area(diag)
end

function partition_cost(bc::BoxedCells, partition)
    p, np = partition, .~partition
    sum(p) * area(bc, p) + sum(np) * area(bc, np)
end

function optimal_partition(bc::BoxedCells)
    function folder(prevstate, partition)
        optpart, optval = prevstate
        checkval = partition_cost(bc, partition)
        checkval < optval ? (partition, checkval) : prevstate
    end
    foldl(folder, partitions(bc), init=(BitVector(undef, length(bc)), Inf32))
end
