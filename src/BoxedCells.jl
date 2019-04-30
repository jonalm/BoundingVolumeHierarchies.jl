# used to build the Bounding Volume Hierarchy

"""
BoxedCells wraps around a set of face indices of a mesh, and the AABBs (axis
aligned bounding boxes) of the corresponding faces, identified by min/max
coordinates.
"""
struct BoxedCells
    min::Matrix{FloatT}
    max::Matrix{FloatT}
    center::Matrix{FloatT}
    indices::Vector{Int}
end


"""
BoxedCells constructor from a triangular mesh defined by

    - vertices: vertex coordinates
    - faces: triplets of indices (which maps vertices to corners of the face triangle)
"""
function BoxedCells(vertices, faces)
    triplets = (vertices[f] for f in faces)
    min_    = [m[i] for m in (min.(t...) for t in triplets), i in 1:3]'
    max_    = [m[i] for m in (max.(t...) for t in triplets), i in 1:3]'
    center_ = 0.5 * (min_ + max_)
    BoxedCells(min_, max_, center_, 1:length(faces))
end

Base.length(bc::BoxedCells) = length(bc.indices)

function Base.sort!(bc::BoxedCells, i::Int)
    perm          = sortperm(bc.center[i, :])
    bc.min[:]     = bc.min[:, perm]
    bc.max[:]     = bc.max[:, perm]
    bc.center[:]  = bc.center[:, perm]
    bc.indices[:] = bc.indices[perm]
    nothing
end

function Base.reverse!(bc::BoxedCells)
    bc.min[:] = bc.min[:, end:-1:1]
    bc.max[:] = bc.max[:, end:-1:1]
    bc.center[:] = bc.center[:, end:-1:1]
    bc.indices[:] = bc.indices[end:-1:1]
    nothing
end

function set_area!(area::Vector, bc::BoxedCells)
    tmpmin, tmpmax = bc.min[:,1], bc.max[:,1]
    area[1] = _area(tmpmax .- tmpmin)
    for i in 2:(length(bc)-1)
        tmpmin[:] = min.(tmpmin, bc.min[:,i])
        tmpmax[:] = max.(tmpmax, bc.max[:,i])
        area[i] = _area(tmpmax .- tmpmin)
    end
    nothing
end

function optimal_split(bc::BoxedCells, axis::Int, buffer1::Vector{FloatT}, buffer2::Vector{FloatT})
    N = length(bc)
    @assert N-1 == length(buffer1) == length(buffer2)
    sort!(bc, axis)
    set_area!(buffer1, bc)
    reverse!(bc)
    set_area!(buffer2, bc)
    return minimum((cost=i*a1_ + (N-i)*a2_, index=i, sortaxis=axis)
                   for (i, (a1_, a2_)) in enumerate(zip(buffer1,reverse(buffer2))))
end

function optimal_split(bc::BoxedCells)
    N = length(bc)
    # create buffers
    buffer1 = Vector{Float32}(undef, N-1)
    buffer2 = Vector{Float32}(undef, N-1)
    min(optimal_split(bc, 1, buffer1, buffer2),
        optimal_split(bc, 2, buffer1, buffer2),
        optimal_split(bc, 3, buffer1, buffer2))
end

function subdivide(bc::BoxedCells, optsplit)
    i, axis = optsplit.index, optsplit.sortaxis
    sort!(bc, axis)
    (BoxedCells(bc.min[:,1:i], bc.max[:,1:i], bc.center[:,1:i], bc.indices[1:i]),
     BoxedCells(bc.min[:,i+1:end],bc.max[:,i+1:end], bc.center[:,i+1:end], bc.indices[i+1:end]))
end
