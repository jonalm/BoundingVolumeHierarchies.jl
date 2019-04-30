struct BVHParam
    SAHfrac::FloatT
    BVHParam(SAHfrac=12.0f0) = new(SAHfrac) # default values
end

struct BVH{N, C}
    aabb::AABB
    children::NTuple{N, C} # either N=2 and C=BVH, or N=1 and C=FaceIndices
    BVH(bv::AABB, children...) = new{length(children), eltype(children)}(bv, children)
end
children(n::BVH) = n.children

struct FaceIndices
    inds::Vector{Int}
end
children(n::FaceIndices) = ()

# _Branch and _Leaf are only used to construct the BVH recursively
struct _Branch{T} data::T; end
struct _Leaf{T} data::T; end

_children(::_Leaf, param::BVHParam) = ()
function _children(n::_Branch, bp::BVHParam)
    bc = n.data
    N, A = length(bc), area(AABB(bc))
    optsplit = optimal_split(bc)
    # optsplit.cost / A approximates the expected number of checks when partitioning the cells
    if  N - optsplit.cost/A > bp.SAHfrac
        bc1, bc2 = subdivide(bc, optsplit)
        return (_Branch(bc1), _Branch(bc2))
    else
        return (_Leaf(bc),)
    end
end
_buildBVH(n::_Leaf, ::BVHParam) = FaceIndices(n.data.indices)
_buildBVH(n::_Branch, bp::BVHParam) = BVH(AABB(n.data), [_buildBVH(c, bp) for c in _children(n, bp)]...)

buildBVH(vertices, faces, bp::BVHParam) = _buildBVH(_Branch(BoxedCells(vertices, faces)), bp)
buildBVH(vertices, faces) = buildBVH(vertices, faces, BVHParam())

resetBVH(bvh::BVH, vertices, faces) = nothing

"""
Returns an iterator over all subsets of face indices such that

`f(x::AABB) == true`

for all parent axis aligned bounding boxes `x`.
"""
faceindices(f, n::BVH{N, C}) where {N, C<:FaceIndices} = (c.inds for c in n.children)
faceindices(f, n::BVH{N, C}) where {N, C<:BVH} = Iterators.flatten(faceindices(f, c)
    for c in children(n) if isa(c, FaceIndices) || f(c.aabb))
faceindices(head::BVH{N, C}) where {N, C<:BVH} = filter_faceindices(x->true, head)

"""
Returns an iterator over a filtered subset of the bounding volumes in the BVH
such that

`f(x::AABB) == true`

for all parent axis aligned bounding boxes `x`.
"""
function boundingvolumes(f, head::BVH)
    Channel() do c
        if f(head.aabb)
            queue = BVH[head]
            while !isempty(queue)
                current = pop!(queue)
                put!(c, current.aabb)
                append!(queue, [c for c in current.children if isa(c, BVH) && f(c.aabb)])
            end
        end
    end
end
boundingvolumes(head::BVH) = boundingvolumes(x->true, head)
