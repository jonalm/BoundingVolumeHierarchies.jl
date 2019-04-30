struct BVHParam
    SAHfrac::FloatT
    BVHParam(SAHfrac=12.0f0) = new(SAHfrac) # default values
end

struct BVH{N, C}
    aabb::AABB
    children::NTuple{N, C} # either N=2 and C=BVH, or N=1 and C=FaceIndices
    BVH(bv::AABB, children...) = new{length(children), eltype(children)}(bv, children)
end

struct FaceIndices
    inds::Vector{Int}
end

printnode(io::IO, n::BVH)         = print(io, "⧄")
printnode(io::IO, n::FaceIndices) = print(io, length(n.inds))
children(n::FaceIndices)          = ()
children(n::BVH)                  = n.children

# underscore _Branch and _Leaf are only used to construct the BVH recursively
struct _Branch{T} data::T; end
struct _Leaf{T}   data::T; end

_children(::_Leaf, param::BVHParam) = ()
function _children(n::_Branch, bp::BVHParam)
    bc = n.data
    N, A = length(bc), area(AABB(bc))
    optsplit = optimal_split(bc)
    # optsplit.cost / A approximates the expected number of checks when partitioning the cells
    if  N - optsplit.cost/A > bp.SAHfrac # see note at the top of this file
        bc1, bc2 = subdivide(bc, optsplit)
        return (_Branch(bc1), _Branch(bc2))
    else
        return (_Leaf(bc),)
    end
end
_build_BVH(n::_Leaf, ::BVHParam) = FaceIndices(n.data.indices)
_build_BVH(n::_Branch, bp::BVHParam) = BVH(AABB(n.data), [_build_BVH(c, bp) for c in _children(n, bp)]...)

build_BVH(vertices, faces, bp::BVHParam) = _build_BVH(_Branch(BoxedCells(vertices, faces)), bp)
build_BVH(vertices, faces) = build_BVH(vertices, faces, BVHParam()) # using default param
