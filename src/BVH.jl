# The Bounding Volume Hierarchy (BVH) is implemented as a binary tree where
# the `BVH` as nodes and `FaceIndices` as leaves at the bottom of the hierarchy.
#
#  - BVH contain an axis aligned bounding volume and a tuple of children (either
#    two BVHs or one single FaceIndices)
#  - FaceIndices holds the indices of the faces of the original mesh. (A mesh is
#    identified by a vertices buffer, and a faces buffer, containing triplets of
#    indices of the vertex buffer which identifies mesh cells). I.e.
#    `vertices[faces[faceindices.inds]]` gives the associated vertices at a bottom
#    `faceindices` node.
#
# The "Surface Area Heuristic" method is used to build the BVH. The `BVHParam.SAHfrac`
# variable is the computational cost fraction between traversing one level in the BVH and
# checking a mesh cell for intersection. I.e. the cost of checking collision of the bounding
# boxes versus one single cell. (the bounding boxes are essentially composed of 6 cells each,
# so this franction should be ≃ 12)
#
# TODOs:
# 1. The BVM uses only AABBs as of now, can optimize by using an optimal
#    bounding volume on each branch of the tree.
# 2. The BVM is currently a binary tree, could potentially be improved
#    by a varying number of children at each node.
# 3. Optimize the `BVHParam.SAHfrac`.
#

struct BVHParam
    SAHfrac::FloatT
    BVHParam(SAHfrac=12.0f0) = new(SAHfrac) # default values
end

struct BVH{N, C}
    boundingvolume::AABB
    children::NTuple{N, C}
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
    n, a = length(bc), area(AABB(bc))
    partition, cost = optimal_partition(bc)
    # cost/a approximates the expected number of checks when partitioning the cells
    if  n - cost/a > bp.SAHfrac # see note at the top of this file
        bc1, bc2 = subdivide(bc, partition)
        return (_Branch(bc1), _Branch(bc2))
    else
        return (_Leaf(bc),)
    end
end
_build_BVH(n::_Leaf, ::BVHParam) = FaceIndices(n.data.indices)
_build_BVH(n::_Branch, bp::BVHParam) = BVH(AABB(n.data), [_build_BVH(c, bp) for c in _children(n, bp)]...)

build_BVH(vertices, faces, bp::BVHParam) = _build_BVH(_Branch(BoxedCells(vertices, faces)), bp)
build_BVH(vertices, faces) = build_BVH(vertices, faces, BVHParam())
