module BoundingVolumeHierarchies

using LinearAlgebra

import AbstractTrees: children, printnode
import GeometryTypes: faces, vertices, area, volume
import GeometryTypes

# will only consider 3 dimension and use Float32 for coordinates
const FloatT      = Float32
const PointT      = GeometryTypes.Point{3, FloatT}

# triangle mesh cell
const TriCellT    = NTuple{3, PointT}
const TriFaceIdxT = GeometryTypes.Face{3,GeometryTypes.OffsetInteger{-1,UInt32}}

 # rectangular mesh cell
const RectCellT     = NTuple{4, PointT}
const RectFaceIdxT  = GeometryTypes.Face{4,GeometryTypes.OffsetInteger{-1,UInt32}}

# helper functions:
# d is assumed to be diagonal of cuboid
_area(d)   = 2(d[1]*d[2] + d[2]*d[3] + d[3]*d[1])
_volume(d) = d[1]*d[2]*d[3]

include("BoxedCells.jl")
include("AABB.jl") # axis aligned bounded box
include("BVH.jl")

a=3
export
    a,
    # BoundingVolumes.jl
    AABB,
    vertices,
    faces,
    # BoundingVolumeHierarchy.jl
    BVHParam,
    build_BVH,
    BVH
    #filter_faceindices,
    #faceindices,
    #filter_boundingvolumes,
    #boundingvolumes,
    # Intersection.jl
    #Segment,
    #intersection

end # module
