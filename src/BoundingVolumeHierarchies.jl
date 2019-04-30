module BoundingVolumeHierarchies

using LinearAlgebra
import GeometryTypes: area, Face, faces, OffsetInteger, vertices, volume, Point

# will only consider 3D and use Float32 for coordinates
const FloatT = Float32
const PointT = Point{3, FloatT}
const TriCellT = NTuple{3, PointT}
const RectCellT = NTuple{4, PointT}
const TriFaceIdxT = Face{3, OffsetInteger{-1,UInt32}}
const RectFaceIdxT = Face{4, OffsetInteger{-1,UInt32}}

# helper functions: d is assumed to be diagonal of cuboid
_area(d) = 2(d[1]*d[2] + d[2]*d[3] + d[3]*d[1])
_volume(d) = d[1]*d[2]*d[3]

include("BoxedCells.jl")
include("AABB.jl")
include("BVH.jl")
include("Intersection.jl")

export
    # BoundingVolumes.jl
    AABB,
    vertices,
    faces,
    # BoundingVolumeHierarchy.jl
    BVHParam,
    buildBVH,
    BVH,
    faceindices,
    boundingvolumes,
    # Intersection.jl
    Segment,
    intersection

end # module
