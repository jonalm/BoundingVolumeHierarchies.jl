"""
Axis Aligned Bounding Box, defined by the min/max coordinates.
"""
struct AABB
    min::PointT
    max::PointT
end

AABB(b::BoxedCells) = AABB(minimum(b.min, dims=2) .- eps(FloatT),
                           maximum(b.max, dims=2) .+ eps(FloatT))

diagonal(bb::AABB) = bb.max .- bb.min
area(bb::AABB) = _area(diagonal(bb))
volume(bb::AABB) = _volume(diagonal(bb))

function vertices(bb::AABB)
    a, b = bb.min, bb.max
    PointT[a, (b[1], a[2], a[3]), (a[1], b[2], a[3]), (a[1], a[2], b[3]),
              (a[1], b[2], b[3]), (b[1], a[2], b[3]), (b[1], b[2], a[3]), b]
end

faces(::AABB) = RectFaceIdxT[(1,3,7,2), (1,2,6,4), (1,4,5,3), (8,6,2,7), (8,7,3,5), (8,5,4,6)]
normals(::AABB) = PointT[(0,0,-1), (0,-1,0), (-1,0,0), (1,0,0), (0,1,0), (0,0,1)]
inside(point::PointT, aabb::AABB) =  all(aabb.min .< point .< aabb.max)
