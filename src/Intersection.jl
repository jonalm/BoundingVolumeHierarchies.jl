struct Segment
    a::PointT
    b::PointT
end

_normal(cell) = normalize(cross(cell[2] - cell[1], cell[3] - cell[1]))

function intersection(s::Segment, cell::TriCellT, cell_normal)
    dot(cell_normal, s.a-cell[1]) <= zero(FloatT) && return false
    dot(cell_normal, s.b-cell[1]) >= zero(FloatT) && return false
    BA = s.a - s.b
    dot(cross(cell[2]-cell[1], s.b-cell[2]), BA) < zero(FloatT) && return false
    dot(cross(cell[3]-cell[2], s.b-cell[3]), BA) < zero(FloatT) && return false
    dot(cross(cell[1]-cell[3], s.b-cell[1]), BA) < zero(FloatT) && return false
    return true
end

function intersection(s::Segment, cell::RectCellT, cell_normal)
    dot(cell_normal, s.a-cell[1]) <= zero(FloatT) && return false
    dot(cell_normal, s.b-cell[1]) >= zero(FloatT) && return false
    BA = s.a - s.b
    dot(cross(cell[2]-cell[1], s.b-cell[2]), BA) < zero(FloatT) && return false
    dot(cross(cell[3]-cell[2], s.b-cell[3]), BA) < zero(FloatT) && return false
    dot(cross(cell[4]-cell[3], s.b-cell[4]), BA) < zero(FloatT) && return false
    dot(cross(cell[1]-cell[4], s.b-cell[1]), BA) < zero(FloatT) && return false
    return true
end

function intersection_faces(s::Segment, aabb::AABB)
    ve, fa, no = vertices(aabb), faces(aabb), normals(aabb)
    any(intersection(s, ve[f], n) for (f,n) in zip(fa, no))
end

intersection(s::Segment, cell::T) where {T <: Union{TriCellT, RectCellT}} =
    intersection(s, cell, _normal(cell))

intersection(s::Segment, aabb::AABB) =
    inside(s.a, aabb) || inside(s.b, aabb) || intersection_faces(s, aabb)

function intersection(s::Segment, bvh::BVH, vertices, faces)
    for inds in faceindices(aabb->intersection(s, aabb), bvh)
        any(intersection(s, vertices[faces[i]]) for i in inds) && return true
    end
    return false
end

function intersection(s::Segment, bvh::BVH, vertices, faces, normals)
    for inds in faceindices(aabb->intersection(s, aabb), bvh)
        any(intersection(s, vertices[faces[i]], normals[i]) for i in inds) && return true
    end
    return false
end
