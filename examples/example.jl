using BenchmarkTools
using BoundingVolumeHierarchies
using LinearAlgebra
using Makie
using StatsBase

import GeometryTypes: HomogenousMesh
import BoundingVolumeHierarchies: PointT, _normal

##

catmesh = Makie.loadasset("cat.obj")
catverts, catfaces = catmesh.vertices, catmesh.faces
catnormals = [_normal(catverts[f]) for f in catfaces] # mesh cell normals

catBVH = buildBVH(catverts, catfaces)


## Plot the cat mesh together with the bounding boxes in the Bounding Volume Hierarchy

scene = Scene(center=false)

wireframe!(scene, catmesh, linewidth=0.5, color=:black)

for bv in boundingvolumes(catBVH)
    wireframe!(scene, HomogenousMesh(vertices(bv), faces(bv)), linewidth = 0.5, color=:red)
end

cam = Makie.cameracontrols(scene)
cam.upvector[] = Point(0.,1.,0.)
cam.eyeposition[] = Point(3,2.,4)
update_cam!(scene, cam)

display(scene)


## benchmark naive method vs using a BVH

# segments from a fixed source to a random point constrained to a plane
# such that the catmesh is between the source and the plane
create_segments() =  [Segment(PointT(1, 0.4, 0.4),
                              PointT(-0.4, rand(), 2*rand().-0.7)) for _ in 1:10_000]

segments = create_segments()

# check all meshcell for each segment
naiveintersection(s, verts, faces, normals) = any(intersection(s, verts[f], n) for (f,n) in zip(faces, normals))

naive_hits = @btime [naiveintersection(s, catverts, catfaces, catnormals) for s in segments]
bvh_hits = @btime [intersection(s, catBVH, catverts, catfaces, catnormals) for s in segments]
@assert all(bvh_hits .== naive_hits)


##

# Plot the mesh together with a scatter of the end points of a set of segments
# all segments starts at the same center point (large orange ball)
# all segments end on a plane on the opposite side of the cat mesh,
# end ponints are colored black if the segment intersects with the catmesh
# and orange otherwise.

scene = Scene(center=false)

wireframe!(scene, catmesh, linewidth=0.5, color=:black)
scatter!(scene, [s.b for (s,h) in zip(segments, bvh_hits) if h], color=:black, markersize=0.01)
scatter!(scene, [s.b for (s,h) in zip(segments, bvh_hits) if !h], color=:orange, markersize=0.01)
scatter!(scene, [segments[1].a], color=:orange)

cam = Makie.cameracontrols(scene)
cam.upvector[] = Point(0.,1.,0.)
cam.eyeposition[] = Point(3,2.,4)
update_cam!(scene, cam)

display(scene)
