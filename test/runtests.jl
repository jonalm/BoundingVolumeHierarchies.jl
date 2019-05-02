using BoundingVolumeHierarchies
using LinearAlgebra
using Random
using StatsBase
using Test

import BoundingVolumeHierarchies: intersection_faces, RectCellT, TriCellT

Random.seed!(1234)

@testset "test random intersection Segment vs AABB" begin
    aabb = AABB((-1,-1,-1), (1,1,1))
    b = (0,0,0)
    for _ in 1:1000
		a = 3*normalize(randn(3)) # random vector, |a| = 3
		@test BoundingVolumeHierarchies.intersection_faces(Segment(a,b), aabb)
		@test !BoundingVolumeHierarchies.intersection_faces(Segment(b,a), aabb)
		@test intersection(Segment(b,a), aabb)
		@test intersection(Segment(a,b), aabb)
    end
end

@testset "test random intersection Segment vs TriCell" begin
	t = TriCellT(((0,0,0), (1,0,0),(0,1,0)))
	xy = [(x=x_, y=rand()*(1-x_)) for x_ in rand(200)]
	for _ in 1:1000
	    p = sample(xy, 2)
	    @test intersection(Segment((p[1].x, p[1].y, 1), (p[2].x, p[2].y, -1)), t)
	    @test !intersection(Segment((p[1].x, p[1].y, -1), (p[2].x, p[2].y, 1)), t)
	end

end

@testset "test random intersection Segment vs RectCell" begin
	t = RectCellT(((0,0,0), (1,0,0), (1,1,0), (0,1,0)))
	xy = [(x=rand(), y=rand()) for _ in rand(200)]
	for _ in 1:1000
	    p = sample(xy, 2)
	    @test intersection(Segment((p[1].x, p[1].y, 1), (p[2].x, p[2].y, -1)), t)
	    @test !intersection(Segment((p[1].x, p[1].y, -1), (p[2].x, p[2].y, 1)), t)
	end

end
