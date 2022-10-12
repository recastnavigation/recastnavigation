# Configuring Recast with rcConfig
The following describes best practices for configuring Recast.  They're broken down by the fields in `rcConfig` and are ordered by the sequence in which you should consider them.

## Determine Agent Size
First you should decide the size of your agent's logical cylinder. If your game world uses meters as units, a reasonable starting point for a human-sized agent might be a radius of `0.4` and a height of `2.0`.

## cs
The voxelization cell size `cs` defines the voxel size along both axes of the ground plane: x and z in Recast. This value is usually derived from the character radius `r`. A recommended starting value for `cs` is either `r/2` or `r/3`. Smaller values of `cs` will increase rasterization resolution and navmesh detail, but total generation time will increase exponentially.  In outdoor environments, `r/2` is often good enough.  For indoor scenes with tight spaces you might want the extra precision, so a value of `r/3` or smaller may give better results.

The initial instinct is to reduce this value to something very close to zero to maximize the detail of the generated navmesh. This quickly becomes a case of diminishing returns, however. Beyond a certain point there's usually not much perceptable difference in the generated navmesh, but huge increases in generation time.  This hinders your ability to quickly iterate on level designs and provides little benefit.  The general recommendation here is to use as large a value for `cs` as you can get away with.

## ch
The voxelization cell height `ch` is defined separately in order to allow greater precision in height tests. A good starting point for `ch` is half the `cs` value. Smaller `ch` values ensure that the navmesh properly connects areas that are only separated by a small curb or ditch.  If small holes are generated in your navmesh around where there are discontinuities in height (for example, stairs or curbs), you may want to decrease the cell height value to increase the vertical rasterization precision of Recast.

## walkableHeight
This value defines the height `h` of the agent in voxels. This value should be calculated as `ceil(h / ch)`.  Note this is based on `ch` not `cs` since it's a height value.

## walkableClimb
The `walkableClimb` value defines the maximum height of ledges and steps that the agent can walk up. Given a designer-defined `maxClimb` distance in world units, this value should be calculated as `ceil(maxClimb / ch)`.  Again, note that this is using `ch` not `cs` because it's a height-based value.

## walkableRadius
The parameter `walkableRadius` defines the agent radius in voxels.  Most often, this value should be calculated as `ceil(r/cs)`. Note this is based on `cs` since the agent radius is always parallel to the ground plane.

If the `walkableRadius` value is greater than zero, the edges of the navmesh will be pushed away from all obstacles by this amount.

A non-zero `walkableRadius` allows for much simpler runtime navmesh collision checks. The game only needs to check that the center point of the agent is contained within a navmesh polygon.  Without this erosion, runtime navigation checks need to collide the geometric projection onto the navmesh of agent's logical cylinder with the boundary edges of the navmesh polygons.

Navmesh erosion is performed on the voxelized world representation, so some precision can be lost there. This step allows simpler checks at runtime. 

If you want to have tight-fitting navmesh, or want to reuse the same navmesh for multiple agents with differing radii, you can use a `walkableRadius` value of zero.

## walkableSlopeAngle
The parameter `walkableSlopeAngle` is to filter out areas of the world where the ground slope would be too steep for an agent to traverse. This value is defined as a maximum angle in degrees that the surface normal of a polgyon can differ from the world's up vector.  This value must be within the range `[0, 90]`.

## maxEdgeLen
In certain cases, long outer edges may decrease the quality of the resulting triangulation, creating very long thin triangles. This can sometimes be remedied by limiting the maximum edge length, causing the problematic long edges to be broken up into smaller segments. 

The parameter `maxEdgeLen` defines the maximum edge length and is defined in terms of voxels. A good value for `maxEdgeLen` is something like `walkableRadius * 8`. A good way to adjust this value is to first set it really high and see if your data creates long edges. If it does, decrease `maxEdgeLen` until you find the largest value which improves the resulting tesselation.

## maxSimplificationError
When the rasterized areas are converted back to a vectorized representation, the `maxSimplificationError` describes how loosely the simplification is done.  The simplification process uses the [Ramer–Douglas-Peucker algorithm](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm), and this value describes the max deviation in voxels. 

Good values for `maxSimplificationError` are in the range `[1.1, 1.5]`.  A value of `1.3` is a good starting point and usually yields good results. If the value is less than `1.1`, some sawtoothing starts to appear at the generated edges.  If the value is more than `1.5`, the mesh simplification starts to cut some corners it shouldn't.

## minRegionSize
Watershed partitioning is really prone to noise in the input distance field. In order to get nicer areas, the areas are merged and small disconnected areas are removed after the water shed partitioning. The parameter `minRegionSize` describes the minimum isolated region size that is still kept. A region is removed if the number of voxels in the region is less than the square of `minRegionSize`.

## mergeRegionSize
The triangulation process works best with small, localized voxel regions. The parameter `mergeRegionSize` controls the maximum voxel area of a region that is allowed to be merged with another region.  If you see small patches missing here and there, you could lower the minRegionSize.
