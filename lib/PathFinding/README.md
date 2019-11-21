# Pathfinding

The pathfinding module is a wrapper for *Recast* and *Detour*, which respectively provide the navmesh generation and path planning. The major classes in this module are the `NavMeshBuilder`, the `NavMeshNavigator`, and the `NavMeshShader`. The builder handles navmesh generation, the navigator is a wrapper for the Detour functionality, and the shader integrates with the Lighthouse2 `RenderAPI` to give a visual representation of a navmesh.

## Usage

#### NavMeshBuilder

A `NavMeshBuilder` instance is initialized with a directory in which it keeps the resulting navmeshes. A navmesh can be generated using `NavMeshBuilder::Build`, which needs a `HostScene` pointer. The resulting navmesh can be saved using `NavMeshBuilder::Serialize` and loaded with `NavMeshBuilder::Deserialize`. After building, errors can be checked by calling `NavMeshBuilder::GetError`, which is of type `navmesh_common.h/NavMeshErrorCode`. More detailed information on the error can be found in the terminal. Calling `NavMeshBuilder::DumpLog` prints detailed build statistics of the last build.

The generation procedure itself consists of 7 parts. The first step is rasterization, in which the scene is converted into a voxel representation. The choice of voxel size, which can be set by the user, is a trade-off between computation time and level of detail. After rasterization, the voxel model is filtered for walkable voxels based on the parameters set by the user. Recast then places 2D regions on top of these chunks. Steps 4-6 convert these regions into connected convex polygons, represented by two meshes: the polygon mesh and the detail mesh. The polygon mesh is a crude representation of traversability and polygon connections, which is used for pathfinding. The detail mesh stores the exact surface height of each point on the polygon. The final step combines these two meshes into one navmesh that can be used by Detour. When the pmesh and dmesh have been manually edited, or when off-mesh connections have been added with `NavMeshBuilder::AddOffMeshConnection`, this last step has to be redone to refresh the Detour data. Hence, editing the navmesh requires the pmesh and dmesh to still be there. More info on the navmesh generation can be found in the official [Recast documentation](http://masagroup.github.io/recastdetour/group__recast.html).

The parameters for the building process are stored in `NavMeshBuilder::m_config`, which are saved and loaded alongside the navmesh. This `NavMeshConfig` struct contains the following parameters:
* `m_width`/`m_height`/`m_tileSize`/`m_borderSize`: Set by the building process; represents the voxel array dimensions
* `m_cs`/`m_ch`: The voxel cell size (width/depth) and cell height respectively
* `m_bmin`/`m_bmax`: The dimensions of the axis aligned bounding box within which the navmesh should remain
* `m_walkableSlopeAngle`: The maximum slope the agent can traverse
* `m_walkableHeight`: The minimum height undernath which the agent can walk, expressed in voxels
* `m_walkableClimb`: The maximum number of voxels the agent can climb (e.g. stairs)
* `m_walkableRadius`: The minimum width through which the agent can traverse (i.e. the agent's width)
* `m_maxEdgeLen`: The maximum length of any polygon edge.
* `m_maxSimplificationError`: The maximum distance a polygon edge can differ from that in the original contour set.
* `m_minRegionArea`: The minimum number of connected voxels that constitute a surface (filters islands)
* `m_mergeRegionArea`: The minimum number of connected voxels needed to not be merged with a neighboring surface
* `m_maxVertsPerPoly`: The maximum number of polygon vertices (3-6)
* `m_detailSampleDist`: The sampling distance used in creating the detail mesh
* `m_detailSampleMaxError`: The maximum distance the detail mesh surface can deviate from the heightfield.
* `m_partitionType`: The partitioning method to use. The options are:
    * **Watershed partitioning** (best for precomputed navmeshes and open areas)
        * the classic Recast partitioning
        * creates the nicest tessellation, but usually the slowest
        * partitions the heightfield into nice regions without holes or overlaps
        * the are some corner cases where this method produces holes and overlaps:
            * holes may appear when a small obstacle is close to large open area (triangulation won't fail)
            * overlaps may occur on narrow spiral corridors (i.e stairs) and triangulation may fail
    * **Monotone partitioning** (fast navmesh generation)
        * fastest
        * partitions the heightfield into regions without holes and overlaps (guaranteed)
        * creates long thin polygons, which sometimes causes paths with detours
    * **Layer partitioning** (best for navmeshes with small tiles)
        * quite fast
        * partitions the heighfield into non-overlapping regions
        * relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
        * produces better triangles than monotone partitioning
        * does not have the corner cases of watershed partitioning
        * can be slow and create a bit ugly tessellation (still better than monotone) if you have large open areas with small obstacles (not a problem if you use tiles)
* `m_keepInterResults`: Whether or not to keep the intermediate results, such as the voxel model, pmesh, and dmesh
* `m_filterLowHangingObstacles`: Whether to filter for low hanging obstacles
* `m_filterLedgeSpans`: Whether to filter for ledge spans
* `m_filterWalkableLowHeightSpans`: Whether to filter for low height spans
* `m_printBuildStats`: Whether to print detailed build statistics
* `m_id`: A string identifying the navmesh. Used in the filename while saving. Loading a navmesh requires the user to set the ID first, as it looks for the file containing this ID.

#### NavMeshNavigator

A `NavMeshNavigator` instance provides the runtime path planning functionality. It can be initialized by loading a precomputed navmesh, or directly asking one from the builder using `NavMeshBuilder::GetNavigator`.

You can find a path by calling `NavMeshNavigator::FindPath` or `NavMeshNavigator::FindPathConstSize`, the former being more convenient, and the latter being slightly more optimized. Both of these functions return a series of `PathNode` structs, which consist of a target location and a reference to the associated polygon. They also return a boolean indicating whether the final target appears to be reachable (true when in doubt). The path can be constrained by providing a `dtQueryFilter` instance to indicate the types of polygons that cannot be traversed by this agent (e.g. riverbed, ziplines). The flags used by this filter are user defined, and manually assigned to polygons.

Other functions include `NavMeshNavigator::FindNearestPoly`, to find the navmesh polygon closest to a given position, and `NavMeshNavigator::FindClosestPointOnPoly`, to find the point on a given poly closest to a given position.

#### NavMeshAgents

A `NavMeshAgents` instance keeps track of all agents and their updates. An `Agent` can be added by calling `NavMeshAgents::AddAgent` with a `NavMeshNavigator` and a `RigidBody`, and removed using an `Agent` pointer by calling `NavMeshAgents::RemoveAgent`. Note that the `Agent` owns neither its navmesh nor its rigidbody. One navmesh can be used by multiple agents of the same type, and the rigidbodies are owned and updated by the physics engine (currently `PhysicsPlaceholder`).

An agent can be given a target with `Agent::SetTarget`, which can either be a static target or, when given a pointer to a position, a dynamic target. There are two update functions: `Agent::UpdateMovement` should be called before every physics update to allow the agent to move; `Agent::UpdateNavigation` can be called less frequently, or even sporadically depending on the applicatoin, to update the agent's path. Both of these update cycles abbreviated by the `NavMeshAgents::UpdateAgentMovement` and `NavMeshAgents::UpdateAgentBehavior` functions, the latter of which uses an internal timer to only update at a given interval. The booleans they return indicate whether any of the agent's states have actually changed (e.g. when none of the agents have a target, both functions will return false).

#### NavMeshShader

The `NavMeshShader` provides a graphical representation of all of the above classes. An instance can be constructed by passing a directory with the object meshes (found in PathFinding/assets), and a pointer to the renderer. Please note that for this class to be interactive, the renderer will need to support scene probing, since many functions rely on instance IDs, mesh IDs, or scene positions.

To shade a navmesh, pass a `NavMeshNavigator` pointer to `NavMeshShader::UpdateMesh` before calling either `NavMeshShader::AddNavMeshToScene()` (recommended) or `NavMeshShader::AddNavMeshToGL()` (not recommended). In order for highlights and GL shading to show, the main loop should call `NavMeshShader::DrawGL()` after rendering. Additionally, when using agents, the main loop should call `NavMeshShade::UpdateAgentPositions` after the physics update to move all agents to their new location.

The shader has individual Add- and Remove functions for polygons, edges, vertices, and agents. Any of these objects can be highlighted by calling their respective `Select` functions, which return a pointer to the selected object. Call `NavMeshShader::Deselect` to remove all highlights.

Paths can be drawn by passing a pointer to a precalculated path to `NavMeshShader::SetPath`. Similarly, the path start- and end beacon can be set by passing the `float3` pointers to `NavMeshShader::SetPathStart` and `NavMeshShader::SetPathEnd`. The shader will automatically detect changes to any of the data.

To fascilitate graphical navmesh editing, the shader can add-, move-, and remove one temporary vertex with `NavMeshShader::SetTmpVert` and `NavMeshShader::RemoveTmpVert`, as well as add off-mesh connections during rutime with `NavMeshShader::AddTmpOMC`.

<br/>
<br/>

## Backlog

* Add code documentation to readme ❗

#### NavMeshBuilder
* building
    * give builder closure that automatically annotates polygons based on parameters
* off-mesh connections
    * debug m_navmesh recalculation ❗
    * test saving/loading ❗
* editing
    * find out how to apply changes to vertices
    * save/load `m_pmesh` and `m_dmesh` to allow editing of loaded projects

#### NavMeshNavigator / Agent
* BUG: when close to unreachable goal (above), path update plans vertical path
* Don't call 'arrive' behavior on every path corner
* Add behavior (flee/follow)


#### NavMeshShader
* BUG: agents/vertices won't select after rebuild
    * instIDs differ ❗
* make meshes transparent
* off-mesh connections
    * BUG: OMC edge wrong mesh? ❗
    * make OMCs added during runtime temporary/removable
    * how to check directionality in `NavMeshShader::AddOMCsToScene` and `NavMeshShader::AddTmpOMC`
* remove old navmesh mesh from render core on `Clear` to save memory
* why are there no detail verts?
* OpenGL highlights cost 10-20 fps
* BUG: OpenGL shading fails when the camera is too close (when a vertex is behind the camera) (`Camera::WorldToScreenPos` issue)