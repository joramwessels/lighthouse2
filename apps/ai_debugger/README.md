# AI Debugger
The AI Debugger app fascilitates the generation-, augmentation-, and debugging of navigation meshes using a graphical interface.

Dependencies:
* The pathfinding module, which requires the recastnavigation submodule
* Any render core that implements scene probing

![ScreenShot](../../screenshots/ai_debugger.png)

## Usage

You can move the camera using WASD, and rotate using the arrow keys. Hold SHIFT to move faster.

#### Building

Navmeshes can be generated using the 'build' tab. The `NavMeshBuilder` class is automatically given the `HostScene` of the renderer, which is initialized in `main.cpp/PrepareScene()`. The following navmesh building parameters can be altered in the 'bulid' tab:

* `AABB min/max`: The bounding box limiting the navmesh. Defaults to the scene bounds when 0.
* `cell size`: The width and depth of the voxel grids during rasterization.
* `cell height`: The height of the voxel grids during rasterization.
* `max slope`: The maximum slope this agent can traverse.
* `min height`: The minimum height the agent needs (i.e. the agent's height) expressed in voxels.
* `max climb`: The maximum number of voxels the agent can climb (e.g. stairs).
* `min radius`: The minimum width/depth the agent needs (i.e. the agent's width).
* `filtering`: Whether to filter for low hanging obstacles/ledge spans/low height spans.
* `partition type`: The method used for partitioning. The options are:
    * Watershed partitioning
    * Monotone partitioning
    * Layer partitioning
* `min region area`: The minimum number of connected voxels that constitue a traversable surface. Surfaces with less voxels will be discarded.
* `min merged region`: The minimum number of voxels needed to be a separate region. When smaller than this, the region will (if possible) be merged with a neighboring region.
* `max edge length`: The maximum length any navmesh edge can have, expressed in voxels.
* `max simpl err`: The maximum distance a border edge can deviate from the generated contours.
* `max verts per poly`: The maximum number of vertices in a polygon, ranging from 3 to 6.
* `detail sample dist`: The sampling distance used to create the detail mesh.
* `detail max err`: The maximum distance the detail mesh surface can deviate from the heightfield.
* `NavMesh ID`: A string that identifies this navmesh. When saving, the file name will include this ID, and when loading, the app looks for the file with this ID.
* `Print build stats`: Whether to print detailed build statistics.
* `error code`(read only): If any errors occur, this will display "ERROR". Consult the terminal for further information.

Navmeshes can be built, saved, loaded, and cleared with the buttons underneath these options. Please note that, currently, only the navmesh itself is saved. Loading a navmesh does not recover the intermediate build results required to edit it.

#### Editing

To edit a navmesh, first generate one using the build menu and activate `EDIT` mode using the uppermost button on the edit menu. To select a polygon, edge, or vertex, hold SHIFT and left click the object. The edit menu now displays the details of the object. *(Editing hasn't been implemented yet)*.

To add an off mesh connection, hold CTRL and left click to set the starting point, then right click to set the end. Once both points have been set, the OMC is added to the builder. You can hit 'Apply Changes'and switch to the debug menu to test it, and then switch back to the edit or build menu to save it.

#### Debugging

To test the navmesh, activate the debug menu using the uppermost button. Paths can be drawn by holding CTRL. Left click to set the start, right click to set the end.

To add an agent, hold SHIFT while right clicking anywhere on the navmesh. Hold SHIFT and left click to select an agent. When an agent is selected, hold CTRL and right click to give it a target. The debug menu displays all relevant details on the agent and its path.

<br/>
<br/>

## Backlog

#### Building
* extensive QA testing for all build params
* give instances/meshes a flag that can exclude them from navmesh generation

#### Editing
* navmesh editing
    * finish AntTweakBar vert hiding ❗
    * spawn unit axis at selected vert/edge
    * controlls to drag unit axis
    * make local copy of changed objects
        * Make NavMeshShader verts reference this local copy
    * apply changes in AntTweakBar callback
* off-mesh connections
    * debug traversal ❗
    * test saving/loading ❗
    * how to check directionality in `navmesh_shader.cpp/NavMeshShader::AddOMCsToScene` and `navmesh_shader.cpp/NavMeshShader::AddTmpOMC`
    * make OMCs added to shader during runtime temporary/removable
    * make OMCs visible and editable in the edit menu (radius, directionality)
* edit menu
    * add polygon flags/areas ❗

#### Debugging
* add agent editing (dtQueryFilter, speed)

#### Shading
* fix instance deleting ❗
* make meshes transparant
    * BUG: OMC edge wrong mesh? ❗
* are calls to SynchronizeSceneData() necessary after add operation?
* OpenGL highlights cost 10-20 fps
* OpenGL highlights lag behind 1 frame due to scene probing delay

#### UI
* add mouse movement
    * middle mouse draggin for up/down
    * right mouse dragging for rotation (except when CTRL SHIFT)
* typing w/a/s/d in navmesh ID field moves camera
* automatic conversion tab in build menu
    * agent height/radius/climb in world coordinates
    * conversion to voxels before building
* add pause button to disables agents/physics updates
* don't refresh navmesh when nothing has been edited