from omni.isaac.kit import SimulationApp

# Start up Isaac Sim
kit = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.prims import XformPrim, RigidPrim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import open_stage_as_new

import asyncio

async def create_simple_scene():
    # Open a new empty stage
    open_stage_as_new()

    # Initialize the World object
    world = World(stage_units_in_meters=1.0) # 1.0 means 1 meter in USD
    await world.initialize_simulation()

    # Add a ground plane
    # We use a built-in asset for simplicity
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        print("Could not find Isaac Sim assets folder")
        kit.shutdown()
        return

    ground_plane_path = assets_root_path + "/Props/Shapes/Motion_Board.usd"
    world.scene.add_ground_plane(prim_path="/World/defaultGroundPlane", usd_path=ground_plane_path)

    # Add a simple rigid body (e.g., a cube)
    cube_prim = world.scene.add_default_ground_plane(prim_path="/World/myCube", position=[0.0, 0.0, 1.0], scale=[0.5, 0.5, 0.5])
    # For rigid bodies, you often want a physics material
    # world.scene.add_physics_material(prim_path="/PhysicsMaterial", static_friction=0.5, dynamic_friction=0.5, restitution=0.1)
    # cube_prim.apply_physics_material("/PhysicsMaterial")

    print("Scene created successfully! Press Play in Isaac Sim UI or run world.run() to simulate.")

    # You can keep the simulation running for interaction
    # while True:
    #     await world.step_async()

# Run the async function
asyncio.run(create_simple_scene())

# Keep the app running if not in headless mode
kit.run()
kit.shutdown()
