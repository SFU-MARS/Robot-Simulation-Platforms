"""
Writes the QBot worlds (SDF 1.6, static boxes 0.4 m high, so the LiDAR plane at 0.207 m cuts them) into
turtlebot3/turtlebot3_controls/worlds/:

  qbot_arena.world     -- the Quanser arena, 3.6 x 3.6 m inside: an L-shaped obstacle in the middle (a concave
                          corner) and a pillar near a corner
  qbot_occluded.world  -- 6 x 6 m inside: three screens, each with a box hidden behind one end, and two pillars

The QBot is 0.57 m wide and its obstacles are grown into square-cornered configuration-space obstacles (the faces
are extended by the footprint radius at both ends), so a passage is either about 1 m wide or closed (below 0.75 m:
behind the hidden boxes, between a pillar and the walls of a corner).

    cd nav_core && python scripts/make_qbot_worlds.py
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "turtlebot3" / "turtlebot3_controls" / "worlds"

HEADER = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="{name}">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.7 0.7 0.7 1</ambient>
      <background>0.9 0.9 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 5 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -1</direction>
    </light>

    <!-- {size} m x {size} m floor; its top surface is at z = 0. -->
    <model name="map_floor">
      <static>true</static>
      <link name="floor">
        <collision name="floor_collision">
          <pose>0 0 -0.025 0 0 0</pose>
          <geometry><box><size>{floor} {floor} 0.05</size></box></geometry>
        </collision>
        <visual name="floor_visual">
          <pose>0 0 -0.025 0 0 0</pose>
          <geometry><box><size>{floor} {floor} 0.05</size></box></geometry>
          <material><ambient>0.82 0.82 0.82 1</ambient><diffuse>0.82 0.82 0.82 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

BOX = """
    <!-- {name}: x = [{x0:.3f}, {x1:.3f}], y = [{y0:.3f}, {y1:.3f}]{note} -->
    <model name="{name}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} 0.2 0 0 0</pose>
      <link name="body">
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} 0.4</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} 0.4</size></box></geometry>
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

FOOTER = """  </world>
</sdf>
"""


def box(name, x0, x1, y0, y1, note="", rgb="0.8 0.05 0.05"):
    return BOX.format(name=name, x0=x0, x1=x1, y0=y0, y1=y1, cx=(x0 + x1) / 2, cy=(y0 + y1) / 2, sx=x1 - x0,
                      sy=y1 - y0, note=f"; {note}" if note else "", rgb=rgb)


def walls(half, t=0.05):
    """Perimeter walls around the inside [-half, half]^2."""
    return (box("wall_n", -half - t, half + t, half, half + t, "perimeter")
            + box("wall_s", -half - t, half + t, -half - t, -half, "perimeter")
            + box("wall_w", -half - t, -half, -half, half, "perimeter")
            + box("wall_e", half, half + t, -half, half, "perimeter"))


def arena() -> str:
    half = 1.8
    body = walls(half)
    blue = "0.1 0.2 0.8"
    body += box("l_bottom", -0.2, 0.6, -0.3, -0.2, "L-shaped obstacle in the middle, with l_side: a concave corner "
                "open to the upper left; 1.2 m or more to the walls", blue)
    body += box("l_side", 0.5, 0.6, -0.2, 0.6, "L-shaped obstacle", blue)
    body += box("pillar_1", -1.05, -0.75, 0.75, 1.05, "pillar near the north-west corner (the corner itself is "
                "closed: 0.75 m to the walls), 1.1 m from the L", blue)
    return HEADER.format(name="qbot_arena", size=2 * half, floor=2 * half + 0.2) + body + FOOTER


def occluded() -> str:
    half = 3.0
    body = walls(half)
    blue, green = "0.1 0.2 0.8", "0.1 0.6 0.1"
    body += box("screen_1", -2.0, 0.4, -1.275, -1.125, "screen; 1.7 m to the south wall", blue)
    body += box("hidden_1", 0.65, 1.05, -1.05, -0.65, "box behind the east end of screen_1", green)
    body += box("screen_2", -0.4, 2.0, 0.525, 0.675, "screen; 1.65 m above screen_1, 1.0 m to the east wall", blue)
    body += box("hidden_2", -1.05, -0.65, 0.75, 1.15, "box behind the west end of screen_2", green)
    body += box("screen_3", -1.975, -1.825, 1.5, 3.0, "screen from the north wall; 1.0 m to the west wall", blue)
    body += box("hidden_3", -1.75, -1.35, 1.55, 1.95, "box behind the south end of screen_3", green)
    body += box("pillar_1", 2.1, 2.5, -2.5, -2.1, "pillar near the south-east corner (closed: 0.5 m to the walls)",
                blue)
    body += box("pillar_2", 0.7, 1.1, 1.65, 2.05, "free-standing pillar; 0.95 m to the north wall, 0.975 m to "
                "screen_2", blue)
    return HEADER.format(name="qbot_occluded", size=2 * half, floor=2 * half + 0.2) + body + FOOTER


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    for name, text in (("qbot_arena.world", arena()), ("qbot_occluded.world", occluded())):
        (OUT / name).write_text(text)
        print("wrote", OUT / name)
