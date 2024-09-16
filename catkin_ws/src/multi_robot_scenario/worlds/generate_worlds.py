import sys


def generate_world_file(size):
    half_size = size / 2.0
    world_content = (
        """<?xml version="1.0" encoding="UTF-8"?>

        <!-- This world file defines a Gazebo simulation environment with a {size}x{size} rectangle of walls centered at (0, 0). 
        The walls are 2 meters tall and there is no ceiling. -->

        <sdf version="1.6">
        <world name="default">
            <light name="sun" type="directional">
            <cast_shadows>1</cast_shadows>
            <pose frame="">0 0 10 0 -0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.5 -1</direction>
            </light>

            <!-- Ground Plane -->
            <model name="ground_plane">
            <static>1</static>
            <link name="link">
                <collision name="collision">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    <size>100 100</size>
                    </plane>
                </geometry>
                <surface>
                    <friction>
                    <ode>
                        <mu>100</mu>
                        <mu2>50</mu2>
                    </ode>
                    </friction>
                </surface>
                <max_contacts>10</max_contacts>
                </collision>
                <visual name="visual">
                <cast_shadows>0</cast_shadows>
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    <size>100 100</size>
                    </plane>
                </geometry>
                <material>
                    <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>Gazebo/Grey</name>
                    </script>
                </material>
                </visual>
            </link>
            </model>

            <!-- Walls -->
            <model name="wall_1">
            <static>1</static>
            <link name="link">
                <pose>0 """
        + str(half_size)
        + """ 1 0 0 0</pose>
                <collision name="collision">
                <geometry>
                    <box>
                    <size>"""
        + str(size)
        + """ 0.1 2</size>
                    </box>
                </geometry>
                </collision>
                <visual name="visual">
                <geometry>
                    <box>
                    <size>"""
        + str(size)
        + """ 0.1 2</size>
                    </box>
                </geometry>
                </visual>
            </link>
            </model>

            <model name="wall_2">
            <static>1</static>
            <link name="link">
                <pose>0 -"""
        + str(half_size)
        + """ 1 0 0 0</pose>
                <collision name="collision">
                <geometry>
                    <box>
                    <size>"""
        + str(size)
        + """ 0.1 2</size>
                    </box>
                </geometry>
                </collision>
                <visual name="visual">
                <geometry>
                    <box>
                    <size>"""
        + str(size)
        + """ 0.1 2</size>
                    </box>
                </geometry>
                </visual>
            </link>
            </model>

            <model name="wall_3">
            <static>1</static>
            <link name="link">
                <pose>"""
        + str(half_size)
        + """ 0 1 0 0 1.5708</pose>
                <collision name="collision">
                <geometry>
                    <box>
                    <size>"""
        + str(size)
        + """ 0.1 2</size>
                    </box>
                </geometry>
                </collision>
                <visual name="visual">
                <geometry>
                    <box>
                    <size>"""
        + str(size)
        + """ 0.1 2</size>
                    </box>
                </geometry>
                </visual>
            </link>
            </model>

            <model name="wall_4">
            <static>1</static>
            <link name="link">
                <pose>-"""
        + str(half_size)
        + """ 0 1 0 0 1.5708</pose>
                <collision name="collision">
                <geometry>
                    <box>
                    <size>"""
        + str(size)
        + """ 0.1 2</size>
                    </box>
                </geometry>
                </collision>
                <visual name="visual">
                <geometry>
                    <box>
                    <size>"""
        + str(size)
        + """ 0.1 2</size>
                    </box>
                </geometry>
                </visual>
            </link>
            </model>

            <gravity>0 0 -9.8</gravity>
            <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
            <atmosphere type="adiabatic"/>
        </world>
        </sdf>
        """
    )
    # file_name = "train_rect_map_" + str(int(size)) + "x" + str(int(size)) + ".world"
    file_name = "training_medium_rect" + ".world"
    with open(file_name, "w") as file:
        file.write(world_content)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python generate_worlds.py <size>")
        sys.exit(1)

    size = float(sys.argv[1])
    generate_world_file(size)
    print("Generated rect_map_{}x{}.world".format(size, size))
