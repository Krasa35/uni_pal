#   Unified Pal Application

Main goal of this application is to develop flexible palletizing/depalletizing/pick&place feature
for every robot with just one source code - Thesis topic in Polish:

**Elastyczny system paletyzacji oparty o ROS2**.

##  Opening repository
> [!WARNING]
> Repository is fully customizable - branch is always developed for native Linux system but inside `.devcontainer` you can find other possibilites to open the repository - all **should** work.

##  CONFIG.YAML
-   system:
    |  |   |  |
    | :------  | :------: | :------: |
    | username | **STRING**     | |
    | simulation | **BOOL**      | true or false |
-   robot:
    |  |   |  |
    | :------  | :------: | :------: |
    | type | **STRING**     | "universal_robots" or "techman_robots" |
-   scene:
    -   element

        required
        |  |   |  |
        | :------  | :------: | :------: |
        | position | **STRING**     | "\<x> \<y> \<z>" e.g. "0.5 0.5 1.2" in meters|
        | orientation | **STRING**      | "\<roll> \<pitch> \<yaw>" e.g. "0.5 0.5 1.2" in radians|
        | parent | **STRING**      | link name |

        optional, but required:

        *string* or *mesh*
        |  |   |  |
        | :------  | :------: | :------: |
        | size | **STRING**     | "\<width> \<length> \<height>" e.g. "0.5 0.5 1.2" in meters|
        | mesh | **STRING**      | file path to .stl file |


<details>
<summary><b> Linux </b></summary>

</details>

<details>
<summary><b> Windows + Docker Desktop </b></summary>
</details>

<details>
<summary><b> WSL + Docker</b></summary>
</details>

After building devcontainer below steps need to be performed for every installation.
```bash
python3 /home/ws/src/uni_pal_generator/scripts/urdf_generator.py
colcon build
```