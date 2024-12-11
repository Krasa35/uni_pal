#   CONFIG.YAML SETUP & PARAMETERS
>[!CAUTION]
>Running `urdf_generator.py` script deletes every previously generated description file. Make sure you backed up your work!

>[!WARNING]
>**REMEMBER!** Every shape uses its center in x, y, z as default origin and every mesh uses point declared in .stl as its origin - make sure to properly adjust it in CAD software!
-   system:
    |  |   |  |
    | :------  | :------: | :------: |
    | username | **STRING**     | |
    | simulation | **BOOL**      | true or false |
-   robot:

    required
    |  |   |  |
    | :------  | :------: | :------: |
    | type | **STRING**     | "universal_robots" or "techman_robots" |
    | parent | **STRING**     | link which robot is attached to, make sure you properly declare origin offset or make additional frame thus there is no access to robots position and orientation directly |

    fully optional:
    |  |   |  |
    | :------  | :------: | :------: |
    | origin_xyz | **STRING**     | "\<x> \<y> \<z>" e.g. "0.5 0.5 1.2" in meters|
    | origin_rpy | **STRING**     | "\<roll> \<pitch> \<yaw>" e.g. "0.5 0.5 1.2" in radians|

-   scene:
    -   element:

        required
        |  |   |  |
        | :------  | :------: | :------: |
        | position | **STRING**     | "\<x> \<y> \<z>" e.g. "0.5 0.5 1.2" in meters|
        | orientation | **STRING**      | "\<roll> \<pitch> \<yaw>" e.g. "0.5 0.5 1.2" in radians|
        | parent | **STRING**      | link name |

        optional, but required 1 of a type:

        *size* or *mesh*
        |  |   |  |
        | :------  | :------: | :------: |
        | size | **STRING**     | "\<width> \<length> \<height>" e.g. "0.5 0.5 1.2" in meters generates the box with thos dimensions|
        | mesh | **STRING**      | file path to .stl file |


        fully optional:
        |  |   |  |
        | :------  | :------: | :------: |
        | origin_xyz | **STRING**     | "\<x> \<y> \<z>" e.g. "0.5 0.5 1.2" in meters|
        | origin_rpy | **STRING**     | "\<roll> \<pitch> \<yaw>" e.g. "0.5 0.5 1.2" in radians|