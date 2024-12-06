#   Unified Pal Application

Main goal of this application is to develop flexible palletizing/depalletizing/pick&place feature
for every robot with just one source code - Thesis topic in Polish:

**Elastyczny system paletyzacji oparty o ROS2**.

##  Opening repository
> [!WARNING]
> Repository is fully customizable - branch is always developed for native Linux system but inside `.devcontainer` you can find other possibilites to open the repository - all **should** work.

##  CONFIG.YAML
Description of configuration .yaml parameters is available in `.github/samples` directory [README](.github/samples/README.md)
>[!CAUTION]
>Running `urdf_generator.py` script deletes every previously generated description file. Make sure you backed up your work!

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
In order to see your work below command needs to be run
```bash
ros2 launch uni_pal urdf.test.launch.py
```