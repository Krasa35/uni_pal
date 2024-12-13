import os
import shutil
from jinja2 import Template
from templates import macros
import glob

def generate_launch(file_path, launch_type, robot_model_, robot_ip_="xxx.xxx.xxx.xxx"):
  template = Template(macros.launch_templates[launch_type])
  launch_content = template.render(
      robot_model=robot_model_,
      robot_ip=robot_ip_
  )
  with open(file_path, 'w') as launch_file:
      launch_file.write(launch_content)
      print(f"Launch file generated at {file_path}")
  
