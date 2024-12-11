import os
import shutil
from jinja2 import Template
from templates import macros
import glob

def generate_launch(file_path, launch_type, robot_model_):
  template = Template(macros.launch_templates[launch_type])
  launc_content = template.render(
      robot_model=robot_model_
  )
  with open(file_path, 'w') as launch_file:
      launch_file.write(launc_content)
      print(f"Launch file generated at {file_path}")
  
