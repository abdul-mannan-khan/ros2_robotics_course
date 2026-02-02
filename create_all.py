import os

BASE="/home/it-services/auto_ws/src/ros2_robotics_course"

def wf(path,content,exe=False):
    os.makedirs(os.path.dirname(path),exist_ok=True)
    with open(path,"w") as f:
        f.write(content)
    if exe:
        os.chmod(path,0o755)

files={}
