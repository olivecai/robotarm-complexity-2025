import libtmux
import time
from os import path

if __name__ == '__main__':
    print("Initializing...")
    server = libtmux.Server(
        config_file=path.expandvars("/home/user/vs/catkin_ws/scripts/.tmux.conf")
    )
    if server.has_session("sim"):
        print("Session has sim... exiting...")
        exit()
    else:
        session = server.new_session("sim", start_directory="/home/user/vs", attach=False)
        
    # terminals for the simulation to start
    terminals = {
        "core": "roscore",
        #"rqt": "rqt",
        "camera1": "rosrun visualservo camera_node.py _cam_id:=4", # TODO: change this to take in idx somehow. 
        "tracking": "rosrun visualservo tracking_node.py _cam_id:=4", 
        #"camera2": "rosrun visualservo camera_node.py _cam_id:=4", # TODO: change this to take in idx somehow. 
    }
    
    for name, cmd in terminals.items():
        window = session.new_window(name, attach=False)
        window.select_layout(layout="tiled")
        pane = window.panes[0]
        time.sleep(0.1)
        pane.send_keys(cmd, suppress_history=True)