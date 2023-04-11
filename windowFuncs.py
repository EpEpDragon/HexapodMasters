import subprocess
import gi
gi.require_version('Gdk', '3.0')
from gi.repository import Gdk

def get_active_window_title():
    return subprocess.getoutput("xprop -id $(xprop -root _NET_ACTIVE_WINDOW | cut -d ' ' -f 5) WM_NAME | cut -d '\"' -f 2")

def adjust_window(name,pos_x, pos_y, size_x, size_y):
    subprocess.run(['wmctrl', '-r', name, '-e', f"0,{pos_x},{pos_y},{size_x},{size_y}"])

def get_screen_resolution():
    screen = Gdk.Screen.get_default()
    return [screen.get_monitor_geometry(0).height, screen.get_monitor_geometry(0).width]
    # return [screen.get_width(), screen.get_height()]


BAR_SIZE = 40
def move_size_window(window_name, monitor:int, pos_x:float, pos_y:float, size_x:float, size_y:float) -> None:
    """Move and size window to the given monitor, dimentions given in percentage"""
    screen = Gdk.Screen.get_default()
    
    n_monitors = screen.get_n_monitors()
    if monitor == -1:
        monitor = n_monitors - 1
    elif monitor > n_monitors:
        print("Monitor out of bounds, default to monitor 0")
        monitor = 0

    x_pad : int = BAR_SIZE
    y_pad : int = 0
    n : int = 0
    while n < monitor:
        x_pad += screen.get_monitor_geometry(n).width
        y_pad += screen.get_monitor_geometry(n).height
        n += 1
    pix_x = screen.get_monitor_geometry(monitor).width - BAR_SIZE
    pix_y = screen.get_monitor_geometry(monitor).height
    subprocess.run(['wmctrl', '-r', window_name, '-e', f"0,{int(pos_x*pix_x + x_pad)},{int(pos_y*pix_y + y_pad)},{int(size_x*pix_x)},{int(size_y*pix_y)}"])


# def active():
#     screen = Gdk.Screen.get_default()
#     return screen.get_active_window()