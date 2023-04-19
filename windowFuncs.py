from sys import platform
import subprocess
import screeninfo

# if platform in ['Windows','win32','cywin']:
#     import win32gui

# def get_active_window_title():
#     if platform in ['linux','linux2']:
#         return subprocess.getoutput("xprop -id $(xprop -root _NET_ACTIVE_WINDOW | cut -d ' ' -f 5) WM_NAME | cut -d '\"' -f 2")
#     elif platform in ['Windows', 'win32', 'cygwin']:
#         return GetWindowText(GetForegroundWindow())
#     else:
#         print("OS Not supported")

def adjust_window(name,pos_x, pos_y, size_x, size_y):
    if platform in ['Windows','win32','cywin']:
        print("adjust_window not implemented on Windows OS")
    else:
        subprocess.run(['wmctrl', '-r', name, '-e', f"0,{pos_x},{pos_y},{size_x},{size_y}"])

# def get_screen_resolution():
#     if platform in ['linux','linux2']:
#         screen = Gdk.Screen.get_default()
#         return [screen.get_monitor_geometry(0).height, screen.get_monitor_geometry(0).width]
#     elif platform in ['Windows', 'win32', 'cygwin']:
#         return [GetSystemMetrics(0), GetSystemMetrics(1)]
#     else:
#         print("OS Not supported")

BAR_SIZE = 40
def move_size_window(window_name, monitor:int, pos_x:float, pos_y:float, size_x:float, size_y:float) -> None:
    """Move and size window to the given monitor, dimentions given in percentage"""
    # screen = Gdk.Screen.get_default()
    monitors = screeninfo.get_monitors()
    # n_monitors = screen.get_n_monitors()
    n_monitors = len(monitors)
    if monitor == -1:
        monitor = n_monitors - 1
    elif monitor > n_monitors:
        print("Monitor out of bounds, default to monitor 0")
        monitor = 0

    # if monitor == 0:
    #     x_pad : int = BAR_SIZE
    # else:
    #     x_pad : int = 0
    # y_pad : int = 0
    # n : int = 0
    # while n < monitor:
    #     x_pad += monitors(n).width
    #     y_pad += monitors(n).height
    #     n += 1
    # if monitor == 0:
    #     pix_x = screen.get_monitor_geometry(monitor).width - BAR_SIZE
    # else:
    #     pix_x = screen.get_monitor_geometry(monitor).width
    pix_x = monitors[monitor].width
    pix_y = monitors[monitor].height
    x_pad = monitors[monitor].x
    y_pad = monitors[monitor].y
    adjust_window(window_name,int(pos_x*pix_x + x_pad),int(pos_y*pix_y + y_pad),int(size_x*pix_x),int(size_y*pix_y))
    # subprocess.run(['wmctrl', '-r', window_name, '-e', f"0,{int(pos_x*pix_x + x_pad)},{int(pos_y*pix_y + y_pad)},{int(size_x*pix_x)},{int(size_y*pix_y)}"])