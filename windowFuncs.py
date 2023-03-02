from sys import platform
import subprocess
import screeninfo
import cv2
import warnings


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
        warnings.warn("Not implemented on Windows OS")
        return
    subprocess.run(['wmctrl', '-r', name, '-e', f"0,{pos_x},{pos_y},{size_x},{size_y}"])


def get_window_size(name):
    """Get window size including its frame in pixels"""
    if platform in ['Windows','win32','cywin']:
        warnings.warn("Not implemented on Windows OS")
        return
    w = int(subprocess.getoutput(f"xwininfo -name '{name}' | grep Width | cut -d ' ' -f 4"))
    h = int(subprocess.getoutput(f"xwininfo -name '{name}'| grep Height | cut -d ' ' -f 4"))
    frame = get_window_frame(name)
    return [w+sum(frame[0:2]),h+sum(frame[2:4])]


def get_screen_margins():
    if platform in ['Windows','win32','cywin']:
        warnings.warn("Not implemented on Windows OS")
        return
    return list(map(int, subprocess.getoutput("xprop -root _NET_WORKAREA | sed 's/[[:space:]]*//g' | cut -d '=' -f 2").split(',')))


def get_window_frame(name):
    """Return window frame as [left, right, top, bottom]"""
    if platform in ['Windows','win32','cywin']:
        warnings.warn("Not implemented on Windows OS")
        return
    return list(map(int, subprocess.getoutput(f"xprop -id $(wmctrl -l | grep '{name}' | cut -d ' ' -f 1) | grep FRAME | sed 's/[[:space:]]*//g' | cut -d '=' -f 2").split(',')))


def get_monitor(monitor):
    monitors = screeninfo.get_monitors()
    return monitors[monitor]


def move_size_window(window_name:str, monitor:int, pos_x:float, pos_y:float, size_x:float=0, size_y:float=0, is_cv2=False) -> None:
    """Move and size window to the given monitor using relative coordinates/sizes""" 
    if platform in ['Windows','win32','cywin']:
        warnings.warn("Not implemented on Windows OS")
        return    
    monitors = screeninfo.get_monitors()
    n_monitors = len(monitors)
    if monitor == -1:
        monitor = n_monitors - 1
    elif monitor > n_monitors:
        print("Monitor out of bounds, default to monitor 0")
        monitor = 0
    
    screen_margins = get_screen_margins()
    window_frame = get_window_frame(window_name)
    tot_win_margin = [sum(window_frame[0:2]), sum(window_frame[2:4])] # left, right, top and bottom frames in width and height margins

    # Workable pixel area
    if monitor == 0:
        pix_x = monitors[monitor].width - screen_margins[0]
        pix_y = monitors[monitor].height - screen_margins[1]
    else:
        pix_x = monitors[monitor].width
        pix_y = monitors[monitor].height

    # Get padding for positioning, includes screen margins and window frame
    x_pad = monitors[monitor].x + window_frame[0]
    y_pad = monitors[monitor].y + window_frame[2]
    if monitor == 0:
        x_pad += screen_margins[0]
        y_pad += screen_margins[1]

    if is_cv2:
        cv2.moveWindow(window_name, int(pos_x*pix_x + x_pad), int(pos_y*pix_y + y_pad))
        return
    adjust_window(window_name, int(pos_x*pix_x + x_pad), int(pos_y*pix_y + y_pad), int(size_x*pix_x - tot_win_margin[0]), int(size_y*pix_y - tot_win_margin[1]))