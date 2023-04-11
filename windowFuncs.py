import os
import subprocess

def get_active_window_title():
    return subprocess.getoutput("xprop -id $(xprop -root _NET_ACTIVE_WINDOW | cut -d ' ' -f 5) WM_NAME | cut -d '\"' -f 2")

def adjust_window(name,pos_x, pos_y, size_x, size_y):
    subprocess.run(['wmctrl', '-r', name, '-e', f"0,{pos_x},{pos_y},{size_x},{size_y}"])
