from rapprentice.colorize import colorize
import subprocess

def call_and_print(cmd,color='green',check=True):
    print colorize(cmd, color, bold=True)
    if check: subprocess.check_call(cmd, shell=True)
    else: subprocess.call(cmd, shell=True)
