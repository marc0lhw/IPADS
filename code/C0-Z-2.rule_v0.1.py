import time
from datetime import datetime

print("------------------------------")
print("[*] Status :     monitoring...")
time.sleep(10)

current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
print(f"[*] Status :     detected    [*] {current_time}")
time.sleep(0.5)

print("[*] Rule# :      WL number 01")
print("[*] ATTACK INFO")
print("     - Rule Type : set")
print("     - Attack name : Unauthenticated Publisher Duplicating")
print("     - Node name :   wm_motion_controller_node")
print("     - Topic name :  /can/control_hardware")

time.sleep(5)
print("------------------------------")
print("[*] Status :     monitoring...")

time.sleep(1000)