from articulation_socket import ActionReceiverThread,ArticulationSender
import numpy as np
import time
HOST = '127.0.0.1'
PORT = 65434
DOF= 6
receiver_thread = ActionReceiverThread('127.0.0.1', 65434,6)
receiver_thread.daemon = True 
receiver_thread.start()
time.sleep(1)

while True:
    current_action = receiver_thread.get_latest_action()
    print(f" 最新动作 -> {current_action}")
    time.sleep(0.02)

