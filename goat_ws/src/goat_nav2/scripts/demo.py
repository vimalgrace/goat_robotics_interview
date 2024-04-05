import tkinter as tk
import threading
import time

def timer_thread():
    global timer_running
    timer_running = True
    while timer_running:
        # Update the timer label every second
        time.sleep(1)
        root.after(0, update_timer)

def update_timer():
    global timer_value
    timer_value += 1
    timer_label.config(text=f"Timer: {timer_value}s")

def stop_timer():
    global timer_running
    timer_running = False

# Create the tkinter window
root = tk.Tk()

# Create a label to display the timer value
timer_value = 0
timer_label = tk.Label(root, text=f"Timer: {timer_value}s")
timer_label.pack()

# Create a button to stop the timer
stop_button = tk.Button(root, text="Stop Timer", command=stop_timer)
stop_button.pack()

# Start the timer thread
timer_thread = threading.Thread(target=timer_thread)
timer_thread.start()

# Start the tkinter event loop
root.mainloop()