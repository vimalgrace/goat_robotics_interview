import tkinter as tk
import threading
import time


def timer_thread():
    root.after(1000,update_timer)

def update_timer():
    global timer_value
    timer_value += 1
    timer_label.config(text = f"Timer : {timer_value}")


root = tk.Tk()

timer_value = 0
timer_label = tk.Label(root, text = f"Timer : {timer_value}s")

table_3_order_status = tk.Label(root,text = f"ty45y5y")
table_3_order_status.grid(row = 3, column = 2,padx=10, pady=10)
table_3_order_status.config(text = f"fgegrer")
# timer_label.pack()



timer_thread = threading.Thread(target = timer_thread)
timer_thread.start()



root.mainloop()