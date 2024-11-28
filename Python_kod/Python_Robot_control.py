import tkinter as tk
from tkinter import messagebox, scrolledtext
import serial as serial
from datetime import datetime
import time

connection_status = 0
busy_state = 0


def run_program():
    global busy_state
    global program_active_label
    if connection_status == 1:
        program_name_text = program_name.get()


        program_active_label.destroy()
        program_active_label = tk.Label(root, text="Active             ", fg = "green")
        program_active_label.place(x = 680, y = 280)
        #program_active_label.config(text="Active",fg="green")
        with open(program_name_text + '.txt', 'r') as file:
            for line in file:
                command = line.strip()
                command = command.replace('*', '')
                command_ls = command.split(",")

                if command_ls[0] == "delay":
                    time.sleep(int(command_ls[1]) / 1000)
                    #continue

                if command_ls[0] != "delay":
                    send_message(line.strip())
                    busy_state = 1 

                    while busy_state:
                        read_from_serial()
                        time.sleep(0.05)
        program_active_label.destroy()
        #program_active_label.config(text="Not active",fg="red")
        program_active_label = tk.Label(root, text=" Not active", fg = "red")
        program_active_label.place(x = 680, y = 280)
    else:
        messagebox.showinfo("Warning", "Robot not connected")

def send_message(message):
    global busy_state
    if connection_status == 1:
        try:
            arduino.write((message + '\n').encode())
            #print(f"Sent: {message}")
            busy_state = 1  
        except:
            messagebox.showinfo("Warning", "Error sending the message")
    else:
        messagebox.showinfo("Warning", "Robot not connected")

def read_from_serial():
    global busy_state
    if connection_status == 1 and arduino.in_waiting > 0:
        try:
            message = ""
            while arduino.in_waiting > 0:
                char = arduino.read().decode('utf-8')
                message += char

            if message:
                busy_state = 0
                update_screen(message)
        except Exception as e:
            messagebox.showinfo("Error", f"Error reading from serial port: {e}")
    
    root.after(50, read_from_serial)


def move_joint():
    x_val = x_entry.get()
    y_val = y_entry.get()
    z_val = z_entry.get()
    speed_val = speed_entry.get()
    accel_val = accel_entry.get()
    tcp_val = tcp_entry.get()

    send_message("joint,"+ str(x_val) + "," + str(y_val) + "," + str(z_val)+ "," + str(tcp_val) + "," + str(speed_val)+ "," + str(accel_val) + "*") 

def move_joint_t():
    x_val = x_entry.get()
    y_val = y_entry.get()
    z_val = z_entry.get()
    speed_val = speed_entry.get()
    accel_val = accel_entry.get()
    tcp_val = tcp_entry.get()

    send_message("joint_t,"+ str(x_val) + "," + str(y_val) + "," + str(z_val)+ "," + str(tcp_val) + "," + str(speed_val)+ "," + str(accel_val) + "*") 


def move_linear():
    x_val = x_entry.get()
    y_val = y_entry.get()
    z_val = z_entry.get()
    speed_val = speed_entry.get()
    waypoints_val = accel_entry.get()
    tcp_val = tcp_entry.get() 

    send_message("linear,"+ str(x_val) + "," + str(y_val) + "," + str(z_val)+ "," + str(tcp_val) + "," + str(speed_val)+ "," + str(waypoints_val) + "*") 


def homing():
    send_message("homing*")

def connect():
    global arduino
    global connection_status

    if connection_status == 1:
        messagebox.showinfo("Connection", "Already connected")

    if connection_status == 0:
        try:
            COM_port = COM_entry.get()
            arduino = serial.Serial(COM_port,115200)
            messagebox.showinfo("Connection", "Connected")
            connect_status_label.config(text="Connected",fg="green")
            connection_status = 1

        except:
            messagebox.showinfo("Connection", "Cannot connect")


def vaccuum_on():
    send_message('valve_on*')

def vaccuum_off():
    send_message('valve_off*')

def clear_output():
    messages_area.configure(state='normal')
    messages_area.delete('1.0', tk.END)
    messages_area.configure(state='disabled') 


def update_screen(message):
    global busy_state
    message = message.rstrip()
    message.replace('\r\n', '')
    message.replace('*', '')
    timestamp = datetime.now().strftime('%H:%M:%S')
    ls_odg = message.split(",")
    messages_area.configure(state='normal')
    messages_area.insert('1.0', timestamp  + '     ' + ls_odg[0].strip() + '\n')
    #print("update screen")
    #print(ls_odg)
    if len(ls_odg) > 5:
        ls_odg[6].replace('*', '')
        joint1_entry.config(state='normal')
        joint2_entry.config(state='normal')
        joint3_entry.config(state='normal')
        xa_entry.config(state='normal')
        ya_entry.config(state='normal')
        za_entry.config(state='normal')
        joint1_entry.delete(0, tk.END)
        joint1_entry.insert(0, ls_odg[1])
        joint2_entry.delete(0, tk.END)
        joint2_entry.insert(0, ls_odg[2])
        joint3_entry.delete(0, tk.END)
        joint3_entry.insert(0, ls_odg[3])
        xa_entry.delete(0, tk.END)
        xa_entry.insert(0, ls_odg[4])
        ya_entry.delete(0, tk.END)
        ya_entry.insert(0, ls_odg[5])
        za_entry.delete(0, tk.END)
        za_entry.insert(0, ls_odg[6].replace('*', ''))
        joint1_entry.config(state='readonly')
        joint2_entry.config(state='readonly')
        joint3_entry.config(state='readonly')
        xa_entry.config(state='readonly')
        ya_entry.config(state='readonly')
        za_entry.config(state='readonly')
    messages_area.configure(state='disabled')
    messages_area.yview(tk.END)
    messages_area.update_idletasks()


root = tk.Tk()
root.title("Robot Control")
root.geometry("800x600")

messages_area = scrolledtext.ScrolledText(root, wrap=tk.WORD, width=50, height=20, font=("Arial", 12))
messages_area.place(x=0, y=200)
messages_area.configure(state='disabled')


COM_label = tk.Label(root, text="COM port:")
COM_label.grid(row=0, column=0)

COM_entry = tk.Entry(root)
COM_entry.grid(row=0, column=1)

connect_button = tk.Button(root, text="Connect", command=connect)
connect_button.grid(row=0, column=2)

connect_status_label = tk.Label(root, text="Disconnected", fg="red")
connect_status_label.grid(row=0, column=3)

program_name = tk.Entry(root)
program_name.place(x = 560, y = 250)

program_label = tk.Label(root, text="Program name:")
program_label.place(x = 470, y = 250)

run_program_button = tk.Button(root, text="Run program", command=run_program)
run_program_button.place(x = 680, y = 245)

program_active_label = tk.Label(root, text="Not active", fg = "red")
program_active_label.place(x = 680, y = 280)


x_label = tk.Label(root, text="X[mm]:")
x_label.grid(row=2, column=0)

y_label = tk.Label(root, text="Y[mm]:")
y_label.grid(row=3, column=0)

z_label = tk.Label(root, text="Z[mm]:")
z_label.grid(row=4, column=0)

joint1_label = tk.Label(root, text="Joint 1[deg]:")
joint1_label.grid(row=2, column=2)

joint2_label = tk.Label(root, text="Joint 2[deg]:")
joint2_label.grid(row=3, column=2)

joint3_label = tk.Label(root, text="Joint 3[deg]:")
joint3_label.grid(row=4, column=2)

xa_label = tk.Label(root, text="X [mm]:")
xa_label.grid(row=2, column=4)

ya_label = tk.Label(root, text="Y [mm]:")
ya_label.grid(row=3, column=4)

za_label = tk.Label(root, text="Z [mm]:")
za_label.grid(row=4, column=4)

x_entry = tk.Entry(root)
x_entry.grid(row=2, column=1)

y_entry = tk.Entry(root)
y_entry.grid(row=3, column=1)

z_entry = tk.Entry(root)
z_entry.grid(row=4, column=1)

joint1_var = tk.StringVar()
joint1_entry = tk.Entry(root, textvariable=joint1_var, state='readonly')
joint1_entry.grid(row=2, column=3)

joint2_var = tk.StringVar()
joint2_entry = tk.Entry(root, textvariable=joint2_var, state='readonly')
joint2_entry.grid(row=3, column=3)

joint3_var = tk.StringVar()
joint3_entry = tk.Entry(root, textvariable=joint3_var, state='readonly')
joint3_entry.grid(row=4, column=3)

x_var = tk.StringVar()
xa_entry = tk.Entry(root, textvariable=x_var, state='readonly')
xa_entry.grid(row=2, column=5)

y_var = tk.StringVar()
ya_entry = tk.Entry(root, textvariable=y_var, state='readonly')
ya_entry.grid(row=3, column=5)

z_var = tk.StringVar()
za_entry = tk.Entry(root, textvariable=z_var, state='readonly')
za_entry.grid(row=4, column=5)

movej_button = tk.Button(root, text="Move Joint", command=move_joint)
movej_button.place(x = 500, y = 150)

movejt_button = tk.Button(root, text="Move Joint T", command=move_joint_t)
movejt_button.place(x = 580, y = 150)

movel_button = tk.Button(root, text="Move Linear", command=move_linear)
movel_button.place(x = 670, y = 150)

homing_button = tk.Button(root, text="Homing", command=homing)
homing_button.grid(row=1, column=0)

vacum_on_btn = tk.Button(root, text="Vaccuum ON", command=vaccuum_on)
vacum_on_btn.place(x = 500, y = 200)

vacum_off_btn = tk.Button(root, text="Vaccuum OFF", command=vaccuum_off)
vacum_off_btn.place(x = 600, y = 200)

speed_label = tk.Label(root, text="Speed [step/s]:")
speed_label.grid(row=5, column=0)
speed_entry = tk.Entry(root)
speed_entry.grid(row=5, column=1)

accel_label = tk.Label(root, text="Acceleration [step/s^2]:")
accel_label.grid(row=5, column=2)
accel_entry = tk.Entry(root)
accel_entry.grid(row=5, column=3)

tcp_label = tk.Label(root, text="Tool offset [mm]:")
tcp_label.grid(row=5, column=4)
tcp_entry = tk.Entry(root)
tcp_entry.grid(row=5, column=5)

clear_output_btn = tk.Button(root, text="Clear output", command=clear_output)
clear_output_btn.place(x = 380, y = 170)

root.after(50, read_from_serial)

root.mainloop()