import rospy

from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
import tkinter as tk

# Initialize ROS publisher
pub_pl = rospy.Publisher('phantom_hsv_low', Vector3, queue_size=10)
pub_ph = rospy.Publisher('phantom_hsv_high', Vector3, queue_size=10)
pub_il = rospy.Publisher('inserter_hsv_low', Vector3, queue_size=10)
pub_ih = rospy.Publisher('inserter_hsv_high', Vector3, queue_size=10)
rospy.init_node('hsv_sliders', anonymous=True)
# Create tkinter GUI window
root = tk.Tk()
hsv_pl = Vector3()
hsv_ph = Vector3()
hsv_il = Vector3()
hsv_ih = Vector3()

phantom_low_p = rospy.get_param("phantom_low_p", [0,0,143])
phantom_high_p = rospy.get_param("phantom_high_p", [180,59,255])
inserter_low_p = rospy.get_param("inserter_low_p", [28,144,82])
inserter_high_p = rospy.get_param("inserter_high_p", [151,255,156])
hsv_pl.x = phantom_low_p[0]
hsv_pl.y = phantom_low_p[1]
hsv_pl.z = phantom_low_p[2]

hsv_ph.x = phantom_high_p[0]
hsv_ph.y = phantom_high_p[1]
hsv_ph.z = phantom_high_p[2]

hsv_il.x = inserter_low_p[0]
hsv_il.y = inserter_low_p[1]
hsv_il.z = inserter_low_p[2]

hsv_ih.x = inserter_high_p[0]
hsv_ih.y = inserter_high_p[1]
hsv_ih.z = inserter_high_p[2]


# Phantom low sliders
def slider_command_h_pl(val):
    """Value callback from slider."""
    hsv_pl.x = float(val)
    pub_pl.publish(hsv_pl)

def slider_command_s_pl(val):
    """Value callback from slider."""
    hsv_pl.y = float(val)
    pub_pl.publish(hsv_pl)


def slider_command_v_pl(val):
    """Value callback from slider."""
    hsv_pl.z = float(val)
    pub_pl.publish(hsv_pl)

# Phantom high sliders
def slider_command_h_ph(val):
    """Value callback from slider."""
    hsv_ph.x = float(val)
    pub_ph.publish(hsv_ph)

def slider_command_s_ph(val):
    """Value callback from slider."""
    hsv_ph.y = float(val)
    pub_ph.publish(hsv_ph)


def slider_command_v_ph(val):
    """Value callback from slider."""
    hsv_ph.z = float(val)
    pub_ph.publish(hsv_ph)

# Inserter low sliders
def slider_command_h_il(val):
    """Value callback from slider."""
    hsv_il.x = float(val)
    pub_il.publish(hsv_il)

def slider_command_s_il(val):
    """Value callback from slider."""
    hsv_il.y = float(val)
    pub_il.publish(hsv_il)


def slider_command_v_il(val):
    """Value callback from slider."""
    hsv_il.z = float(val)
    pub_il.publish(hsv_il)

# Inserter high sliders
def slider_command_h_ih(val):
    """Value callback from slider."""
    hsv_ih.x = float(val)
    pub_ih.publish(hsv_ih)

def slider_command_s_ih(val):
    """Value callback from slider."""
    hsv_ih.y = float(val)
    pub_ih.publish(hsv_ih)


def slider_command_v_ih(val):
    """Value callback from slider."""
    hsv_ih.z = float(val)
    pub_ih.publish(hsv_ih)


# Phantom Low GUI
slider_h_pl = tk.Scale(root, from_=0, to=180, orient='horizontal', length='200',
                  tickinterval=50, command=slider_command_h_pl)

slider_s_pl = tk.Scale(root, from_=0, to=255, orient='horizontal', length='200',
                  tickinterval=50, command=slider_command_s_pl)

slider_v_pl = tk.Scale(root, from_=0, to=255, orient='horizontal', length='200',
                  tickinterval=50, command=slider_command_v_pl)

# Phantom High GUI
slider_h_ph = tk.Scale(root, from_=0, to=180, orient='horizontal', length='200',
                  tickinterval=50, command=slider_command_h_ph)

slider_s_ph = tk.Scale(root, from_=0, to=255, orient='horizontal', length='200',
                  tickinterval=50, command=slider_command_s_ph)

slider_v_ph = tk.Scale(root, from_=0, to=255, orient='horizontal', length='200',
                  tickinterval=50, command=slider_command_v_ph)

# Inserter Low GUI
slider_h_il = tk.Scale(root, from_=0, to=180, orient='horizontal', length='200',
                  tickinterval=50, command=slider_command_h_il)

slider_s_il = tk.Scale(root, from_=0, to=255, orient='horizontal', length='200',
                    tickinterval=50, command=slider_command_s_il)

slider_v_il = tk.Scale(root, from_=0, to=255, orient='horizontal', length='200',
                    tickinterval=50, command=slider_command_v_il)

# Inserter High GUI
slider_h_ih = tk.Scale(root, from_=0, to=180, orient='horizontal', length='200',
                    tickinterval=50, command=slider_command_h_ih)

slider_s_ih = tk.Scale(root, from_=0, to=255, orient='horizontal', length='200',
                    tickinterval=50, command=slider_command_s_ih)

slider_v_ih = tk.Scale(root, from_=0, to=255, orient='horizontal', length='200',
                    tickinterval=50, command=slider_command_v_ih)

# Label and pack Phantom sliders    
lbl_h_pl = tk.Label(root, text='HSV low, phantom')
lbl_h_pl.pack()
slider_h_pl.set(phantom_low_p[0])
slider_s_pl.set(phantom_low_p[1])
slider_v_pl.set(phantom_low_p[2])
slider_h_pl.pack()
slider_s_pl.pack()
slider_v_pl.pack()

lbl_h_ph = tk.Label(root, text='HSV high, phantom')
lbl_h_ph.pack()
slider_h_ph.set(phantom_high_p[0])
slider_s_ph.set(phantom_high_p[1])
slider_v_ph.set(phantom_high_p[2])
slider_h_ph.pack()
slider_s_ph.pack()
slider_v_ph.pack()


# Label and pack Inserter sliders
lbl_h_il = tk.Label(root, text='HSV low, inserter')
lbl_h_il.pack()
slider_h_il.set(inserter_low_p[0])
slider_s_il.set(inserter_low_p[1])
slider_v_il.set(inserter_low_p[2])
slider_h_il.pack()
slider_s_il.pack()
slider_v_il.pack()

lbl_h_ih = tk.Label(root, text='HSV high, inserter')
lbl_h_ih.pack()
slider_h_ih.set(inserter_high_p[0])
slider_s_ih.set(inserter_high_p[1])
slider_v_ih.set(inserter_high_p[2])
slider_h_ih.pack()
slider_s_ih.pack()
slider_v_ih.pack()



while not rospy.is_shutdown():
    root.update_idletasks()
    root.update()
