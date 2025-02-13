#!/usr/bin/env python

import rospy
import tf
import threading
import geometry_msgs.msg
import tf_conversions
import tkinter as tk
from tkinter import filedialog
from tkinter import ttk
import yaml
import sys
import os


class GUI():
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Camera two tf calibrator')
        self.root.geometry('350x180')
        self.root.resizable(False, False)
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=5)
        self.root.columnconfigure(2, weight=1)

        self.precision = 100.0
        self.slide_length = 200.0

        # Variables to hold the current pose
        self.current_x = tk.DoubleVar()
        self.current_y = tk.DoubleVar()
        self.current_z = tk.DoubleVar()
        self.current_pitch = tk.DoubleVar()
        self.current_yaw = tk.DoubleVar()
        self.current_roll = tk.DoubleVar()

        # Labels for the sliders
        self.slider_label_x = ttk.Label(self.root, text='x:')
        self.slider_label_y = ttk.Label(self.root, text='y:')
        self.slider_label_z = ttk.Label(self.root, text='z:')
        self.slider_label_pitch = ttk.Label(self.root, text='pitch:')
        self.slider_label_yaw = ttk.Label(self.root, text='yaw:')
        self.slider_label_roll = ttk.Label(self.root, text='roll:')

        # Layout for labels and sliders
        self.slider_label_x.grid(column=0, row=0, sticky='w')
        self.slider_label_y.grid(column=0, row=1, sticky='w')
        self.slider_label_z.grid(column=0, row=2, sticky='w')
        self.slider_label_pitch.grid(column=0, row=3, sticky='w')
        self.slider_label_yaw.grid(column=0, row=4, sticky='w')
        self.slider_label_roll.grid(column=0, row=5, sticky='w')

        # Sliders to control translation and rotation
        self.slider_x = ttk.Scale(self.root, from_=-15.0 * self.precision, to=15.0 * self.precision,
                                  length=self.slide_length, orient='horizontal', command=self.set_x,
                                  variable=self.current_x)
        self.slider_y = ttk.Scale(self.root, from_=-15.0 * self.precision, to=15.0 * self.precision,
                                  length=self.slide_length, orient='horizontal', command=self.set_y,
                                  variable=self.current_y)
        self.slider_z = ttk.Scale(self.root, from_=-3.0 * self.precision, to=3.0 * self.precision,
                                  length=self.slide_length, orient='horizontal', command=self.set_z,
                                  variable=self.current_z)
        self.slider_pitch = ttk.Scale(self.root, from_=-7.0, to=7.0 * self.precision, length=self.slide_length,
                                      orient='horizontal', command=self.set_pitch, variable=self.current_pitch)
        self.slider_yaw = ttk.Scale(self.root, from_=-7.0, to=7.0 * self.precision, length=self.slide_length,
                                    orient='horizontal', command=self.set_yaw, variable=self.current_yaw)
        self.slider_roll = ttk.Scale(self.root, from_=-7.0, to=7.0 * self.precision, length=self.slide_length,
                                     orient='horizontal', command=self.set_roll, variable=self.current_roll)

        # Grid positions for sliders
        self.slider_x.grid(column=1, row=0, sticky='w')
        self.slider_y.grid(column=1, row=1, sticky='w')
        self.slider_z.grid(column=1, row=2, sticky='w')
        self.slider_pitch.grid(column=1, row=3, sticky='w')
        self.slider_yaw.grid(column=1, row=4, sticky='w')
        self.slider_roll.grid(column=1, row=5, sticky='w')

        # Labels to display current values of the sliders
        self.current_x_label = ttk.Label(self.root, text='')
        self.current_y_label = ttk.Label(self.root, text='')
        self.current_z_label = ttk.Label(self.root, text='')
        self.current_pitch_label = ttk.Label(self.root, text='')
        self.current_yaw_label = ttk.Label(self.root, text='')
        self.current_roll_label = ttk.Label(self.root, text='')

        # Grid positions for current values labels
        self.current_x_label.grid(column=2, row=0, sticky='w')
        self.current_y_label.grid(column=2, row=1, sticky='w')
        self.current_z_label.grid(column=2, row=2, sticky='w')
        self.current_pitch_label.grid(column=2, row=3, sticky='w')
        self.current_yaw_label.grid(column=2, row=4, sticky='w')
        self.current_roll_label.grid(column=2, row=5, sticky='w')

        # Update labels with initial values
        self.update_labels()

        # Create a TransformBroadcaster
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(1000.0)

        # Start a separate thread for publishing transforms
        self.gui_thread = threading.Thread(target=self.node_publisher)
        self.gui_thread.start()

        # Button to save the current transform to a YAML file
        self.save_button = ttk.Button(self.root, text="Save Transform", command=self.save_transform)
        self.save_button.grid(column=1, row=6, sticky='w', padx=10, pady=10)

        # Load pre-configured transform from a YAML file (if exists)
        self.load_config()

    def node_publisher(self):
        while not rospy.is_shutdown():
            # Read values from sliders
            x = self.current_x.get() / self.precision
            y = self.current_y.get() / self.precision
            z = self.current_z.get() / self.precision
            pitch = self.current_pitch.get() / self.precision
            roll = self.current_roll.get() / self.precision
            yaw = self.current_yaw.get() / self.precision

            # Send the transform to TF
            try:
                self.br.sendTransform((x, y, z),
                                      tf_conversions.transformations.quaternion_from_euler(pitch, roll, yaw),
                                      rospy.Time.now(),
                                      "cameratwo_link",  # Change to the appropriate frame name
                                      "world")
            except:
                pass
            self.rate.sleep()

    def set_x(self, event):
        self.update_labels()

    def set_y(self, event):
        self.update_labels()

    def set_z(self, event):
        self.update_labels()

    def set_pitch(self, event):
        self.update_labels()

    def set_yaw(self, event):
        self.update_labels()

    def set_roll(self, event):
        self.update_labels()

    def update_labels(self):
        # Update the labels with the current values
        self.current_x_label.configure(text='{: .2f}'.format(self.current_x.get() / self.precision))
        self.current_y_label.configure(text='{: .2f}'.format(self.current_y.get() / self.precision))
        self.current_z_label.configure(text='{: .2f}'.format(self.current_z.get() / self.precision))
        self.current_pitch_label.configure(text='{: .2f}'.format(self.current_pitch.get() / self.precision))
        self.current_yaw_label.configure(text='{: .2f}'.format(self.current_yaw.get() / self.precision))
        self.current_roll_label.configure(text='{: .2f}'.format(self.current_roll.get() / self.precision))

    def save_transform(self):
        # Get the current transform values
        x = self.current_x.get() / self.precision
        y = self.current_y.get() / self.precision
        z = self.current_z.get() / self.precision
        pitch = self.current_pitch.get() / self.precision
        roll = self.current_roll.get() / self.precision
        yaw = self.current_yaw.get() / self.precision

        # Convert to quaternion
        quaternion = tf_conversions.transformations.quaternion_from_euler(pitch, roll, yaw)
        quaternion = [float(q) for q in quaternion]
        # Save the transform data to a YAML file
        transform_data = {
            'translation': {'x': x, 'y': y, 'z': z},
            'rotation': {
                'pitch': pitch, 
                'yaw': yaw, 
                'roll': roll, 
            }
        }

        # File dialog to select the file location
        file_path = filedialog.asksaveasfilename(defaultextension=".yaml", filetypes=[("YAML files", "*.yaml")])

        if file_path:
            with open(file_path, 'w') as file:
                yaml.dump(transform_data, file, default_flow_style=False)
            print(f"Transform saved to {file_path}")

    def load_config(self):
        config_path = os.path.join(os.path.dirname(__file__), '../config/cameratwo_tf_conf.yaml')
        if os.path.exists(config_path):
            with open(config_path, 'r') as file:
                config_data = yaml.load(file, Loader=yaml.FullLoader)

            # Apply the configuration if it's valid
            if 'translation' in config_data and 'rotation' in config_data:
                translation = config_data['translation']
                rotation = config_data['rotation']

                # Set sliders based on loaded values
                self.current_x.set(translation['x'] * self.precision)
                self.current_y.set(translation['y'] * self.precision)
                self.current_z.set(translation['z'] * self.precision)
                self.current_pitch.set(rotation['pitch'] * self.precision)
                self.current_yaw.set(rotation['yaw'] * self.precision)
                self.current_roll.set(rotation['roll'] * self.precision)

                self.update_labels()
                print(f"Loaded configuration from {config_path}")
        else:
            print(f"No configuration file found at {config_path}")


def main():
    rospy.init_node('dynamic_transform_publisher_with_gui')
    gui = GUI()
    gui.root.mainloop()


if __name__ == '__main__':
    main()



