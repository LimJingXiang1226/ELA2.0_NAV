import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped
import customtkinter as tk
import os
from math import sqrt

class Ella2Gui(Node):

    def __init__(self):
        super().__init__('ela2_gui')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.registered_goal = {"Origin": {"x" : 0.0, "y" : 0.0, "qz" : 0.0, "qw" : 1.0}}
        
        # GUI
        self.create_main_window()

        # Update Register Goal
        self.update_register_goal()

    #Create Main Window
    def create_main_window(self):
        # SET GUI TO DARK THEME
        tk.set_appearance_mode("dark")
        
        # Create Main Window
        self.root = tk.CTk()
        self.root.title("Ella2 GUI")
        
        # Window Size
        self.root.geometry("320x290")
        self.root.resizable(False, False)

        # Window Label
        title_label = tk.CTkLabel(master=self.root, text="E.L.A 2.0 GUI", font=("Times New Roman", 24))
        title_label.grid(row=0, column=0, padx=15, pady=15)
        
        # Create a Frame
        button_frame = tk.CTkFrame(master=self.root, width=280, height=150)
        button_frame.grid(row=1, column=0, padx=20, pady=5)

        # Register Goal Button
        register_button = tk.CTkButton(master=button_frame, text="Register Goal", font=("Times New Roman", 24), command=self.open_register_goal_window)
        register_button.grid(row=0, column=0, padx=15, pady=15)
        register_button.configure(width=250, height=50)

        # Set Goal Button
        set_button = tk.CTkButton(master=button_frame, text="Set Goal", font=("Times New Roman", 24), command=self.open_set_goal_window)
        set_button.grid(row=1, column=0, padx=15, pady=0)
        set_button.configure(width=250, height=50)

        # Remove Goal Button
        remove_button = tk.CTkButton(master=button_frame, text="Remove Goal", font=("Times New Roman", 24), command=self.open_remove_goal_window)
        remove_button.grid(row=2, column=0, padx=15, pady=15)
        remove_button.configure(width=250, height=50)

    # Register Goal Pose
    def get_current_pose(self):
        try:
            # Get latest time available in the buffer
            latest_time = self.tf_buffer.get_latest_common_time('map', 'base_footprint')

            # Lookup transform with adjusted time
            map_base_tf = self.tf_buffer.lookup_transform('map', 'base_footprint', 
                                                          latest_time, 
                                                          timeout=rclpy.duration.Duration(seconds=1.0))

            # Return position and orientation
            self.current_pose_x = map_base_tf.transform.translation.x
            self.current_pose_y = map_base_tf.transform.translation.y
            self.current_pose_qz = map_base_tf.transform.rotation.z
            self.current_pose_qw = map_base_tf.transform.rotation.w
            self.get_logger().warn(f"Robot Current Pose Get")

            return True

        except Exception as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")

            return False
        
    def open_register_goal_window(self):
        # Hide main window
        self.root.withdraw()

        # Create new register goal window
        self.register_goal_window = tk.CTk()
        self.register_goal_window.title("Register Goal")

        # Window Size
        self.register_goal_window.geometry("320x520")
        self.register_goal_window.resizable(True, True)

        # Register Goal Label
        register_label = tk.CTkLabel(master=self.register_goal_window, text="Register Goal Window", font=("Times New Roman", 24))
        register_label.grid(row=0, column=0, padx=15, pady=15)

        # Create a Frame
        button_frame = tk.CTkFrame(master=self.register_goal_window, width=280, height=150)
        button_frame.grid(row=1, column=0, padx=20, pady=5)

        # Entry Fields
        goal_name_label = tk.CTkLabel(master=button_frame, text="Goal Name :", font=("Times New Roman", 16))
        goal_name_label.grid(row=0, column=0, padx=0, pady=5)
        self.goal_name_entry = tk.CTkEntry(master=button_frame)
        self.goal_name_entry.grid(row=0, column=1, padx=0, pady=5)

        x_label = tk.CTkLabel(master=button_frame, text="X-Axis :", font=("Times New Roman", 16))
        x_label.grid(row=1, column=0, padx=0, pady=5)
        self.x_entry = tk.CTkEntry(master=button_frame, validate="key", 
                                   validatecommand=(self.register_goal_window.register(self.validate_float), '%P'))
        self.x_entry.grid(row=1, column=1, padx=0, pady=5)

        y_label = tk.CTkLabel(master=button_frame, text="Y-Axis :", font=("Times New Roman", 16))
        y_label.grid(row=2, column=0, padx=0, pady=5)
        self.y_entry = tk.CTkEntry(master=button_frame, validate="key", 
                                   validatecommand=(self.register_goal_window.register(self.validate_float), '%P'))
        self.y_entry.grid(row=2, column=1, padx=0, pady=5)

        qz_label = tk.CTkLabel(master=button_frame, text="Quantration Z :", font=("Times New Roman", 16))
        qz_label.grid(row=3, column=0, padx=0, pady=5)
        self.qz_entry = tk.CTkEntry(master=button_frame, validate="key", 
                                    validatecommand=(self.register_goal_window.register(self.validate_float), '%P'))
        self.qz_entry.grid(row=3, column=1, padx=0, pady=5)

        qw_label = tk.CTkLabel(master=button_frame, text="Quanteration W :", font=("Times New Roman", 16))
        qw_label.grid(row=4, column=0, padx=0, pady=5)
        self.qw_entry = tk.CTkEntry(master=button_frame)
        self.qw_entry.grid(row=4, column=1, padx=0, pady=5)

        # Register Current Pose as Goal Button
        Current_Pose_button = tk.CTkButton(master=button_frame, text="Register Current Pose", font=("Times New Roman", 24), command=self.write_current_pose_to_file)
        Current_Pose_button.grid(row=5, column=0, padx=15, pady=5, columnspan=2)
        Current_Pose_button.configure(width=250, height=50)

        # Register Entry Pose as Goal Button
        Entry_Pose_button = tk.CTkButton(master=button_frame, text="Register Entry Pose", font=("Times New Roman", 24), command=self.write_entry_to_file)
        Entry_Pose_button.grid(row=6, column=0, padx=15, pady=5, columnspan=2)
        Entry_Pose_button.configure(width=250, height=50)

        # Exit Button
        exit_button = tk.CTkButton(master=button_frame, text="Exit", font=("Times New Roman", 24), command=self.close_register_goal_window)
        exit_button.grid(row=7, column=0, padx=15, pady=5, columnspan=2)
        exit_button.configure(width=250, height=50)

        # Create a Frame
        info_frame = tk.CTkFrame(master=self.register_goal_window, width=280, height=150)
        info_frame.grid(row=2, column=0, padx=20, pady=5)
        self.info_label = tk.CTkLabel(master=info_frame, text="             Register ...             ", font=("Times New Roman", 24))
        self.info_label.grid(row=0, column=0, padx=15, pady=15)

        # Run register goal window
        self.register_goal_window.mainloop()

    def close_register_goal_window(self):
        # Destroy register goal window and show main window
        self.register_goal_window.destroy()
        self.root.deiconify()

    def validate_float(self, value):
        try:
            if value == "":
                return True
            float(value)
            return True
        except ValueError:
            return False

    def write_entry_to_file(self):
        goal_name = self.goal_name_entry.get()
        if goal_name == "":
            goal_name = "Goal_Coordinate"
        
        if goal_name == "Origin":
            goal_name = "Origin 1"

        x_value = self.x_entry.get()
        if x_value == "":
            x_value = 0.0
        
        y_value = self.y_entry.get()
        if y_value == "":
            y_value = 0.0
        
        qz_value = self.qz_entry.get()
        if qz_value == "":
            qz_value = 0.0
        
        qw_value = self.qw_entry.get()
        if qw_value == "":
            qw_value = sqrt(1-float(qz_value)**2)
        
        # Format values to two decimal points
        x_value = f"{float(x_value):.2f}"
        y_value = f"{float(y_value):.2f}"
        qz_value = f"{float(qz_value):.2f}"
        qw_value = f"{float(qw_value):.2f}"
        
        username = os.getlogin()
        base_path = f"/home/{username}/GoalCoordinates"
        filename = f"{goal_name}.txt"
        filepath = os.path.join(base_path, filename)
        if not os.path.exists(base_path):
            os.makedirs(base_path)

        with open(filepath, 'w') as file:
            file.write(f"{x_value}\t{y_value}\t{qz_value}\t{qw_value}\n")
        
        self.info_label.configure(text="      Register Succesful       ")
    
    def write_current_pose_to_file(self):
        goal_name = self.goal_name_entry.get()
        if goal_name == "":
            goal_name = "Goal_Coordinate"

        if goal_name == "Origin":
            goal_name = "Origin 1"
        
        if self.get_current_pose():
            x_value = self.current_pose_x
            y_value = self.current_pose_y
            qz_value = self.current_pose_qz
            qw_value = self.current_pose_qw
            self.info_label.configure(text="      Register Succesful       ")
        else:
            self.info_label.configure(text="           Register Fail           ")
            return

        # Format values to two decimal points
        x_value = f"{float(x_value):.2f}"
        y_value = f"{float(y_value):.2f}"
        qz_value = f"{float(qz_value):.2f}"
        qw_value = f"{float(qw_value):.2f}"
        
        username = os.getlogin()
        base_path = f"/home/{username}/GoalCoordinates"
        base_path = "/home/ljx/GoalCoordinates"
        filename = f"{goal_name}.txt"
        filepath = os.path.join(base_path, filename)
        if not os.path.exists(base_path):
            os.makedirs(base_path)

        with open(filepath, 'w') as file:
            file.write(f"{x_value}\t{y_value}\t{qz_value}\t{qw_value}\n")

    # Set Goal Pose
    def open_set_goal_window(self):
        
        # Update Register Goal
        self.update_register_goal()
        
        # Hide main window
        self.root.withdraw()

        # Create new set goal window
        self.set_goal_window = tk.CTk()
        self.set_goal_window.title("Set Goal")

        # Window Size
        self.set_goal_window.geometry("320x240")
        self.set_goal_window.resizable(False, False)

        # Set Goal Label
        set_label = tk.CTkLabel(master=self.set_goal_window, text="Set Goal Window", font=("Times New Roman", 24))
        set_label.grid(row=0, column=0, padx=15, pady=15)

        # Create a Frame
        button_frame = tk.CTkFrame(master=self.set_goal_window, width=280, height=150)
        button_frame.grid(row=1, column=0, padx=20, pady=5)


        # Dropdown list for registered goals
        registered_goal_names = list(self.registered_goal.keys())
        self.goal_dropdown = tk.CTkComboBox(master=button_frame,
                                       values=registered_goal_names,
                                       font=("Times New Roman", 24),
                                       dropdown_font=("Times New Roman", 24),
                                       width=250,
                                       height=30)
        self.goal_dropdown.grid(row=0, column=0, padx=15, pady=5)

        # Set Goal Button
        set_goal_button = tk.CTkButton(master=button_frame, text="Set Goal", font=("Times New Roman", 24), command=self.set_goal)
        set_goal_button.grid(row=1, column=0, padx=15, pady=5)
        set_goal_button.configure(width=250, height=50)

        # Exit Button
        exit_button = tk.CTkButton(master=button_frame, text="Exit", font=("Times New Roman", 24), command=self.close_set_goal_window)
        exit_button.grid(row=2, column=0, padx=15, pady=5)
        exit_button.configure(width=250, height=50)

        # Run register goal window
        self.set_goal_window.mainloop()

    def close_set_goal_window(self):
        # Destroy set goal window and show main window
        self.set_goal_window.destroy()
        self.root.deiconify()

    def update_register_goal(self):

        username = os.getlogin()
        base_path = f"/home/{username}/GoalCoordinates"
        
        # Check if the directory exists
        if not os.path.exists(base_path):
            self.get_logger().warn(f"Directory {base_path} does not exist.")
            return
        
        # Initialize or clear existing registered goals
        self.registered_goal = {"Origin": {"x" : 0.0, "y" : 0.0, "qz" : 0.0, "qw" : 1.0}}

        # Iterate over files in the directory
        for filename in os.listdir(base_path):
            filepath = os.path.join(base_path, filename)

            # Skip directories (if any)
            if os.path.isdir(filepath):
                continue
            
            # Attempt to read and parse the file
            try:
                with open(filepath, 'r') as file:
                    line = file.readline().strip()
                    if line:
                        # Split the line into components
                        values = line.split('\t')
                        
                        # Ensure we have exactly 4 values
                        if len(values) == 4:
                            # Convert values to float
                            x_value = float(values[0])
                            y_value = float(values[1])
                            qz_value = float(values[2])
                            qw_value = float(values[3])

                            goal_name = os.path.splitext(filename)[0]

                            # Add to registered goals dictionary
                            self.registered_goal[goal_name] = {
                                'x': x_value,
                                'y': y_value,
                                'qz': qz_value,
                                'qw': qw_value
                            }
                        else:
                            self.get_logger().warn(f"Ignoring invalid data in file {filename}")
            
            except Exception as e:
                self.get_logger().warn(f"Error reading file {filename}: {str(e)}")
        
        print(self.registered_goal)

    def set_goal(self):
        selected_goal_name = self.goal_dropdown.get()
        
        if selected_goal_name in self.registered_goal:
            goal_data = self.registered_goal[selected_goal_name]
            
            # Create a PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'  # Assuming the frame_id for the goal pose
            pose_msg.pose.position.x = goal_data['x']
            pose_msg.pose.position.y = goal_data['y']
            pose_msg.pose.orientation.z = goal_data['qz']
            pose_msg.pose.orientation.w = goal_data['qw']
            
            # Publish the message
            self.publisher_.publish(pose_msg)
            self.get_logger().info(f"Published goal pose for '{selected_goal_name}'")

        else:
            self.get_logger().warn(f"Selected goal '{selected_goal_name}' not found in registered goals.")

    # Remove Goal Pose
    def open_remove_goal_window(self):
        
        # Update Register Goal
        self.update_register_goal()
        
        # Hide main window
        self.root.withdraw()

        # Create new set goal window
        self.remove_goal_window = tk.CTk()
        self.remove_goal_window.title("Remove Goal")

        # Window Size
        self.remove_goal_window.geometry("320x240")
        self.remove_goal_window.resizable(False, False)

        # Remove Goal Label
        remove_label = tk.CTkLabel(master=self.remove_goal_window, text="Remove Goal Window", font=("Times New Roman", 24))
        remove_label.grid(row=0, column=0, padx=15, pady=15)

        # Create a Frame
        button_frame = tk.CTkFrame(master=self.remove_goal_window, width=280, height=150)
        button_frame.grid(row=1, column=0, padx=20, pady=5)


        # Dropdown list for registered goals
        registered_goal_names = list(self.registered_goal.keys())
        self.goal_dropdown = tk.CTkComboBox(master=button_frame,
                                       values=registered_goal_names,
                                       font=("Times New Roman", 24),
                                       dropdown_font=("Times New Roman", 24),
                                       width=250,
                                       height=30)
        self.goal_dropdown.grid(row=0, column=0, padx=15, pady=5)

        # Set Goal Button
        remove_goal_button = tk.CTkButton(master=button_frame, text="Remove Goal", font=("Times New Roman", 24), command=self.remove_register_goal)
        remove_goal_button.grid(row=1, column=0, padx=15, pady=5)
        remove_goal_button.configure(width=250, height=50)

        # Exit Button
        exit_button = tk.CTkButton(master=button_frame, text="Exit", font=("Times New Roman", 24), command=self.close_remove_goal_window)
        exit_button.grid(row=2, column=0, padx=15, pady=5)
        exit_button.configure(width=250, height=50)

        # Run register goal window
        self.remove_goal_window.mainloop()

    def close_remove_goal_window(self):
        # Destroy set goal window and show main window
        self.remove_goal_window.destroy()
        self.root.deiconify()

        # Update Register Goal
        self.update_register_goal()

    def remove_register_goal(self):
        
        username = os.getlogin()
        base_path = f"/home/{username}/GoalCoordinates"
        
        # Check if the directory exists
        if not os.path.exists(base_path):
            self.get_logger().warn(f"Directory {base_path} does not exist.")
            return
        
        goal_name = self.goal_dropdown.get()
        
        if goal_name not in self.registered_goal:
            self.get_logger().warn(f"Goal {goal_name} does not exist in registered goals.")
            return

        filename = goal_name + ".txt"
        filepath = os.path.join(base_path, filename)
        
        if os.path.exists(filepath):
            os.remove(filepath)
            self.get_logger().info(f"Removed file: {filepath}")
        else:
            self.get_logger().warn(f"File {filepath} does not exist.")
        
        if goal_name in self.registered_goal:
            del self.registered_goal[goal_name]
        
        # Update the dropdown list
        registered_goal_names = list(self.registered_goal.keys())
        self.goal_dropdown.configure(values=registered_goal_names)
        self.goal_dropdown.set(registered_goal_names[0] if registered_goal_names else "")

        self.update_register_goal()


    # Run
    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = Ella2Gui()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
