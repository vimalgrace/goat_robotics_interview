#!/usr/bin/env python3


from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from geometry_msgs.msg import PoseStamped
import tf_transformations
import time
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import os
import tkinter as tk
from threading import Thread


class GoatNavigator(BasicNavigator):

    def __init__(self):
        super().__init__("ebot_navigator")

        self.set_initial_pose()

        self.callback_group = ReentrantCallbackGroup()
        self.callback_group_1 = MutuallyExclusiveCallbackGroup()

        # Intializing tkinter here
        self.root = tk.Tk()
        self.root.geometry('490x270')
        self.root.resizable(0,0)

        # setting flags here
        self.table_1_order_status = "Not Ordered"
        self.table_2_order_status = "Not Ordered"
        self.table_3_order_status = "Not Ordered"

        self.order_1_received = False
        self.order_2_received = False
        self.order_3_received = False

        self.order_1_cancelled = False
        self.order_2_cancelled = False
        self.order_3_cancelled = False

        self.current_status_of_robot = "Home Position"

        self.confirmation_text = "Give Confirmation"

        self.target_timer = 3.0

        self.current_table_name = None
        self.going_to_kitchen = False
        self.going_to_home = False

        self.orders_list = {}

        self.table_1_pose = [-1.28, 2.04, -1.57]
        self.table_2_pose = [5.1, 2.2, 0.0]
        self.table_3_pose = [5.1, -3.08, 0.0]

        # self.table_1_pose = [-4.0, 0.8, 0.0]
        # self.table_2_pose = [-4.0, 2.0, 0.0]
        # self.table_3_pose = [-4.0, -0.15, 0.0]

        self.kitchen_pose = [-7.5, -2.8, 1.57]
        # self.kitchen_pose = [-5.0, 0.8, 0.0]

        self.wait_time = 5.0
        self.order_confirmed = False
        self.must_go_to_kitchen = False

        self.reached_kitchen = False
        self.going_to_table = False
        self.current_navigating_position = None
    
        self.timer_thread = Thread(target = self.update_timer)
        self.timer_thread.start()

        self.activate_gui()

    def reset_flags(self):

        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function reset the flags present in init method to default state
        
        
        '''


        self.table_1_order_status = "Not Ordered"
        self.table_2_order_status = "Not Ordered"
        self.table_3_order_status = "Not Ordered"

        self.order_1_received = False
        self.order_2_received = False
        self.order_3_received = False

        self.order_1_cancelled = False
        self.order_2_cancelled = False
        self.order_3_cancelled = False

        self.current_status_of_robot = "Home Position"

        self.confirmation_text = "Give Confirmation"

        self.target_timer = 3.0

        self.current_table_name = None
        self.going_to_kitchen = False
        self.going_to_home = False

        self.orders_list = {}

        # self.table_1_pose = [-1.28, 2.04, -1.57]
        # self.table_2_pose = [5.1, 2.2, 0.0]
        # self.table_3_pose = [5.1, -3.08, 0.0]

        self.table_1_pose = [-4.0, 0.8, 0.0]
        self.table_2_pose = [-4.0, 2.0, 0.0]
        self.table_3_pose = [-4.0, -0.15, 0.0]

        # self.kitchen_pose = [-7.5, -2.8, 1.57]
        self.kitchen_pose = [-5.0, 0.8, 0.0]

        self.wait_time = 5.0
        self.order_confirmed = False
        self.must_go_to_kitchen = False

        self.reached_kitchen = False
        self.going_to_table = False

        self.current_navigating_position = None
    

        
    def update_timer(self):
        '''
            Inputs:    
                msg  :  None
            
            Output:
                None

            Description:
                This is run by another thread and executed continuously.
        
        
        '''


        while True:
            time.sleep(0.5)
            # print(f"order_1 = {self.table_1_order_status}")
            # print(f"order_2 = {self.table_2_order_status}")
            # print(f"order_3 = {self.table_3_order_status}")
            # print(self.orders_list)
            if len(self.orders_list) > 0:
                print("Entered")
                self.go_to_kitchen_position(self.kitchen_pose)

                if self.reached_kitchen:
                    self.confirm_order(text = "Kitchen")

                    if not self.order_confirmed:
                        self.orders_list = {}
                        self.timeout_confirmation.config(text = f"Order Not Approved")
                        self.cancel_table_orders()
                        self.order_confirmed = False
                        print("continue...")
                        self.go_to_home_position()
                        continue

                    else:
                        self.timeout_confirmation.config(text = f"Received food from the kitchen")

                
                for table_name, table_position in list(self.orders_list.items()):
                    self.go_to_table_position(table_name, table_position)

                if self.must_go_to_kitchen:
                    self.go_to_kitchen_position(self.kitchen_pose)
                    self.must_go_to_kitchen = False
                    
                    if len(self.orders_list) > 0:
                        continue
                    

                self.go_to_home_position()



    def cancel_table_orders(self):
        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                cancel the table orders in tkinter gui
        
        
        '''

        self.table_1_order_status_label.config(text = f"Order Aborted")
        self.table_2_order_status_label.config(text = f"Order Aborted")
        self.table_3_order_status_label.config(text = f"Order Aborted")

    def reset_table_orders(self):

        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                Reset the table orders to default in tkinter gui
        
        
        '''


        self.table_1_order_status_label.config(text = f"Not Ordered")
        self.table_2_order_status_label.config(text = f"Not Ordered")
        self.table_3_order_status_label.config(text = f"Not Ordered")
        self.timeout_confirmation.config(text = f"Order confirmation status bar")
        

    def activate_gui(self):
        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function created the tkinter widget , labels and buttons
        
        
        '''


        self.reset_button = tk.Button(self.root, text = "R",command = self.reset_all)
        self.reset_button.grid(row = 0, column = 3)

        self.status_label = tk.Label(self.root, text = f"Current_status : {self.current_status_of_robot}",padx=5, pady=5)
        self.status_label.grid(row = 0,columnspan=3)
        # status_label.pack(side = "top")

        # Table order buttons
        table_1_order = tk.Button(self.root,text = "Order_1",command = self.callback_table_1_order)
        table_1_order.grid(row = 1, column=0,padx=10, pady=10)

        table_2_order = tk.Button(self.root,text = "Order_2",command = self.callback_table_2_order)
        table_2_order.grid(row = 1, column=1,padx=10, pady=10)

        table_3_order = tk.Button(self.root,text = "Order_3",command = self.callback_table_3_order)
        table_3_order.grid(row = 1, column=2,padx=10, pady=10)

        # Table cancel order buttons

        table_1_order_cancel = tk.Button(self.root,text = "Cancel Order_1",command = self.callback_table_1_order_cancel)
        table_1_order_cancel.grid(row = 2,column=0,padx=10, pady=10)

        table_2_order_cancel = tk.Button(self.root,text = "Cancel Order_2",command = self.callback_table_2_order_cancel)
        table_2_order_cancel.grid(row = 2,column=1,padx=10, pady=10)

        table_3_order_cancel = tk.Button(self.root,text = "Cancel Order_3",command = self.callback_table_3_order_cancel)
        table_3_order_cancel.grid(row = 2,column=2,padx=10, pady=10)

        # Order status button

        self.table_1_order_status_label = tk.Label(self.root,text = f"{self.table_1_order_status}")
        self.table_1_order_status_label.grid(row = 3, column = 0,padx=10, pady=10)

        self.table_2_order_status_label = tk.Label(self.root,text = f"{self.table_2_order_status}")
        self.table_2_order_status_label.grid(row = 3, column = 1,padx=10, pady=10)

        self.table_3_order_status_label = tk.Label(self.root,text = f"{self.table_3_order_status}")
        self.table_3_order_status_label.grid(row = 3, column = 2,padx=10, pady=10)

        # self.order_confirmation = tk.Button(self.root,text = f"{self.confirmation_text}",command = self.callback_confirm_order)
        # self.order_confirmation.grid(row = 4,columnspan=3,padx=10, pady=10)

        self.timeout_confirmation = tk.Label(self.root,text = f"Order confirmation status bar")
        self.timeout_confirmation.grid(row = 5,columnspan=3,padx=10, pady=10)


        self.root.mainloop()


    def reset_all(self):
        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function resets the entire tkinter app
        
        
        '''

        try:
            for _ in range(3):
                self.cancelTask()
           
            self.reset_flags()
            self.reset_table_orders()

            self.go_to_home_position()

            self.reset_flags()
            self.reset_table_orders()

        except Exception as e:
            print(f"Error is {e}")


    def callback_table_1_order(self):
        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function accepts table order 1
        
        
        '''

        self.order_1_received = True
        self.table_1_order_status = "Ordered"
        self.table_1_order_status_label.config(text = f"{self.table_1_order_status}")

        if "table_1" not in self.orders_list:
            self.orders_list["table_1"] = self.table_1_pose


    def callback_table_2_order(self):
        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function accepts table order 2
        
        
        '''

        self.order_2_received = True
        self.table_2_order_status = "Ordered"
        self.table_2_order_status_label.config(text = f"{self.table_2_order_status}")

        if "table_2" not in self.orders_list:
            self.orders_list["table_2"] = self.table_2_pose

    def callback_table_3_order(self):

        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function accepts table order 3
        
        
        '''

        self.order_3_received = True
        self.table_3_order_status = "Ordered"
        self.table_3_order_status_label.config(text = f"{self.table_3_order_status}")

        if "table_3" not in self.orders_list:
            self.orders_list["table_3"] = self.table_3_pose


    def callback_table_1_order_cancel(self):

        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function cancels table order 1
        
        
        '''

        self.order_1_cancelled = True
        self.table_1_order_status = "Order Cancelled"
        self.table_1_order_status_label.config(text = f"{self.table_1_order_status}")

        if "table_1" in self.orders_list:
            self.orders_list.pop("table_1")

        if self.current_table_name == "table_1":
            self.cancelTask()

    def callback_table_2_order_cancel(self):

        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function cancels table order 2
        
        
        '''

        self.order_2_cancelled = True
        self.table_2_order_status = "Order Cancelled"
        self.table_2_order_status_label.config(text = f"{self.table_2_order_status}")

        if "table_2" in self.orders_list:
            self.orders_list.pop("table_2")


        if self.current_table_name == "table_2":
            self.cancelTask()


    def callback_table_3_order_cancel(self):

        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function cancels table order 3
        
        
        '''


        self.order_3_cancelled = True
        self.table_3_order_status = "Order Cancelled"
        self.table_3_order_status_label.config(text = f"{self.table_3_order_status}")

        if "table_3" in self.orders_list:
            self.orders_list.pop("table_3")

        if self.current_table_name == "table_3":
            self.cancelTask()

    

    
    def confirm_order(self,text):

        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function waits for the confirmation from kitchen and table
        
        
        '''

        self.order_confirmation = tk.Button(self.root,text = f"{self.confirmation_text}",command = self.callback_confirm_order)
        self.order_confirmation.grid(row = 4,columnspan=3,padx=10, pady=10)


        self.timeout_confirmation.config(text = f"Waiting for confirmation from {text} ({self.wait_time} seconds)")
        self.order_confirmed = False
        start = time.time()
        while not (abs(time.time() - start) > self.wait_time):
            time.sleep(0.1)

            if self.order_confirmed:
                break
        
        # self.order_confirmation = tk.Button(self.root,text = f"",command = self.callback_confirm_order)
        self.order_confirmation.grid_forget()

    def callback_confirm_order(self):
        self.timeout_confirmation.config(text = f"Order Confirmed")
        self.order_confirmed = True
        

    def change_footprint(self,footprint_value):
        os.system(f"ros2 param set /local_costmap/local_costmap footprint '{footprint_value}'")
        os.system(f"ros2 param set /global_costmap/global_costmap footprint '{footprint_value}'")
        self.get_logger().info("Changed the robot footprint")



    def create_pose_stamped(self,x,y,yaw):

        '''
            Inputs:    
                x           :  float
                y           :  float
                yaw         :  float
            
            output:
                pose_stamped object

            Description:
                This function returns the pose stamped object for the given x,y,and yaw values
        
        
        '''
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0

        q_x,q_y,q_z,q_w = tf_transformations.quaternion_from_euler(0.0,0.0,yaw)

        pose_stamped.pose.orientation.x = q_x
        pose_stamped.pose.orientation.y = q_y
        pose_stamped.pose.orientation.z = q_z
        pose_stamped.pose.orientation.w = q_w
        return pose_stamped


    def wait_and_display_result(self,msg = "Goal succeeded"):

        '''
            Inputs:    
                msg  :  string
            
            Output:
                None

            Description:
                This function waits for the current nav2 goal to complete
        
        
        '''

        while not self.isTaskComplete():
           
            print("Task in progress...")

            if self.going_to_kitchen:
                if not self.must_go_to_kitchen:
                    if not (len(self.orders_list) > 0):
                        self.cancelTask()
                        self.going_to_kitchen = False
                        self.reached_kitchen = False

            if self.going_to_home:
                if len(self.orders_list) > 0:
                    self.cancelTask()
                    self.going_to_home = False

        result = self.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(msg)

            if self.going_to_kitchen:
                self.reached_kitchen = True

            if self.going_to_home:
                self.status_label.config(text = f"Reached Home")
                self.going_to_home = False
                self.reset_table_orders()

            if self.going_to_table:
                self.status_label.config(text = f"Reached {self.current_table_name}")

                self.confirm_order(text = self.current_table_name)

                if self.order_confirmed:
                    self.update_respective_table(self.current_table_name, "Accepted")
                    self.status_label.config(text = f"Order Accepted")
                else:
                    self.timeout_confirmation.config(text = f"Order not accepted by {self.current_table_name}")
                    self.status_label.config(text = f"Order Rejected")
                    self.update_respective_table(self.current_table_name,"Rejected")
                    self.must_go_to_kitchen = True
                
                self.going_to_table = False

            

        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")

            if self.going_to_table:
                self.must_go_to_kitchen = True

        
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")

            if self.going_to_kitchen:
                self.go_to_kitchen_position(self.current_navigating_position)

            elif self.going_to_table:
                self.go_to_table_position(self.current_table_name,self.current_navigating_position)

            elif self.going_to_home:
                self.go_to_home_position()

        else:
            self.get_logger().info("Goal has invalid return status!")

    
    def set_initial_pose(self):

        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function sets the initial pose for robot
        
        
        '''

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = -8.4
        initial_pose.pose.position.y = 3.2
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.z = 0.0095
        initial_pose.pose.orientation.w = 1.0

        self.setInitialPose(initial_pose)
        print("Initial pose is set")

        self.waitUntilNav2Active()
        print("Nav2 is in active state")


    def update_respective_table(self, table_name,text):
        '''
            Inputs:    
                table_name: string - name of table
                text: string - message to update
            
            Output:
                None

            Description:
                This function updates the respective table labels
        
        
        '''

        if table_name == "table_1":
            self.table_1_order_status = f"Order {text}"
            self.table_1_order_status_label.config(text = f"{self.table_1_order_status}")

        elif table_name == "table_2":
            self.table_2_order_status = f"Order {text}"
            self.table_2_order_status_label.config(text = f"{self.table_2_order_status}")

        elif table_name == "table_3":
            self.table_3_order_status = f"Order {text}"
            self.table_3_order_status_label.config(text = f"{self.table_3_order_status}")



    def go_to_table_position(self,table_name,table_pose):

        '''
            Inputs:    
                table_name: string - name of table
                table_pose: list - [x,y,yaw] of table
            
            Output:
                None

            Description:
                This function make the robot to navigate to the respective table
        
        
        '''


        self.current_navigating_position = table_pose

        self.going_to_table = True

        self.status_label.config(text = f"Going to {table_name}")

        self.current_table_name = table_name

        print(f"Target table position = {table_pose}")

        table_pose_stamped = self.create_pose_stamped(*table_pose)

        self.goToPose(table_pose_stamped)

        self.wait_and_display_result(f"Reached {table_name}")


        if table_name in self.orders_list:
            self.orders_list.pop(table_name)

        self.order_confirmed = False
        
        self.current_table_name = None

        self.going_to_table = False

        


    def go_to_kitchen_position(self,kitchen_pose):

        '''
            Inputs:    
                kitchen_pose: list - [x,y,yaw] of kitchen position
            
            Output:
                None

            Description:
                This function make the robot to navigate to the respective table
        
        
        '''



        

        self.current_navigating_position = kitchen_pose

        self.status_label.config(text = f"Going to kitchen")

        self.going_to_kitchen = True

        print(f"Going to kitchen pose = {kitchen_pose}")

        kitchen_pose_stamped = self.create_pose_stamped(*kitchen_pose)

        self.goToPose(kitchen_pose_stamped)

        self.wait_and_display_result("Kitchen Pose reached.")

        self.going_to_kitchen = False
        

        self.status_label.config(text = f"Reached Kitchen")



    def go_to_home_position(self):

        '''
            Inputs:    
                None
            
            Output:
                None

            Description:
                This function make the robot to navigate to the kitchen
        
        
        '''

        

        

        self.status_label.config(text = f"Going to home")

        self.going_to_home = True

        print("Going to Home Position")

        home_pose = [-8.4, 3.16, 0.0]

        home_pose = self.create_pose_stamped(*home_pose)

        self.current_navigating_position = home_pose

        self.goToPose(home_pose)

        self.wait_and_display_result("Home position reached.")

        


def main():
    '''
            Inputs:    
                None
            
            output:
                None

            Description:
                This is the main function where the instance of GoatNavigator class is created.
        
        
        '''
    


    rclpy.init()
    node = GoatNavigator()

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()
 
    exit(0)




if __name__ == "__main__":
    main()

