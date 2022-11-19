class ActivityExecutor():

    def __init__(self, activity_plan):
        self.activity_plan = activity_plan
        self.activity_idx = 0

    def next_activity(self):
        activity = self.activity_plan[self.activity_idx]
        if activity.name == 'move-to-base':
            self.move_to_base(activity.parameters)
        elif activity.name == 'move-gripper':
            self.move_gripper(activity.parameters)
        elif activity.name == 'close-drawer':
            self.close_drawer(activity.parameters)
        elif activity.name == 'grab-item':
            self.grab_item(activity.parameters)
        elif activity.name == 'release-object':
            self.release_object(activity.parameters)
        elif activity.name == 'move-item-to-drawer':
            self.move_item_to_drawer(activity.parameters)
        elif activity.name == 'move-gripper-from-drawer':
            self.move_gripper_from_drawer(activity.parameters)

    def move_to_base(self, params):
        pass

    def move_gripper(self, params):
        pass

    def grab_handle(self, params):
        pass

    def open_drawer(self, params):
        pass

    def close_drawer(self, params):
        pass
    
    def grab_item(self, params):
        pass
    
    def release_object(self, params):
        pass
    
    def move_item_to_drawer(self, params):
        pass
    
    def move_gripper_from_drawer(self, params):
        pass
    
    


