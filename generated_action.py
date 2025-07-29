def move_custom(self):
    linear = 1.0
    angular = 0.5
    duration = 2.0
    self.send_cmd(linear, 0, duration)
    self.send_cmd(0, angular, duration)
