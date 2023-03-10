import time


class Trajectory:
    def __init__(self, endpoints, steps=30, accuracy=0.1):
        self.endpoints = endpoints  # [Start Next Next ... Next End] in format [x,y,z]
        self.steps = steps  # No of steps between two points
        self.accuracy = accuracy  # Accuracy
        self.destination = endpoints[0]
        self.curr_index = 1
        self.end_index = len(endpoints)

    # Check if current point is close to destination
    def checkCloseness(self, cp):
        dp = self.destination
        if (
            abs(cp[0] - dp[0]) <= self.accuracy
            and abs(cp[1] - dp[1]) <= self.accuracy
            and abs(cp[2] - dp[2]) <= self.accuracy
        ):
            return True
        return False

    # Returns destination point
    def getDestination(self, current_point):
        if (not self.checkCloseness(current_point)) or (
            self.curr_index >= self.end_index
        ):
            return self.destination

        # If next end point is reached increase index
        if self.checkCloseness(self.endpoints):
            self.destination = self.endpoints[self.curr_index]
            self.curr_index += 1
            return self.getDestination(current_point)

        i = self.curr_index

        # X Change
        step = abs(self.endpoints[i][0] - self.endpoints[i - 1][0]) / self.steps
        if self.endpoints[i][0] >= self.endpoints[i - 1][0]:
            self.destination[0] = min(self.endpoints[i][0], self.destination[0] + step)
        else:
            self.destination[0] = max(self.endpoints[i][0], self.destination[0] - step)

        # Y Change
        step = abs(self.endpoints[i][1] - self.endpoints[i - 1][1]) / self.steps
        if self.endpoints[i][1] >= self.endpoints[i - 1][1]:
            self.destination[1] = min(self.endpoints[i][1], self.destination[1] + step)
        else:
            self.destination[1] = max(self.endpoints[i][1], self.destination[1] - step)

        # Z Change
        step = abs(self.endpoints[i][2] - self.endpoints[i - 1][2]) / self.steps
        if self.endpoints[i][2] >= self.endpoints[i - 1][2]:
            self.destination[2] = min(self.endpoints[i][2], self.destination[2] + step)
        else:
            self.destination[2] = max(self.endpoints[i][2], self.destination[2] - step)

        return self.destination


class Slot:
    def __init__(self):
        self.slot = 0
        self.slot_timer = time.time()
        self.slot_array = [5, 1.7, 0.5, 2.3, 0.5, 1.2, 0.5, 2.5, 0]

    def check_command(self, command):
        # print(self.slot_timer+self.slot_array[self.slot])
        # print(time.time())
        if self.slot >= len(self.slot_array):
            command = [1500, 1500, 900, 1500]
            return command
        if (self.slot_timer + self.slot_array[self.slot]) < time.time():
            self.slot_timer = time.time()
            self.slot += 1
            print(f"Next slot {self.slot}")
        if self.slot == 1:
            command[1] += 100
        if self.slot == 3:
            command[0] += 75
        if self.slot == 5:
            command[1] -= 75
        if self.slot == 7:
            command[0] -= 100
        return command
