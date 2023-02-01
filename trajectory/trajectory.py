class Trajectory:
    def __init__(self, endpoints, stepsize, accuracy):
        self.endpoints = endpoints  # [Start Next Next ... Next End]
        self.stepsize = stepsize  # Stepsize
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
        i = self.curr_index

        # X Change
        if self.endpoints[i][0] >= self.endpoints[i - 1][0]:
            self.destination[0] += self.stepsize
            if self.destination[0] > self.endpoints[i][0]:
                self.curr_index += 1
                return self.getDestination(current_point)
        else:
            self.destination[0] -= self.stepsize
            if self.destination[0] < self.endpoints[i][0]:
                self.curr_index += 1
                return self.getDestination(current_point)

        # Y Change
        if self.endpoints[i][1] >= self.endpoints[i - 1][1]:
            self.destination[1] += self.stepsize
            if self.destination[1] > self.endpoints[i][1]:
                self.curr_index += 1
                return self.getDestination(current_point)
        else:
            self.destination[1] -= self.stepsize
            if self.destination[1] < self.endpoints[i][1]:
                self.curr_index += 1
                return self.getDestination(current_point)

        # Z Change
        if self.endpoints[i][2] >= self.endpoints[i - 1][2]:
            self.destination[2] += self.stepsize
            if self.destination[2] > self.endpoints[i][2]:
                self.curr_index += 1
                return self.getDestination(current_point)
        else:
            self.destination[2] -= self.stepsize
            if self.destination[2] < self.endpoints[i][2]:
                self.curr_index += 1
                return self.getDestination(current_point)

        return self.destination
