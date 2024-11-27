import time 

class PID:
    def __init__(self, kp, ki, kd, setpoint = 0, output_limits = (None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.setpoint = setpoint
        self.output_limits = output_limits

        self.prev_error = 0
        self.integral = 0
        self.prev_time = None


    def compute(self, curr_val, curr_time = None):

        error = self.setpoint - curr_val

        # print(f'Error: {error}')

        if curr_time == None:
            curr_time = time.time()
        if self.prev_time == None:
            self.prev_time = curr_time

        delta_time = curr_time - self.prev_time

        if delta_time <= 0:
            delta_time = 1e-16 # avoids division by zero

        proportional = min(3, max(-3, self.kp * error))

        min_output, max_output = self.output_limits

        
        self.integral += error * delta_time
        # avoid integral windup by clamping
        if self.output_limits != (None, None):
            min_output, max_output = self.output_limits
            self.integral = max(min_output / self.ki, min(self.integral, max_output / self.ki))
        
        

        integral = self.ki * self.integral

        derivative = self.kd * (error - self.prev_error) / delta_time 

        output = proportional + integral + derivative

        # print(F"Prop: {proportional}, Output: {output}")

        # clamp output
        if min_output is not None:
            output = max(min_output, output)
        if max_output is not None:
            output = min(max_output, output)

        self.prev_error = error
        self.prev_time = curr_time

        return output

            

            