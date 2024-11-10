import math

#wrist to elbow: 91mm

#89.6 - 11.2 origin to shoulder

def cosd(n):
    return math.cos(n/180*math.pi)

def sind(n):
    return math.sin(n/180*math.pi)

def tand(n):
    return math.tan(n/180*math.pi)

def acosd(n):
    return math.acos(n)/math.pi*180

def asind(n):
    return math.asin(n)/math.pi*180

def atand(n):
    return math.atan(n)/math.pi*180

class InverseKinematics:

    def compute(self, height, u, v):
        #tilt angle of the platform
        theta = math.sqrt(u**2 + v**2)
        A = [0, 0, height]
        B = [0, 0, A[2] - 9/cosd(theta)]
        
        theta_c = v
        theta_d = -atand(cosd(30)*sind(u) + sind(30)*sind(v))
        theta_e = -atand(-cosd(30)*sind(u) + sind(30)*sind(v))
        #print(theta_d, theta_e)
        motor_1 = self._compute_arm_angle(B[2], theta_c)
        motor_2 = self._compute_arm_angle(B[2], theta_d)
        motor_3 = self._compute_arm_angle(B[2], theta_e)

        return [motor_1, motor_2, motor_3]

    def _compute_arm_angle(self, height, theta):
        Mx = 78.4
        My = -94
        r = 121.603
        upper_length = 75
        forearm_length = 91

        Wx = r*cosd(theta)
        Wy = height - r*sind(theta)

        d = math.sqrt((Wx - Mx)**2 + (Wy - My)**2)

        theta_2 = atand((Wx - Mx)/(Wy - My))
        theta_3 = acosd((d**2 + upper_length**2 - forearm_length**2)/(2*d*upper_length))
        return theta_2 + theta_3

class MotorOutputs:
    MOTOR_1_OFFSET = 1880
    MOTOR_2_OFFSET = 1767
    MOTOR_3_OFFSET = 1800
    MAX_ROTATION = 130
    STEPS_PER_DEGREE = 4095/360

    def compute_motor_outputs(angle1, angle2, angle3):
        output1 = int(MotorOutputs.MOTOR_1_OFFSET -angle1*MotorOutputs.STEPS_PER_DEGREE)
        output2 = int(MotorOutputs.MOTOR_2_OFFSET -angle2*MotorOutputs.STEPS_PER_DEGREE)
        output3 = int(MotorOutputs.MOTOR_3_OFFSET -angle3*MotorOutputs.STEPS_PER_DEGREE)
        return [output1, output2, output3]

    def update_offsets(offset1, offset2, offset3):
        MotorOutputs.MOTOR_1_OFFSET = offset1
        MotorOutputs.MOTOR_1_OFFSET = offset2
        MotorOutputs.MOTOR_1_OFFSET = offset3

    def angles_valid(angle1, angle2, angle3):
        if angle1 < 0 or angle1 > 133:
            return False
        if angle2 < 0 or angle2 > 133:
            return False
        if angle3 < 0 or angle3 > 133:
            return False
        return True
if __name__ == "__main__":
    ik = InverseKinematics()
    arm_angles = ik.compute(-45.5, 0, 0)
    print(arm_angles)
    print(MotorOutputs.compute_motor_outputs(*arm_angles))