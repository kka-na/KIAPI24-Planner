class PID:
    # def __init__(self, p_gain, i_gain, d_gain, sampling_time):
    #     self.p_gain = p_gain
    #     self.i_gain = i_gain
    #     self.d_gain = d_gain
    #     self.sampling_time = sampling_time

    #     self.previous_error = 0
    #     self.integral_error = 0
    def __init__(self, p_gain, i_gain, d_gain, sampling_time):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.sampling_time = sampling_time
        #print("sampling time : ", sampling_time)

        self.previous_error = 0
        self.integral_error = 0

        ## APID
        self.Kp = p_gain
        self.Ki = i_gain
        self.Kd = d_gain
        #print("Initial P:{:4.2f} I:{:4.2f} D:{:4.2f}".format(self.Kp, self.Ki, self.Kd))

        self.dKp = 0
        self.dKi = 0
        self.dKd = 0

        self.ddKp = 0
        self.ddKi = 0
        self.ddKd = 0

        self.errs = [-1, -1, -1, -1, -1] # k-3, k-2, k-1, k, k+1 
        self.outs = [-1, -1, -1, -1, -1]
        self.ctrls = [-1, -1, -1, -1, -1]

        self.lr = 0.001

        self.cnt = 0


    def change_gains(self, p_gain, i_gain, d_gain):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        #print("changed PID gain: ", self.p_gain, self.i_gain, self.d_gain)

    def get_output_origin(self, target_value, current_value):
        error = (target_value-current_value)
        self.integral_error += error*self.sampling_time
        self.integral_error = max(-100, min(self.integral_error, 100))
        derivative_error = (error-self.previous_error)/self.sampling_time
        output = self.p_gain*error + self.i_gain*self.integral_error + self.d_gain*derivative_error

        self.previous_error = error
        return output

    def get_output(self, target_value, current_value):
        # update
        for i in range(3):
            self.errs[i] = self.errs[i+1]
            self.outs[i] = self.outs[i+1]
            self.ctrls[i] = self.ctrls[i+1]
        self.errs[3] = target_value - current_value
        self.outs[3] = current_value
        self.ctrls[3] = self.ctrls[4]

        if self.cnt < 3:
            self.cnt += 1
            Kp = self.p_gain
            Ki = self.i_gain
            Kd = self.d_gain
            self.ctrls[4] = self.Kp*self.errs[3]
        else:
            # ddK(k-1)
            self.ddKp = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / (self.ctrls[1] - self.ctrls[0])) * \
                        (self.errs[2] - self.errs[1])
            self.ddKi = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / (self.ctrls[1] - self.ctrls[0])) * \
                        (self.errs[2])
            self.ddKd = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / (self.ctrls[1] - self.ctrls[0])) * \
                        (self.errs[2] - 2*self.errs[1] + self.errs[0])
            # dK(k)
            self.dKp = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / (self.ctrls[2] - self.ctrls[1])) * \
                        (self.errs[3] - self.errs[2])
            self.dKi = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / (self.ctrls[2] - self.ctrls[1])) * \
                        (self.errs[3])
            self.dKd = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / (self.ctrls[2] - self.ctrls[1])) * \
                        (self.errs[3] - 2*self.errs[2] + self.errs[1])
            

            # K(k+1)
            f_dKp = self.dKp + self.ddKp
            f_dKi = self.dKi + self.ddKi
            f_dKd = self.dKd + self.ddKd

            threshold = 0.3
            f_dKp = min(max(f_dKp, -threshold*self.p_gain), threshold*self.p_gain)
            f_dKi = min(max(f_dKi, -threshold*self.i_gain), threshold*self.i_gain)
            f_dKd = min(max(f_dKd, -threshold*self.d_gain), threshold*self.d_gain)

            Kp = self.p_gain + f_dKp
            Ki = self.i_gain + f_dKi
            Kd = self.d_gain + f_dKd

            #print("dP:{:4.2f} dI:{:4.2f} dD:{:4.2f}".format(self.dKp + self.ddKp, self.dKi + self.ddKi, self.Kd + self.dKd + self.ddKd))

            # control_amount
            delta_u_k = (Kp * (self.errs[3] - self.errs[2])) + \
                        (Ki * self.errs[3]) + \
                        (Kd * (self.errs[3] - 2*self.errs[2] + self.errs[1]))

            self.ctrls[4] = (self.ctrls[3] + delta_u_k)*self.sampling_time

        if abs(self.errs[3]) > 10:
            self.ctrls[4] = self.p_gain*self.errs[3]

        #print("P:{:4.2f} I:{:4.2f} D:{:4.2f}".format(Kp, Ki, Kd))
        #print("control_amount :", self.ctrls[4])
        return self.ctrls[4]
    