import time
import thread
from debug import *

# [13, CMD_FAMILY, CMD_ID, HEADER_CHECKSUM, BODY, BODY_CHECKSUM]
class RoverController:
    def send_command(cmd_family, cmd_id, body):
        _msg = [13, cmd_family, cmd_id, (cmd_id + cmd_family) % 256]

        for i in body:
            _msg.append(i)

        total = 0
        for i in _msg:
            total += i

        _msg.append(total % 256) # body checksum
        msg = bytearray(_msg)
        # send the msg command
        # TODO: send command here

    def recv_command(cmd_family, cmd_id):
        #do stuff
        # TODO:  check if the command id/family is identical
        # TODO: receive command here
        body = []
        return body


    # Command Family
    class CommandFamily:
        class MotorCommand:
            FAMILY_ID = 0
            # MOTOR COMMANDS
            SET_MOTOR_VEL = 0
            SET_MOTOR_ACC = 1
            SET_MOTOR_POS = 2

            GET_MOTOR_ACC = 3
            GET_MOTOR_VEL = 4
            GET_MOTOR_POS = 5

            GET_MOTOR_TEMP = 6
            GET_MOTOR_CURRENT = 7
            GET_MOTOR_VOLTAGE = 8

            SET_STATE = 9
            GET_STATE = 10

            RESET = 11

        class SensorCommand:
            FAMILY_ID = 1
            # TODO: sensor command table


class MotorController():
    def __init__(self, motor_index):
        self.index = motor_index
        self.acc = 0
        self.vel = 0
        self.pos = 0
        self.recursive_update = False
        self.enabled = False
        self.temperature = 0
        self.current = 0
        self.voltage = 0

        self.disable_when_above_restrictions = True
        self._max_current = 6
        self._max_voltage = 13
        self._max_temperature = 30

        """This is the scaling factor where the low level interface multiplies in order to prevent
            floating numbers"""
        self._velocity_scaling = 1.0
        self._acceleration_scaling = 1.0
        self._position_scaling = 1.0
        self._temp_scaling = 1000
        self._voltage_scaling = 1000
        self.current_scaling = 1000

    def check_status(self):
        if self.current > self._max_current:
            if self.disable_when_above_restrictions:
                self.disable()
            warn("Current limit reached.")
        elif self.current > self._max_current * 0.8:
            warn("Current exceeded %80 of current limit.")


        if self.temperature > self._max_temperature:
            if self.disable_when_above_restrictions:
                self.disable()
            warn("Temperature limit reached.")
        elif self.temperature > self._max_temperature * 0.8:
            warn("Temperature exceeded %80 of temperature limit.")


        if self.voltage > self._max_voltage:
            if self.disable_when_above_restrictions:
                self.disable()
            warn("Voltage limit Reached.")
        elif self.voltage > self._max_voltage * 0.8:
            warn("Voltage exceeded %80 of voltage limit.")

    def reset_controller(self):
        RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.RESET,
            [self.index])

    def update(self, loop=False, loop_rate=1):
        print "updating."
        def _update(controller):
            controller.acc = controller.get_acc()
            controller.vel = controller.get_vel()
            controller.pos = controller.get_pos()
            controller.voltage = controller.get_voltage()
            controller.current = controller.get_current()
            controller.temperature = controller.get_temp()
            controller.check_status()
            while controller.recursive_update:
                controller.acc = controller.get_acc()
                controller.vel = controller.get_vel()
                controller.pos = controller.get_pos()
                controller.voltage = controller.get_voltage()
                controller.current = controller.get_current()
                controller.temperature = controller.get_temp()
                controller.check_status()
                time.sleep(1.0 / loop_rate)
            thread.exit()

        if loop:
            self.recursive_update = True
            thread.start_new_thread(_update, (self,))
        else:
            self.recursive_update = False
            thread.start_new_thread(_update, (self,))

    def terminate_update(self):
        debug("Update_terminated")
        self.recursive_update = False

    def enable(self):
        debug("Controller enabled")
        if not self.enabled:
            # do stuff
            RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
                RoverController.CommandFamily.MotorCommand.SET_STATE, [self.index, 1])
                # TODO: confirmation here OK message from stm
        self.enabled = True

    def disable(self):
        debug("Contorller disabled")
        if self.enabled:
            RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
                RoverController.CommandFamily.MotorCommand.SET_STATE, [self.index, 0])
                # TODO: confirmation here OK message from stm
            # do stuff
        self.enabled = False


    def set_vel(self, rpm):
        debug("Setting vel to " + str(rpm))
        rpm *= self._velocity_scaling

        # sign 0 is +
        # sign 1 is -
        sign = 0
        if rpm < 0:
            sign = 1
            rpm = abs(rpm)

        RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.SET_MOTOR_VEL,
            [self.index, sign, int(rpm / 256), int(rpm % 256)])
            # example:  rpm = 500
            # [1, 244]
            # example:  rpm = 756
            # [2, 244]

    def set_acc(self, acc):
        # degrees/sec^2
        # to be Multiply with 1000
        debug("setting acc to " + str(acc))
        acc *= self._acceleration_scaling
        sign = 0
        if acc < 0:
            sign = 1
            acc = abs(acc)
        RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.SET_MOTOR_ACC,
            [self.index, sign, int(acc / 256), int(acc % 256)])

    def set_pos(self, degree):
        # degree
        # [0-360] -> integer
        debug("setting degree to " + str(degree))
        degree *= self._position_scaling

        sign = 0
        if degree < 0:
            sign = 1
            degree = abs(degree)

        RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.SET_MOTOR_POS,
            [self.index, sign, int(degree / 256), int(degree % 256)])


    def get_vel(self):
        # [index, sign, vel / 256, vel % 256]
        RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_VEL,
            [self.index])
        msg_body = RoverController.recv_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_VEL)
        if msg_body[0] != self.index:
            error("Returned motor index is not identical with sender")
            return None
        velocity = msg_body[2] * 256 + msg_body[3]
        if msg_body[1] == 1:
            velocity *= -1
        return velocity / self._velocity_scaling

    def get_acc(self):
        # [index, sign, acc / 256, acc % 256]

        RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_ACC,
            [self.index])
        msg_body = RoverController.recv_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_ACC)

        if msg_body[0] != self.index:
            error("Returned motor index is not identical with sender")
            return None

        acc = msg_body[2] * 256 + msg_body[3]
        if msg_body[1] == 1:
            acc *= -1
        return acc / self._acceleration_scaling

    def get_pos(self):
        # [index, sign, pos / 256, pos % 256]
        RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_POS,
            [self.index])
        msg_body = RoverController.recv_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_POS)

        if msg_body[0] != self.index:
            error("Returned motor index is not identical with sender")
            return None

        pos = msg_body[2] * 256 + msg_body[3]
        if msg_body[1] == 1:
            pos *= -1
        return pos / self._position_scaling

    def get_temp(self):
        # [index, sign, temperature]
        RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_TEMP,
            [self.index])
        msg_body = RoverController.recv_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_TEMP)
        if msg_body[0] != self.index:
            error("Returned motor index is not identical with sender")
            return None
        temp = msg_body[2]
        if msg_body[1] == 1:
            temp *= -1
        return temp / self._temp_scaling

    def get_current(self):
        # [index, sign, value]
        RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_CURRENT,
            [self.index])
        msg_body = RoverController.recv_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_CURRENT)
        if msg_body[0] != self.index:
            error("Returned motor index is not identical with sender")
            return None

        current = msg_body[2]
        if msg_body[1] == 1:
            current *= -1
        return current / self.current_scaling

    def get_voltage(self):
        # [index, sign, value]
        RoverController.send_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_VOLTAGE,
            [self.index])
        msg_body = RoverController.recv_command(RoverController.CommandFamily.MotorCommand.FAMILY_ID,
            RoverController.CommandFamily.MotorCommand.GET_MOTOR_VOLTAGE)
        if msg_body[0] != self.index:
            error("Returned motor index is not identical with sender")
            return None

        voltage = msg_body[2]
        if msg_body[1] == 1:
            voltage *= -1
        return voltage / self._voltage_scaling
