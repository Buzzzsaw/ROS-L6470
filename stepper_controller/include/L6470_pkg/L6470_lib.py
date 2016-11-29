#! /usr/bin/env python

import spidev

class L6470(object):
    # ==============================================================================================
    # Useful calculation constants 
    # ==============================================================================================
    # Stepping specs
    ROTATION_STEPS = 200
    TICK = 250 * 10 ** -9
    SCALE = 2 ** -18
    
    # Ratio value used to convert an acceleration from step/s^2 to step/tick^2.
    # step/tick^2 = STEP_ACC_RATIO * step/s^2
    STEP_ACC_RATIO = 0.068719
    
    # Ratio value used to convert a speed from step/s to step/tick.
    # step/tick = STEP_SPEED_RATIO * step/s
    STEP_SPEED_RATIO = 67.108864
    
    # Ratio value used to convert a max or min speed from step/s to step/tick.
    # step/tick = MAXMIN_STEP_SPEED_RATIO * step/s
    MAXMIN_STEP_SPEED_RATIO = 0.065536
    
    # Stepper directions
    DIR_CLOCKWISE = 0x01
    DIR_COUNTER_CLOCKWISE = 0x00
    
    # Register actions performed on a GoUntil or a ReleaseSW.
    ACT_RESET = 0x00
    ACT_COPY = 0x01
    
    # ==============================================================================================
    # Stepper commands
    # The parameters that appear in the Cmd byte must be added to these base command values.
    # ==============================================================================================
    CMD_NOP = 0x00
    CMD_SETPARAM = 0x00
    CMD_GETPARAM = 0x20
    CMD_RUN = 0x50
    CMD_STEPCLOCK = 0x58
    CMD_MOVE = 0x40
    CMD_GOTO = 0x60
    CMD_GOTODIR = 0x68
    CMD_GOUNTIL = 0x82
    CMD_RELEASESW = 0x92
    CMD_GOHOME = 0x70
    CMD_GOMARK = 0x78
    CMD_RESETPOS = 0xD8
    CMD_RESETDEVICE = 0xC0
    CMD_SOFTSTOP = 0xB0
    CMD_HARDSTOP = 0xB8
    CMD_SOFTHIZ = 0xA0
    CMD_HARDHIZ = 0xA8
    CMD_GETSTATUS = 0xD0
    
    # ==============================================================================================
    # Stepper registers
    # ==============================================================================================
    REG_ABS_POS = 0x01
    REG_MARK	 = 0x03
    REG_SPEED = 0x04
    REG_ACC		 = 0x05
    REG_DEC		 = 0x06
    REG_MAX_SPEED = 0x07
    REG_MIN_SPEED = 0x08
    REG_KVAL_HOLD = 0x09
    REG_KVAL_RUN = 0x0A
    REG_KVAL_ACC = 0x0B
    REG_KVAL_DEC = 0x0C
    REG_INT_SPEED = 0x0D
    REG_ST_SLP = 0x0E
    REG_FN_SLP_ACC = 0x0F
    REG_FN_SLP_DEC = 0x10
    REG_K_THERM = 0x11
    REG_ADC_OUT = 0x12
    REG_OCD_TH = 0x13
    REG_STALL_TH = 0x14
    REG_FS_SPD	 = 0x15
    REG_STEP_MODE = 0x16
    REG_ALARM_EN = 0x17
    REG_CONFIG = 0x18
    REG_STATUS	 = 0x19
    
    # ==============================================================================================
    # Step mode values
    # ==============================================================================================
    # The SYNC_EN field enables (or disables) a clock signal on the BUSY/SYNC pin at the 
    # frequency defined by the SYNC_SEL field.
    STEP_MODE_SYNC_EN_MASK = 0x80
    STEP_MODE_SYNC_EN 	 = 0x80
    
    # STEP_SEL values mask.
    STEP_MODE_SYNC_SEL_MASK = 0x70
    
    # Possible values for the SYNC_SEL field of the STEP_MODE register.
    # The numbers represent a multiplicand for the FS frequency (ex.: 8 * FS).
    SYNC_SEL_HALF = 0x00
    SYNC_SEL_1 = 0x01
    SYNC_SEL_2 = 0x02
    SYNC_SEL_4	 = 0x03
    SYNC_SEL_8 = 0x04
    SYNC_SEL_16 = 0x05
    SYNC_SEL_32 = 0x06
    SYNC_SEL_64 = 0x07
    
    # Mask for the bits of the STEP_SEL field in the STEP_MODE register.
    STEP_MODE_STEP_SEL_MASK = 0x07
    
    # Possible values for the STEP_SEL field of the STEP_MODE register.
    # The numbers represent the fraction of step (ex.: 1/32 microstep).
    STEP_SEL_FULL = 0x00
    STEP_SEL_HALF = 0x01
    STEP_SEL_4 = 0x02
    STEP_SEL_8	 = 0x03
    STEP_SEL_16 = 0x04
    STEP_SEL_32 = 0x05
    STEP_SEL_64 = 0x06
    STEP_SEL_128 = 0x07
    
    # ==============================================================================================
    # Alarm values
    # ==============================================================================================
    # The ALARM_EN register can be used to configure which alarms are active.
    # The alarm configuration fits on a single bit, so every bit represents a single alarm.
    ALARM_EN_MASK = 0xFF
    ALARM_OVERCURRENT 	 = 0x01
    ALARM_THERMAL_SHUTDOWN = 0x02
    ALARM_THERMAL_WARNING = 0x04
    ALARM_UNDERVOLTAGE = 0x08
    ALARM_STALL_DETECTION_A = 0x10
    ALARM_STALL_DETECTION_B = 0x20
    ALARM_SWITCH_TURN_ON = 0x40
    ALARM_WRONG_IGNORED_CMD = 0x80

    # ==============================================================================================
    # Open and close functions.
    # ==============================================================================================
    # Open
    # Open a new SPI connection to the specified bus and device.
    def open(self, Bus, Device):
        # Open the given SPI device
        self._SPI = spidev.SpiDev()
        self._SPI.open(Bus, Device)
        
        # Configure SPI
        self._SPI.bits_per_word = 8
        self._SPI.max_speed_hz = 100000
        self._SPI.mode = 0b11

    # Close
    # Close the currently opened SPI connection.
    def close(self):
        self._SPI.close()

    # ==============================================================================================
    # Utility functions
    # Meant to be used as private functions.
    # ==============================================================================================
    # Send command with optional params.
    def sendCmd(self, command, params=[]):
        # Send command byte to the controller.
        byte = [command]
        self._SPI.xfer2(byte)

        responseValue = 0
        length = len(params)
        #Send parameters
        for idx, byte in enumerate(params):
            request = [byte]
            response = self._SPI.xfer2(request)
            responseValue = responseValue | (response[0] << ((length-idx-1)*8))

        return responseValue

    # Send a command with the parameter split in 3 bytes and obtain the response in 3 bytes.
    def sendCmd3(self, command, param):
        # Split in 3 bytes, from MSB to LSB.
        param2 = (param >> 16) & 0xFF
        param1 = (param >> 8) & 0xFF
        param0 = param & 0xFF
        
        # Send bytes to the controller separately and build response value.
        # The bytes have to be sent separately since the Chip Select has to be reset
        # between every byte.
        request = [command]
        response = self._SPI.xfer2(request)
        responseValue = 0x000000
        
        request = [param2]
        response = self._SPI.xfer2(request)
        responseValue = responseValue | (response[0] << 16)
        
        request = [param1]
        response = self._SPI.xfer2(request)
        responseValue = responseValue | (response[0] << 8)
        
        request = [param0]
        response = self._SPI.xfer2(request)
        responseValue = responseValue | response[0]
        
        return responseValue

    # Convert an acceleration from step/tick^2 to step/s^2.
    def accToStepSec(self, StepTick2):
        # Controller gives an acceleration in step/tick^2 formatted in unsigned fixed point 0.40.
        # Equation: StepSec2 = (StepTick2 * 2^-40)/TICK^2
        # Equivalent to: StepTick2 / 0.068719
        StepSec2 = StepTick2 / self.STEP_ACC_RATIO
        
        return StepSec2

    # Convert an acceleration from step/s^2 to step/tick^2.
    def accToStepTick(self, StepSec2):
        # Controller expects an acceleration in step/tick^2 formatted in unsigned fixed point 0.40.
        # Equation: StepTick2 = (StepSec2 * TICK^2)/2^-40
        # Equivalent to: StepTick2 * 0.068719
        StepTick2 = int(round(StepSec2 * self.STEP_ACC_RATIO))
        
        # Limit speed to a 12 bits number
        return min(StepTick2, 0x00000FFF)

    # Convert a speed from step/tick to step/s.
    def speedToStepSec(self, StepTick):
        # Controller gives a speed in step/tick formatted in unsigned fixed point 0.28.
        # Equation: StepSec = (StepTick * 2^-28)/TICK
        # Equivalent to: StepTick / 67.108864
        StepSec = StepTick / self.STEP_SPEED_RATIO
        
        return StepSec

    # Convert a speed from step/s to step/tick.
    def speedToStepTick(self, StepSec):
        # Controller expects speed in step/tick formatted in unsigned fixed point 0.28.
        # Equation: StepTick = (StepSec * TICK)/(2^-28)
        # Equivalent to: StepSec * 67.108864
        StepTick = int(round(StepSec * self.STEP_SPEED_RATIO))
        
        # Limit speed to a 20 bits number
        return min(StepTick, 0x000FFFFF)

    # Convert a max or min speed from step/tick to step/s.
    # Max or min speed registers use a different encoding than the stepper speed register.
    def maxMinspeedToStepSec(self, StepTick):
        # Controller gives a speed in step/tick formatted in unsigned fixed point 0.18.
        # Equation: StepSec = (StepTick * 2^-18)/TICK
        # Equivalent to: StepTick / 0.065536
        StepSec = StepTick / self.MAXMIN_STEP_SPEED_RATIO
        
        return StepSec

    # Convert a max or min speed from step/s to step/tick.
    # Max or min speed registers use a different encoding than the stepper speed register.
    def maxMinspeedToStepTick(self, StepSec):
        # Controller expects speed in step/tick formatted in unsigned fixed point 0.18.
        # Equation: StepTick = (StepSec * TICK)/(2^-18)
        # Equivalent to: StepSec * 0.065536
        StepTick = StepSec * self.MAXMIN_STEP_SPEED_RATIO
        
        # Limit speed to a 20 bits number
        return min(int(round(StepTick)), 0x000FFFFF)

    # Convert a value to array with the MSB first
    def valueToMSBytes(self, Value, NbBytes):
        msb = []
        for i in xrange(NbBytes):
            shift = 8*(NbBytes-i-1)
            msb.append((Value >> shift) & 0xFF)
        return msb

    # ==============================================================================================
    # Stepper commands
    # ==============================================================================================
    # Nop command
    # Send a nop command, which does absolutely nothing at all.
    def nop(self):
        self.sendCmd(self.CMD_NOP)

    # SetParam command
    # Set the given register to the given value.
    def setParam(self, Param, Value, nbBytes):
        # Only accept Param as a 5 bits number.
        if Param > 0x1F:
            return

        self.sendCmd(self.CMD_SETPARAM | Param, self.valueToMSBytes(Value, nbBytes))

    # GetParam command
    # Obtain the value contained in the given register.
    def getParam(self, Param, nbBytes):
        # Only accept Param as a 5 bits number.
        if Param > 0x1F:
            return

        return self.sendCmd(self.CMD_GETPARAM | Param, self.valueToMSBytes(0, nbBytes))

    # Run command
    # Speed in in step/s. As a reference, there are 200 steps in a full rotation.
    def run(self, Direction, Speed):
        if Direction != self.DIR_CLOCKWISE and Direction != self.DIR_COUNTER_CLOCKWISE:
            print "L6470.run: Invalid direction %d" % (Direction)
            return
        if Speed <= 0:
            print "L6470.run: Invalid speed %d" % (Speed)
            return

        # Convert from step/s to step/tick
        SpeedTick = self.speedToStepTick(Speed)
                
        self.sendCmd3(self.CMD_RUN | Direction, SpeedTick)

    # StepClock command
    # Switch the device to Step-Clock mode in the given direction.
    # In Step-Clock mode, the stepper moves by a microstep on each rising edge of the STCK pin.
    # The Step-Clock mode is aborted by motion commands such as Run, Move and GoTo.
    def stepClock(self, Direction):
        if Direction != self.DIR_CLOCKWISE and Direction != self.DIR_COUNTER_CLOCKWISE:
            return
                
        self.sendCmd(self.CMD_STEPCLOCK | Direction)

    # Move command
    # Move the stepper by the given number of microsteps in agreement with the selected step mode 
    # (full, half, quarter, etc.).
    def move(self, Direction, Steps):
        if Direction != self.DIR_CLOCKWISE and Direction != self.DIR_COUNTER_CLOCKWISE:
            return
        if Steps <= 0:
            return
                
        # Limit steps to a 22 bits number
        Steps = min(Steps, 0x003FFFFF)
        
        self.sendCmd3(self.CMD_MOVE | Direction, int(Steps))

    # GoTo command
    # Move the stepper to the given absolute position in agreement with the selected step mode 
    # (full, half, quarter, etc.).
    # This function uses the shortest path possible to get to that position.
    def goTo(self, Position):
        # Limit posiition to a 22 bits number
        Position = min(Position, 0x003FFFFF)
        
        self.sendCmd3(self.CMD_GOTO, int(Position))

    # GoToDir command
    # Same ad GoTo, but with a forced rotation direction, which depending on that direction might 
    # not result in the shortest path.
    def goToDir(self, Direction, Position):
        if Direction != self.DIR_CLOCKWISE or Direction != self.DIR_COUNTER_CLOCKWISE:
            return

        # Limit posiition to a 22 bits number
        Position = min(Position, 0x003FFFFF)
        
        self.sendCmd3(self.CMD_GOTODIR | Direction, int(Position))

    # GoUntil command
    # Move the stepper at the given speed until a falling edge is detected on the SW pin of the 
    # controller (external switch activated). The SW_MODE in the CONFIG register determines if a 
    # hard stop or a soft stop is performed.
    # The action determines whether the ABS_POS register is reset (L6470.ACT_RESET) or copied
    # (L6470.ACT_COPY) in the MARK register.
    # The speed is in steps/s.
    def goUntil(self, Action, Direction, Speed):
        if Action != self.ACT_RESET and Action != self.ACT_COPY:
            return
        if Direction != self.DIR_CLOCKWISE and Direction != self.DIR_COUNTER_CLOCKWISE:
            return
        if Speed <= 0:
            return

        # Convert from step/s to step/tick
        SpeedTick = self.speedToStepTick(Speed)
                
        self.sendCmd3(self.CMD_GOUNTIL | Action | Direction, SpeedTick)

    # ReleaseSW command
    # Move the stepper at minimum speed in the given direction until a rising edge is detected
    # on the SW pin (external switch deactivated), then performs a hard stop.
    # The action determines whether the ABS_POS register is reset (L6470.ACT_RESET) or copied
    # (L6470.ACT_COPY) in the MARK register.
    # This minimum speed is the highest value between the MIN_SPEED register and 5 steps/s.
    def releaseSW(self, Action, Direction):
        if Action != self.ACT_RESET and Action != self.ACT_COPY:
            return
        if Direction != self.DIR_CLOCKWISE and Direction != self.DIR_COUNTER_CLOCKWISE:
            return
                
        self.sendCmd(self.CMD_RELEASESW | Action | Direction)

    # GoHome command
    # Equivalent to GoTo(0), but  with 3 bytes less to send.
    def goHome(self):
        self.sendCmd(self.CMD_GOHOME)

    # GoMark command
    # Equivalent to GoTo(MARK), but  with 3 bytes less to send.
    def goMark(self):
        self.sendCmd(self.CMD_GOMARK)

    # ResetPos command
    # Reset the ABS_POS register to zero.
    # The zero position is also defined as HOME position.
    def resetPos(self):
        self.sendCmd(self.CMD_RESETPOS)

    # ResetDevice command
    # Reset the stepper controller to power-up conditions.
    def resetDevice(self):
        self.sendCmd(self.CMD_RESETDEVICE)

    # SoftStop command
    # Stop the stepper using the decceleration curve defined by the DEC register.
    def softStop(self):
        self.sendCmd(self.CMD_SOFTSTOP)

    # HardStop command
    # Stop the stepper immediately.
    def hardStop(self):
        self.sendCmd(self.CMD_HARDSTOP)

    # SoftDisengage
    # Disengage the stepper after a decceleration curve as defined by the DEC register.
    def softDisengage(self):
        self.sendCmd(self.CMD_SOFTHIZ)

    # HardDisengage
    # Disengage the stepper immediately.
    def hardDisengage(self):
        self.sendCmd(self.CMD_HARDHIZ)

    # GetStatus command
    # Fetch the 16 bits value in the STATUS register.
    # Reset every warning flag and error state. Using GetParam(STATUS) does not reset these flags.
    def status(self):
        # Obtain status.
        Status = self.sendCmd3(self.CMD_GETSTATUS, 0x000000)
        
        # Keep only the two last bytes of response.
        return Status & 0x0000FFFF

    # ==============================================================================================
    # Stepper command shortcuts
    # These functions are shortcuts for using the stepper commands with a specific register.
    # ==============================================================================================
    # CurrentPosition command
    # Obtain the current absolute position of the stepper.
    def currentPosition(self):
        # Obtain position, in two's complement form.
        Pos = self.getParam(self.REG_ABS_POS, 3)
        
        # Convert to int value.
        if Pos & (1 << 21):  # If sign bit is set:
            Pos = Pos - (1 << 22)  # Compute negative value.
                
        return Pos

    # MarkPosition command
    # Obtain the current marked position of the controller.
    # The mark position is a saved position to which the GoMark command can easily go.
    def markPosition(self):
        # Obtain position, in two's complement form.
        Mark = self.getParam(self.REG_MARK, 3)
        
        # Convert to int value.
        if Mark & (1 << 21):  # If sign bit is set:
            Mark = Mark - (1 << 22)  # Compute negative value.
                
        return Mark

    # CurrentSpeed command
    # Obtain the speed at which the stepper is currently moving, in step/s.
    def currentSpeed(self):
        # Obtain value in SPEED register in step/tick and convert to step/s.
        Speed = self.speedToStepSec(self.getParam(self.REG_SPEED, 3))
        
        return Speed

    # CurrentAcceleration command
    # Obtain the configured acceleration, in step/s^2.
    # This acceleration value is used for soft starts of the stepper.
    def currentAcceleration(self):
        # Obtain value in ACC register in step/tick^2 and convert to step/s^2.
        Acc = self.accToStepSec(self.getParam(self.REG_ACC, 2))
        
        return Acc

    # CurrentDeceleration command
    # Obtain the configured deceleration, in step/s^2.
    # This deceleration value is used for soft stops of the stepper.
    def currentDeceleration(self):
        # Obtain value in DEC register in step/tick^2 and convert to step/s^2.
        Dec = self.accToStepSec(self.getParam(self.REG_DEC, 2))
        
        return Dec

    # SetMinSpeed command
    # Set a new minimum speed for the stepper, in step/s.
    # The Optimization parameter set the low-speed optimization flag. If set to True, low-speed
    # optimization will be activated and the minimum speed value will be ignored by the controller.
    def setMinSpeed(self, MinSpeed, Optimized):
        if MinSpeed < 0:
            return
        
        MinSpeed = min(MinSpeed, 976)
        
        # Add the low speed optimization bit if specified.
        if Optimized == True:
            MinSpeed += 0x1000
        
        self.setParam(self.REG_MIN_SPEED, self.maxMinspeedToStepTick(MinSpeed), 2)

    # SetMaxSpeed command
    # Set a new maximum speed for the stepper, in step/s.
    def setMaxSpeed(self, MaxSpeed):
        if MaxSpeed < 0:
            return
        
        MaxSpeed = min(MaxSpeed, 15625)
        
        self.setParam(self.REG_MAX_SPEED, self.maxMinspeedToStepTick(MaxSpeed), 2)

    # SetThresholdSpeed command
    # Set a new threshold speed for the stepper, in step/s.
    # When the actual speed exceeds this value, the step mode is automatically switched to 
    # full-step two-phase on.
    def setThresholdSpeed(self, ThresholdSpeed):
        if ThresholdSpeed < 0:
            return
        
        # Limit value to 10 bits.
        ThresholdSpeed = min(ThresholdSpeed, 15625)

        self.setParam(self.REG_FS_SPD, self.maxMinspeedToStepTick(ThresholdSpeed), 2)


    # SetKvals
    # Range is 0.996*12V=11.952V with a resolution of 0.004*12V=0.048V
    # Reg range is KVAL_HOLD, KVAL_RUN, KVAL_ACC, KVAL_DEC
    # RegVal range is 0-255 
    def setKvalHold(self, RegVal):
        if RegVal > 0xFF:
            RegVal = 0x0  
        self.setParam(self.REG_KVAL_HOLD, RegVal, 1)

    def setKvalRun(self, RegVal):
        if RegVal > 0xFF:
            RegVal = 0x0  
        self.setParam(self.REG_KVAL_RUN, RegVal, 1)

    def setKvalAcc(self, RegVal):
        if RegVal > 0xFF:
            RegVal = 0x0  
        self.setParam(self.REG_KVAL_ACC, RegVal, 1)

    def setKvalDec(self, RegVal):
        if RegVal > 0xFF:
            RegVal = 0x0  
        self.setParam(self.REG_KVAL_DEC, RegVal, 1)

     
    #The INT_SPEED register contains the speed value at which the BEMF compensation curve
    # changes slope. The available range is from 0 to 976.5 step/s with a resolution of 0.0596 step/s.
    # 14 bit register.
    def setIntersectSpeed(self, RegVal):
        if RegVal > 0x3FFF:
            RegVal = 0x0 
        self.setParam(self.REG_INT_SPEED, RegVal, 2)

    #The ST_SLP register contains the BEMF compensation curve slope that is used when the
    #speed is lower than the intersect speed (see Section 7.4). Its value is expressed in s/step
    #and the available range is from 0 to 0.004 with a resolution of 0.000015.
    def setStartSlope(self, RegVal):
        if RegVal > 0xFF:
            RegVal = 0x0 
        self.setParam(self.REG_ST_SLP, RegVal, 1)


    #The FN_SLP_ACC register contains the BEMF compensation curve slope that is used when
    #the speed is greater than the intersect speed during acceleration (see Section 7.4). Its value
    #is expressed in s/step and the available range is from 0 to 0.004 with a resolution of
    #0.000015.
    def setAccFinalSlope(self, RegVal):
        if RegVal > 0xFF:
            RegVal = 0x0 
        self.setParam(self.REG_FN_SLP_ACC, RegVal, 1)


    #The FN_SLP_DEC register contains the BEMF compensation curve slope that is used when
    #the speed is greater than the intersect speed during deceleration (see Section 7.4). Its value
    #is expressed in s/step and the available range is from 0 to 0.004 with a resolution of
    #0.000015.
    def setDecFinalSlope(self, RegVal):
        if RegVal > 0xFF:
            RegVal = 0x0 
        self.setParam(self.REG_FN_SLP_DEC, RegVal, 1)


    #The K_THERM register contains the value used by the winding resistance thermal drift
    #compensation system (see Section 7.6 on page 37).
    #The available range is from 1 to 1.46875 with a resolution of 0.03125, as shown in Table 13.
    def setKThermComp(self, RegVal):
        if RegVal > 0xFF:
            RegVal = 0x0 
        self.setParam(self.REG_K_THERM, RegVal, 1)

    #The ADC_OUT register contains the result of the analog-to-digital conversion of the ADCIN
    #pin voltage; the result is available even if the supply voltage compensation is disabled.
    def getADCOut(self, RegVal):
        if RegVal > 0xFF:
            RegVal = 0x0 
        self.setParam(self.ADC_OUT, RegVal, 1)

    #The OCD_TH register contains the overcurrent threshold value (see Section 6.9 on page
    #29). The available range is from 375 mA to 6 A, in steps of 375 mA, as shown in Table 15.
    def setOCDThreshold(self, RegVal):
        if RegVal > 0xFF:
            RegVal = 0x0 
        self.setParam(self.REG_OCD_TH, RegVal, 1)

    #The STALL_TH register contains the stall detection threshold value (see Section 7.2 on
    #page 35). The available range is from 31.25 mA to 4 A with a resolution of 31.25 mA.
    def setStallThreshold(self, RegVal):
        if RegVal > 0xFF:
            RegVal = 0x0 
        self.setParam(self.REG_STALL_TH, RegVal, 1)


    # SetStepMode command
    # The step mode determines the movement unit of the stepper (microsteps).
    # See the STEP_SEL values for options.
    def setStepMode(self, StepMode):
        # Default to full step if invalid step mode.
        if StepMode > self.STEP_MODE_STEP_SEL_MASK or StepMode < 0:
            StepMode = self.STEP_SEL_FULL
                
        SyncEn = self.STEP_MODE_SYNC_EN_MASK & 0x00
        SyncSel = self.STEP_MODE_SYNC_SEL_MASK & self.SYNC_SEL_1  # Hardcoded to full FS.
        StepSel = self.STEP_MODE_STEP_SEL_MASK & StepMode
                
        self.setParam(self.REG_STEP_MODE, SyncEn | SyncSel | StepSel, 1)

    # SetAlarms command
    # Configure which alarms are active on the controller.
    # See the ALARM values for options.
    def setAlarms(self, Alarms):
        # Limit alarms to 8 bits
        Alarms = min(Alarms, 0xFF)
                
        self.setParam(self.REG_ALARM_EN, Alarms, 1)

    # SetConfig command
    # Not implemented for now.
    # See datasheet for default values.
    def setConfig(self):
        return

    # GetStatus command
    # Fetch the 16 bits value in the STATUS register.
    # This command does not reset the warning flags and error states.
    def getStatus(self):
        # Obtain status.
        return self.getParam(self.REG_STATUS, 2)
