from smbus2 import SMBus
import time, math, csv

# I2C address of the MPU-6050 (0x68 or 0x69 depending on the AD0 pin)
IMU_ADDR = 0x68

# Initialize the I2C bus (use 1 for /dev/i2c-1)
I2C_BUS_NUMBER = 1
bus = SMBus(I2C_BUS_NUMBER)

aX_cal = 0
aY_cal = 0
aZ_cal = 0
gX_cal = 0
gY_cal = 0
gZ_cal = 0

g = 9.8
deg2rad = math.pi / 180.0
calibrated = False

CALIBRATION_DEPTH = 200
CALIBRATION_INTERVAL_MS = 0.01

q = [1.0, 0, 0, 0]
# Free parameters in the Mahony filter and fusion scheme,
# Kp for proportional feedback, Ki for integral
Kp = 30.0
Ki = 1.0

# https://en.wikipedia.org/wiki/Covariance_matrix
# https://www.cuemath.com/algebra/covariance-matrix/
# represents the joint variability (ie randomness) of two variables, or the
# approximate strength of the linear relationship between the measurements
# C(i,j) = 0, i & j have no relationship
# diagonal C(i,i) represents measurement i's own uncertainty

# velocity in x, y, z, angular velocity about X axis, Y axis, Z axis
#                   vx  vy vz wx wy wz
twistCovariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, # vx
                   0.0, 0.05, 0.0, 0.0, 0.0, 0.0, # vy
                   0.0, 0.0, 0.05, 0.0, 0.0, 0.0, # vz
                   0.0, 0.0, 0.0, 0.05, 0.0, 0.0, # wx
                   0.0, 0.0, 0.0, 0.0, 0.05, 0.0, # wy
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.05] # wz

# x, y, z, rotation about X axis, Y axis, Z axis
#                  x   y  z  rl  pt yw
poseCovariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, # x
                  0.0, 0.05, 0.0, 0.0, 0.0, 0.0, # y
                  0.0, 0.0, 0.05, 0.0, 0.0, 0.0, # z
                  0.0, 0.0, 0.0, 0.05, 0.0, 0.0, # roll
                  0.0, 0.0, 0.0, 0.0, 0.05, 0.0, # pitch
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.05] # yaw

# reading raw data ------------------------------------------------
def read_word_2c(register):
    high_byte = bus.read_byte_data(IMU_ADDR, register)
    low_byte = bus.read_byte_data(IMU_ADDR, register + 1)
    value = (high_byte << 8) + low_byte
    if value >= 0x8000:
        return -((0xFFFF - value) + 1)
    else:
        return value

def read_raw_data():
    accelerometer_x = read_word_2c(0x3B)
    accelerometer_y = read_word_2c(0x3D)
    accelerometer_z = read_word_2c(0x3F)
    temperature = read_word_2c(0x41)
    gyro_x = read_word_2c(0x43)
    gyro_y = read_word_2c(0x45)
    gyro_z = read_word_2c(0x47)
    return accelerometer_x, accelerometer_y, accelerometer_z, temperature, gyro_x, gyro_y, gyro_z

def read_raw_stream():
    bus.write_byte_data(IMU_ADDR, 0x6B, 0)  # Wake up the MPU-6050

    while True:
        accelerometer_x, accelerometer_y, accelerometer_z, temperature, gyro_x, gyro_y, gyro_z = read_raw_data()
        print("aX = {} | aY = {} | aZ = {} | tmp = {:.2f} | gX = {} | gY = {} | gZ = {}".format(
            int162str( accelerometer_x ),
            int162str( accelerometer_y ),
            int162str( accelerometer_z ),
            temp2cel( temperature ),
            int162str( gyro_x ),
            int162str( gyro_y ),
            int162str( gyro_z )
        ))

        time.sleep(1)

def int162str(i):
    return "{:6d}".format(i)



# reading calibrated, useful data ------------------------------
def get_SI_data():
    raw_aX, raw_aY, raw_aZ, temp, raw_gX, raw_gY, raw_gZ = read_raw_data()

    aX = g*(acc2g(raw_aX))
    aY = g*(acc2g(raw_aY))
    aZ = g*(acc2g(raw_aZ))

    tempC = temp2cel(temp)

    gX = gyro2SI(raw_gX)
    gY = gyro2SI(raw_gY)
    gZ = gyro2SI(raw_gZ)

    return aX, aY, aZ, tempC, gX, gY, gZ

def get_calibrated_data():
    if not calibrated:
        calibrate_imu()

    aX_SI, aY_SI, aZ_SI, tempC, gX_SI, gY_SI, gZ_SI = get_SI_data()

    aX = aX_SI + aX_cal
    aY = aY_SI + aY_cal
    aZ = aZ_SI + aZ_cal

    gX = gX_SI + gX_cal
    gY = gY_SI + gY_cal
    gZ = gZ_SI + gZ_cal


    return aX, aY, aZ, tempC, gX, gY, gZ

def read_calibrated_stream():
    while True:
        aX, aY, aZ, tempC, gX, gY, gZ = get_calibrated_data()
        print(f"A:{{x={aX:3.2f} | y={aY:3.2f} | z={aZ:3.2f}}} "
              f"G:{{x={gX:3.2f} | y={gY:3.2f} | z={gZ:3.2f}}}"
        )
        time.sleep(0.5)

def calibrate_imu():
    print('Calibrating stationary')
    global aX_cal, aY_cal, aZ_cal, gX_cal, gY_cal, gZ_cal, calibrated

    aX_cal = 0
    aY_cal = 0
    aZ_cal = 0

    gX_cal = 0
    gY_cal = 0
    gZ_cal = 0

    for i in range(CALIBRATION_DEPTH):
        aX, aY, aZ, tempC, gX, gY, gZ = get_SI_data()
        aX_cal += aX
        aY_cal += aY
        aZ_cal += aZ

        gX_cal += gX
        gY_cal += gY
        gZ_cal += gZ

        time.sleep(CALIBRATION_INTERVAL_MS)

    aX_cal /= -CALIBRATION_DEPTH
    aY_cal /= -CALIBRATION_DEPTH
    aZ_cal /= -CALIBRATION_DEPTH

    aX_cal = 0 + aX_cal
    aY_cal = 0 + aY_cal
    aZ_cal = g + aZ_cal

    gX_cal /= -CALIBRATION_DEPTH
    gY_cal /= -CALIBRATION_DEPTH
    gZ_cal /= -CALIBRATION_DEPTH

    aX_cal = 0 + aX_cal
    gY_cal = 0 + gY_cal
    gZ_cal = 0 + gZ_cal

    print(f"A_cal {{dX:{aX_cal:3.2f} dY{aY_cal:3.2f} dZ:{aZ_cal:3.2f}}}")
    print(f"G_cal {{dX:{gX_cal:3.2f} dY{gY_cal:3.2f} dZ:{gZ_cal:3.2f}}}")
    calibrated = True

def acc2g(acc):
    return acc / 0x4000 #2^14

def temp2cel(temperature):
    return temperature / 340.0 + 36.53

def gyro2SI(gyro):
    return deg2rad*gyro / 131.0



# roll pitch yaw calculations -----------------------------------
def mahony_update(ax, ay, az, gx, gy, gz, deltat):
    global q
    recipNorm = 0
    ix = 0.0
    iy = 0.0
    iz = 0.0  #integral feedback terms

    # Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    tmp = ax * ax + ay * ay + az * az
    if (tmp > 0.0):
        # Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recipNorm = 1.0 / math.sqrt(tmp)
        ax *= recipNorm
        ay *= recipNorm
        az *= recipNorm

        # Estimated direction of gravity in the body frame (factor of two divided out)
        vx = q[1] * q[3] - q[0] * q[2]
        vy = q[0] * q[1] + q[2] * q[3]
        vz = q[0] * q[0] - 0.5 + q[3] * q[3]

        # Error is cross product between estimated and measured direction of gravity in body frame
        # (half the actual magnitude)
        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)

        # Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0):
            ix += Ki * ex * deltat  # integral error scaled by Ki
            iy += Ki * ey * deltat
            iz += Ki * ez * deltat
            gx += ix  # apply integral feedback
            gy += iy
            gz += iz

        # Apply proportional feedback to gyro term
        gx += Kp * ex
        gy += Kp * ey
        gz += Kp * ez


    # Integrate rate of change of quaternion, q cross gyro term
    deltat = 0.5 * deltat
    gx *= deltat   # pre-multiply common factors
    gy *= deltat
    gz *= deltat
    qa = q[0]
    qb = q[1]
    qc = q[2]
    q[0] += (-qb * gx - qc * gy - q[3] * gz)
    q[1] += (qa * gx + qc * gz - q[3] * gy)
    q[2] += (qa * gy - qb * gz + q[3] * gx)
    q[3] += (qa * gz + qb * gy - qc * gx)

    # renormalise quaternion
    recipNorm = 1.0 / math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    q[0] = q[0] * recipNorm
    q[1] = q[1] * recipNorm
    q[2] = q[2] * recipNorm
    q[3] = q[3] * recipNorm
    return q

def get_rpy(deg=False):
    roll  = math.atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]))
    pitch = math.asin(2.0 * (q[0] * q[2] - q[1] * q[3]))
    #conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
    yaw   = -math.atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]))
    if (yaw < 0): yaw += 2*math.pi #compass circle

    if (deg):
        yaw   /= deg2rad
        pitch /= deg2rad
        roll  /= deg2rad
    return roll, pitch, yaw

def read_rpy_stream(verbose=False, write2csv=False):
    bus.write_byte_data(IMU_ADDR, 0x6B, 0)  # Wake up the MPU-6050
    writer = None

    if write2csv:
        f = open('csv_data.csv', 'w')
        writer = csv.writer(f)
    
    t = time.time()
    while True:
        aX, aY, aZ, tempC, gX, gY, gZ = get_calibrated_data()
        dt = (time.time() - t)
        t = time.time()

        mahony_update(aX, aY, aZ, gX, gY, gZ, dt)
        roll, pitch, yaw = get_rpy()

        if verbose: print(f"{{roll:{roll:3.2f} | pitch:{pitch:3.2f} | yaw:{yaw:3.2f}}}")
        if write2csv: writer.writerow([roll, pitch, yaw, dt, q[0], q[1], q[2], q[3]])
        time.sleep(0.01)

if __name__ == "__main__":
    try:
        calibrate_imu()
        read_rpy_stream(verbose=True, write2csv=False)
    except KeyboardInterrupt:
        print(" Exiting...")
    finally:
        bus.close()
