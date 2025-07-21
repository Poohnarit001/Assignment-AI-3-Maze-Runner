"""my_controller_wall_follower with odometry and CSV logging"""

from controller import Robot
import math
import csv

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    wheel_radius = 0.0205  # meters
    wheel_distance = 0.053  # meters between left and right wheels

    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    left_position_sensor = robot.getPositionSensor('left wheel sensor')
    right_position_sensor = robot.getPositionSensor('right wheel sensor')
    left_position_sensor.enable(timestep)
    right_position_sensor.enable(timestep)

    prox_sensors = []
    for ind in range(8):
        sensor = robot.getDistanceSensor(f'ps{ind}')
        sensor.enable(timestep)
        prox_sensors.append(sensor)

    x, y, theta = 0.0, 0.0, 0.0
    prev_left = 0.0
    prev_right = 0.0

    # เปิดไฟล์ CSV สำหรับเขียน (เขียนหัวก่อน)
    with open('odometry_log.csv', mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['time_ms', 'x_m', 'y_m', 'theta_rad'])

        time_ms = 0
        while robot.step(timestep) != -1:
            left_wall = prox_sensors[5].getValue() > 80
            left_corner = prox_sensors[6].getValue() > 80
            front_wall = prox_sensors[7].getValue() > 80

            left_speed = max_speed
            right_speed = max_speed

            if front_wall:
                left_speed = max_speed
                right_speed = -max_speed
            else:
                if left_wall:
                    left_speed = max_speed
                    right_speed = max_speed
                else:
                    left_speed = max_speed / 8
                    right_speed = max_speed

            if left_corner:
                left_speed = max_speed
                right_speed = max_speed / 8

            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

            left_pos = left_position_sensor.getValue()
            right_pos = right_position_sensor.getValue()

            d_left = (left_pos - prev_left) * wheel_radius
            d_right = (right_pos - prev_right) * wheel_radius
            prev_left = left_pos
            prev_right = right_pos

            d_center = (d_left + d_right) / 2.0
            delta_theta = (d_right - d_left) / wheel_distance

            theta += delta_theta
            x += d_center * math.cos(theta)
            y += d_center * math.sin(theta)

            # บันทึกข้อมูลลง CSV
            csv_writer.writerow([time_ms, x, y, theta])

            print(f"Time: {time_ms} ms | Position -> x: {x:.3f}, y: {y:.3f}, theta: {math.degrees(theta)%360:.2f} deg")

            time_ms += timestep

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
