from controller import Robot, Camera, DistanceSensor

robot = Robot()
time_step = int(robot.getBasicTimeStep())


camera = robot.getDevice("camera")
camera.enable(time_step)


proximity_sensors = []
for i in range(8):
    ps = robot.getDevice(f"ps{i}")
    ps.enable(time_step)
    proximity_sensors.append(ps)


left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

WANDER_SPEED = 4.0                 
TARGET_REACHED_THRESHOLD = 150     
MIN_SPEED = 0.5                    
KP = 0.01                          
last_error = 0.0                   

def rgb_to_hsv(r, g, b):
    
    r_norm, g_norm, b_norm = r/255.0, g/255.0, b/255.0
    cmax = max(r_norm, g_norm, b_norm)
    cmin = min(r_norm, g_norm, b_norm)
    delta = cmax - cmin

    if delta == 0:
        h = 0
    elif cmax == r_norm:
        h = (60 * (((g_norm - b_norm) / delta) % 6))
    elif cmax == g_norm:
        h = (60 * (((b_norm - r_norm) / delta) + 2))
    else:
        h = (60 * (((r_norm - g_norm) / delta) + 4))
    
    s = 0 if cmax == 0 else delta / cmax
    v = cmax
    return h, s, v

def detect_color():
    
    image = camera.getImage()
    if image is None:
        return None

    width = camera.getWidth()
    height = camera.getHeight()
    
    red_sum, green_sum, blue_sum = 0, 0, 0
    pixel_count = 0

    
    for x in range(width // 3, 2 * width // 3, 5):
        for y in range(height // 3, 2 * height // 3, 5):
            red_sum += camera.imageGetRed(image, width, x, y)
            green_sum += camera.imageGetGreen(image, width, x, y)
            blue_sum += camera.imageGetBlue(image, width, x, y)
            pixel_count += 1

    avg_red = red_sum / pixel_count
    avg_green = green_sum / pixel_count
    avg_blue = blue_sum / pixel_count

    h, s, v = rgb_to_hsv(avg_red, avg_green, avg_blue)
    if s < 0.3:  
        return None

    
    if (h < 20 or h > 340):
        return "red"
    elif 70 <= h <= 150:
        return "green"
    elif 200 <= h <= 260:
        return "blue"
    else:
        return None

def get_target_offset(target_color):
   
    image = camera.getImage()
    if image is None:
        return None

    width = camera.getWidth()
    height = camera.getHeight()
    center_x = width / 2.0
    matching_pixels = []
    step = 5 
    for x in range(0, width, step):
        for y in range(0, height, step):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)
            h, s, v = rgb_to_hsv(r, g, b)
            if s < 0.2:  
                continue
            if target_color == "red":
                if not (h < 20 or h > 340):
                    continue
            elif target_color == "green":
                if not (70 <= h <= 150):
                    continue
            elif target_color == "blue":
                if not (200 <= h <= 260):
                    continue
            matching_pixels.append(x)
    
    if not matching_pixels:
        return None
    
    average_x = sum(matching_pixels) / len(matching_pixels)
    error = center_x - average_x  

def approach_and_lock(target_color):
 
    global last_error

    front_value = max(proximity_sensors[0].getValue(), proximity_sensors[7].getValue())
    
    if front_value >= TARGET_REACHED_THRESHOLD:
        base_speed = MIN_SPEED
    else:
        base_speed = WANDER_SPEED - (WANDER_SPEED - MIN_SPEED) * (front_value / TARGET_REACHED_THRESHOLD)
    
    error = get_target_offset(target_color)
    if error is None:
        error = last_error  
    last_error = error

    turn_adjustment = KP * error
    left_speed = base_speed - turn_adjustment
    right_speed = base_speed + turn_adjustment

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    

    print("Target:", target_color, "Error:", error, "L_speed:", left_speed, "R_speed:", right_speed)
    
    return base_speed, front_value, error

def avoid_obstacles():

    psValues = [sensor.getValue() for sensor in proximity_sensors]
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0
    left_obstacle = psValues[6] > 80.0 or psValues[7] > 80.0

    if left_obstacle:
        left_motor.setVelocity(0.6 * WANDER_SPEED)
        right_motor.setVelocity(-0.4 * WANDER_SPEED)
    elif right_obstacle:
        left_motor.setVelocity(-0.4 * WANDER_SPEED)
        right_motor.setVelocity(0.6 * WANDER_SPEED)
    else:
        left_motor.setVelocity(0.5 * WANDER_SPEED)
        right_motor.setVelocity(0.5 * WANDER_SPEED)


state = "SEARCH_RED"

while robot.step(time_step) != -1:
    color_seen = detect_color()
    
   
    if state == "SEARCH_RED":
        if color_seen == "red":
            print("Red seen; switching to approach mode.")
            state = "APPROACH_RED"
        else:
            avoid_obstacles()
    
  
    elif state == "APPROACH_RED":
        if color_seen == "red":
            base_speed, front_val, error = approach_and_lock("red")
            if front_val >= TARGET_REACHED_THRESHOLD:
                print("The trash is found.")
                state = "SEARCH_GREEN"
        else:
            
            left_motor.setVelocity(WANDER_SPEED/2)
            right_motor.setVelocity(WANDER_SPEED/2)
            state = "SEARCH_RED"
    

    elif state == "SEARCH_GREEN":
        if color_seen == "green":
            print("Green seen; switching to approach mode.")
            state = "APPROACH_GREEN"
        else:
            avoid_obstacles()
    
    
    elif state == "APPROACH_GREEN":
        if color_seen == "green":
            base_speed, front_val, error = approach_and_lock("green")
            if front_val >= TARGET_REACHED_THRESHOLD:
                print("The trashcan is found.")
                state = "SEARCH_HOME"
        else:
            left_motor.setVelocity(WANDER_SPEED/2)
            right_motor.setVelocity(WANDER_SPEED/2)
            state = "SEARCH_GREEN"
    
   
    elif state == "SEARCH_HOME":
        if color_seen == "blue":
            print("Blue seen; switching to approach mode.")
            state = "APPROACH_HOME"
        else:
            avoid_obstacles()
    
    
    elif state == "APPROACH_HOME":
        if color_seen == "blue":
            base_speed, front_val, error = approach_and_lock("blue")
            if front_val >= TARGET_REACHED_THRESHOLD:
                print("The trashcan task is completed successfully.")
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                break 
        else:
            left_motor.setVelocity(WANDER_SPEED/2)
            right_motor.setVelocity(WANDER_SPEED/2)
            state = "SEARCH_HOME"