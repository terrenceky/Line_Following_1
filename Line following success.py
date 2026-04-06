import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import time
import csv

# --- MOTOR SETUP ---
GPIO.setmode(GPIO.BCM)
ENA, IN1, IN2 = 12, 23, 24
ENB, IN3, IN4 = 13, 17, 27

GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)
pwmA = GPIO.PWM(ENA, 1000); pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(0); pwmB.start(0)

# --- GLOBAL PID VARIABLES ---
last_error = 0

# --- CAMERA SETUP ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (160, 120)})
picam2.configure(config)
picam2.start()


def get_line_error():
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    roi = gray[75:115, 0:160]  
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    
    # --- SHADOW FILTERING LOGIC ---
    # 1. Use Otsu to find the ideal split
    ret, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    # 2. SHADOW GUARD: If the calculated Otsu threshold (ret) is too high, 
    # it means the "darkest" thing it found isn't actually black (like a line),
    # but just a shadow. We force the screen to black if the contrast is too low.
    # Typically, a black line on a light floor has a threshold value < 100.
    if ret > 158: 
        thresh = np.zeros_like(thresh) # Force total black
    
    # Clean up noise (smaller kernel is better for thin lines)
    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.erode(thresh, kernel, iterations=1)
    thresh = cv2.dilate(thresh, kernel, iterations=1) 
    
    debug_view = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    cam_center = 80 

    M = cv2.moments(thresh)
    pixel_count = M['m00'] / 255  

    # Display Pixel Count and the Threshold Value for tuning
    cv2.putText(debug_view, f"P: {int(pixel_count)} T: {int(ret)}", (5, 15), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    cv2.line(debug_view, (cam_center, 0), (cam_center, 40), (0, 0, 255), 1)

    error = None
    
    # Threshold for a real line - adjust based on your P value
    # If the line is missing, pixel_count will now be 0
    if pixel_count > 10000:
        error = "FINISH"
    elif 200 < pixel_count < 2500: 
        cx = int(M['m10'] / M['m00'])
        cv2.circle(debug_view, (cx, 20), 5, (0, 255, 0), -1) 
        error = cx - cam_center
    else:
        error = None
    
    cv2.imshow("Camera View (Cleaned)", debug_view)
    return error, pixel_count




def move_robot(error, pixel_count, searching=False):
    global last_error
    
    Kp, Kd = 7.5, 3.8
    BASE_SPEED = 40
    PIVOT_SPEED = 70
    PIVOT_THRESHOLD = 35
    LOW_PIXEL_COUNT = 500 
    
    if error is not None and error != "FINISH":
        side_memory = "RIGHT" if error > 0 else "LEFT"
    else:
        side_memory = "RIGHT" if last_error > 0 else "LEFT"

    is_sharp_turn = False
    # Trigger pivot if searching (error is None) or if pixel count is outside valid range
    if searching or pixel_count < LOW_PIXEL_COUNT or (error is not None and error != "FINISH" and abs(error) > PIVOT_THRESHOLD):
        is_sharp_turn = True

    if is_sharp_turn:
        l_pwr = PIVOT_SPEED if side_memory == "RIGHT" else -PIVOT_SPEED
        r_pwr = -PIVOT_SPEED if side_memory == "RIGHT" else PIVOT_SPEED
            
        GPIO.output(IN1, GPIO.LOW if l_pwr > 0 else GPIO.HIGH)
        GPIO.output(IN2, GPIO.HIGH if l_pwr > 0 else GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW if r_pwr > 0 else GPIO.HIGH)
        GPIO.output(IN4, GPIO.HIGH if r_pwr > 0 else GPIO.LOW)
    else:
        derivative = error - last_error
        steering = (error * Kp) + (derivative * Kd)
        last_error = error
        
        l_pwr = BASE_SPEED + steering
        r_pwr = BASE_SPEED - steering
        
        GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.HIGH)
        
        l_pwr = max(0, min(100, l_pwr))
        r_pwr = max(0, min(100, r_pwr))

    pwmA.ChangeDutyCycle(abs(l_pwr))
    pwmB.ChangeDutyCycle(abs(r_pwr))
    
    return l_pwr, r_pwr

def stop_motors():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwmA.ChangeDutyCycle(0); pwmB.ChangeDutyCycle(0)

# --- PRE-START SETUP ---
log_file = open("robot_tuning_log.csv", mode="w", newline="")
log_writer = csv.writer(log_file)
log_writer.writerow(["Time", "Error", "L_Motor", "R_Motor"])

print("Systems Online.")
input(">>> Place robot on track and press ENTER to start! <<<")

# --- MAIN LOOP ---
try:
    start_time = time.time()
    while True:
        result, p_count = get_line_error()
        status_img = np.zeros((180, 400, 3), dtype="uint8")

        if result == "FINISH":
            print("Finish line detected!")
            stop_motors()
            break 
        
        elif result is not None:
            l_pwr, r_pwr = move_robot(result, p_count)
            msg, clr = f"Error: {result}", (0, 255, 0)
        
        else:
            # Force search logic when result is None (shadows or line lost)
            l_pwr, r_pwr = move_robot(None, 0, searching=True)
            msg, clr = "LOST: SEARCHING...", (0, 165, 255)

        # --- DASHBOARD LOGIC ---
        cv2.putText(status_img, msg, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, clr, 2)
        cv2.putText(status_img, f"Pixels: {int(p_count)}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(status_img, f"L: {l_pwr:.1f}% R: {r_pwr:.1f}%", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow("Robot Dashboard", status_img)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    log_file.close()
    stop_motors()
    pwmA.stop(); pwmB.stop()
    picam2.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
