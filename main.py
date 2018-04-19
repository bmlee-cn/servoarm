from pid import PID
import time,sensor,image,pyb
from servoarm import Servoarm

#define public vars
actions = {}
cx_array = []
cy_array = []
h_array = []
cx_pid = PID(p=0.6, i=0.2, d=0.03, imax=200)
cy_pid = PID(p=0.3, i=0.01, d=0.1, imax=100)
h_pid = PID(p=3,i=0.5, imax=50)
h_threshold = 25
face_mode = RGB_mode = False
filter_stages = 6
threshold = [50, 50, 0, 0, 0, 0] # Middle L, A, B values.
face_cascade = image.HaarCascade("frontalface", stages=25)



#Define internal functions
def recursion_filter(array,weight=1.0):
    foo=0
    del array[0]
    if weight < 1:
        foo=int((sum(array[:-1])/(len(array)-1)*weight+array[-1]*(1-weight)))
    else:
        foo=int(sum(array)/len(array))
    return foo

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

def face_detect():
    print("Detecting face...")
    sensor.reset()
    sensor.set_contrast(1)
    sensor.set_gainceiling(16)
    sensor.set_framesize(sensor.HQVGA)
    sensor.set_pixformat(sensor.GRAYSCALE)

    clock = time.clock()

    LED.on()
    for i in range(60):
        img = sensor.snapshot()
        objects = img.find_features(face_cascade, threshold=0.75, scale_factor=1.25)

        # Draw objects
        if objects:
            for r in objects:
                img.draw_rectangle(r)
                cx=r[0]+r[2]//2
                cy=r[1]+r[3]//2
                img.draw_cross(cx,cy)
            obj = find_max(objects)
            cx_array.append(obj[0]+obj[2]//2)
            cy_array.append(obj[1]+obj[3]//2)
            h_array.append(obj[2])
            h_threshold = obj[2]
            if len(cx_array) == filter_stages:
                LED.off()
                print('Face detected.\nStart tracking...')
                return True
    LED.off()
    print('No face detected.\nTrying color mode...')
    return False

def color_detect():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(60)
    sensor.set_auto_gain(False) # must be turned off for color tracking
    sensor.set_auto_whitebal(False) # must be turned off for color tracking

    # Capture the color thresholds for whatever was in the center of the image.
    r = [(320//2)-(50//2), (240//2)-(50//2), 50, 50] # 50x50 center of QVGA.

    print("Auto algorithms done. Hold the object you want to track in front of the camera in the box.")
    print("MAKE SURE THE COLOR OF THE OBJECT YOU WANT TO TRACK IS FULLY ENCLOSED BY THE BOX!")
    for i in range(60):
        if (i % 5 == 0) : LED.toggle()
        img = sensor.snapshot()
        img.draw_rectangle(r)

    print("Learning thresholds...")
    for i in range(60):
        img = sensor.snapshot()
        hist = img.get_histogram(roi=r)
        lo = hist.get_percentile(0.01) # Get the CDF of the histogram at the 1% range (ADJUST AS NECESSARY)!
        hi = hist.get_percentile(0.99) # Get the CDF of the histogram at the 99% range (ADJUST AS NECESSARY)!
        # Average in percentile values.
        threshold[0] = (threshold[0] + lo.l_value()) // 2
        threshold[1] = (threshold[1] + hi.l_value()) // 2
        threshold[2] = (threshold[2] + lo.a_value()) // 2
        threshold[3] = (threshold[3] + hi.a_value()) // 2
        threshold[4] = (threshold[4] + lo.b_value()) // 2
        threshold[5] = (threshold[5] + hi.b_value()) // 2
        img.draw_rectangle(r)

        blobs = img.find_blobs([threshold], pixels_threshold=100, area_threshold=100, merge=True, margin=10)
        if blobs:
            for blob in blobs:
                img.draw_rectangle(blob.rect())
                img.draw_cross(blob.cx(), blob.cy())
            blob = find_max(blobs)
            cx_array.append(blob.cx())
            cy_array.append(blob.cy())
            h_array.append(blob.w())
            h_threshold= blob.w()
            if len(cx_array) == filter_stages:
                print("Thresholds learned...")
                print("Tracking colors...")
                return True

    print("Failed to learned thresholds...")
    print("Abrading...")
    return False


myarm = Servoarm()
clock = time.clock()
LED = pyb.LED(3)

myarm.init()
time.sleep(500)

face_mode = face_detect() #Try to start with face tracking first, return val is FaceCascade object
if not face_mode: RGB_mode=color_detect() #Return val is detected color threshold

print(face_mode, RGB_mode)

while face_mode or RGB_mode:
    clock.tick()
    roi = None
    img = sensor.snapshot()
    if face_mode:
        objects = img.find_features(face_cascade, threshold=0.75, scale_factor=1.25)
        if objects: roi = find_max(objects)
    else:
        blobs = img.find_blobs([threshold], pixels_threshold=250, area_threshold=250, merge=True, margin=10)
        if blobs:
            blob = find_max(blobs)
            roi = blob[0:4]
    if roi is not None:
        _cx_=roi[0]+roi[2]//2
        _cy_=roi[1]+roi[3]//2
        img.draw_rectangle(roi)
        img.draw_cross(_cx_,_cy_)
        #添加滤波列表
        cx_array.append(_cx_)
        cy_array.append(_cy_)
        h_array.append(roi[2])
        #调用滤波函数,可加权重
        cx_ = recursion_filter(cx_array,0.2)
        cy_ = recursion_filter(cy_array,0.2)
        h_area = recursion_filter(h_array)
        #获取PID输出
        x_error = cx_ - img.width()//2
        y_error = cy_ - img.height()//2
        h_error = h_area - h_threshold
        x_out = cx_pid.get_pid(x_error,-1)
        y_out = cy_pid.get_pid(y_error,1)
        h_out = h_pid.get_pid(h_error,1)
        #获取动作字典
        actions=myarm.action(x_out,y_err=y_out,h_err=h_out)
        print(roi[2],h_threshold,h_out)
        #执行动作
        myarm.execute(actions)
        h_threshold = h_area
