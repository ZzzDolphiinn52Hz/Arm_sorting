#include <webots/camera.h>
#include <webots/camera_recognition_object.h>

from controller import Supervisor

TIME_STEP = 32
robot = Supervisor()

# =========================
# CAMERA SETUP
# =========================

CAMERA_NAME = "camera"   

camera = robot.getDevice(CAMERA_NAME)
camera.enable(TIME_STEP)

camera.recognitionEnable(TIME_STEP)

print("Camera enabled:", CAMERA_NAME)
print("Camera width:", camera.getWidth())
print("Camera height:", camera.getHeight())

def detect_cube():
        objects = camera.getRecognitionObjects()
        if len(objects) == 0:
            return None
    
        # Return the first detected object (or filter by model/color)
        obj = objects[0]
        pos_on_image = obj.getPositionOnImage()   # pixel x, y
        size_on_image = obj.getSizeOnImage()       # pixel w, h
        position_3d  = obj.getPosition()          # x, y, z relative to robot
        model_name   = obj.getModel()
    
        return {
            "cx": pos_on_image[0],
            "cy": pos_on_image[1],
            "area": size_on_image[0] * size_on_image[1],
            "position_3d": position_3d,
            "model": model_name
        }

    
def wait_until_cube_ready(max_steps=500):
    print("Tracking and following the cube along the conveyor...")
    
    image_center_x = camera.getWidth() / 2
    image_center_y = camera.getHeight() / 2 # Lấy tâm theo chiều dọc (720 / 2 = 360)
    
    PICK_ZONE_X = 15  
    PICK_ZONE_Y = 250  
    MIN_CUBE_AREA = 150
    
    Kp = -0.0015  

    for i in range(max_steps):
        result = detect_cube()
        
        if result is not None:
            cx, cy, area = result["cx"], result["cy"], result["area"]
            error_x = cx - image_center_x
            error_y = abs(cy - image_center_y)
            
            # 1. Lấy cấu hình khớp hiện tại của robot
            current_q = get_current_joint_positions()
            
            # 2. Điều chỉnh khớp xoay gốc (Base Joint) bám theo tọa độ X của vật
            current_q[0] += Kp * error_x
            move_ur_to(current_q)
            
            # IN THÊM CY VÀ ERROR_Y ĐỂ BẠN DỄ GIÁM SÁT QUA CONSOLE
            if i % 10 == 0:
                print(f"Tracking -> cx: {round(cx,1)}, cy: {round(cy,1)}, error_y: {round(error_y,1)}, Base Angle: {round(current_q[0],3)}")
            
            # 3. Kiểm tra điều kiện khóa mục tiêu
            if abs(error_x) < PICK_ZONE_X and error_y < PICK_ZONE_Y and area > MIN_CUBE_AREA:
                print(f"-> TARGET LOCKED! Khóa mục tiêu tại góc Base: {round(current_q[0],3)}. Tiến hành gắp!")
                return current_q[0]  # Trả về góc chính xác để hạ kẹp xuống gắp
        else:
            if i % 20 == 0:
                print("Searching for cube...")
        
        if robot.step(TIME_STEP) == -1:
            return None
            
    print("Timeout: Cube moved past or was missed.")
    return None
    
def preview_and_detect_cube(n_steps=300):
    print("Preview and detect white cube...")

    for i in range(n_steps):
        result, max_brightness, brightest_pixel = detect_white_cube()

        if result is not None:
            cx, cy, area = result

            if i % 10 == 0:
                print(
                    "White cube detected:",
                    "cx =", round(cx, 1),
                    "cy =", round(cy, 1),
                    "area =", area,
                    "max_brightness =", round(max_brightness, 1),
                    "brightest_rgb =", brightest_pixel
                )
        else:
            if i % 10 == 0:
                print(
                    "No white cube detected",
                    "max_brightness =", round(max_brightness, 1),
                    "brightest_rgb =", brightest_pixel
                )

        if robot.step(TIME_STEP) == -1:
            return False

    return True
    
UR_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

GRIPPER_NAMES = [
    "finger_1_joint_1",
    "finger_2_joint_1",
    "finger_middle_joint_1"
]

ur_motors = []
ur_sensors = []

for name in UR_JOINT_NAMES:
    motor = robot.getDevice(name)
    sensor = robot.getDevice(name + "_sensor")
    motor.setVelocity(0.8)
    sensor.enable(TIME_STEP)
    ur_motors.append(motor)
    ur_sensors.append(sensor)

gripper_motors = []

for name in GRIPPER_NAMES:
    motor = robot.getDevice(name)
    motor.setVelocity(0.5)
    gripper_motors.append(motor)


def move_ur_to(q):
    for motor, pos in zip(ur_motors, q):
        motor.setPosition(pos)


def wait_steps(n):
    for _ in range(n):
        if robot.step(TIME_STEP) == -1:
            return False
    return True


def open_gripper():
    for motor in gripper_motors:
        motor.setPosition(0.05)


def close_gripper():
    for motor in gripper_motors:
        motor.setPosition(0.8)
        
def get_current_joint_positions():
    return [sensor.getValue() for sensor in ur_sensors]


def smooth_move_ur_to(target_q, steps=100):
    start_q = get_current_joint_positions()

    for i in range(steps + 1):
        ratio = i / steps

        q = []
        for start, target in zip(start_q, target_q):
            value = start + ratio * (target - start)
            q.append(value)

        move_ur_to(q)

        if robot.step(TIME_STEP) == -1:
            return False

    return True
    
def wait_until_reached(target_q, tolerance=0.02, max_steps=300):
    for _ in range(max_steps):
        current_q = [sensor.getValue() for sensor in ur_sensors]

        errors = []
        for current, target in zip(current_q, target_q):
            errors.append(abs(current - target))

        if max(errors) < tolerance:
            return True

        if robot.step(TIME_STEP) == -1:
            return False

    return False
    
def dynamic_descend_and_pick(steps=25):
    print("Đang lao xuống gắp và liên tục điều chỉnh góc theo vật...")
    start_q = get_current_joint_positions()
    target_q = list(PICK_DOWN)
    image_center_x = camera.getWidth() / 2
    Kp = -0.0015  # Sử dụng cùng hệ số P-control với hàm đợi [cite: 41]
    
    for i in range(steps + 1):
        ratio = i / steps
        current_q = get_current_joint_positions()
        
        next_q = []
        # Lấy góc khớp gốc thực tế hiện tại làm điểm tựa để sửa sai số
        base_angle = current_q[0]
        
        # Đọc dữ liệu camera liên tục NGAY TRONG LÚC ĐANG HẠ TAY
        result = detect_cube()
        if result is not None:
            cx = result["cx"]
            error_x = cx - image_center_x
            base_angle += Kp * error_x  # Bù trừ góc xoay thời gian thực [cite: 43]
            
        next_q.append(base_angle)
        
        # Nội suy mượt mà các khớp nâng/hạ và cổ tay (khớp 1 đến 5) [cite: 11]
        for j in range(1, 6):
            val = start_q[j] + ratio * (target_q[j] - start_q[j])
            next_q.append(val)
            
        move_ur_to(next_q)
        if robot.step(TIME_STEP) == -1:
            return False
            
    return True
    
def check_if_picked_successfully():
    # Chờ một vài bước để cánh tay nâng lên ổn định hình ảnh
    wait_steps(10) 
    
    result = detect_cube()
    if result is None:
        print("-> [XÁC NHẬN]: KHÔNG thấy vật trong kẹp. Gắp trượt hoàn toàn!")
        return False
        
    # Lấy tọa độ 3D relative của vật đối với camera 
    pos_3d = result["position_3d"]
    import math
    # Tính khoảng cách hình học từ tâm camera đến khối hộp
    distance = math.sqrt(pos_3d[0]**2 + pos_3d[1]**2 + pos_3d[2]**2)
    
    print(f"Khoảng cách vật tới camera sau khi nhấc: {round(distance, 3)} mét")
    
    # Nếu vật nằm sát kẹp (thường khoảng cách vật lý trong Webots sẽ < 0.25 mét)
    if distance < 0.25:
        print("-> [XÁC NHẬN]: Đã gắp THÀNH CÔNG khối hộp! Tiến hành mang về thùng.")
        return True
    else:
        print("-> [XÁC NHẬN]: Gắp hụt! Vật nhìn thấy là vật đang trôi tiếp dưới băng tải.")
        return False
    
def goto_pose(target_q, steps=100, tolerance=0.02):
    smooth_move_ur_to(target_q, steps=steps)
    wait_until_reached(target_q, tolerance=tolerance)
    
def preview_camera(n_steps=200):
    print("Preview camera before picking...")

    for _ in range(n_steps):
        if robot.step(TIME_STEP) == -1:
            return False

    return True
    
HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0] 

PICK_ABOVE = [-0.000, -1.210, 1.370, -1.770, -1.590, -0.000] 

PICK_DOWN = [-0.000, -1.010, 1.590, -2.170, -1.553, -0.020]

BIN_ABOVE = [3.060, -1.550, 1.050, -1.050, -1.490, -0.080]  

BIN_DOWN = [3.060, -1.550, 1.450, -1.450, -1.490, -0.080] 

SAFE_MID = [2.410, -1.570, 1.150, -1.210, -1.570, 0.000]

while robot.step(TIME_STEP) != -1:
    # 1. Quay về vị trí HOME [cite: 13]
    goto_pose(HOME, steps=100)

    # 2. Mở gripper [cite: 14]
    open_gripper()
    wait_steps(50)

    # 3. Di chuyển tới vị trí đợi phía trên băng tải [cite: 14]
    goto_pose(PICK_ABOVE, steps=100)
    
    # 3.1 Bám đuổi khối hộp trên cao cho đến khi thẳng hàng [cite: 53]
    tracked_pan = wait_until_cube_ready(max_steps=500)
    if tracked_pan is None:
        continue  # Bỏ qua lượt nếu không tìm thấy mục tiêu [cite: 53]
    
    # 4. SỬ DỤNG HÀM MỚI: Hạ tay xuống gắp chủ động thời gian thực
    dynamic_descend_and_pick(steps=25) 

    # 5. Đóng gripper để gắp vật [cite: 14]
    close_gripper()
    wait_steps(80)

    # 6. Nâng vật lên thẳng đứng theo góc hiện tại [cite: 14]
    current_pan = get_current_joint_positions()[0] # Lấy góc thực tế lúc vừa kẹp xong [cite: 10]
    dynamic_pick_above = list(PICK_ABOVE)
    dynamic_pick_above[0] = current_pan
    goto_pose(dynamic_pick_above, steps=30)
    
    # 7. KIỂM TRA TRẠNG THÁI GẮP THỰC TẾ
    if check_if_picked_successfully():
        # LỘ TRÌNH KHI GẮP THÀNH CÔNG: Mang ra thùng thả [cite: 14]
        goto_pose(SAFE_MID, steps=80)
        goto_pose(BIN_ABOVE, steps=100)
        goto_pose(BIN_DOWN, steps=80)
        
        open_gripper()  # Thả vật [cite: 15]
        wait_steps(80)
        
        goto_pose(BIN_ABOVE, steps=80)
        goto_pose(SAFE_MID, steps=80)
    else:
        # LỘ TRÌNH KHI GẮP TRƯỢT: Không chạy qua thùng đỏ nữa, tránh lãng phí thời gian
        print("Hủy quy trình thả thùng. Quay về HOME chuẩn bị săn mục tiêu mới...")
        
    # Quay lại vị trí HOME ban đầu [cite: 15]
    goto_pose(HOME, steps=100)