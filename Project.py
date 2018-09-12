import time
import random
import cozmo
import numpy
import socket
import cv2
import sys
from cozmo.util import degrees, distance_mm, radians, speed_mmps, Vector2
from cozmo.lights import Color, Light
import asyncio

try:
    from PIL import Image, ImageColor, ImageDraw, ImageStat
except ImportError:
    sys.exit('Cannot import from PIL: Do `pip3 install --user Pillow` to install')

def toBinaryImage(img):
    image_array = numpy.asarray(img)
    image_array.flags.writeable = True
    processedImg = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)
    a, processedImg = cv2.threshold(processedImg, 100, 255, cv2.THRESH_BINARY)
    element = cv2.getStructuringElement(cv2.MORPH_RECT, (15,15))
    processedImg = cv2.morphologyEx(processedImg, cv2.MORPH_CLOSE, element)
    return processedImg

def ImageToBinary(image):
    width = image.width
    height = image.height
    mList = []
    for y in range(0,height):
         row = []
         for x in range(0,width):
             pixel = image.getpixel((x, y))
             if pixel[0]>230 and pixel[1]>230 and pixel[2]>230:
                   row.append(1)
             else:
                   row.append(0)
         mList.append(row)
    return mList

# 求解当前轨迹的中线点
def detectRoute(biMatrix):
   
    img_height = len(biMatrix)
    img_width = len(biMatrix[0])
   
    # 求解区域限制在图像下方1/7处
    baseline = int(img_height *6 / 7)

    # 搜索宽度设置为5
    searchWidth = 5
    col_l_sum = 0
    col_r_sum = 0

    # 图像中遍历搜索追踪的轨迹中线
    for i in range(baseline-searchWidth, baseline+searchWidth):
        limit = 30
        col_l = img_width
        col_r = 0
        for j in range(limit, img_width-limit):

            # 单通道图像
            pixel = biMatrix[i][j]
            #print(pixel)
            # pixel == 255 追踪轨迹为浅色
            # pixel < 255 追踪轨迹为深色
            if (pixel == 1):
                if j < col_l:
                    col_l = j
                if j > col_r:
                    col_r = j
        col_l_sum = col_l_sum+col_l
        col_r_sum = col_r_sum+col_r

    # 求解的轨迹中心点
    center = (col_l_sum/searchWidth/2 + col_r_sum/searchWidth/2) / 2
    #print(center)
    return center

def check_red(image):
    width = image.width
    height = image.height
    y = height/2;
    #for x in range(0,width):
    x = width/2
    pixel = image.getpixel((x, y))
    if pixel[1]<200 and pixel[2]<200:
        return True
    else:
        return False


    print(pixel)


def check_red_1(image):
    width = image.width
    height = image.height
    y = height-1;
    #for x in range(0,width):
    x = width/2
    pixel = image.getpixel((x, y))
    if pixel[1]<200 and pixel[2]<200:
        return True
    else:
        return False    

class CozmoProject():
    def __init__(self, robot: cozmo.robot.Robot,host,port):
        self.robot = robot
        # 创建 socket 对象
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 

        # 获取本地主机名
        self.host = host 

        # 设置端口号
        self.port = port
        self.s.connect((self.host, self.port))
        self.food_number_observed = -1
        self.cube = None
        self.distance = 0
        self.prefer = [80, 80, 20]  # 食客的喜好
        self.robot_tot = [2, 2, 2]   # 机器人训练数据
        self.robot_take = [2, 2, 2]  # 机器人训练数据
        self.count = 0   # 运行次数上限
        self.image = None   
        self.checked = False
        self.robot.camera.image_stream_enabled = True
        self.robot.camera.color_image_enabled = True
        self.robot.add_event_handler(cozmo.world.EvtNewCameraImage, self.get_last_image)
    
    async def object_detect(self):
        try:
            self.cube = self.robot.world.wait_for_observed_light_cube(timeout=0.05)
        except:
            pass
    
    async def object_check(self):
        try:
            self.checked = self.robot.world.wait_for_observed_light_cube(timeout=0.5) != None
        except:
            pass
    
    async def observe_cube(self):
        while True:
            await self.object_detect()
            if self.cube != None:
               self.food_number_observed = self.cube.object_id-1
               break
   
    def take_judge(self,food_number):
        p = self.robot_take[food_number]/self.robot_tot[food_number]*100
        r = random.randint(1, 100)
        if r > p:
            return False
        return True


    def judge(self,food_number):
        r = random.randint(1, 100)
        self.robot_tot[food_number] += 1
        if r > self.prefer[food_number]:
            return False
        self.robot_take[food_number] += 1
        return True
    
    def send_message(self,message):
        
        self.s.send(message.encode('utf-8'))
        


    def carMoveWithPathTrack(self,center,width,distance):
        # 期望沿轨迹中心巡线
        expect = width / 2

        # 允许追踪偏差
        deviation = 15

        # 偏差修正过程，可以使用比例调节，这里简单处理
        # 偏差修正的差速给定值
        # 修正方向可能需要确认
        if (abs(center-expect)<deviation):
            self.robot.drive_straight(distance_mm(distance), speed_mmps(180)).wait_for_completed()
            self.distance+=distance
            #print(self.distance)
        elif (center - expect > 0):
            #turn right
            self.robot.turn_in_place(degrees(-3)).wait_for_completed()
        elif center - expect < 0:
            #turn left
            self.robot.turn_in_place(degrees(3)).wait_for_completed()
        else:
            pass

    def get_last_image(self,evt,**kwargs):
        image = self.robot.world.latest_image.raw_image
        return image
        #downsized_image = image.resize((DOWNSIZE_WIDTH, DOWNSIZE_HEIGHT), resample = Image.LANCZOS)
   

    def run(self):
        
        #发现食物id
     
        asyncio.set_event_loop(asyncio.new_event_loop())
        loop = asyncio.get_event_loop()
        tasks = [  
            asyncio.ensure_future(self.observe_cube()),
            ]
        loop.run_until_complete(asyncio.wait(tasks))  
        loop.close()
        food_number = self.food_number_observed
 
        try:
             self.send_message('2')
        except Exception as e:
             print("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")
             print(e)
             pass
       

       
        ###################################
        ###
        ###   在这里加   ##m################
        ###    

        while True:
            # 最多演示30次
            self.count += 1
            if self.count > 1:
                break
            
            
            # 操作cozmo拿取
            # ——这里添加一条圆盘·停止·旋转的语句——
            #  0 代表停止
            #  1 代表转动，，可以自己定义
       
            try:
                self.send_message('4')
                self.robot.pickup_object(self.cube,use_pre_dock_pose=True, num_retries=10).wait_for_completed()
            except Exception as e:
                print(e)
                print("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGg")
                pass
        
            # ——↑操作cozmo拿取——
            #time.sleep(3)   # 拿取动作大约占用3秒，视具体情况修改
            # ——这里添加一条圆盘·开始·旋转的语句——
            
            try:
                self.send_message('2')
            except Exception as e:
                print("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")
                print(e)
                pass
            
            # cozmo向客人移动
            # 移动部分
            self.robot.turn_in_place(degrees(180)).wait_for_completed()
            while self.distance<400:
                time.sleep(0.5)
                image = self.get_last_image(cozmo.world.EvtNewCameraImage)
                biMatrix = ImageToBinary(image)
                width = image.width
                center = detectRoute(biMatrix)
                self.carMoveWithPathTrack(center,width,40)
                if self.distance>240:
                    break
            self.distance = 0

            flag = False

            while flag == False:
                #self.robot.drive_straight(distance_mm(4), speed_mmps(180)).wait_for_completed()
                time.sleep(0.5)
                image = self.get_last_image(cozmo.world.EvtNewCameraImage)
                biMatrix = ImageToBinary(image)
                width = image.width
                center = detectRoute(biMatrix)
                self.carMoveWithPathTrack(center,width,4)
                time.sleep(0.5)
                image = self.get_last_image(cozmo.world.EvtNewCameraImage)
                if check_red_1(image):
                    flag = True
            
            # 操控舵机部分
            # True表示喜欢，False表示不喜欢
            # 向舵机发送喜好信息，True时向1号舵机发送命令，False时向2号舵机发送命令
            #if like:
            #    self.send_message('3')
            #else:
            #    self.send_message('4')
            # ——待补充——
           
            # 操作cozmo放下
            # ——检测cozmo面前是否有空间放置方块——
         
                # ——当可以放置时——
                # ——停止转盘——
                # ——操作cozmo放下方块——
              # 放下动作大约占用3秒，视具体情况修改
                # ——启动转盘——
         
            
            # cozmo返回原地
            # 移动部分
            # ——待补充——
              # 大约移动5秒，视具体情况修改
      
          





def cozmo_program(robot: cozmo.robot.Robot):
    test = CozmoProject(robot,'192.168.1.84',21)
    test.run()
 
    
cozmo.robot.Robot.drive_off_charger_on_connect = True
cozmo.run_program(cozmo_program, use_viewer = True, force_viewer_on_top = True)
