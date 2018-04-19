# ServoArm 

基于OpenMV3 M7控制机械臂跟踪小球，或者追踪人脸，一个简单的示例，I2C连接PCA9685模块，控制四个舵机，当然可以更多

其中用到的PID、pca9685、I2C等模块以及接线参考了[openmv.cc 星瞳科技](http://openmv.cc) 上的例程

机械臂模块为servoarm.py，调用方法：
~~~ sh
from servoarm import Servoarm
myarm = Servoarm() #创建机械臂实例，我的机械臂肩关节用了双舵机，可自省去掉
myarm.init()       #执行一个自检动作

#根据blob移动计算出的偏移量，计算出每个关节舵机的移动位置，返回给action变量
action = myarm.action(x_err, y_err, z_err)  

myarm.execute(action) #舵机执行动作

~~~