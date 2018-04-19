from machine import I2C, Pin
from servo import Servos
import math

class Servoarm:
    def __init__(self, dualservo='shoulder', offset=-40):
        self.offset = offset    #定义左右舵机偏置
        self.dualservo = dualservo
        '''定义舵机关节接线'''
        self.servos = {'wist':1,'elbow':2,'pan':3,'shoulder_l':4,'shoulder_r':5}
        '''定义各关节属性字典，名称:[最小,最大,初始,锁定,顶点，当前］,每次调用舵机函数更新当前脉宽'''
        self.joints = {
            'wist':[800,2100,1650,False,2100,1650],
            'elbow':[750,2100,1000,False,1800,1000],
            'pan':[1000,2100,1550,False,1550,1550],
            'shoulder':[800,1970,1800,False,1425,1800]
            }
        self._i2c = I2C(sda=Pin('P5'),scl=Pin('P4'))
        self.servo = Servos(self._i2c)

    def init(self):
        for joint in self.joints.keys():
            self.arm_set(joint,self.joints[joint][2])

    def arm_set(self,servokey,value):
        value = min(self.joints[servokey][1],max(self.joints[servokey][0],value))
        if servokey == self.dualservo:
            self.servo.position(self.servos['shoulder_l'],us=value)
            self.servo.position(self.servos['shoulder_r'],
                                us=sum(self.joints['shoulder'][:2])-value+self.offset)
        else:
            self.servo.position(self.servos[servokey],us=value)
        self.joints[servokey][-1]=value

    def action(self,x_err=None,y_err=None,h_err=None):
        actions={}
        for joint in self.joints.keys():
            actions[joint] = self.joints[joint][-1]

        if h_err is not None:
            actions['shoulder'] = min(self.joints['shoulder'][1],
                                    max(self.joints['shoulder'][0],
                                    actions['shoulder'] + int(h_err) ))
            actions['elbow'] = min(self.joints['elbow'][1],
                                max(self.joints['elbow'][0],
                                actions['elbow']+self.joints['shoulder'][-1]-actions['shoulder']))
            print(max(self.joints['shoulder'][0],actions['shoulder']),actions['shoulder']-int(h_err))
#            actions['wist'] += int(h_err)-
#            actions['wist'] = self.joints['wist'][-1] + int(h_err)
            h_ouut = h_err
        if x_err is not None:
            actions['pan'] = self.joints['pan'][-1] + int(x_err)
        if y_err is not None:
            if (self.joints['elbow'][0] < actions['elbow']-y_err < self.joints['elbow'][-2]) and not self.joints['elbow'][-3]:
                actions['elbow'] -= int(y_err)
                actions['wist'] -= int(y_err)
                self.joints['wist'][-2] = min(self.joints['wist'][1],
                                            max(self.joints['wist'][0],
                                            actions['wist'] ))
#                self.joints['elbow'][-3]=False
                print('aaaaa',self.joints['wist'][-2])
            elif self.joints['elbow'][-3]:
                if actions['wist'] - self.joints['wist'][-2] < y_err:
                    self.joints['elbow'][-3] = False
                    actions['wist'] = max(self.joints['wist'][-2],actions['wist']+int(y_err))
                    actions['elbow'] = int(y_err-(actions['wist']-self.joints['wist'][-2]))
                else:
                    actions['wist'] += int(y_err)
                print('bbbb')
            else:
                actions['elbow'] = min(self.joints['elbow'][-2],
                                    max(self.joints['elbow'][0],
                                    actions['elbow'] - int(y_err)))
                actions['wist'] += actions['elbow']-(self.joints['elbow'][-1] -int(y_err))
                self.joints['elbow'][-3] = True
                print('cccccc')
        return actions

        '''
            rd_shld = math.radians((self.joints['shoulder'][-1]-self.joints['shoulder'][-2])/10)
            rd_elb = math.radians((self.joints['elbow'][-1]-self.joints['elbow'][-2])/10)
            b_shld = math.sin(rd_shld)
            b_elb = math.sin(rd_shld)
            delta_shld = math.degrees(math.asin((b_shld+y_err/2)/self.joints['shoulder'][-3]))
            delta_elb = math.degrees(math.asin((b_elb+y_err/2)/self.joints['elbow'][-3]))
        '''

    def execute(self,actionlists):
        #传递各关节动作位置字典,舵机名称:舵机位置us
        for act in actionlists.keys():
            self.arm_set(act,actionlists[act])
