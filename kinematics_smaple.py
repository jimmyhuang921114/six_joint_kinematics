# -*- coding: utf-8 -*-
import math
import numpy as np
from matplotlib import pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D

class kinematics():
    def __init__(self, link):
        self.link = link                        ## 各軸連桿長度的陣列
        self.joint = [0, 0, 0, 0, 0, 0]         ## 各軸旋轉角度的陣列
        self.end_position = []                  ## 末端點的位置 [x, y, z]
        self.end_orientation = []               ## 末端點的姿態 [Pitch, Roll, Yaw]
        self.dof = 6                            ## 手臂自由度
        self.all_pos = np.zeros((1,3))          ## 各軸位置資訊
        self.all_or_n = np.zeros((1,3))         ## 各軸X向量資訊
        self.all_or_o = np.zeros((1,3))         ## 各軸Y向量資訊
        self.all_or_a = np.zeros((1,3))         ## 各軸Z向量資訊
        self.wrist = [0, 0, 0]                  ## 手腕位置 [x, y, z]

    def demo(self, end_position, end_orientation):
        self.end_position = end_position        ## 儲存末端點位置
        self.end_orientation = end_orientation  ## 儲存末端點姿態
        self.inverse_kinamatics()               ## 逆運動學
        self.forward_kinematics()               ## 正運動學
        self.plot()                             ## 繪圖

    ### 計算逆運動學
    def inverse_kinamatics(self):
        ###  定義手臂末端點姿態矩陣(旋轉順序)
        ###  定義 Roll, Pitch, Yaw旋轉軸並且放到相對應的 Rz, Ry, Rx
        ###  end_o為大地座標對手臂的旋轉矩陣
        Ry
        Rx
        Rz
        end_o
        R_YPR = end_o*Rx*Ry*Rz

        ###  計算手腕中心
        ###  分別計算手腕中心的位置並且放到 wrist陣列中
        self.wrist[0] 
        self.wrist[1] 
        self.wrist[2] 

        ###  逆位置運動學
        ###  使用逆位置運動學求出第一、二、三軸的角度
        self.joint[0]
        self.joint[2]
        self.joint[1]

        ###  逆姿態運動學
        ###  求出 R0_3(第一到三軸的旋轉矩陣)，並且透過 R0_3與 R_YPR求出 R3_6
        R0_3
        R3_6
        ###  使用 R3_6分別計算出四、五、六軸的角度
        self.joint[3]
        self.joint[4]
        self.joint[5]

    ###  DH連桿表
    def dh_list(self):
        ###  製作 dh連桿表陣列，每一列為各軸，每一行分別為 a, α, d, θ
        ###  並且將製作好的 dh連桿表回傳回去
        ######                    a          α            d            θ     #####
        return dh

    ### 計算正運動學
    def forward_kinematics(self):
        for i in range(0, self.dof):
            ###  輸入各軸的角度以及 dh連桿表的資訊到齊次轉換矩陣中來獲得各軸 A矩陣(齊次轉換矩陣)
            A
            ###  計算各軸旋轉矩陣 T
            T
            ###  從T矩陣中儲存各軸的位置(pos), x向量(n), y向量(o), z向量(a)
            self.all_pos
            self.all_or_n
            self.all_or_o
            self.all_or_a

    ### 計算齊次轉換矩陣
    def GenerateTransformationMatrices(self, Theta, dh):
        ###  將旋轉的角度和連桿資訊放到齊次轉換矩陣中來獲得 A矩陣，並且回傳A矩陣
        A
        return A

    ### 畫圖
    def plot(self):
        ax = plt.subplot(111, projection='3d')
        ###  畫出各關節的位置
        ax.scatter()
        ###  將各關節的位置進行連線
        ax.plot()

        ###  畫出各關節的姿態
        for i in range(0, self.dof+1):
            ###  畫出 x向量(n)
            ax.plot()
            ###  畫出 y向量(o)
            ax.plot()
            ###  畫出 z向量(a)
            ax.plot()

        ax.set_xlim(-150, 150)
        ax.set_ylim(-150, 150)
        ax.set_zlim(  0 , 150)
        ax.set_zlabel('Z')
        ax.set_ylabel('Y')
        ax.set_xlabel('X')
        plt.show()

    ###  測試正運動學
    def demo_forward(self,joint):
        self.joint = joint
        self.forward_kinematics()
        self.plot()

if __name__ == "__main__":
    ###  手臂連桿的長度
    link = [10, 60, 50, 20]
    six_dof_link = kinematics(link)

    ###  手臂各關節角度
    joint = [0, 0, 0, 0, 0, 0]
    # six_dof_link.demo_forward(joint)

    ###  手臂末端點位置
    x = 50
    y = 60
    z = 30
    end_position = [x, y, z]
    ###  手臂末端點姿態
    pitch = math.radians(0)
    roll  = math.radians(0)
    yaw   = math.radians(0)
    end_orientation = [pitch, roll, yaw]

    six_dof_link.demo(end_position, end_orientation)
    
