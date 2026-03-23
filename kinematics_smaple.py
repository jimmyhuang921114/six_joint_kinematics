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





""" =======================逆運動學計算========================"""

    ### 計算逆運動學
    def inverse_kinamatics(self):
        ###  定義手臂末端點姿態矩陣(旋轉順序)
        ###  定義 Roll, Pitch, Yaw旋轉軸並且放到相對應的 Rz, Ry, Rx
        ###  end_o為大地座標對手臂的旋轉矩陣
        "====================A.計算末段旋轉矩陣=================="
        deg_p = 
        deg_y = 
        deg_r =


        R_y = np.matrix([   [ , , ],
                            [ , , ],
                            [ , , ], 
        ]) 
        R_x = np.matrix([   [ , , ],
                            [ , , ],
                            [ , , ], 
        ]) 
        R_z = np.matrix([   [ , , ],
                            [ , , ],
                            [ , , ], 
        ]) 



        R_YPR = end_o*Rx*Ry*Rz #求出整體的旋轉矩陣

        ###  計算手腕中心
        ###  分別計算手腕中心的位置並且放到 wrist陣列中

        #計算前三軸的手臂位置,並將末端位置減去前三軸的位置



        """ ============ 逆位置運動學 ============="""
        ###  使用逆位置運動學求出第一、二、三軸的位置

        self.wrist[0] = self.end_position[0] - self.link[3]*R_YPR[0, 2]
        self.wrist[1] = self.end_position[1] - self.link[3]*R_YPR[1, 2]
        self.wrist[2] = self.end_position[2] - self.link[3]*R_YPR[2, 2]


        # 如果第一軸跟第二軸的角度等於0,將第一軸角度設定為0
        if( abs(self.wrist[0]) < 0.0001 and abs(self.wrist[1]) < 0.0001):
            self.joint[0] = 0
        else:
            self.joint[0] = math.degrees(math.atan2(-self.wrist[0], self.wrist[1]))


        cos_d = (self.wrist[0]**2 + self.wrist[1]**2 + \
                (self.wrist[2]-self.link[0])**2 -(self.link[1]**2+self.link[2]**2) )/ \
                (2*self.link[1]*self.link[2]) 
        
        self.joint[2] = math.degrees(math.atan2( -1*math.sqrt(1-cos_d**2) , cos_d ))
        self.joint[1] = math.degrees(math.atan2( (self.wrist[2]-self.link[0]) , math.sqrt( self.wrist[0]**2+self.wrist[1]**2 ) )) - \
                        math.degrees(math.atan2( self.link[2]*math.sin(math.radians(self.joint[2])) , (self.link[1]+self.link[2]*math.cos(math.radians(self.joint[2])))))




        """ ============= 逆姿態運動學 ============="""
        ###  求出 R0_3(第一到三軸的旋轉矩陣)，並且透過 R0_3與 R_YPR求出 R3_6

        dh = self.dh_list() #先去定義出你的D-H連桿表

        """ * 需要去計算相較於手臂基準的角度，計算出絕對角度"""
        """ -----------------------------------------------
            相對旋轉角度 + 手臂原始角度
        ------------------------------------------------"""
        s_1  = math.sin( math.radians(self.joint[0]) + dh[0, 3] )
        c_1  = math.cos( math.radians(self.joint[0]) + dh[0, 3] )
        s_23 = math.sin( math.radians(self.joint[1]) + math.radians(self.joint[2]) + dh[1, 3] + dh[2, 3] )
        c_23 = math.cos( math.radians(self.joint[1]) + math.radians(self.joint[2]) + dh[1, 3] + dh[2, 3] )




        # 計算前三軸角度
        R0_3 = np.array([ [ c_1*c_23 ,  s_1 , c_1*s_23 ],
                          [ s_1*c_23 , -c_1 , s_1*s_23 ],
                          [   s_23   ,   0  ,   -c_23  ]] )


        """---------將剛剛計算完的前三軸旋轉矩陣，將零到六軸的旋轉矩陣乘上剛剛算出來的旋轉矩陣，即可獲得三至六軸的旋轉矩陣"""
        R3_6 = np.dot(R0_3.T, R_YPR)

        ###  使用 R3_6分別計算出四、五、六軸的角度
        # 將rotation martix轉換維RPY
        #判斷ptich數值是否為正負90度


        if( abs( R3_6[2, 2] ) > 0.99999): # 如果今天pitch為正負90度的情況下 
            self.joint[3] = 0
            self.joint[4] = math.degrees(math.atan2( -math.sqrt(1 - R3_6[2, 2]**2), -R3_6[2, 2]) )
            self.joint[5] = math.degrees(math.atan2( R3_6[2, 1] , R3_6[2, 0] ))



        else: # 如果今天pitch不是正負90度的情況ㄒ
            self.joint[3] = math.degrees(math.atan2( -R3_6[1, 2] ,  R3_6[0, 2] ))
            self.joint[4] = math.degrees(math.atan2( -math.sqrt(1 - R3_6[2, 2]**2), -R3_6[2, 2]) )
            self.joint[5] = math.degrees(math.atan2( -R3_6[2, 1] , R3_6[2, 0] ))


    ###  DH連桿表
    def dh_list(self):
        ###  製作 dh連桿表陣列，每一列為各軸，每一行分別為 a, α, d, θ
        ###  並且將製作好的 dh連桿表回傳回去
        pi = math.pi
        ######                    a          α            d            θ     #####
        """ ========================== B.完成DH連桿表填寫 ========================="""
        dh = np.array([ [ , , , ],
                        [ , , , ],
                        [ , , , ],
                        [ , , , ],
        ])
        return dh



""" =======================正運動學驗證========================"""

    ### 計算正運動學
    def forward_kinematics(self):
        T = np.identity(4) # 開設4 * 4的矩陣
        dh = self.dh_list() # 獲取D-H連桿表

        #計算各軸角度
        for i in range(0, self.dof):
            ###  輸入各軸的角度以及 dh連桿表的資訊到齊次轉換矩陣中來獲得各軸 A矩陣(齊次轉換矩陣)
            A = self.GenerateTransformationMatrices(math.radians(self.joint[i]), dh[i])
            ###  計算各軸旋轉矩陣 T
            T = np.dot(T, A)


            ###  從T矩陣中儲存各軸的位置(pos), x向量(n), y向量(o), z向量(a)
            self.all_pos = np.vstack((self.all_pos, T[0:3,3]))
            self.all_or_n = np.vstack((self.all_or_n, T[0:3,0]))
            self.all_or_o = np.vstack((self.all_or_o, T[0:3,1]))
            self.all_or_a = np.vstack((self.all_or_a, T[0:3,2]))
            print(T)




    ### 計算齊次轉換矩陣
    def GenerateTransformationMatrices(self, Theta, dh):
        ###  將旋轉的角度和連桿資訊放到齊次轉換矩陣中來獲得 A矩陣，並且回傳A矩陣
        """ ============ C.填入齊次轉換矩陣 ========================="""
        c_theta = math.cos( Theta + dh[3] ) 
        s_theta = math.sin( Theta + dh[3] )
        c_alpha = math.cos( dh[1] )
        s_alpha = math.sin( dh[1] )
        A = np.array([  [ , , , ],
                        [ , , , ],
                        [ , , , ],
                        [ , , , ],
        ])

        return A




""" =======================matplotlib畫圖========================"""
    def plot(self):
        ax = plt.subplot(111, projection='3d')
        self.all_pos = np.around(self.all_pos, decimals=3)
        self.all_or_n = np.around(self.all_or_n, decimals=3)
        self.all_or_o = np.around(self.all_or_o, decimals=3)
        self.all_or_a = np.around(self.all_or_a, decimals=3)
        print(self.all_pos)
        ax.scatter(self.all_pos[:,0], self.all_pos[:,1], self.all_pos[:,2], c='r')
        ax.plot(self.all_pos[:,0], self.all_pos[:,1], self.all_pos[:,2], c='y')

        for i in range(0, self.dof+1):
            flag_n = self.all_pos[i, :]
            flag_n = np.vstack((flag_n, self.all_pos[i, :] + self.all_or_n[i, :]*20))
            ax.plot(flag_n[:,0], flag_n[:,1], flag_n[:,2], c='r')

            flag_a = self.all_pos[i, :]
            flag_a = np.vstack((flag_a, self.all_pos[i, :] + self.all_or_a[i, :]*20))
            ax.plot(flag_a[:,0], flag_a[:,1], flag_a[:,2], c='b')

            flag_o = self.all_pos[i, :] 
            flag_o = np.vstack((flag_o, self.all_pos[i, :] + self.all_or_o[i, :]*20))
            ax.plot(flag_o[:,0], flag_o[:,1], flag_o[:,2], c='g')

        # ax.set_xlim(-150, 150)
        # ax.set_ylim(-150, 150)
        # ax.set_zlim(  0 , 150)
        ax.set_zlabel('Z')
        ax.set_ylabel('Y')
        ax.set_xlabel('X')
        plt.show()

    def demo_forward(self,joint):
        self.joint = joint
        self.forward_kinematics()
        self.plot()
        print(self.all_pos)





""" =======================參數設定========================"""

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
    
