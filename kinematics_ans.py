# -*- coding: utf-8 -*-
import math
import numpy as np
from matplotlib import pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D


# aa = np.matrix([1,2,3],[4,5,6])
class kinematics():
    def __init__(self, link):
        self.link = link
        self.joint = [0, 0, 0, 0, 0, 0]
        self.end_position = []
        self.end_orientation = []
        self.dof = 6
        self.all_pos = np.zeros((1,3))
        self.all_or_n = np.zeros((1,3))
        self.all_or_o = np.zeros((1,3))
        self.all_or_a = np.zeros((1,3))

        self.wrist = [0, 0, 0]

    def demo(self, end_position, end_orientation):
        self.end_position = end_position
        self.end_orientation = end_orientation
        self.inverse_kinamatics()
        self.forward_kinematics()
        self.plot()

    ### 計算逆運動學
    def inverse_kinamatics(self):
        ###  定義手臂末端點姿態矩陣(旋轉順序)
        deg_p = self.end_orientation[0] 
        deg_y = self.end_orientation[2] 
        deg_r = self.end_orientation[1] 

        Ry = np.matrix([[  math.cos(deg_p),  0, math.sin(deg_p)],
                        [        0        ,  1,        0       ],
                        [ -math.sin(deg_p),  0, math.cos(deg_p)]
        ])
        Rx = np.matrix([[ 1,        0        ,        0        ],
                        [ 0,  math.cos(deg_y), -math.sin(deg_y)],
                        [ 0,  math.sin(deg_y),  math.cos(deg_y)]
        ])
        Rz = np.matrix([[  math.cos(deg_r), -math.sin(deg_r), 0],
                        [  math.sin(deg_r),  math.cos(deg_r), 0],
                        [        0        ,        0        , 1]
        ])
        end_o = np.matrix([ [ 0, 1, 0],
                            [ 0, 0, 1],
                            [ 1, 0, 0]
        ])
        R_YPR = end_o*Rx*Ry*Rz

        ###  計算手腕中心
        self.wrist[0] = self.end_position[0] - self.link[3]*R_YPR[0, 2]
        self.wrist[1] = self.end_position[1] - self.link[3]*R_YPR[1, 2]
        self.wrist[2] = self.end_position[2] - self.link[3]*R_YPR[2, 2]

        ###  逆位置運動學
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

        ###  逆姿態運動學
        dh = self.dh_list()

        s_1  = math.sin( math.radians(self.joint[0]) + dh[0, 3] )
        c_1  = math.cos( math.radians(self.joint[0]) + dh[0, 3] )
        s_23 = math.sin( math.radians(self.joint[1]) + math.radians(self.joint[2]) + dh[1, 3] + dh[2, 3] )
        c_23 = math.cos( math.radians(self.joint[1]) + math.radians(self.joint[2]) + dh[1, 3] + dh[2, 3] )

        R0_3 = np.array([ [ c_1*c_23 ,  s_1 , c_1*s_23 ],
                          [ s_1*c_23 , -c_1 , s_1*s_23 ],
                          [   s_23   ,   0  ,   -c_23  ]] )

        R3_6 = np.dot(R0_3.T, R_YPR)

        if( abs( R3_6[2, 2] ) > 0.99999):
            self.joint[3] = 0
            self.joint[4] = math.degrees(math.atan2( -math.sqrt(1 - R3_6[2, 2]**2), -R3_6[2, 2]) )
            self.joint[5] = math.degrees(math.atan2( R3_6[2, 1] , R3_6[2, 0] ))
        else:
            self.joint[3] = math.degrees(math.atan2( -R3_6[1, 2] ,  R3_6[0, 2] ))
            self.joint[4] = math.degrees(math.atan2( -math.sqrt(1 - R3_6[2, 2]**2), -R3_6[2, 2]) )
            self.joint[5] = math.degrees(math.atan2( -R3_6[2, 1] , R3_6[2, 0] ))

    ###  DH連桿表
    def dh_list(self):
        pi = math.pi
        ######                    a          α            d            θ     #####
        dh = np.array([ [         0     ,   pi/2  ,  self.link[0] ,   pi/2   ],
                        [   self.link[1],     0   ,       0       ,     0    ],
                        [         0     ,  -pi/2  ,       0       ,  -pi/2   ],
                        [         0     ,   pi/2  ,  self.link[2] ,     0    ],
                        [         0     ,  -pi/2  ,       0       ,     0    ],
                        [         0     ,     0   ,  self.link[3] ,     0    ]
        ])
        return dh

    ### 計算正運動學
    def forward_kinematics(self):
        T = np.identity(4)
        dh = self.dh_list()
        for i in range(0, self.dof):
            A = self.GenerateTransformationMatrices(math.radians(self.joint[i]), dh[i])
            T = np.dot(T, A)
            self.all_pos = np.vstack((self.all_pos, T[0:3,3]))
            self.all_or_n = np.vstack((self.all_or_n, T[0:3,0]))
            self.all_or_o = np.vstack((self.all_or_o, T[0:3,1]))
            self.all_or_a = np.vstack((self.all_or_a, T[0:3,2]))
            print(T)

    ### 計算齊次轉換矩陣
    def GenerateTransformationMatrices(self, Theta, dh):
        c_theta = math.cos( Theta + dh[3] )
        s_theta = math.sin( Theta + dh[3] )
        c_alpha = math.cos( dh[1] )
        s_alpha = math.sin( dh[1] )
        A = np.array([ [ c_theta  , -s_theta*c_alpha ,  s_theta*s_alpha ,  dh[0]*c_theta ],
                       [ s_theta  ,  c_theta*c_alpha , -c_theta*s_alpha ,  dh[0]*s_theta ],
                       [    0     ,      s_alpha     ,        c_alpha   ,      dh[2]     ],
                       [    0     ,         0        ,           0      ,        1       ]
                    ])
        return A

    ### 畫圖
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
if __name__ == "__main__":
    ###  手臂連桿的長度
    link = [10, 60, 50, 20]
    ###  手臂末端點位置
    x = 40
    y = 40
    z = 10
    end_position = [x, y, z]
    ###  手臂末端點姿態
    pitch = math.radians(0)
    roll  = math.radians(0)
    yaw   = math.radians(-45)
    end_orientation = [pitch, roll, yaw]
    print("末端角度")
    print(end_orientation)
    joint = [0, 0, 0, 0, 0, 0]
    six_dof_link = kinematics(link)
    # six_dof_link.demo_forward(joint)
    six_dof_link.demo(end_position, end_orientation)
    
