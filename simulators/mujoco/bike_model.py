import math 
from enum import Enum
import numpy as np
import control as ct
import matplotlib.pyplot as plt
import ecbc_toolbox as ecbc
from math import cos,sin,atan,sqrt

class WheelieGround:
    def __init__(self, dt:float, waist_pos:float):
        self.mb = 1.70488 # 1.70208
        self.mf = 5.91501 # 6.27033
        self.mrw = 0.43239 # 0.41106
        self.mfw = 0.35533
        self.lb = 0.3
        self.lf = 0.3
        self.db = 0.14306 # 0.14332
        self.df = 0.16170 # 0.16954
        self.Ibb = 0.06089 # 0.02664
        self.Ifb = 0.20020 # 0.05828
        self.Irw = 0.00140 # 0.00139
        self.Ifw = 0.00135
        self.r = 0.09
        self.g = 9.8
        self.dt = dt
        self.waist_pos = waist_pos
        
        self.updateSysParam(self.waist_pos)
        pass

    def updateSysParam(self, waist_pos=0):
        mb = self.mb
        mf = self.mf
        mfw = self.mfw
        db = self.db
        df = self.df
        lb = self.lb
        lf = self.lf
        Ibb = self.Ibb
        Ifb = self.Ifb
        Ifw = self.Ifw
        mc = mb + mf + mfw
        xml = (mb*db + mf*(lb+df*cos(waist_pos)) + mfw*(lb+lf*cos(waist_pos)))/mc
        yml = (mb*0  + mf*df*sin(waist_pos) + mfw*lf*sin(waist_pos))/mc
        center_ang = math.atan2(yml, xml)
        lc = sqrt(pow(xml,2)+pow(yml,2))
        Ic = (Ibb+Ifb+Ifw+
              mb*(pow(db-xml,2)+pow(0-yml,2))+ 
              mf*(pow(lb+df*cos(waist_pos)-xml,2)+pow(df*sin(waist_pos)-yml,2)) + 
              mfw*(pow(lb+lf*cos(waist_pos)-xml,2)+pow(lf*sin(waist_pos)-yml,2)))

        self.Ib = Ic
        self.Iw = self.Irw
        self.l = lc
        self.M  = self.mrw
        self.m  = self.mb + self.mf + self.mfw


        M = self.M
        m = self.m
        l = self.l
        Ib = self.Ib
        Iw = self.Iw
        g = self.g
        r = self.r
        Imc = m*l*r
        Iw2 = Iw + (M+m)*r*r
        self.A = np.array([[0, 1, 0], 
                           [m*g*l/(m*l*l + Ib - Imc*Imc/Iw2), 0, 0],
                           [-(Imc/Iw2)*m*g*l/(m*l*l + Ib - Imc*Imc/Iw2), 0, 0]])
        self.Bu = np.array([[0],
                            [-(Imc/Iw2 + 1)/(m*l*l + Ib - Imc*Imc/Iw2)],
                            [ ((m*l*l + Ib + Imc)/Iw2)/(m*l*l + Ib - Imc*Imc/Iw2)]])
        self.Bd = np.array([[0, 0],
                            [m*g*l/(m*l*l + Ib - Imc*Imc/Iw2), -(Imc/Iw2 + 1)/(m*l*l + Ib - Imc*Imc/Iw2)],
                            [-(Imc/Iw2)*m*g*l/(m*l*l + Ib - Imc*Imc/Iw2), ((m*l*l + Ib + Imc)/Iw2)/(m*l*l + Ib - Imc*Imc/Iw2)]])
        # self.Bd = np.hstack((self.Bd, self.Bu))
        self.Cm = np.eye(self.A.shape[0])
        self.Co = np.array([[0, 0, 1]])

        self.sys = ecbc.LinearDistSys(self.A, self.Bu, self.Bd, self.Cm, self.Co, self.dt)
        pass
