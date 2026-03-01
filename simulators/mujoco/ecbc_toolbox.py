import math
from enum import Enum
import numpy as np
import control as ct
import matplotlib.pyplot as plt
import warnings

class LinearDistSys:
    """
    x: state, u: control, d: disturbance, ym: measured output, yo: controlled output
    dot_x = A  x + Bu u + Bd d
       ym = Cm x
       yo = Co y
    """
    def __init__(self, A, Bu, Bd, Cm, Co, dt):
        self.updateSysParam(A, Bu, Bd, Cm, Co, dt)
        self.resetState()
        self.controller = None
    
    def __repr__(self):
        return (f"linear system with disturbance. dt = {self.dt}\n"
                f"A : {self.A}\n"
                f"Bu: {self.Bu}\n"
                f"Bd: {self.Bd}\n"
                f"Cm: {self.Cm}\n"
                f"Co: {self.Co}\n")
    
    def updateSysParam(self, A, Bu, Bd, Cm, Co, dt):
        self.A  = A   # state transition matrix
        self.Bu = Bu if Bu is not None else np.zeros((A.shape[0], 0))
                      # control input matrix,
        self.Bd = Bd if Bd is not None else np.zeros((A.shape[0], 0))
                      # disturbance input matrix (D)
        self.Cm = Cm if Cm is not None else np.eye(A.shape[0])
                      # measured output matrix (M)
        self.Co = Co if Cm is not None else np.eye(A.shape[0])
                      # controlled output matrix (O) (controlled/tracked/concerned output)
        self.dt = dt  # discretization time step
        self.nx = self.A.shape[0]  # 
        self.nu = self.Bu.shape[1] #
        self.nd = self.Bd.shape[1] #
        self.nm = self.Cm.shape[0] #
        self.no = self.Co.shape[0] #
    
    def setController(self, controller):
        self.controller = controller

    def step(self, control=None, state_add=None, disturbance=None, reference=None):
        if self.controller is None:
            self.yr = self.yr
            self.u = control.reshape(self.nu, 1) if control is not None else np.zeros((self.nu, 1))
        else:
            self.yr = reference.reshape(self.no, 1)
            self.u = self.controller.stepAndGetControl(self.ym, self.yr, self.t)

        self.d = disturbance.reshape(self.nd, 1) if disturbance is not None else np.zeros((self.nd, 1))
        self.xadd = state_add.reshape(self.nx, 1) if state_add is not None else np.zeros((self.nx, 1))
            
        dx = self.A @ self.x + self.xadd
        if self.nu > 0:
            dx += self.Bu @ self.u
        if self.nd > 0:
            dx += self.Bd @ self.d
        self.x += dx * self.dt
        self.ym = self.Cm @ self.x
        self.yo = self.Co @ self.x
        self.t += self.dt
        # print(f"x:{self.x}, dx:{dx}, state_add:{state_add}, dt:{self.dt}, A:{self.A}, Bu:{self.Bu}, Cm:{self.Cm}")

        self.T  = np.hstack((self.T, np.array([[self.t]])))
        self.X  = np.hstack((self.X, self.x))
        self.Xadd = np.hstack((self.Xadd, self.xadd))
        self.U  = np.hstack((self.U, self.u))
        self.D  = np.hstack((self.D, self.d))
        self.Ym = np.hstack((self.Ym, self.ym))
        self.Yo = np.hstack((self.Yo, self.yo))
        self.Yr = np.hstack((self.Yr, self.yr))
    
    def simNstep(self, nstep, init_state=None, Control=None, Disturbance=None, Reference=None, StateAdd=None):
        self.resetState(init_state)
        tempU = Control if Control is not None else np.zeros((self.nu, nstep))
        tempD = Disturbance if Disturbance is not None else np.zeros((self.nd, nstep))
        tempYr = Reference if Reference is not None else np.zeros((self.no, nstep))
        tempXadd = StateAdd if StateAdd is not None else np.zeros((self.nx, nstep))
        if (tempU.shape[1] != nstep or tempD.shape[1] != nstep or 
            tempYr.shape[1] != nstep or tempXadd.shape[1] != nstep):
            raise ValueError(f"Control, Disturbance, Reference and StateAdd must "
                             f"have {nstep} columns.")
        for i in range(0, nstep):
            self.step(tempU[:,i], tempXadd[:,i], tempD[:,i], tempYr[:,i])
    
    def resetState(self, init_state=None):
        self.t    = 0
        self.x    = np.zeros((self.nx, 1)) if init_state is None else init_state.reshape((self.nx, -1))
        self.xadd = np.zeros((self.nx, 1))
        self.u    = np.zeros((self.nu, 1))
        self.d    = np.zeros((self.nd, 1))
        self.ym   = self.Cm @ self.x
        self.yo   = self.Co @ self.x
        self.yr   = np.zeros((self.no, 1))
        self.T    = np.array([[self.t]])
        self.X    = self.x
        self.Xadd = np.zeros((self.nx, 0))
        self.U    = np.zeros((self.nu, 0))
        self.D    = np.zeros((self.nd, 0))
        self.Ym   = self.ym
        self.Yo   = self.yo
        self.Yr   = self.yr
        
    def findEquilibrium(self, d_est, yo_ref):
        """
        find equilibrium (x_e, u_e) satisfying 
            A  x_e + Bu u_e + Bd d_est = 0
            Co x_e          -   yo_ref = 0
        """
        Quo = np.block([[self.A, self.Bu], [self.Co, np.zeros((self.no, self.nu))]])
        rQuo = np.linalg.matrix_rank(Quo)
        Qudo = np.hstack((Quo, np.vstack((self.Bd, np.zeros((self.no, self.nd))))))
        rQudo = np.linalg.matrix_rank(Qudo)
        rhs = np.vstack((-self.Bd @ d_est, yo_ref))
        if rQuo == self.nx + self.no:
            ze  = np.linalg.inv(Quo) @ rhs
        elif rQuo == rQudo:
            ze, _, _, _  = np.linalg.lstsq(Quo, rhs, rcond=None)
            if yo_ref != np.zeros_like(yo_ref):
                warnings.warn("The disturbance doesn't require compensation.\n"
                    "But 'yo' can only track zero. Otherwise, there is no equilibrium solusion, so a least square solution is provided.\n"
                    "In the zero-tracking case, the disturbance doesn't affect the steady-state of the controlled output.", 
                    UserWarning)
        else:
            raise ValueError("The disturbance cannot be completely compensated. "
                             "Adding more actuators or changing the controlled output could be helpful.")
        return (ze[:self.nx, 0].reshape(-1,1), ze[-self.nu:, 0].reshape(-1,1))
    
    def plotResult(self, state=True, control=False, disturbance=False,
                   measuredOut=True, controlledOut=True):
        def pad1(Z):
            return np.hstack((Z, Z[:,-1]))
        def plotMline(T, Z, label_prefix=""):
            for i in range(Z.shape[0]):
                plt.plot(T[0, :], Z[i, :], label=label_prefix+f'line {i+1}')
        if state:
            plt.figure()
            plotMline(self.T, self.X)
            plt.legend()
            plt.title("state")
        if control:
            plt.figure()
            plotMline(self.T, pad1(self.U))
            plt.legend()
            plt.title("control")
        if disturbance:
            plt.figure()
            plotMline(self.T, pad1(self.D))
            plt.legend()
            plt.title("disturbance")
        if measuredOut:
            plt.figure()
            plotMline(self.T, self.Ym)
            plt.legend()
            plt.title("measured output")
        if controlledOut:
            plt.figure()
            plotMline(self.T, self.Yo, "Actual ")
            plotMline(self.T, self.Yr, "Reference ")
            plt.legend()
            plt.title("controlled output")

class SignalGenerator:
    def __init__(self, sig_type, **kwargs):
        self.sig_type = sig_type
        self.kwargs = kwargs
        self.out = 0
        if self.sig_type == 'sin':
            wc = kwargs.get('wc')
            amp = kwargs.get('amp')
            phase = kwargs.get('phase')
            offset = kwargs.get('offset')
            if (wc is None) or (amp is None):
                raise ValueError("For 'sin' signal, y = amp*sin(wc*t + phase) + offset, "
                                 "'freq' and 'amp' must be provided.")
            self.wc = wc
            self.amp = amp
            self.phase = 0 if phase is None else phase
            self.offset = 0 if offset is None else offset

        elif self.sig_type == 'pwm':
            period = kwargs.get('period')
            duty_ratio = kwargs.get('duty_ratio')
            amp = kwargs.get('amp')
            offset = kwargs.get('offset')
            if (period is None) or (duty_ratio is None) or (amp is None):
                raise ValueError("For 'pwm' signal, y = amp*(1 if (t % period < duty_ratio) else -1.0) + offset, "
                                 "'period', 'duty_ratio' and 'amp' must be provided.")
            self.period = period
            self.duty_ratio = duty_ratio
            self.amp = amp
            self.offset = 0 if offset is None else offset

        elif self.sig_type == 'step':
            init_value = kwargs.get('init_value')
            step_value = kwargs.get('step_value')
            step_time = kwargs.get('step_time')
            if (init_value is None) or (step_value is None) or (step_time is None):
                raise ValueError("For 'step' signal, y = init_value if (t <= step_time) else step_value, "
                                 "'init_value', 'step_value' and 'step_time' must be provided.")
            self.init_value = init_value
            self.step_value = step_value
            self.step_time = step_time
        else:
            raise ValueError(f"Unknown signal type '{self.sig_type}'. Use 'sin' "
                             f"or 'pwm' or 'step'.")

    def valueAtT(self, t):
        if self.sig_type == 'sin':
            self.out = self.amp * math.sin(self.wc * t + self.phase) + self.offset
            pass
        elif self.sig_type == 'pwm':
            self.out = self.amp*(1.0 if ((t % self.period)/self.period < self.duty_ratio) else -1.0) + self.offset
            pass
        elif self.sig_type == 'step':
            self.out = self.init_value if (t <= self.step_time) else self.step_value
            pass
        else:
            self.out = 0
        return self.out

class TransferFunction:
    pass

class LowpassFilter:
    """
    First-order low-pass filter; for higher order use LowPassFilterSs

    Methods:
    - stepAndGetOutput(newin): perform filtering
    """
    def __init__(self, dt, Ts):
        self.dt = dt
        self.Ts = Ts
        self.pole = 1.0/self.Ts
        self.xin = 0
        self.xout = 0
        self.first_step = True

    def stepAndGetOutput(self, newin):
        # (s + p) y = p x, yk - yl = p(x - y) dt
        self.xin = newin
        self.xout = self.xout 
        if self.first_step:
            self.xout = self.xin
            self.first_step = False
        else:
            self.xout = self.xout + (self.xin - self.xout) * self.dt / (self.Ts + self.dt)
            # self.xout = self.xout + (self.xin - self.xout) * self.pole * self.dt
        return self.xout

class Controller:
    def resetState(self, init_state=None):
        pass
    def updateSysAndGain(self, new_sys, method, **kwargs):
        pass
    def updateGain(self, method, **kwargs):
        pass
    # def stepAndGetControl(self, state, refstate, t):
    #     pass
    pass

class PIDController(Controller):
    """
    PID controller

    Attributes:
    -
    -
    -

    Methods:
    -
    -
    -
    """
    def __init__(self, dt, kp=0, kd=0, ki=0):
        self.dt = dt
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.integral = 0
        self.derivative = 0
        self.ref = 0
        self.act = 0
        self.err = 0
        self.last_err = 0
        self.first_step = True
        self.out = 0
    
    def resetIntegrator(self, new_int = 0):
        self.integral = new_int

    def stepAndGetOutput(self, act, ref):
        self.ref = ref
        self.act = act
        self.last_err = self.err
        self.err = self.act - self.ref
        if self.first_step:
            self.derivative = 0
            self.first_step = False
        else:
            self.derivative = (self.err - self.last_err) / self.dt
        # Derivative with filter: y = 1/T (x - lpfx)
        self.integral += self.err * self.dt
        self.output = -(self.err * self.kp + self.derivative * self.kd + self.integral * self.ki)
        return self.output

class LinearController(Controller):
    """
    Linear state-feedback controller.

    Attributes:
    - refsys (LinearDistSys) linear system model with disturbance
    - method (str)        gain computation method: 'place_multiple_poles', 'place_distinct_poles', 'lqr'.
    - wc, poles, Q, R     parameters for different gain methods
    
    Methods:
    - updateGain(method, kwargs)    update gain
    - stepAndGetControl(state, refstate, t)   get control input
    """
    def __init__(self, sys, method, **kwargs):
        self.refsys = sys
        self.method = method
        self.kwargs = kwargs
        self.K = []
        self.updateGain(method, **kwargs)
    
    def resetState(self, init_state=None):
        pass

    def updateSysAndGain(self, sys, method=None, **kwargs):
        self.refsys.updateSysParam(sys.A, sys.Bu, sys.Bd, 
                                sys.Cm, sys.Co, sys.dt)
        if method is None:
            self.updateGain(self.method, **(self.kwargs))
        else:
            self.updateGain(method, **kwargs)

    def getPoles(self):
        return np.linalg.eigvals(self.refsys.A - self.refsys.Bu @ self.K)

    def updateGain(self, method, **kwargs):
        """
        Compute feedback gain. Select required parameters depending on the chosen method.

        Parameters:
        - algorithm (str): method selection 
          ('place_multiple_poles', 'place_distinct_poles', 'lqr')
        - **kwargs: method-specific parameters
          - place_multiple_poles: wc, repeated poles
          - place_distince_poles: poles (array), distinct poles
          - lqr                 : Q, R, weighting matrices

        Returns:
        - (K, poles)
        """
        self.method = method
        self.kwargs = kwargs
        ctrb_ref = np.linalg.matrix_rank(ct.ctrb(self.refsys.A, self.refsys.Bu))
        if ctrb_ref < self.refsys.nx:
            raise ValueError(f"The reference system\n A: {self.refsys.A}\n Bu: {self.refsys.Bu}\n is not controllable.")
        if method == 'place_multiple_poles':
            wc = kwargs.get('wc')
            if wc is None:
                raise ValueError("For 'place_multiple_poles' algorithm, 'wc' must be provided.")
            self.wc = wc
            self.poles = np.full(self.refsys.nx, self.wc)
            self.Q = None
            self.R = None
            # self.K = ct.place_varga(self.refsys.A, self.refsys.Bu, self.poles)
            # self.K = ct.place_acker(self.refsys.A, self.refsys.Bu, self.poles)
            self.K = ct.place(self.refsys.A, self.refsys.Bu, self.poles)

        elif method == 'place_distinct_poles':
            poles = kwargs.get('poles_ctr')
            if poles is None:
                raise ValueError("For 'place_distinct_poles' algorithm, 'poles_ctr' must be provided.")
            self.wc = None
            self.poles = poles
            self.Q = None
            self.R = None
            # self.K = ct.place_varga(self.refsys.A, self.refsys.Bu, self.poles)
            # self.K = ct.place_acker(self.refsys.A, self.refsys.Bu, self.poles)
            self.K = ct.place(self.refsys.A, self.refsys.Bu, self.poles)

        elif method == 'lqr':
            Q = kwargs.get('Qc')
            R = kwargs.get('Rc')
            if Q is None or R is None:
                raise ValueError("For 'lqr' algorithm, both 'Qc' and 'Rc' must be provided.")
            self.wc = None
            self.Q = Q
            self.R = R
            self.K, _, self.poles = ct.lqr(self.refsys.A, self.refsys.Bu, self.Q, self.R)
        
        else:
            raise ValueError(f"Unknown method '{method}'. Use 'place_multiple_poles' "
                             f"or 'place_distinct_poles' or 'lqr'.")
        
        # print(f"controller K: {self.K}")
        flat_str = ', '.join(f'{x:.3e}' for x in self.K.ravel())
        print(f'controller K: {{{flat_str}}}')
        
        return self.K, self.poles 
    
    def stepAndGetControl(self, state, refstate, t):
        if self.K is None:
            raise ValueError("please calculate gain using 'updateCtrGain' correctly.")
        # print(refstate)
        # print(state)
        # print(self.K @ (refstate - state))
        return (self.K @ (refstate - state))
        
class SMController(Controller):
    r"""
    Sliding-mode controller.

    Attributes:
    - refsys (LinearDistSys) linear system model with disturbance
    - sliding_mode_method sliding mode type
    -- SM, sliding mode
    --- SMM_SM_poly_0_n1, sliding surface characteristic polynomial coefficients: a_0, \cdots, a_{n-2}, s = a_0 x_1 + \cdots + a_{n-2} x_{n-1}
    -- TSM, terminal sliding mode
    -- NTSM, non-singular terminal sliding mode
    -- FTSM, fast terminal sliding mode
    - reaching_phase_method reaching-phase method
    -- SAT, saturation u = umax if s < 0 else -umax
    --- RPM_SAT_umax input limit
    -- CONST, constant u = -gain*sign(s)
    --- RPM_CONST_gain 
    - dechatter_method chattering reduction method
    -- NONE, no chattering reduction
    -- HIGH_GAIN, sign(s) = s/(|s|+e)
    --- DCM_HIGH_GAIN_epsilon e
    -- BOUND_LAYER, sign(s) = sign(s) if |s| > e else ks * s
    --- DCM_BOUND_LAYER_epsilon e
    --- DCM_BOUND_LAYER_ks      ks
    
    Methods:
    - updateCtrlFunc(kwargs)                  update control law
    - updateSysAndCtrlFunc(sys, kwargs)       update system and control law
    - stepAndGetControl(state, refstate, t)   get control input
    """
    def __init__(self, sys, **kwargs):
        self.refsys = sys
        self.kwargs = kwargs
        self.updateSysAndCtrlFunc(sys, **kwargs)

    def deriveCtrlCanonSystem(self):
        sys = self.refsys
        if sys.nu > 1:
            raise ValueError(f"Systems with multiple input are unsupported.")
        Mctrb = ct.ctrb(sys.A, sys.Bu)
        rankMctrb = np.linalg.matrix_rank(ct.ctrb(self.refsys.A, self.refsys.Bu))
        if rankMctrb < sys.nx:
            raise ValueError(f"The reference system\n A: {self.refsys.A}\n Bu: {self.refsys.Bu}\n is not controllable."
                             f"Thus, it dosn't have a controllable canonical form.")
        # print(f"Mctrb: {Mctrb}")
        b = np.linalg.inv(Mctrb)[-1, :]
        Ti = b.copy()
        for i in range(1, sys.nx):
            b = b @ sys.A
            Ti = np.vstack((Ti, b))
        # print(f"Ti: {Ti}")
        T = np.linalg.inv(Ti)
        nA = Ti @ sys.A @ T
        nBu = np.zeros_like(sys.Bu)
        nBu[-1] = 1
        nBd = Ti @ sys.Bd
        nCm = sys.Cm @ T
        nCo = sys.Co @ T
        dt = sys.dt
        self.canonsys = LinearDistSys(nA, nBu, nBd, nCm, nCo, dt)
        self.Ti = Ti
        self.T = T
    
    def resetState(self, init_state=None):
        pass

    def updateSysAndCtrlFunc(self, sys, **kwargs):
        self.refsys.updateSysParam(sys.A, sys.Bu, sys.Bd, 
                                sys.Cm, sys.Co, sys.dt)
        self.deriveCtrlCanonSystem()
        self.updateCtrlFunc(**kwargs)

    def updateCtrlFunc(self, **kwargs):
        """
        Compute control law. Select required parameters depending on the chosen method.
        """
        self.kwargs = kwargs
        
        self.sliding_mode_method = kwargs.get('sliding_mode_method')
        # SM, sliding mode, sliding mode;
        # TSM, terminal SM, terminal sliding mode; 
        # NTSM, non-singular TSM, non-singular terminal sliding mode;
        # FTSM, fast TSM, fast terminal sliding mode; 
        if self.sliding_mode_method == "SM":
            self.SMM_SM_poly_0_n1 = kwargs.get('SMM_SM_poly_0_n1')
            if self.SMM_SM_poly_0_n1[-1, 0] == 0:
                raise ValueError(f"Invalid polynomial. The last element must be non-zero")
            else:
                self.SMM_SM_poly_0_n1 = self.SMM_SM_poly_0_n1 / self.SMM_SM_poly_0_n1[-1, 0]
            self.func_s = lambda x: (self.SMM_SM_poly_0_n1 @ x).item()
            def func_ctrl_equi(x):
                tmp = (-self.canonsys.A[-1,:] @ x[:,0] - self.SMM_SM_poly_0_n1[0, :-1] @ x[1:, 0]).item()
                return tmp
            self.func_ctrl_equi = func_ctrl_equi
            pass
        elif self.sliding_mode_method == "TSM":
            pass
        elif self.sliding_mode_method == "NTSM":
            pass
        elif self.sliding_mode_method == "FTSM":
            pass
        else:
            raise ValueError(f"Unknown sliding mode method, avaliable methods: [SM, TSM, NTSM, FTSM] .")
        
        self.dechatter_method = kwargs.get('dechatter_method')
        func_sign = lambda s: 1.0 if s > 0 else -1.0
        # BOUND_LAYER, boundary layer, sign(s)
        if self.dechatter_method == "NONE":
            self.func_sign = func_sign
        elif self.dechatter_method == "HIGH_GAIN":
            self.DCM_HIGH_GAIN_epsilon = kwargs.get('DCM_HIGH_GAIN_epsilon')
            self.func_sign = lambda s: s / (abs(s) + self.DCM_HIGH_GAIN_epsilon)
        elif self.dechatter_method == "BOUND_LAYER":
            self.DCM_BOUND_LAYER_epsilon = kwargs.get('DCM_BOUND_LAYER_epsilon')
            self.DCM_BOUND_LAYER_ks = kwargs.get('DCM_BOUND_LAYER_ks')
            self.func_sign = lambda s: func_sign(s) if abs(s) > self.DCM_BOUND_LAYER_epsilon else s * self.DCM_BOUND_LAYER_ks
        else:
            raise ValueError(f"Unknown dechatter method, avaliable methods: [NONE, BOUND_LAYER, HIGH_GAIN] .")


        self.reaching_phase_method = kwargs.get('reaching_phase_method')
        # SAT, saturation, apply max control (saturate)
        # CONST, constant, given sign gain
        if self.reaching_phase_method == "SAT":
            self.RPM_SAT_umax = kwargs.get('RPM_SAT_umax')
            # TODO: saturation only needs to be applied at the final command
            self.func_ctrl_reach = lambda s: -1e6 * self.func_sign(s)
            self.func_ctrl = lambda x: -self.RPM_SAT_umax if self.func_sign(self.func_s(x)) > 0 else self.RPM_SAT_umax
            pass
        elif self.reaching_phase_method == "CONST":
            self.RPM_CONST_gain = kwargs.get('RPM_CONST_gain')
            self.func_ctrl_reach = lambda s: -self.RPM_CONST_gain * self.func_sign(s)
            self.func_ctrl = lambda x: self.func_ctrl_equi(x) + self.func_ctrl_reach(self.func_s(x))
            pass
        else:
            raise ValueError(f"Unknown reaching phase method, avaliable methods: [SAT, CONST] .")
    
    def stepAndGetControl(self, state, refstate, t):
        new_refstate = self.Ti @ refstate
        new_state = self.Ti @ state
        err_state = new_state - new_refstate
        self.s = self.func_s(err_state)
        return np.array([[self.func_ctrl(new_state - new_refstate)]])

class Observer:
    def updateGain(self, method, **kwargs):
        pass
    def updateSysAndGain(self, new_sys, method, **kwargs):
        pass
    def stepAndGetEstimation(self, total_control, measured_output):
        pass
    def resetState(self, init_state=None):
        pass
    pass

class LinearObserver(Observer):
    """
    Linear state observer.

    Attributes:
    - sys (LinearDistSys) linear system with disturbance
    - method (str)        gain computation method: 'place_multiple_poles', 'place_distinct_poles', 'lqr'.
    - wo, poles, Q, R     parameters for different gain methods
    
    Methods:
    - updateGain(method, **kwargs)                update gain
    - stepAndGetEstimation(control, output)     get estimated state
    """
    def __init__(self, sys, method, est_dist_required=False, **kwargs):
        self.est_dist_required = est_dist_required
        if self.est_dist_required:
            nxe = sys.nx + sys.nd # augmented state dimension
            Ae  = np.block([[sys.A, sys.Bd], 
                            [np.zeros((sys.nd, nxe))]])
            Be  = np.vstack([sys.Bu, np.zeros((sys.nd, sys.nu))])
            Cme = np.block([sys.Cm, np.zeros((sys.nm, sys.nd))])
            Coe = np.eye(nxe)
            self.insys = LinearDistSys(Ae, Be, None, Cme, Coe, sys.dt)
        else:
            self.insys = sys
        self.insys.resetState()
        self.refsys = sys
        self.method = method
        self.kwargs = kwargs
        self.updateGain(self.method, **kwargs)
        
    def resetState(self, init_state=None):
        if init_state is not None:
            if init_state.shape[0] == self.insys.nx:
                self.insys.resetState(init_state)
            elif init_state.shape[0] == self.refsys.nx:
                self.insys.resetState(np.vstack((init_state, np.zeros((self.refsys.nd, 1)))))
        else:
            self.insys.resetState()

    def updateSysAndGain(self, sys, method=None, **kwargs):
        self.refsys.updateSysParam(sys.A, sys.Bu, sys.Bd, sys.Cm, sys.Co, sys.dt)
        if self.est_dist_required:
            nxe = sys.nx + sys.nd # augmented state dimension
            Ae  = np.block([[sys.A, sys.Bd], 
                            [np.zeros((sys.nd, nxe))]])
            Be  = np.vstack([sys.Bu, np.zeros((sys.nd, sys.nu))])
            Cme = np.block([sys.Cm, np.zeros((sys.nm, sys.nd))])
            Coe = np.eye(nxe)
            self.insys.updateSysParam(Ae, Be, None, Cme, Coe, sys.dt)
        else:
            self.insys.updateSysParam(sys.A, sys.Bu, sys.Bd, 
                                                   sys.Cm, sys.Co, sys.dt)
        if method is None:
            # self.updateGain(self.method, **(self.kwargs))
            pass
        else:
            self.updateGain(method, **kwargs)

    def updateGain(self, method, **kwargs):
        """
        Compute feedback gain. Select required parameters depending on the chosen method.

        Parameters:
        - algorithm (str): method selection 
          ('place_multiple_poles', 'place_distinct_poles', 'lqr')
        - **kwargs: method-specific parameters
          - place_multiple_poles: wo, repeated poles
          - place_distince_poles: poles (array), distinct poles
          - lqr                 : Q, R, weighting matrices

        Returns:
        - (L, poles)
        """
        self.method = method
        self.kwargs = kwargs
        obsv_ref = np.linalg.matrix_rank(ct.obsv(self.refsys.A, self.refsys.Cm))
        if obsv_ref < self.refsys.nx:
            raise ValueError(f"The reference system\n A: {self.refsys.A}\n Cm: {self.refsys.Cm}\n is not observable.")
        elif self.est_dist_required:
            obsv_in = np.linalg.matrix_rank(ct.obsv(self.insys.A, self.insys.Cm))
            if obsv_in < self.insys.nx:
                raise ValueError(f"The extend system\n A: {self.insys.A}\n Cm: {self.insys.Cm}\n is not observable "
                                 f"(i.e. disturbance is not observable).")
        if method == 'place_multiple_poles':
            wo = kwargs.get('wo')
            if wo is None:
                raise ValueError("For 'place_multiple_poles' algorithm, 'wo' must be provided.")
            self.wo = wo
            self.poles = np.full(self.insys.nx, self.wo)
            self.Q = None
            self.R = None
            # self.L = ct.place_varga(self.insys.A.T, self.insys.Cm.T, self.poles).T
            # self.L = ct.place_acker(self.insys.A.T, self.insys.Cm.T, self.poles).T
            self.L = ct.place(self.insys.A.T, self.insys.Cm.T, self.poles).T
            # print(f"des poles: {self.poles}")
            print(f"act poles: {np.linalg.eigvals(self.insys.A - self.L @ self.insys.Cm)}")

        elif method == 'place_distinct_poles':
            poles = kwargs.get('poles_obs')
            if poles is None:
                raise ValueError("For 'place_distinct_poles' algorithm, 'poles_obs' must be provided.")
            self.wo = None
            self.poles = poles
            self.Q = None
            self.R = None
            # self.L = ct.place_varga(self.insys.A.T, self.insys.Cm.T, self.poles).T
            # self.L = ct.place_acker(self.insys.A.T, self.insys.Cm.T, self.poles).T
            self.L = ct.place(self.insys.A.T, self.insys.Cm.T, self.poles).T
            # print(f"des poles: {self.poles}")
            print(f"act poles: {np.linalg.eigvals(self.insys.A - self.L @ self.insys.Cm)}")

        elif method == 'lqr':
            Q = kwargs.get('Qo')
            R = kwargs.get('Ro')
            if Q is None or R is None:
                raise ValueError("For 'lqr' algorithm, both 'Qo' and 'Ro' must be provided.")
            self.wo = None
            self.Q = Q
            self.R = R
            self.L, _, self.poles = ct.lqr(self.insys.A.T, self.insys.Cm.T, self.Q, self.R)
            self.L = self.L.T
            print(f"act poles: {np.linalg.eigvals(self.insys.A - self.L @ self.insys.Cm)}")

        else:
            raise ValueError(f"Unknown method '{method}'. Use 'place_multiple_poles' "
                             f"or 'place_distinct_poles' or 'lqr'.")
        # print(f"observer L: {self.L}")
        # flatten row-wise and format as a string
        # flat_str = ', '.join(f'{x:.3e}' for x in self.L.ravel())
        # print(f'observer L: {{{flat_str}}}')
        lines = []
        for row in self.L:
            line = ', '.join(f'{x:.4e}' for x in row)  # scientific notation with 4 decimals
            lines.append(line)

        # merge into final string
        formatted = '{\n' + ',\n'.join(lines) + '\n}'
        print(f'observer L: {formatted}')
        return self.L, self.poles 
    
    def stepAndGetEstimation(self, total_control, measured_output):
        self.insys.step(control=total_control, state_add=self.L @ (measured_output - self.insys.Cm @ self.insys.x))
        if self.est_dist_required:
            return (self.insys.x[0:self.refsys.nx, 0].reshape(self.refsys.nx,1), 
                    self.insys.x[-self.refsys.nd:, 0].reshape(self.refsys.nd,1))
        else:
            return (self.insys.x, None)

# class LinearDiffer(LinearObserver):
#     """
#     Linear differentiator (difference + 2nd-order low-pass filter).
    
#     Methods:
#     - stepAndGetEstimation(signal)     get estimated derivative
#     """
class LinearDiffer(LinearObserver):
    """
    Linear differentiator (difference + 2nd-order low-pass filter).
    
    Methods:
    - stepAndGetEstimation(signal)     get estimated derivative
    """
    def __init__(self, dt:float, wf: float, order = 2):
        self.dt = dt
        self.wf = wf
        self.order = order
        A = np.zeros((order, order))
        A[:-1, 1:] = np.eye(order-1)
        for i in range(order):
            A[-1, i] = - math.comb(order, i) * math.pow(-wf, order-i)
        Bu = np.zeros((order, 1))
        Bu[-1, 0] = math.pow(-wf, order)
        Cm = np.zeros((1, order))
        Cm[0, 0] = 1.0
        Co = np.zeros((1, order))
        Co[0, 1] = 1.0
        self.sys = LinearDistSys(A, Bu, None, Cm, Co, dt)
        self.first_step = True

    def stepAndGetEstimation(self, newin):
        if self.first_step:
            self.first_step = False
            init_x = np.zeros((self.order, 1))
            init_x[0,0] = newin
            self.sys.resetState(init_x)
            return 0.0
        else:
            self.sys.step(np.array([newin]), None, None, None)
            return self.sys.x[1,0]
    # def __init__(self, dt, wo):
    #     A = np.array([[0.0, 1.0],
    #                   [0.0, 0.0]])
    #     Bu = np.array([[0.0],
    #                    [0.0]])
    #     Cm = np.array([[1.0, 0.0]])
    #     Co = np.array([[0.0, 1.0]])
    #     sys = LinearDistSys(A, Bu, None, Cm, Co, dt)
    #     self.last_meas = 0
    #     self.step_cnt = 0
    #     self.dt = dt
    #     self.wo = wo
    #     super().__init__(sys, method='place_multiple_poles', est_dist_required=False, wo=wo)
    #     # super().__init__(sys, method='lqr', est_dist_required=False, Qo=np.diag([0,10000]), Ro=np.diag([1]))
    
    # def stepAndGetEstimation(self, measured_signal):
    #     if self.step_cnt == 0:
    #         self.step_cnt += 1
    #         self.last_meas = measured_signal
    #         return 0.0
    #     elif self.step_cnt == 1:
    #         self.step_cnt += 1
    #         diff = (measured_signal - self.last_meas)/self.dt
    #         self.resetState(np.array([[measured_signal],[diff]]))
    #         return diff
    #     else:
    #         est_state, _ = super().stepAndGetEstimation(None, measured_signal)
    #         return est_state[1,0]
        
class LowPassFilterSs:
    def __init__(self, dt:float, wf: float, order = 2):
        self.dt = dt
        self.wf = wf
        self.order = order
        A = np.zeros((order, order))
        A[:-1, 1:] = np.eye(order-1)
        for i in range(order):
            A[-1, i] = - math.comb(order, i) * math.pow(-wf, order-i)
        Bu = np.zeros((order, 1))
        Bu[-1, 0] = math.pow(-wf, order)
        Co = np.zeros((1, order))
        Co[0, 0] = 1.0
        Cm = Co
        self.sys = LinearDistSys(A, Bu, None, Cm, Co, dt)
        self.first_step = True

    def stepAndGetOutput(self, newin):
        if self.first_step:
            self.first_step = False
            init_x = np.zeros((self.order, 1))
            init_x[0,0] = newin
            return newin
        else:
            self.sys.step(np.array([newin]), None, None, None)
            return self.sys.x[0,0]


class EcbcController(Controller):
    def __init__(self, sys: LinearDistSys,
                  controller: Controller, 
                  observer: Observer,
                  **kwargs):
        self.refsys = sys    # reference system
        self.intsys = None   # no internal system
        self.ctrler = controller
        self.obsver = observer
        self.kwargs = kwargs
        enable_comp = kwargs.get('enable_comp')
        comp_init_time = kwargs.get('comp_init_time')
        self.enable_comp = True if enable_comp is None else enable_comp
        self.comp_init_time = 0 if comp_init_time is None else comp_init_time
        self.last_step_sig = False
        self.step_x_equ = []
        self.step_u_equ = []
        self.resetState()
        # TODO
        lpf_Ts = kwargs.get('lpf_Ts')
        self.equi_lpf = LowpassFilter(dt=self.refsys.dt, Ts=lpf_Ts)
        
    def resetState(self, init_state=None, reset_record=True):
        # In controller properties: refsys stores nominal plant parameters; insys evolves as the internal system.
        # self.refsys.resetState(init_state) 
        self.obsver.resetState(init_state)
        self.ctrler.resetState(init_state)
        self.u_pre = np.zeros([self.refsys.nu, 1])
        if reset_record:
            self.t    = 0
            self.T    = np.zeros([1, 0])
            self.Yr   = np.zeros([self.refsys.no, 0])
            self.Ym   = np.zeros([self.refsys.nm, 0])
            self.Xest = np.zeros([self.refsys.nx, 0])
            self.Dest = np.zeros([self.refsys.nd, 0])
            self.Xequ = np.zeros([self.refsys.nx, 0])
            self.Uequ = np.zeros([self.refsys.nu, 0])
            self.U    = np.zeros([self.refsys.nu, 0])

    def updateSysAndGain(self, sys, ctr_method=None, obs_method=None, **kwargs):
        self.refsys.updateSysParam(sys.A, sys.Bu, sys.Bd, 
                                    sys.Cm, sys.Co, sys.dt)
        self.obsver.updateSysAndGain(sys, obs_method, **kwargs)
        self.ctrler.updateSysAndGain(sys, ctr_method, **kwargs)
    
    def updateGain(self, ctr_method, obs_method, **kwargs):
        self.obsver.updateGain(obs_method, **kwargs)
        self.ctrler.updateGain(ctr_method, **kwargs)
    
    def enableComp(self, enable_comp):
        self.enable_comp = enable_comp

    def stepAndGetControl(self, ym_cur, yo_ref, t, u_act=None, enable_step_cmp=False, step_sig=False):
        self.yo_ref = yo_ref
        self.ym_cur = ym_cur
        if u_act is not None:
            self.x_est, self.d_est = self.obsver.stepAndGetEstimation(u_act, self.ym_cur)
        else:    
            self.x_est, self.d_est = self.obsver.stepAndGetEstimation(self.u_pre, self.ym_cur)
        if self.enable_comp and t > self.comp_init_time:
            self.x_equ, self.u_equ = self.refsys.findEquilibrium(self.d_est, self.yo_ref)
        else:
            self.x_equ, self.u_equ = self.refsys.findEquilibrium(np.zeros((self.refsys.nd, 1)), self.yo_ref)
        
        if enable_step_cmp:
            if step_sig and not self.last_step_sig:
                self.step_x_equ = self.x_equ
                self.step_u_equ = self.u_equ
            # self.step_x_equ[0] = self.equi_lpf.stepAndGetOutput(self.step_x_equ[0])
            self.u_cur = self.step_u_equ + self.ctrler.stepAndGetControl(self.ym_cur, self.step_x_equ, t)
            self.last_step_sig = step_sig
            pass
        else:
            self.x_equ[0] = self.equi_lpf.stepAndGetOutput(self.x_equ[0])
            # self.u_cur = self.u_equ + self.ctrler.stepAndGetControl(self.x_est, self.x_equ, t)
            # TODO reduced-order observer
            # self.u_cur = self.u_equ + self.ctrler.stepAndGetControl(self.x_est, self.x_equ, t)
            self.u_cur = self.u_equ + self.ctrler.stepAndGetControl(self.ym_cur, self.x_equ, t)
        self.u_pre = self.u_cur

        self.t   += self.refsys.dt
        self.T    = np.hstack((self.T, np.array([[self.t]])))
        self.Yr   = np.hstack((self.Yr, self.yo_ref))
        self.Ym   = np.hstack((self.Ym, self.ym_cur))
        self.Xest = np.hstack((self.Xest, self.x_est))
        self.Dest = np.hstack((self.Dest, self.d_est))
        self.Xequ = np.hstack((self.Xequ, self.x_equ))
        self.Uequ = np.hstack((self.Uequ, self.u_equ))
        self.U    = np.hstack((self.U, self.u_cur))

        return self.u_cur

    def plotResult(self, state=True, control=True, disturbance=True, output=True):
        def pad1(Z):
            return np.hstack((Z, Z[:,-1]))
        def plotMline(T, Z, label_prefix=""):
            for i in range(Z.shape[0]):
                plt.plot(T[0, :], Z[i, :], label=label_prefix+f'line {i+1}')
        if state:
            plt.figure()
            plotMline(self.T, self.Xest, "estimated ")
            plotMline(self.T, self.Xequ, "equilibrium ")
            plt.legend()
            plt.title("state")
        if control:
            plt.figure()
            plotMline(self.T, self.U, "actual ")
            plotMline(self.T, self.Uequ, "equilibrium ")
            plt.legend()
            plt.title("control")
        if disturbance:
            plt.figure()
            plotMline(self.T, self.Dest, "estimated ")
            plt.legend()
            plt.title("disturbance")
        if output:
            plt.figure()
            plotMline(self.T, self.Ym, "measured ")
            plotMline(self.T, self.Yr, "reference ")
            plt.legend()
            plt.title("output")

class GesobcController(EcbcController):
    def __init__(self, sys: LinearDistSys, ctr_method: str, obs_method: str, **kwargs):
        """
        add_control_dist: add disturbance on control input to match disturbance
        """
        # if add_control_dist:
        #     # TODO: check the rank of [Bu, Bd]
        #     Bd_new = np.hstack((sys.Bd, sys.Bu))
        #     self.refsys = LinearDistSys(sys.A, sys.Bu, Bd_new, sys.Cm, sys.Co, sys.dt)
        # else:
        #     self.refsys = sys
        self.refsys = sys
        ctrler = LinearController(self.refsys, ctr_method, **kwargs)
        obsver = LinearObserver(self.refsys, obs_method, est_dist_required=True, **kwargs)
        super().__init__(self.refsys, ctrler, obsver, **kwargs)

# class Memo:
#     def __init__(self, dt):
#         self.p = 0
#         self.pl = 0
#         self.v = 0
#         self.vl = 0
#         self.a = 0
#         self.dt = dt
#         self.isFirstUpdate = True
#         self.isSecondUpdate = True
    
#     def pos(self):
#         return self.p
    
#     def vel(self):
#         return self.v
    
#     def update(self, pn):
#         self.pl = self.p
#         self.p = pn
#         self.vl = self.v
#         if self.isFirstUpdate:
#             self.v = 0
#             self.a = 0
#             self.isFirstUpdate = False
#         elif self.isSecondUpdate:
#             self.v = (self.p - self.pl)/self.dt
#             self.a = 0
#             self.isSecondUpdate = False
#         else:
#             self.v = (self.p - self.pl)/self.dt
#             self.a = (self.v - self.vl)/self.dt
            
#     def reset(self):
#         self.p = 0
#         self.pl = 0
#         self.v = 0
#         self.vl = 0
#         self.a = 0
#         self.isFirstUpdate = True
#         self.isSecondUpdate = True

class Memo:
    def __init__(self, dt, diff_method='linear_obsv', **kwargs):
        self.p = 0
        self.pl = 0
        self.v = 0
        self.dt = dt
        self.isFirstUpdate = True
        self.diff_method = diff_method
        if diff_method == 'linear_obsv':
            wo = kwargs.get('wo')
            if wo is None:
                raise ValueError("For 'linear_obsv' differentiator, 'wo' must be provided.")
            self.differ = LinearDiffer(dt, wo)
    
    @property
    def pos(self):
        return self.p
    
    @property
    def vel(self):
        return self.v
    
    def update(self, pn):
        self.pl = self.p
        self.p = pn
        if self.diff_method == 'linear_obsv':
            self.v = self.differ.stepAndGetEstimation(self.p)
        elif self.diff_method == 'diff':
            if self.isFirstUpdate:
                self.v = 0
                self.isFirstUpdate = False
            else:
                self.v = (self.p - self.pl)/self.dt
        else:
            raise ValueError(f"Unknown diff method '{self.diff_method}'. Please use 'linear_obsv' or 'diff'.")

            
    def reset(self):
        self.p = 0
        self.pl = 0
        self.v = 0
        self.isFirstUpdate = True
    

        

    
        
