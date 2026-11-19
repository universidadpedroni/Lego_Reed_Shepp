# =============================================================================
# smc_control.py - Controladores por MODOS DESLIZANTES (SMC) para el AUTO LEGO
# =============================================================================
# Modulo puro (solo math): corre igual en MicroPython (umath) y en CPython, asi
# el mismo codigo que se simula en sim_smc.py es el que va al hub.
#
# CONVENCION (matematica, la de la odometria del dashboard):
#   - rumbo theta: CCW positivo (= -heading del IMU, que es "compass" CW+)
#   - kappa (curvatura) > 0: gira a la izquierda
#   - e_y > 0: el auto esta a la izquierda de la referencia
#   - gear = +1 adelante, -1 marcha atras
# El signo del comando de direccion que sale de aca es matematico (+ = izquierda).
# El firmware lo manda con steer(-pct) porque Pybricks/IMU usan compass CW+.
#
# Referencia teorica y deduccion completa: SMC/SMC_diseno.md
# =============================================================================
try:
    import umath as _m
except ImportError:
    import math as _m


def sat(x):
    """Saturacion unitaria: sign(x) suavizado en la capa limite |x|<1."""
    if x > 1.0:
        return 1.0
    if x < -1.0:
        return -1.0
    return x


def wrap_pi(a):
    """Lleva un angulo a [-pi, pi)."""
    while a >= _m.pi:
        a -= 2.0 * _m.pi
    while a < -_m.pi:
        a += 2.0 * _m.pi
    return a


class LateralSMC:
    """SMC de DIRECCION: regula juntos el error lateral e_y y el de rumbo e_theta.

    Rumbo geometrico: e_g = e_theta - kappa_ref*e_s   (rumbo del camino en el punto
                      mas cercano; sin esto el atraso a lo largo de la traza e_s
                      sesga el error lateral en las curvas: e_y ~ kappa*e_s/lam)
    Superficie:      s = e_g + lam * gear * e_y
    Equivalente:     kappa_eq  = kappa_ref*cos(e_g)/(1 - kappa_ref*e_y) - gear*lam*sin(e_g)
    Control:         kappa_cmd = kappa_eq - gear * (eta*sat(s/phi) + ki*z)
    Direccion:       delta = atan(L*kappa_cmd)  ->  pct = 100*delta/delta_max

    Con eta > (rho*|kappa_eq|max + d)/(1-rho) se cumple s*ds/dt <= -|v|*eta*(1-rho)*|s|
    fuera de la capa limite (rho = error relativo de ganancia de direccion,
    d = sesgo de curvatura). Sobre s=0: de_y/dt = -|v|*lam*e_y (converge sin
    importar el sentido de marcha porque la superficie lleva el factor 'gear').
    """

    def __init__(self, L, delta_max_deg, lam, eta, phi, ki=0.0):
        self.L = L
        self.delta_max = _m.radians(delta_max_deg)
        self.lam = lam      # [1/cm]  polo de e_y sobre la superficie: |v|*lam [1/s]
        self.eta = eta      # [1/cm]  ganancia conmutada (en curvatura)
        self.phi = phi      # [rad]   espesor de la capa limite
        self.ki = ki        # [1/(cm*rad*cm)] integral opcional dentro de la capa
        self.z = 0.0
        self.s = 0.0
        self.pct_eq = 0.0    # % de direccion aportado por el control equivalente
        self.pct_sw = 0.0    # % aportado por el termino conmutado (+ integral)

    def reset(self):
        """Llamar al arrancar cada segmento (el signo de la superficie cambia
        con el gear y el integrador no debe arrastrarse)."""
        self.z = 0.0
        self.s = 0.0
        self.pct_eq = 0.0
        self.pct_sw = 0.0

    def update(self, gear, e_theta, e_y, e_s, kappa_ref, ds_abs, kappa_ff=None):
        """gear: +1/-1. e_theta [rad], e_y y e_s [cm] (atraso/adelanto a lo largo de la traza), kappa_ref [1/cm] firmado,
        ds_abs [cm] = arco recorrido en este tick (|v|*dt), para el integrador,
        kappa_ff [1/cm] = curvatura anticipada para el feedforward (None = kappa_ref).
        Devuelve el comando de direccion en % (+ = izquierda, saturado a +-100)."""
        g = 1.0 if gear >= 0 else -1.0
        e_g = wrap_pi(e_theta - kappa_ref * e_s)
        s = e_g + self.lam * g * e_y
        if kappa_ff is None:
            kappa_ff = kappa_ref      # anticipacion opcional (preview) de la curvatura
        k_eq = kappa_ff * _m.cos(e_g) / (1.0 - kappa_ref * e_y) - g * self.lam * _m.sin(e_g)
        sw = self.eta * sat(s / self.phi)
        if self.ki > 0.0 and abs(s) < self.phi:        # integra solo dentro de la capa
            self.z += s * ds_abs
            z_max = 0.5 * self.eta / self.ki
            if self.z > z_max:
                self.z = z_max
            elif self.z < -z_max:
                self.z = -z_max
        k_cmd = k_eq - g * (sw + self.ki * self.z)
        delta = _m.atan(self.L * k_cmd)
        pct = 100.0 * delta / self.delta_max
        if pct > 100.0:
            pct = 100.0
        elif pct < -100.0:
            pct = -100.0
        self.s = s
        self.pct_eq = 100.0 * _m.atan(self.L * k_eq) / self.delta_max
        self.pct_sw = pct - self.pct_eq          # eq + sw = comando (para la telemetria)
        return pct


class SpeedSMC:
    """SMC de TRACCION: sigue posicion y velocidad de la referencia con un solo
    comando de potencia.

    Superficie:  s_v = (v_ref - v) + c*(pos_ref - pos)       (grado relativo 1)
    Modelo nominal del conjunto motor+auto:  dv/dt = (K*u - v)/tau
    Control:     u = (v + tau*(a_ref + c*(v_ref - v)))/K + eta*sat(s_v/phi)

    K [cm/s por %] y tau [s] se identifican con un escalon de potencia (ver el
    .md). eta cubre el error de K y tau (bateria, carga): condicion
    eta*K/tau > cota del termino no modelado.
    """

    def __init__(self, K, tau, c, eta, phi, u_max=100.0):
        self.K = K
        self.tau = tau
        self.c = c          # [1/s]  polo del error de posicion sobre la superficie
        self.eta = eta      # [%]    ganancia conmutada
        self.phi = phi      # [cm/s] capa limite
        self.u_max = u_max
        self.s = 0.0

    def reset(self):
        self.s = 0.0

    def update(self, e_pos, v_ref, v_meas, a_ref=0.0):
        e_v = v_ref - v_meas
        s = e_v + self.c * e_pos
        u_eq = (v_meas + self.tau * (a_ref + self.c * e_v)) / self.K
        u = u_eq + self.eta * sat(s / self.phi)
        if u > self.u_max:
            u = self.u_max
        elif u < -self.u_max:
            u = -self.u_max
        self.s = s
        return u
