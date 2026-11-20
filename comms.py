# =============================================================================
# comms.py - COMUNICACION CON EL DASHBOARD (por stdin / stdout sobre BLE)
# =============================================================================
# Entrada: read_command() arma una linea de comando sin bloquear el lazo de control.
#          Metodo oficial de Pybricks: usys.stdin + uselect.poll
#          (ver el tutorial "Hub to PC Communication").
# Salida:  send_telemetry() / send_battery() imprimen lineas de texto que el
#          dashboard interpreta. El formato de las lineas esta en la cabecera de
#          main_rs.py y en dashboard.html.
# =============================================================================
from usys import stdin
from uselect import poll

from hardware import battery

_poller = poll()
_poller.register(stdin)
_inbuf = ''


def read_command():
    """Arma una linea completa desde stdin sin bloquear. Devuelve la linea
    (sin \\n) o None si todavia no hay una completa."""
    global _inbuf
    while _poller.poll(0):
        ch = stdin.buffer.read(1)
        if not ch:
            break
        b = ch[0]
        if b == 10 or b == 13:     # \n o \r  -> fin de linea
            if not _inbuf:         # \r\n o linea vacia: no emitir doble
                continue
            line = _inbuf
            _inbuf = ''
            return line.strip()
        else:
            _inbuf += chr(b)       # ASCII: chr() evita bytes(list) y decode()
    return None


def send_battery():
    """Estado de la bateria al dashboard: 'BAT mv,ma'."""
    mv, ma = battery()
    print("BAT {},{}".format(mv, ma))


def send_telemetry(t, pos_ref, pos_veh, pos_err, pos_cmd,
                   st_ref_deg, st_veh, st_err, st_cmd,
                   x, y, pP, pI, sP, sI, ey, ss):
    """Una linea T de telemetria (17 campos, ver cabecera de main_rs.py)."""
    print("T {:.2f},{:.2f},{:.2f},{:.2f},{},{:.2f},{:.2f},{:.2f},{},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.2f},{:.3f}".format(
        t, pos_ref, pos_veh, pos_err, pos_cmd,
        st_ref_deg, st_veh, st_err, st_cmd,
        x, y, pP, pI, sP, sI, ey, ss))
