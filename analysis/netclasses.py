"""Net classes, part roles and placement islands for a board.

Nets get a class: ground, switch node, fast signal (rank 2..5), current / supply (high, medium, low), analog
(sensitive, rank 1..3) or plain signal (GPIO, I2C/UART/PWM). Currents are estimated from the circuit (regulators,
LEDs, connectors) unless given. Parts get a role (chip = island anchor, connector, mechanical, small part) and a
function derived from the net graph (decoupling, crystal load, series resistor, pull-up, divider, feedback, shunt,
switching inductor, bootstrap, ...). Each anchor forms an island with its small parts; decoupling capacitors know the
supply pin they serve.

usage: python3 analysis/netclasses.py <board.json> <netclasses.json> [design_rules=design_rules.json] [specs=specs.json]
Writes <netclasses.json> plus a readable summary next to it (.txt).

specs.json (optional, all keys optional) overrides the automatic decisions, e.g. net classes by regex on the
net name, forced ground nets, extra anchors, part-to-island assignments, fixed parts, rough positions for the placer,
currents per part or pad, voltages and switching frequencies. The task generator learning/tasks.py writes such a
file for every generated board.
"""
import json, sys, re, os, fnmatch, collections, math

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from common.values import capacity, resistance, prefix, tokens, base_name, package, type_

CONNECTOR_PFX = {'J', 'CN', 'CON', 'P', 'USB', 'HDR', 'X', 'XS'}
CONNECTOR_HINT = re.compile(r'conn|jst|header|fpc|ffc|usb|tag.?connect|tc20[35]0|molex|hirose|terminal|socket|pinhead|pin_head|_ph_|_sh_|\bph\b', re.I)
MECH_FP = re.compile(r'MountingHole|Mounting_Hole|Fiducial|LOGO|Logo|OSHW|TestPoint|Measurement_Point', re.I)


OPERATE_HINT = re.compile(r'(^|[^A-Z])LED|WS281|SK68|APA10|BUTTON|SWITCH|TASTER|BUZZER|BEEPER|SPEAKER|DISPLAY|OLED|LCD|POTENTIOMETER|ENCODER', re.I)
MECH_PFX = {'H', 'MH', 'FID', 'REF', 'LOGO', 'TP', 'SW', 'BTN', 'S', 'BT', 'MK', 'LS', 'K', 'G', 'KIBUZZARD'}
PASSIVE_PFX = {'R', 'C', 'L', 'FB', 'D', 'F', 'Y', 'Q', 'TVS', 'RN', 'RV', 'VR', 'NTC', 'PTC', 'FL', 'Z', 'ESD', 'DS', 'NT', 'JP', 'SJ', 'RA', 'CA'}
CHIP_PFX = {'U', 'IC', 'MCU', 'DAC', 'ADC', 'OPV', 'OP', 'AMP', 'MOD', 'CPU', 'MEM', 'LDO', 'BUCK', 'REG', 'PMIC'}
ANALOG_CHIP = re.compile(r'OPA|OPV|RS85|LTC|LT1|AD8|ADA|MCP6|TLV9|TLV2|LMV|LM35|LM32|NE55|OP0|THS|INA|ADS|MAX11|TAD5|TLV320|PCM18|PCM51|CS53|CS43|AK5|AK4|PI3A|TS5A|ADG|DG4|MAX46', re.I)

GPIO_T = {'EN', 'ENABLE', 'SHDN', 'SHDNZ', 'RST', 'RESET', 'NRST', 'NRESET', 'PG', 'PGOOD', 'GPIO', 'INT', 'IRQ',
          'BOOT', 'WKP', 'WAKE', 'READY', 'CONNECTED', 'ASSERT', 'MD', 'NMI', 'SEL', 'CTRL', 'BUTTON', 'KEY', 'MODE', 'ALERT'}
CLK_T = {'CLK', 'SCK', 'SCLK', 'BCLK', 'BCK', 'MCLK', 'CLKIN', 'CLKOUT', 'MCO'}
XTAL_T = {'XTAL', 'XIN', 'XOUT', 'EXTAL', 'XCIN', 'XCOUT', 'OSC', 'OSCIN', 'OSCOUT', 'XI', 'XO', 'XTAL1', 'XTAL2', 'XC1', 'XC2'}
HS_IF = ('XSPI', 'QSPI', 'OSPI', 'SDIO', 'SDMMC', 'MMC', 'EMMC', 'HYPERBUS')
VHS_IF = ('MIPI', 'HDMI', 'LVDS', 'ETH', 'RGMII', 'RMII', 'PCIE', 'SATA', 'DDR', 'SDRAM', 'MII')
AUD_IF = ('I2S', 'SAI', 'TDM', 'PCM', 'PDM')
SWD_T = {'SWCLK', 'SWDIO', 'SWO', 'TCK', 'TMS', 'TDI', 'TDO', 'TRST', 'SWD', 'JTAG'}
SLOW_T = {'SCL', 'SDA', 'I2C', 'TX', 'RX', 'TXD', 'RXD', 'UART', 'CTS', 'RTS', 'PWM', 'WS2812', 'CAN', 'ONEWIRE', '1WIRE', 'DIN', 'DOUT',
          'DTR', 'DSR', 'DCD'}
ON_T3 = {'AIN', 'VREF', 'REF', 'VCM', 'BIAS', 'MICBIAS', 'MIC', 'NTC', 'INP', 'INN', 'INM'}
ON_T2 = {'FB', 'SENSE', 'SNS', 'ISENSE', 'VSENSE', 'ISNS', 'VSNS', 'TRIM', 'HR', 'CTF', 'ANALOG', 'AUDIO'}
PWR_START = ('VIN', 'VBUS', 'VBAT', 'VSYS', 'VCC', 'VDD', 'AVDD', 'DVDD', 'IOVDD', 'VCORE', 'VLED', 'PVIN', 'VMOT', 'VOUT', 'VPP',
             'DREG', 'AREG', 'VREG', 'PWR', 'POWER', 'VSUP', 'SUPPLY')
PWR_HIGH = ('VIN', 'VBUS', 'VBAT', 'VSYS', 'PVIN', 'VLED', 'VMOT', 'PWR', 'POWER')
PWR_SMALL = ('AVDD', 'VREF', 'DREG', 'AREG', 'OPV', 'ADC', 'DAC', 'ANALOG', 'AUDIO', 'BIAS')

WT = {'sw': 3.0, 'fast': 2.0, 'analog': 2.0, 'current_val': 1.5, 'signal': 1.0, 'supply': 0.3}
GPIO_NR = re.compile(r'GPIO\d+|IO\d{1,2}|P[A-K]\d{1,2}')
HF_COIL_UNTIL = 0.2e-6
CONTROL_R_FROM = 1000.0
SMALL_SIGNAL_Q = re.compile(r'SOT-?23(?!-?[5-8])|SC-?70|SOT-?323|SOT-?523|SOT-?416|TO-?92', re.I)
TWO_POLE_T = ('L', 'R', 'C', 'F', 'Y')
DRIVER_CHIP = re.compile(r'DRV8|DRV10|TB6612|TB67|A4988|A3967|TMC2|L29[38]|BTS7|BTN79|VNH\d|MP6500|ULN200|ULN280|TPA31|TPA20|PAM84|MAX9835|MAX9735|MAX9744', re.I)
USB_FS = re.compile(r'ESP32-?S[23]|ESP32-?C[36]|ESP32-?H2|RP2040|RP2350|ATMEGA\d*U\d|AT90USB|SAMD\d|ATSAMD|SAML2|STM32[FLG][0134]|STM32WB|'
                    r'CH32V|CH55\d|CH34[013]|CH910\d|CP210\d|FT23[01]X|FT232R|PL2303|NRF52840|NRF5340|PIC32MX|PIC18F\d+K50|EFM32|LPC11U|GD32F[13]|GD32VF', re.I)
USB_HS = re.compile(r'USB25[01]\d|USB2422|USB2640|USB33[0-4]\d|FT232H|FT2232H|FT4232H|FT60[01]|CY7C68013|CYUSB|FE1\.1|GL85[02]|GL3520|VL81\d|'
                    r'TUSB\d|LAN95\d\d|LAN78\d\d|AX88179|RTL8153|CY7C65', re.I)
DRIVER_OFF = re.compile(r'OUT|^[AB]O\d|^[AB]\d?OUT|MOTOR|^M\d|SPK|^[LR]OUT|^SW', re.I)

def type_part(ref, f):
    t = type_(ref, f.get('fp', ''))
    if t in TWO_POLE_T and (f.get('n_pads') or 0) > 6 and type_('X', f.get('fp', '')) != t: return 'U'
    return t








SHUNT_U = 0.1
SHUNT_P = {'0201': 0.05, '0402': 0.063, '0603': 0.1, '0805': 0.125, '1206': 0.25, '1210': 0.5, '1812': 0.5, '2010': 0.75, '2512': 1.0,
           '2725': 2.0, '2728': 2.0, '3920': 3.0}
SHUNT_P_OTHERWISE = 2.0
SHUNT_R_MIN = 1e-3
FUSE_OPERATION = 0.6
CONNECTOR_A = (
    (r'XT90', 45.0, None), (r'XT60', 30.0, None), (r'XT30', 15.0, None), (r'USB.?C|TYPE.?C', 0.75, 3.0),
    (r'MICRO.?USB|USB.?MICRO|MICROB', 1.0, 1.8), (r'MINI.?USB|USB.?MINI', 0.5, 0.5), (r'USB', 0.5, 1.5),
    (r'BARREL|DC.?JACK|JACK.?DC|HOHL|PJ-0\d\d', 3.0, 3.0), (r'TERMINAL|SCREW|KLEMME|PHOENIX|WAGO|KF\d', 10.0, None),
    (r'MINI.?FIT|5566', 9.0, None), (r'MICRO.?FIT|43045|43650', 5.0, None), (r'JST.?VH|_VH_|\bVH\b', 10.0, None),
    (r'JST.?XH|_XH_|\bXH\b|B\d+B-XH', 3.0, None), (r'JST.?EH|_EH_|\bEH\b', 3.0, None), (r'JST.?PH|_PH_|\bPH\b|-PH-', 2.0, None),
    (r'JST.?PA\b|_PA_|\dB-PASS', 3.0, None), (r'JST.?ZH|_ZH_|\bZH\b', 1.0, None), (r'JST.?SH|_SH_|\bSH\b|-SH-', 1.0, None), (r'JST.?GH|_GH_|\bGH\b', 1.0, None),
    (r'FPC|FFC', 0.5, None), (r'TAG.?CONNECT|TC20[35]0', 0.1, None), (r'1\.27|1,27|P1\.27', 1.0, None),
    (r'PINHEADER|PIN_HEADER|HEADER|2\.54|PINSOCKET', 3.0, None))
CONNECTOR_OTHERWISE_A = 1.0
REGULATOR_A = (
    (r'TPS548A28|TPS548B22', 15.0), (r'LTC3833|LTC3866', 20.0), (r'TPS56C215|TPS56C230', 12.0), (r'TPS568230', 8.0), (r'TPS56637', 6.0),
    (r'TPS6293\d|TPS5430|TPS5450|LM2596|MP1584|TPS54331|TPS563\d{3}|TPS62130|MP2307|LMR3361|LMR33630', 3.0),
    (r'AP6320\d|SY8089|TPS6208\d|TPS62A0\d|MP2359|RT8059|MP2451', 2.0), (r'LM2576|LM2575|XL1509', 2.0),
    (r'AMS1117|LM1117|NCP1117|LD1117|AZ1117', 1.0), (r'TLV757|AP7361|LP38692', 1.0), (r'^L?78M', 0.5), (r'^L?78L|^79L', 0.1),
    (r'^L?78\d|^79\d', 1.0), (r'TLV755|TLV733|AP2112|RT9013|ME6211|LP5907|TLV702', 0.5), (r'AP2210|LP2985|MIC5219', 0.3),
    (r'XC6206|XC6220|HT7[35]\d\d|MCP1700|MCP1702|MCP1703|TPS7A02|TLV713', 0.25))
REGULATOR_HOUSING_A = ((r'SOT-?223|DPAK|TO-?252|TO-?263|D2PAK|TO-?220', 1.0), (r'SOT-?23|SC-?70|SOT-?353|DFN|SON|X2SON', 0.3))
CONSUMER_A = (
    (r'ESP32|ESP-?WROOM|ESP-?WROVER|ESP32-?S3|ESP32-?C\d', 0.35), (r'ESP8266|ESP-?12', 0.25), (r'RP2040|RP2350', 0.05),
    (r'STM32H7|STM32F7|IMXRT|MIMXRT', 0.3), (r'STM32|GD32|CH32|SAMD|ATSAM|LPC\d', 0.1), (r'NRF52|NRF53|CC26|CC13|EFR32', 0.02),
    (r'ATMEGA|ATTINY|PIC\d', 0.02), (r'W5500|LAN87|KSZ|DP83', 0.15), (r'SX12\d\d|RFM9|LLCC68', 0.12),
    (r'WS281|SK68|APA10|SK98', 0.04), (r'DRV8|TB6612|A4988|TMC2|L298|BTS7', 1.0), (r'TPA31|PAM84|MAX9835|MAX9735', 0.5))
CHIP_PADS_A = ((8, 0.005), (20, 0.01), (64, 0.05), (10 ** 6, 0.1))
OPV_A = 0.002
LED_A = 0.02
LED_POWER_A = ((r'XHP|COB', 1.0), (r'3535|XP-?[EGLM]\d?|XM-?L|OSLON|LUXEON|SST-?\d', 0.35), (r'3030|3020', 0.15))
FAN = re.compile(r'FAN|L(UE|Ü)FTER|MOTOR|PUMP', re.I)
FAN_A = 0.25
SIGNAL_I_MIN = 0.1


def signal_currents(fps, refs_on, TYPE, NET, GNDS):
    out = {}
    for n in sorted(refs_on):
        k = NET.get(n) or {}
        if n in GNDS or k.get('category') in ('current_val', 'supply', 'sw') or k.get('current_A') is not None: continue
        cand = []
        for r in sorted(refs_on[n]):
            f = fps.get(r) or {}; txt = '%s %s' % (f.get('value', '') or '', f.get('fp', '') or '')
            if TYPE.get(r) == 'LED' or prefix(r) == 'LED' or re.search(r'(^|[^A-Z])LED', txt, re.I):
                a = table(LED_POWER_A, txt)
                if a: cand.append((a, 'led_power %s' % r))
            elif TYPE.get(r) == 'R':
                R = resistance(f.get('value', ''))
                if R and SHUNT_R_MIN <= R <= 1.0:
                    m = re.search(r'(0201|0402|0603|0805|1206|1210|1812|2010|2512)', f.get('fp', '') or '')
                    cand.append((min(SHUNT_U / R, math.sqrt((SHUNT_P[m.group(1)] if m else SHUNT_P_OTHERWISE) / R)), 'shunt %s' % r))
            elif (prefix(r) in ('J', 'P', 'CN', 'X') and FAN.search(txt)) or (FAN.search(n) and prefix(r) in ('J', 'P', 'CN', 'X', 'M')):
                cand.append((FAN_A, 'fan %s' % r))
        if cand:
            a, q = max(cand)
            if a >= SIGNAL_I_MIN: out[n] = (round(a, 4), q)
    return out
R_MAX_A = 0.01
TRANSISTOR_A = ((r'SOT-?23|SC-?70|SOT-?323|SOT-?523', 0.5), (r'SOT-?223|SO-?8|SOIC-?8|DFN|PowerPAK|TDSON|SON', 2.0),
                (r'DPAK|TO-?252|TO-?263|D2PAK|TO-?220|TO-?247', 5.0))



FIXED_V = ((r'^(?:L|LM|MC|UA|KA|AN|NJM)?78(?:L|M)?(\d\d)', '78'), (r'AP632\d([35])', 'ap'),
          (r'(?:LM2596|LM2576|LM2575|LM2574|LM2594|XL1509|LM2678|LM2679)[A-Z]*-(\d{1,2}(?:\.\d)?)', 'volt'),
          (r'(?:AMS1117|LM1117|AZ1117|NCP1117|LDL?1117|SPX1117|TLV1117|AP2112|AP2210|AP7361|AP7333|RT9013|RT9193|MIC5219|MIC5205|LP5907|'
           r'LP2985|SPX3819|MCP170[0-3]|MCP1711|MCP1825|ME6211|ME6206|XC6206|XC6210|XC6220|HT7[1357]|TLV7\d\d|TPS7\d\d|ADP\d{3,4})'
           r'[A-Z]{0,5}-?(\d{1,2}\.\d{1,2}|\d{2}|\d(?![\d.]))', 'ldo'))
REGULATOR_VREF = ((r'TPS6293\d|TPS54331|TPS6213\d|TPS6214\d|TPS6215\d|TPS56[23]\d{3}|MP1584|MP2451|MP2315|AP632\d0|AP3211', 0.8),
               (r'TPS5430|TPS5450|TPS5410', 1.221), (r'LM2596|LM2576|LM2575|LM2574|LM2594|XL1509|XL401[56]|LM267[89]', 1.23),
               (r'MP2307', 0.925), (r'MP2359', 0.81), (r'SY8089|SY8088|SY8113|SY8120|RT8059|RT8008|MT3608|TLV6256\d|TPS62A0\d', 0.6),
               (r'LMR3363\d|LMR3361\d|LMR1430', 1.0), (r'AMS1117|LM1117|NCP1117|AZ1117|LDL?1117|LM317|LM108[56]', 1.25),
               (r'MCP1640|MCP16251', 1.21), (r'TPS6102[23]', 0.595), (r'TPS630[02]\d', 0.5))
TVS_V = ((r'(?:SMAJ|SMBJ|SMCJ|SMDJ|SMF|SMLJ)(\d+(?:\.\d+)?)', 1.0),
         (r'(?:P4SMA|P6SMB|1\.5SMC|P6KE|1\.5KE|P4KE)(\d+(?:\.\d+)?)', 0.85))
ZENER_V = r'(?:BZX|BZT|BZV|MMSZ|MMBZ|ZMM|ZPD)\w*?(\d+)V(\d)?'
TVS_OTHERWISE_V = r'(?<![\dA-Z.])(\d{1,2}(?:\.\d)?)V\b'
PD_CHIP = re.compile(r'CH22[14]|IP272\d|IP2716|HUSB238|STUSB45|FUSB302|TPS6598|AP33772|CYPD3177|SW2303', re.I)
RAILS_V = (1.8, 2.5, 3.3, 5.0, 9.0, 12.0, 19.0, 24.0, 36.0, 48.0)


IPC_DT = 10.0
IPC_K_OUTER = 0.048
COPPER_OUTER = 0.035
COPPER_INNER = 0.0175
INNER_FACTOR = 2.3
W_CURRENT_MAX = 2.5
RHO_CU = 1.72e-5
BUDGET_DETOUR = 1.3


def ipc_width(I, dT=IPC_DT, t=COPPER_OUTER):
    from analysis.current_width import ipc2221
    return ipc2221(I, dT, t)


def fixed_voltage(amount):
    w = (amount or '').upper()
    if 'ADJ' in w: return None
    for pat, category in FIXED_V:
        m = re.search(pat, w)
        if not m: continue
        c = m.group(1)
        if category == '78': return 3.3 if c == '33' else (2.5 if c == '25' else float(int(c)))
        if category == 'ap': return {'3': 3.3, '5': 5.0}[c]
        v = float(c) if category == 'volt' or '.' in c or len(c) == 1 else int(c[0]) + int(c[1]) / 10.0
        return v if v >= 0.8 else None
    return None


def rail(v):
    s = [x for x in RAILS_V if x <= v * 1.02]
    return s[-1] if s else None


def value_ampere(txt):
    m = re.search(r'(\d+(?:[.,]\d+)?)\s*(m?)A(?![a-z])', txt or '')
    if not m: return None
    v = float(m.group(1).replace(',', '.')) * (1e-3 if m.group(2) else 1.0)
    return v if v > 0 else None


def fuse_ampere(txt):
    a = value_ampere(txt)
    if a: return a
    m = re.search(r'\d{4}L(\d{3})|MF-?[A-Z]{1,5}(\d{3})|SMD[A-Z]{0,2}(\d{3})', (txt or '').upper())
    if not m: return None
    v = int(next(g for g in m.groups() if g)) / 100.0
    return v if v > 0 else None


def table(match_pattern, txt, standard=None):
    for pat, *w in match_pattern:
        if re.search(pat, txt or '', re.I): return w[0] if len(w) == 1 else tuple(w)
    return standard





SMALL_UNTIL = 220e-9
MEDIUM_UNTIL = 4.7e-6
CRYSTAL_LOAD_UNTIL = 100e-12
SERIES_R_UNTIL = 470.0
SHUNT_PATH_UNTIL = 100.0
DIVIDER_RATIO = 0.1
SETTING_T = {'RT', 'RSET', 'ISET', 'ILIM', 'RLIM', 'IREF', 'RREF', 'RBIAS', 'IBIAS', 'FREQ', 'FSET', 'ROSC', 'PROG', 'RPROG'}
FB_T = {'FB', 'VFB', 'FBK', 'ADJ'}
NO_CRYSTAL = {'R', 'C', 'L', 'D', 'FB', 'F', 'Q', 'TVS', 'LED', 'NT', 'RN', 'ESD', 'NTC', 'PTC', 'JP', 'SJ', 'U', 'J', 'TP', 'SW', 'IC'}
FUNCTIONS = ('decoupling_small', 'crystal', 'crystal_load', 'series_r', 'coil_sw', 'bootstrap', 'decoupling_medium', 'decoupling_large',
              'feedback', 'feedback_c', 'setting_r', 'shunt', 'rc_element', 'divider', 'pullup', 'pulldown', 'misc')


def functions(B, fps, pads_by_ref, pads_by_net, NET, ROLE, TYPE, DCP, ISLAND, PRODUCERS, shunt, supply_pins):
    P = B['pads']
    def category(n): return NET[n]['category'] if n in NET else None
    def kat(n):
        k = NET.get(n)
        if k is None: return None
        if k['category'] == 'gnd' or k.get('kelvin'): return 'G'
        if k['category'] == 'supply': return 'V'
        return 'P' if k['category'] in ('current_val', 'sw') else 'S'
    def net_list(r): return [P[i]['net'] if P[i]['net'] in NET else None for i in pads_by_ref[r]]
    def two_poles(t): return [r for r in sorted(fps) if ROLE[r] == 'small' and TYPE[r] == t and len(pads_by_ref[r]) == 2]
    ZP = {t: set(two_poles(t)) for t in ('R', 'C', 'L')}
    def parts_on(n, t): return sorted({P[i]['ref'] for i in pads_by_net.get(n, [])} & ZP[t])
    def other(r, n):
        ns = net_list(r); return ns[1] if ns[0] == n else ns[0]
    def chip_pins(n): return [(P[i]['ref'], i) for i in pads_by_net.get(n, []) if ROLE[P[i]['ref']] == 'chip']
    def close(r, n, cand):
        me = [i for i in pads_by_ref[r] if P[i]['net'] == n] or pads_by_ref[r]
        return min(cand, key=lambda j: (min(math.hypot(P[i]['x'] - P[j]['x'], P[i]['y'] - P[j]['y']) for i in me), j))
    def pin(j, **kw): return dict(ref=P[j]['ref'], pin=P[j]['num'], net_name=P[j]['net'], **kw)
    def one_pin(n): cp = chip_pins(n); return pin(cp[0][1]) if len(cp) == 1 else None
    def value_r(r): return resistance(fps[r].get('value', ''))
    def value_c(r): return capacity(fps[r].get('value', ''))
    def bn(n): return base_name(n) if n else 'open_items'
    def names(n): return set(tokens(n)) | {t for i in pads_by_net.get(n, []) for t in tokens(P[i].get('pinfunc') or '')}
    def value_text(r): return (fps[r].get('value') or '?').split()[0]
    F = {}
    def assign(r, f, reason=None, supports=None):
        if r in F: return
        F[r] = dict(function=f)
        if reason: F[r]['function_reason'] = reason
        if supports: F[r]['supports'] = supports


    Q_NET = {}
    for r in sorted(fps):
        if ROLE[r] in ('without', 'mechanics', 'chip') or TYPE[r] in NO_CRYSTAL or len(pads_by_ref[r]) > 4: continue
        ns = sorted({n for n in net_list(r) if n and kat(n) != 'G'})
        if len(ns) != 2 or any(kat(n) != 'S' for n in ns): continue
        txt = '%s %s' % (fps[r].get('fp', ''), fps[r].get('value', ''))
        hint = TYPE[r] == 'Y' or prefix(r) in ('Y', 'XTAL', 'XT') or bool(re.search(r'crystal|xtal|quar[zt]|resonator|\d\s*[kM]?Hz', txt, re.I))
        load = [[c for c in parts_on(n, 'C') if kat(other(c, n)) == 'G' and (value_c(c) is None or value_c(c) <= CRYSTAL_LOAD_UNTIL * 1.001)] for n in ns]
        if not hint and not all(load): continue
        way = []
        for n in ns:
            w = {c: (j, None) for c, j in chip_pins(n)}
            for rr in parts_on(n, 'R'):
                m = other(rr, n)
                if m and m != n and kat(m) == 'S':
                    for c, j in chip_pins(m): w.setdefault(c, (j, rr))
            way.append(w)
        gem = sorted(set(way[0]) & set(way[1]))
        if not gem: continue
        chip = gem[0]; js = [way[0][chip], way[1][chip]]
        assign(r, 'crystal', 'at %s:%s and %s:%s' % (chip, P[js[0][0]]['num'], chip, P[js[1][0]]['num']),
              dict(ref=chip, pin=P[js[0][0]]['num'], pins=[P[j]['num'] for j, _ in js], net_name=ns[0], net_list=ns))
        for n, (j, rr), ls in zip(ns, js, load):
            Q_NET[n] = (r, j)
            if rr: assign(rr, 'series_r', 'Series resistor between crystal %s and %s:%s' % (r, chip, P[j]['num']), pin(j, crystal=r))
            for c in ls: assign(c, 'crystal_load', 'Load capacitor from %s at %s (%s)' % (r, bn(n), value_text(c)), pin(j, crystal=r))


    for r in sorted(ZP['R']):
        a, b = net_list(r); v = value_r(r)
        if shunt(r): assign(r, 'shunt', 'Sense resistor %s between %s and %s' % (value_text(r), bn(a), bn(b)))
        elif v is not None and v <= SHUNT_PATH_UNTIL and a != b and 'current_val' in (category(a), category(b)) and all(category(n) == 'current_val' or kat(n) == 'G' for n in (a, b)):
            assign(r, 'shunt', 'Resistor %s in the current path between %s and %s' % (value_text(r), bn(a), bn(b)))
    RN = ZP['R'] - set(F)
    def rs_on(n): return [r for r in parts_on(n, 'R') if r in RN]


    FBN = {}
    for n in sorted(NET):
        if kat(n) != 'S' or n in Q_NET: continue
        rr = rs_on(n)
        if not rr: continue
        cp = chip_pins(n)
        tooG = [r for r in rr if kat(other(r, n)) == 'G']
        tooV = [r for r in rr if kat(other(r, n)) == 'V']
        tooS = [r for r in rr if kat(other(r, n)) in ('S', 'P') and other(r, n) != n]
        upper = [r for r in rr if r not in tooG and other(r, n) not in (n, None) and any(c in PRODUCERS.get(other(r, n), ()) for c, _ in cp)]
        if tooG and (upper or (tooV and cp and names(n) & FB_T)):
            top = upper or tooV
            j = next((j for c, j in cp if any(c in PRODUCERS.get(other(r, n), ()) for r in top)), cp[0][1])
            FBN[n] = j
            for r in top + tooG:
                assign(r, 'feedback', 'Feedback divider %s -> %s -> ground at %s:%s' % (bn(other(top[0], n)), bn(n), P[j]['ref'], P[j]['num']), pin(j))
        elif tooG and tooV:
            for r in tooV + tooG: assign(r, 'divider', 'Voltage divider %s -> %s -> ground' % (bn(other(tooV[0], n)), bn(n)), one_pin(n))
        elif tooG and tooS:
            for rg in tooG:
                cons = value_r(rg)
                high = [r for r in tooS if cons and value_r(r) is not None and value_r(r) >= DIVIDER_RATIO * cons]
                for r in ([rg] + high if high else []):
                    assign(r, 'divider', 'Voltage divider %s -> %s -> ground' % (bn(other(high[0], n)), bn(n)), one_pin(n))
        def series_resistor(r):
            rest = {P[i]['ref'] for i in pads_by_net[n]} - {r}
            return bool(rest) and all(TYPE[x] in ('D', 'LED') for x in rest)
        for r in tooV + tooG:
            if r in F: continue
            goal = 'Supply %s' % bn(other(r, n)) if r in tooV else 'Ground'
            if series_resistor(r): assign(r, 'misc', 'Series resistor of a diode/LED (%s to %s)' % (bn(n), goal))
            elif r in tooV: assign(r, 'pullup', '%s from %s to %s' % (value_text(r), bn(n), bn(other(r, n))), one_pin(n))
            elif len(pads_by_net[n]) == 2 and cp and names(n) & SETTING_T:
                assign(r, 'setting_r', 'Setting resistor %s at %s:%s' % (value_text(r), cp[0][0], P[cp[0][1]]['num']), pin(cp[0][1]))
            else: assign(r, 'pulldown', '%s from %s to ground' % (value_text(r), bn(n)), one_pin(n))
    for r in sorted(RN - set(F)):
        n = next((n for n in net_list(r) if n in FBN), None)
        if n: assign(r, 'feedback', 'at feedback node %s (from %s)' % (bn(n), bn(other(r, n))), pin(FBN[n]))


    for c in sorted(ZP['C']):
        if c in F: continue
        a, b = net_list(c); ka, kb = kat(a), kat(b)
        if c in DCP:
            n = DCP[c]['net_name']; C = DCP[c]['C']; an = ISLAND.get(c)
            level = 'small' if C <= SMALL_UNTIL * 1.001 else ('medium' if C <= MEDIUM_UNTIL * 1.001 else 'large')
            g = '%s on %s' % (si(C), bn(n)) + ('' if value_c(c) else ' (value not readable, estimated from package)'); st = None
            if an in fps:
                sp = supply_pins(an, n)
                coil = [i for l in parts_on(n, 'L') for i in pads_by_ref[l] if P[i]['net'] == n and category(other(l, n)) == 'sw'
                         and any(ch == an for ch, _ in chip_pins(other(l, n)))]
                if sp:
                    j = close(c, n, sp); st = pin(j); g += ', supports %s:%s' % (an, P[j]['num'])
                    if len(sp) > 1: st['pins'] = [P[i]['num'] for i in sp]
                elif coil:
                    j = close(c, n, coil); st = pin(j, regulator=an, over_coil=True); g += ', output capacitor from %s behind %s' % (an, P[j]['ref'])
                else: g += ', island %s without pin on this net' % an
            assign(c, 'decoupling_' + level, g, st); continue
        if any(n in FBN for n in (a, b)):
            n = a if a in FBN else b; par = [r for r in rs_on(n) if other(r, n) == other(c, n)]
            assign(c, 'feedback_c', ('parallel to %s at feedback node %s' % (par[0], bn(n))) if par else 'at feedback node %s' % bn(n), pin(FBN[n]))
            continue
        if a is None or b is None or a == b: assign(c, 'misc', 'Capacitor with open or shorted terminal'); continue
        if 'G' not in (ka, kb):
            for x, y in ((a, b), (b, a)):
                bj = [j for ch, j in chip_pins(y) if any(ch == c2 for c2, _ in chip_pins(x))]
                if category(x) == 'sw' and parts_on(x, 'L') and len(pads_by_net[y]) == 2 and bj:
                    assign(c, 'bootstrap', 'between switching node %s and %s:%s' % (bn(x), P[bj[0]]['ref'], P[bj[0]]['num']), pin(bj[0])); break
            if c in F: continue
            gem = sorted({ch for ch, _ in chip_pins(a)} & {ch for ch, _ in chip_pins(b)})
            if gem and 'V' not in (ka, kb) and (ka, kb) != ('P', 'P') and (rs_on(a) or rs_on(b)):
                assign(c, 'feedback_c', 'between two pins of %s (%s, %s): negative feedback/compensation' % (gem[0], bn(a), bn(b)),
                      pin(next(j for ch, j in chip_pins(a) if ch == gem[0])))
            else: assign(c, 'misc', 'Capacitor between %s and %s (coupling or similar)' % (bn(a), bn(b)))
            continue
        n = b if ka == 'G' else a
        if kat(n) == 'S':
            feeds = [r for r in rs_on(n) if other(r, n) not in (n, None) and kat(other(r, n)) != 'G']
            if feeds: assign(c, 'rc_element', '%s at %s, fed via %s; no decoupling' % (value_text(c), bn(n), ', '.join(feeds)), one_pin(n))
            else: assign(c, 'misc', 'Capacitor %s at %s to ground without series resistor (filter or timing element at the pin)' % (value_text(c), bn(n)), one_pin(n))
        else: assign(c, 'misc', 'Capacitor between %s and %s' % (bn(a), bn(b)))


    for r in sorted(RN - set(F)):
        a, b = net_list(r); v = value_r(r)
        if a is None or b is None or a == b: assign(r, 'misc', 'Resistor with open or shorted terminal'); continue
        rc = [n for n in (a, b) if any(F.get(c, {}).get('function') == 'rc_element' and kat(other(c, n)) == 'G' for c in parts_on(n, 'C'))]
        if kat(a) == 'S' and kat(b) == 'S' and 'fast' in (category(a), category(b)) and (v is None or v <= SERIES_R_UNTIL):
            short = min((a, b), key=lambda n: (len(pads_by_net[n]), 0 if any(P[j].get('pintype') in ('output', 'tri_state') for _, j in chip_pins(n)) else 1, n))
            cp = chip_pins(short) or chip_pins(other(r, short))
            assign(r, 'series_r', '%s in series between %s and %s (fast)' % (value_text(r), bn(a), bn(b)), pin(close(r, P[cp[0][1]]['net'], [j for _, j in cp])) if cp else None)
        elif rc and all(kat(n) in ('S', 'P') for n in (a, b)):
            assign(r, 'rc_element', 'Resistor %s of the RC element at %s (from %s)' % (value_text(r), bn(rc[0]), bn(other(r, rc[0]))), one_pin(rc[0]))
        elif all(kat(n) in ('S', 'P') for n in (a, b)):
            assign(r, 'misc', 'Series resistor %s between %s and %s (not a fast net)' % (value_text(r), bn(a), bn(b)))
        else: assign(r, 'misc', 'Resistor %s between %s and %s' % (value_text(r), bn(a), bn(b)))


    for l in sorted(ZP['L']):
        sw = [n for n in net_list(l) if category(n) == 'sw']
        cp = chip_pins(sw[0]) if sw else []
        if sw: assign(l, 'coil_sw', 'Coil at switching node %s' % bn(sw[0]), pin(close(l, sw[0], [j for _, j in cp])) if cp else None)
        else: assign(l, 'misc', 'Coil/choke between %s and %s (no switching node)' % tuple(bn(n) for n in net_list(l)))
    for r in fps:
        if r not in F: F[r] = dict(function='misc' if ROLE[r] == 'small' else ROLE[r])
    return F


def load(argv):
    B = json.load(open(argv[1]))
    kw = dict(a.split('=', 1) for a in argv[3:] if '=' in a)
    RG = json.load(open(kw['design_rules'])) if 'design_rules' in kw else None
    CONS = json.load(open(kw['specs'])) if 'specs' in kw and os.path.exists(kw['specs']) else {}
    return B, RG, CONS, kw


def loose_connector(B, fps, ROLE, CONS, operate=()):
    try:
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
    except ImportError:
        return set()
    cand = {r for r in fps if ROLE.get(r) in ('connector', 'mechanics') or r in operate}
    hard = {r for r in fps if fps[r].get('locked') or glob_any(r, CONS.get('pinned'))}



    if not str(B.get('src', '')).startswith('task'):
        return set()
    cy = {}
    for c in B.get('courtyards') or []:
        r = c.get('ref')
        if r in cand or r in hard:
            try: cy[r] = unary_union([Polygon(q) for q in c.get('polys') or [] if len(q) >= 3])
            except (ValueError, TypeError): pass
    loose = set()
    for r in sorted(cand - hard):
        g = cy.get(r)
        if g is None or g.is_empty: continue
        for q in sorted((cand | hard) - {r}):
            h = cy.get(q)
            if h is None or h.is_empty or fps[q]['side'] != fps[r]['side']: continue
            if g.intersection(h).area > 0.01:
                loose.add(r); break
    return loose


def glob_any(ref, pats): return any(fnmatch.fnmatchcase(ref, p) for p in pats or [])


def actual_ground_name(name):
    return base_name(name).upper() in ('0V', 'GNDPWR') or any(
        re.fullmatch(r'(A|D|P|S|C)?GND[A-Z]?\d*', t) or t.startswith('VSS')
        for t in tokens(name))


def pair_id(name):
    scope, _, base = name.rpartition('/')
    u = base.upper()
    if re.fullmatch(r'(?:C\d+|V[CS]|VCC|VDD|VSS|VBAT|VIN|VOUT|PWR)[+\-]', u): return None
    if u in ('DP', 'DM', 'DN', 'D+', 'D-', 'DPLUS', 'DMINUS'):
        return (scope, 'USB_D', '+' if u in ('DP', 'D+', 'DPLUS') else '-')
    m = re.fullmatch(r'(.*?)(DP|DM|DN)(\d+)', u)
    if m: return (scope, m[1] + 'D@' + m[3], '+' if m[2] == 'DP' else '-')
    m = re.fullmatch(r'(.+?)[_]([PN])(\d+)', u)
    if m: return (scope, m[1] + '_' + m[3], '+' if m[2] == 'P' else '-')
    m = re.fullmatch(r'(.*?)(DPLUS|DMINUS|DP|DM|DN|[+\-])', u)
    if m and m[1]:
        stem, sign = m[1], m[2]
        if sign in ('DP', 'DM', 'DN', 'DPLUS', 'DMINUS'): stem += 'D'
        return (scope, stem.rstrip('_'), '+' if sign in ('DP', 'DPLUS', '+') else '-')
    m = re.fullmatch(r'(.+?)(?:_([PN])|([PN]))', u)
    if m:
        stem, sign = m[1], m[2] or m[3]
        if m[2] or re.search(r'(?:USB_D|TXD?\d*|RXD?\d*|LVDS\d*|MIPI\w*|HDMI\w*|ETH\w*|IN\d*|SENSE\d*|CLK|CK|DQS\d*)$', stem):
            return (scope, stem, '+' if sign == 'P' else '-')
    return None


def analyze(B, RG=None, CONS=None, impedance=False, disturb_flank=False, pairs=False, sw_compact=False):
    CONS = CONS or {}
    L = int(B.get('cu_layers') or (RG or {}).get('router', {}).get('cu_layers') or 4)
    SIGW = (RG or {}).get('router', {}).get('sigw') or (0.10 if L >= 6 else 0.15)


    SIGMIN = min(SIGW, (RG or {}).get('router', {}).get('sigmin') or SIGW)
    fps = {f['ref']: f for f in B['fps']}
    pads_by_ref = collections.defaultdict(list); pads_by_net = collections.defaultdict(list)
    for i, p in enumerate(B['pads']):
        if p['ref'] not in fps: continue
        pads_by_ref[p['ref']].append(i)
        if p['net'] and not p['net'].startswith('unconnected'): pads_by_net[p['net']].append(i)
    refs_on = {n: {B['pads'][i]['ref'] for i in v} for n, v in pads_by_net.items()}
    TYPE = {r: type_part(r, f) for r, f in fps.items()}


    def role(r):
        f = fps[r]
        if f.get('dnp') or f.get('board_only') and not pads_by_ref[r]: return 'without'
        if glob_any(r, CONS.get('anchor')): return 'chip'
        pf = TYPE[r]; txt = '%s %s %s' % (f.get('fp', ''), f.get('lib', ''), f.get('value', ''))
        if MECH_FP.search(f.get('fp', '')): return 'mechanics'
        if pf in CONNECTOR_PFX or (pf not in PASSIVE_PFX and pf not in CHIP_PFX and CONNECTOR_HINT.search(txt)): return 'connector'
        if pf in MECH_PFX: return 'mechanics'
        if (prefix(r) == 'LED' or pf == 'LED') and f.get('n_pads', 0) >= 4: return 'chip'
        if pf in PASSIVE_PFX and prefix(r) not in PASSIVE_PFX and f.get('n_pads', 0) >= 5: return 'chip'
        if pf in PASSIVE_PFX or pf == 'LED': return 'small'
        if pf in CHIP_PFX or f.get('n_pads', 0) >= 5: return 'chip'
        return 'small'
    ROLE = {r: role(r) for r in fps}
    def operate(r):
        f = fps[r]
        if CONS.get('operate_fixed') is False or ROLE[r] not in ('small', 'chip'): return False
        return prefix(r) in ('LED', 'DS', 'RV', 'VR', 'BZ') or bool(OPERATE_HINT.search('%s %s' % (f.get('fp', ''), f.get('value', ''))))
    OPERATE = {r for r in fps if operate(r)}
    has_nets = {r: any(B['pads'][i]['net'] and not B['pads'][i]['net'].startswith('unconnected') for i in pads_by_ref[r]) for r in fps}
    for r in fps:
        if not has_nets[r] and ROLE[r] in ('small', 'chip'): ROLE[r] = 'without'
    def analog_chip(r):
        f = fps[r]; return ROLE[r] == 'chip' and bool(ANALOG_CHIP.search('%s %s' % (f.get('value', ''), f.get('fp', ''))))
    def shunt(r):
        if TYPE[r] != 'R': return False
        v = resistance(fps[r].get('value', '')); return v is not None and 0 < v <= 1.0
    def hf_coil(r):
        if TYPE[r] != 'L': return False
        v = capacity(fps[r].get('value', ''))
        if v is not None: return v < HF_COIL_UNTIL
        if package(fps[r].get('fp', '')) > 3: return False
        for i in pads_by_ref[r]:
            for j in pads_by_net.get(B['pads'][i]['net'], []):
                c = B['pads'][j]['ref']
                if TYPE.get(c) == 'C' and (capacity(fps[c].get('value', '')) or 1.0) <= CRYSTAL_LOAD_UNTIL * 1.001: return True
        return False
    def is_ind(r): return TYPE[r] == 'L' and not hf_coil(r)


    over = [(re.compile(k), v) for k, v in (CONS.get('net_list') or {}).items()]
    gnd_forced = CONS.get('gnd') or []
    def pintypes(n): return {B['pads'][i].get('pintype', '') for i in pads_by_net[n]}
    def load_pad(r, n):
        ps = [B['pads'][i] for i in pads_by_net[n] if B['pads'][i]['ref'] == r]
        return len(ps) >= 2 or any(p.get('pintype') in ('passive', 'power_in', 'power_out') for p in ps)

    gnd_names = {n for n in pads_by_net if n in gnd_forced or actual_ground_name(n)}

    def fast_pin(n):
        for i in sorted(pads_by_net[n]):
            p = B['pads'][i]; f = (p.get('pinfunc') or '').strip().upper()
            if not f or '/' in f.strip('/~{}') or ROLE[p['ref']] != 'chip': continue
            T = [t for t in re.split(r'[_\-()/{}~\s.,]+', f) if t]
            if not T: continue
            if re.fullmatch(r'USB_?D[PMN+\-]|D[+\-]|DPLUS|DMINUS', f) or (T[0] == 'USB' and T[-1] in ('DP', 'DM', 'DN', 'P', 'N')):
                return dict(category='fast', rank=5, current_val=None, reason='USB pair (pin name %s:%s)' % (p['ref'], f))
            if set(T) & XTAL_T: return dict(category='fast', rank=4, current_val=None, reason='Crystal/oscillator (pin name %s:%s)' % (p['ref'], f), crystal=True)
            if any(t.startswith(VHS_IF) for t in T): return dict(category='fast', rank=5, current_val=None, reason='High speed (pin name %s:%s)' % (p['ref'], f))
            if set(T) & CLK_T or any(t.endswith('CLK') for t in T):
                return dict(category='fast', rank=4, current_val=None, reason='Clock (pin name %s:%s)' % (p['ref'], f))
            if any(t.startswith(HS_IF) for t in T): return dict(category='fast', rank=4, current_val=None, reason='xSPI/SDIO (pin name %s:%s)' % (p['ref'], f))
            if set(T) & {'MOSI', 'MISO', 'COPI', 'CIPO'}: return dict(category='fast', rank=3, current_val=None, reason='SPI (pin name %s:%s)' % (p['ref'], f))
        return None

    def net_class(n):
        base = base_name(n); T = tokens(n); u = base.upper(); refs = refs_on[n]
        for rx, v in over:
            if rx.fullmatch(base) or rx.fullmatch(n): return dict(dict(category='signal', rank=0, current_val=None), **v, reason='Spec')
        if n in gnd_forced or actual_ground_name(n):
            return dict(category='gnd', rank=0, current_val=None, reason='Name')
        if any(hf_coil(r) for r in refs) and not any(is_ind(r) for r in refs):
            return dict(category='fast', rank=4, current_val=None, reason='RF matching (coil < 0.2 uH or placeholder with pF capacitor)')
        if any(re.fullmatch(r'(SW|LX|SWN|PH)\d?', t) for t in T) and any(is_ind(r) for r in refs):
            return dict(category='sw', rank=5, current_val='high', reason='Switching node (name + coil)')
        chips_n = [r for r in refs if ROLE[r] == 'chip']
        c_gnd = [r for r in refs if TYPE[r] == 'C' and any(B['pads'][i]['net'] in gnd_names for i in pads_by_ref[r])]
        if any(is_ind(r) for r in refs) and chips_n and not c_gnd and not any(t.startswith(PWR_START) for t in T) \
                and not any(re.fullmatch(r'\+?\d+V\d*', t) for t in T):
            return dict(category='sw', rank=5, current_val='high', reason='Switching node (chip pin + coil, without C to GND)')
        if any(t in ('BST', 'CBST', 'BOOT', 'BS') for t in T) and len(pads_by_net[n]) == 2 and any(TYPE[r] == 'C' for r in refs):
            return dict(category='sw', rank=4, current_val='medium', reason='Bootstrap')

        Ts = set(T); anyst = lambda pre: any(t.startswith(pre) for t in T)
        if anyst('USB') and (T[-1] in ('P', 'N', 'DP', 'DM', 'DN') or Ts & {'DP', 'DN', 'DM', 'DPLUS', 'DMINUS'}):
            return dict(category='fast', rank=5, current_val=None, reason='USB pair')
        if any(anyst(x) for x in VHS_IF): return dict(category='fast', rank=5, current_val=None, reason='HighSpeed-Interface')
        if any(anyst(x) for x in HS_IF):
            if Ts & CLK_T or any(t.endswith('CLK') for t in T): return dict(category='fast', rank=5, current_val=None, reason='xSPI/SDIO-Clock')
            return dict(category='fast', rank=4, current_val=None, reason='xSPI/SDIO-Data')
        if (Ts & GPIO_T or (any(GPIO_NR.fullmatch(t) for t in T) and not any(t.startswith(PWR_START) for t in T))) and not Ts & CLK_T:
            return dict(category='signal', rank=0, current_val=None, reason='ControlSignal')
        if Ts & XTAL_T: return dict(category='fast', rank=4, current_val=None, reason='Crystal/Oscillator', crystal=True)
        if Ts & SWD_T: return dict(category='fast', rank=2, current_val=None, reason='SWD/JTAG')
        spi = anyst('SPI'); aud = any(anyst(x) for x in AUD_IF)
        if Ts & CLK_T or any(t.endswith('CLK') for t in T):
            return dict(category='fast', rank=4, current_val=None, reason='Clock (%s)' % ('SPI' if spi else 'Audio' if aud else 'CLK'))
        if spi and Ts & {'MOSI', 'MISO', 'SDI', 'SDO', 'SI', 'SO', 'CS', 'NSS', 'SS', 'NCS', 'CE'}:
            return dict(category='fast', rank=3, current_val=None, reason='SPI')
        if spi or any(re.fullmatch(r'N?CS\d*', t) for t in T): return dict(category='fast', rank=3, current_val=None, reason='SPI/CS')
        if aud or Ts & {'LRCK', 'LRCLK', 'FSYNC', 'WS', 'FS', 'SDOUT', 'SDIN', 'SDATA'}:
            return dict(category='fast', rank=3, current_val=None, reason='Audio-Digital (I2S/SAI)')
        if Ts & SLOW_T or anyst('I2C') or anyst('UART'):
            return dict(category='signal', rank=1, current_val=None, reason='slow interface (I2C/UART/PWM)')
        if Ts & GPIO_T: return dict(category='signal', rank=0, current_val=None, reason='ControlSignal')

        opin = bool(re.search(r'(-|\+)IN\)?$|--\)$|-\+\)$', base))
        if opin or Ts & ON_T3 or (Ts & {'IN'} and (anyst('ADC') or anyst('AIN') or anyst('OPV'))) or any(re.fullmatch(r'IN\d+[PNM]?', t) for t in T):
            return dict(category='analog', rank=3, current_val=None, reason='Analog-Input/Reference')
        if Ts & ON_T2: return dict(category='analog', rank=2, current_val=None, reason='Feedback/Measurement')

        pwr_name = any(re.fullmatch(r'\+?\d+V\d*', t) or t.startswith(PWR_START) for t in T) or u.startswith('+')
        pt = pintypes(n)
        if pwr_name or pt & {'power_in', 'power_out'} or re.search(r'power|pwr|supply', B.get('netclass', {}).get(n, ''), re.I):
            if any(t.startswith(PWR_HIGH) for t in T) or any(is_ind(r) or shunt(r) or TYPE[r] == 'F' for r in refs): lvl = 'high'
            elif any(t.startswith(PWR_SMALL) or t in PWR_SMALL for t in T): lvl = 'small'
            else: lvl = 'medium'
            return dict(category='supply', rank=0, current_val=lvl, reason='Supply (%s)' % ('Name' if pwr_name else 'Pin-Type'))


        if any(is_ind(r) for r in refs): return dict(category='current_val', rank=0, current_val='high', reason='Current path (coil)')
        if any(TYPE[r] in ('F', 'PTC') for r in refs): return dict(category='current_val', rank=0, current_val='high', reason='Current path (fuse)')
        if any(shunt(r) for r in refs): return dict(category='current_val', rank=0, current_val='medium', reason='Current path (shunt)')




        control = any(TYPE[r] == 'R' and not shunt(r) and (resistance(fps[r].get('value', '')) or 0) >= CONTROL_R_FROM for r in refs) \
            and not any(r in c_gnd and (capacity(fps[r].get('value', '')) or 0) >= 1e-6 for r in refs) \
            and all(SMALL_SIGNAL_Q.search(fps[r].get('fp', '') or '') for r in refs if TYPE[r] == 'Q')
        if any(TYPE[r] == 'Q' and load_pad(r, n) for r in refs) and not control:
            return dict(category='current_val', rank=0, current_val='medium', reason='Current path (load terminal of a transistor)')
        if any(TYPE[r] == 'Q' for r in refs) and any(ROLE[r] == 'connector' for r in refs) and not control:
            return dict(category='current_val', rank=0, current_val='medium', reason='Current path (transistor at connector)')

        if not control and any(ROLE[r] == 'connector' for r in refs) and any(
                ROLE[B['pads'][i]['ref']] == 'chip' and DRIVER_CHIP.search(fps[B['pads'][i]['ref']].get('value') or '')
                and (not (B['pads'][i].get('pinfunc') or '').strip() or DRIVER_OFF.search(B['pads'][i]['pinfunc'])) for i in pads_by_net[n]):
            return dict(category='current_val', rank=0, current_val='medium', reason='Current path (driver output at connector)')
        sp = fast_pin(n)
        if sp: return sp
        if any(analog_chip(r) for r in refs):
            return dict(category='analog', rank=2, current_val=None, reason='Pin of an analog chip')
        return dict(category='signal', rank=0, current_val=None, reason='Rest (GPIO)')

    NET = {n: net_class(n) for n in pads_by_net}



    LEVELS = {'small': 1, 'medium': 2, 'high': 3}
    for _ in range(6):
        new = False
        for r in sorted(fps):
            if TYPE[r] in ('F', 'PTC') or shunt(r): sides = sorted({B['pads'][i]['net'] for i in pads_by_ref[r]} & set(NET))
            elif TYPE[r] == 'Q': sides = sorted(n for n in {B['pads'][i]['net'] for i in pads_by_ref[r]} & set(NET) if load_pad(r, n))
            else: continue
            if len(sides) != 2: continue
            for a, b in ((sides[0], sides[1]), (sides[1], sides[0])):
                ka, kb = NET[a], NET[b]
                if ka['category'] not in ('supply', 'current_val') or not ka.get('current_val'): continue
                if kb['reason'] == 'Rest (GPIO)' or (kb['category'] == 'current_val' and LEVELS.get(kb.get('current_val'), 0) < LEVELS[ka['current_val']]
                                                   and kb['reason'].startswith('Current path (')):
                    NET[b] = dict(category='current_val', rank=0, current_val=ka['current_val'], reason='Current path (behind %s of %s)' % (r, a)); new = True
        if not new: break
    gnd = [n for n in pads_by_net if NET[n]['category'] == 'gnd']
    gnd = [n for n in gnd_forced if n in pads_by_net] + sorted([n for n in gnd if n not in gnd_forced], key=lambda n: -len(pads_by_net[n]))


    net_ties = sorted(r for r in fps if TYPE[r] == 'NT' or 'nettie' in re.sub(r'[^a-z]', '', fps[r].get('fp', '').lower()))
    for r in net_ties:
        ns = sorted({B['pads'][i]['net'] for i in pads_by_ref[r] if B['pads'][i]['net'] in NET})
        for n in ns:
            if n == (gnd[0] if gnd else None) or n in gnd_forced or NET[n]['category'] != 'gnd' or NET[n]['reason'] == 'Spec': continue
            an = [m for m in ns if m != n and NET[m]['category'] == 'gnd']
            if an: NET[n] = dict(category='analog', rank=3, current_val=None, kelvin=r, reason='Kelvin sense ground (net tie %s at %s)' % (r, base_name(an[0])))
    gnd = [n for n in gnd if NET[n]['category'] == 'gnd']
    GNDS = set(gnd)

    sw_chips = {r for n, k in NET.items() if k['category'] == 'sw' for r in refs_on[n] if ROLE[r] == 'chip'}
    for n, k in NET.items():
        if k['category'] in ('signal', 'analog') and refs_on[n] & sw_chips: k['sensitive_sw'] = True
        if k['category'] == 'analog': k['sensitive_sw'] = True

    if pairs or CONS.get('pairs') is True or impedance or CONS.get('impedance') is True:


        by_base = collections.defaultdict(lambda: collections.defaultdict(set))
        for n, k in sorted(NET.items()):
            if k['category'] not in ('fast', 'analog', 'signal'): continue
            key = pair_id(n)
            if key: by_base[key[:2]][key[2]].add(n)
            elif n.startswith(('Net-(', 'N-')):
                for i in pads_by_net[n]:
                    p = B['pads'][i]; key = pair_id(p.get('pinfunc', ''))
                    if key: by_base[('pin:' + p['ref'], key[1])][key[2]].add(n)

            for i in pads_by_net[n]:
                p = B['pads'][i]; f = fps[p['ref']]
                if ROLE[p['ref']] != 'connector' or 'USB' not in (f.get('fp','')+' '+f.get('value','')).upper(): continue
                pin = p['num']; c = 'TYPE-C' in (f.get('fp','')+' '+f.get('value','')).upper() or 'USB_C' in f.get('fp','').upper()
                positive, negative = ({'A6','B6'}, {'A7','B7'}) if c else ({'3'}, {'2'})
                if pin in positive | negative: by_base[('connector:' + p['ref'], 'USB_D')]['+' if pin in positive else '-'].add(n)
        usb_names = {n for key, signs in by_base.items() if 'USB' in key[1] for names in signs.values() for n in names}
        candidates = collections.defaultdict(set)
        for signs in by_base.values():
            if len(signs['+']) == len(signs['-']) == 1:
                a, b = next(iter(signs['+'])), next(iter(signs['-']))
                if a != b: candidates[a].add(b); candidates[b].add(a)
        for n, partners in sorted(candidates.items()):
            if len(partners) != 1: continue
            pn = next(iter(partners))
            if candidates[pn] != {n}: continue


            if any(TYPE[r] == 'C' and {n, pn}.issubset({B['pads'][i]['net'] for i in pads_by_ref[r]}) for r in refs_on[n] & refs_on[pn]) and any(re.search(r'MAX(?:232|3232)|SP(?:232|3232)', fps[r].get('value',''), re.I) for r in refs_on[n] & refs_on[pn]): continue
            analog = all(NET[q]['category'] == 'analog' or re.search(r'(?:^|[_/])(?:IN\d*|AIN\d*|SENSE\d*|AUDIO|MIC|LINE)', q.upper()) for q in (n, pn))
            usb = n in usb_names or (pair_id(n) and 'USB' in pair_id(n)[1])
            usb = usb or any(USB_FS.search(fps[r].get('value') or '') or USB_HS.search(fps[r].get('value') or '') for r in refs_on[n] & refs_on[pn] if ROLE[r] == 'chip')
            category, rank = ('analog', 3) if analog else ('fast', max(5 if usb else 3, NET[n]['rank'], NET[pn]['rank']))
            k = NET[n]; k.update(pair=pn, pair_kind=category, pair_rank=rank)
            if k['reason'] != 'Spec': k.update(category=category, rank=rank, current_val=None)
    else:
        by_base = {}
        for n in NET:
            m = re.match(r'(.*?)(_?)(P|N|DP|DN|DM|\+|-)$', base_name(n))
            if m and NET[n]['category'] == 'fast':



                scope = n.rpartition('/')[0] if NET[n].get('z_diff_ohm') and (impedance or CONS.get('impedance') is True) else n.rsplit('/', 1)[0]
                by_base.setdefault((scope, m.group(1)), []).append(n)
        for k, v in by_base.items():
            if len(v) == 2:
                NET[v[0]]['pair'] = v[1]; NET[v[1]]['pair'] = v[0]



    for n, k in sorted(NET.items()):
        if k['category'] != 'fast' or not k['reason'].startswith('USB pair'): continue
        chips = {r for r in refs_on[n] if ROLE[r] == 'chip'}
        for r in refs_on[n]:
            if TYPE[r] == 'R' and len(pads_by_ref[r]) == 2:
                chips |= {q for i in pads_by_ref[r] for q in refs_on.get(B['pads'][i]['net'], ()) if ROLE[q] == 'chip'}
        hs = sorted(c for c in chips if USB_HS.search(fps[c].get('value') or ''))
        fs = sorted(c for c in chips if USB_FS.search(fps[c].get('value') or ''))
        if hs: k['tr_ns'] = 0.5; k['tr_source'] = 'USB high-speed (%s)' % hs[0]
        elif fs: k['tr_ns'] = 4.0; k['tr_source'] = 'USB full-speed (%s %s)' % (fs[0], fps[fs[0]].get('value'))

    def router_params(k):
        category = k['category']
        if category == 'sw': return dict(w=0.5, wmin=0.25, pen=2.0, via=1.5, inner=1.8, prio=0 if k['rank'] >= 5 else 1, group=0)
        if category in ('current_val', 'supply'):
            w, wmin, prio = {'high': (1.0, 0.3, 2), 'medium': (0.5, 0.2, 4), 'small': (0.3, 0.15, 5)}[k['current_val']]
            return dict(w=w, wmin=wmin, pen=1.0, via=1.0, inner=1.25, prio=prio, group=2)
        if category == 'fast': return dict(w=SIGW, wmin=SIGMIN, pen=0.0, via=round(1.2 + 0.15 * k['rank'], 2), inner=1.0, prio=8, group=1)
        if category == 'analog': return dict(w=SIGW, wmin=SIGMIN, pen=0.0, via=1.0, inner=0.95, prio=8, group=3)
        if category == 'gnd':
            if L <= 2:
                return dict(w=max(0.35, SIGW), wmin=SIGMIN, pen=0.5, via=1.0, inner=1.0, prio=5, group=5)
            return dict(w=0.35, wmin=0.15, pen=0.5, via=0.0, inner=99.0, prio=7, group=5)
        return dict(w=SIGW, wmin=SIGMIN, pen=0.0, via=0.9, inner=0.9, prio=8, group=4)




    VIA_F = float(CONS.get('via_factor') or {2: 4.0}.get(L, 3.0))
    for n, k in NET.items():
        rp = router_params(k)
        for key in ('w', 'wmin'):
            if key in k: rp[key] = k[key]
        rp['via'] = round(rp['via'] * VIA_F, 3)
        k.update(rp)
    MIN_WIDTH = float((RG or {}).get('router', {}).get('track_min') or 0.0)
    if MIN_WIDTH:
        for k in NET.values(): k['w'] = max(k['w'], MIN_WIDTH); k['wmin'] = max(k['wmin'], MIN_WIDTH)
    for n, b in (CONS.get('width') or {}).items():
        if n in NET:
            b = float(b); NET[n]['w'] = max(NET[n]['w'], b); NET[n]['width_spec'] = b


    anchor = [r for r in fps if ROLE[r] in ('chip', 'connector')]
    nets_of = {r: {B['pads'][i]['net'] for i in pads_by_ref[r] if B['pads'][i]['net'] in NET} for r in fps}
    anchor_on = {n: [r for r in refs_on[n] if r in anchor] for n in NET}
    def core_set(a):
        k = re.sub(r'\d+$', '', re.sub(r'^[A-Za-z]+_', '', a))
        return k.upper() if len(re.sub(r'[^A-Za-z]', '', k)) >= 2 else ''
    def hint(p, a):
        if len(a) >= 4 and re.search(r'(^|[^0-9A-Za-z])' + re.escape(a) + r'(?![0-9A-Za-z])', p): return 4.0
        k = core_set(a)
        pt = tokens(p)
        if k and pt and any(t == k or t.startswith(k) for t in pt[1:] + [pt[0][1:]]): return 2.0
        return 0.0
    def sheet(r): return fps[r].get('sheet') or ''
    def sheet_bonus(p, a):
        sp, sa = sheet(p), sheet(a)
        if not sp or not sa: return 0.0
        if sp == sa: return 3.0
        if sp.startswith(sa) or sa.startswith(sp): return 1.0
        return 0.0
    def dist(p, a): return math.hypot(fps[p]['x'] - fps[a]['x'], fps[p]['y'] - fps[a]['y'])
    def nets_of_all(r): return {B['pads'][i]['net'] for i in pads_by_ref[r]}
    ISLAND = {}
    for r, a in (CONS.get('assignment') or {}).items():
        if r in fps: ISLAND[r] = a if a != 'vacant' else None
    small_all = [r for r in fps if ROLE[r] == 'small']
    small = [r for r in small_all if r not in ISLAND]



    DCP = {}
    for r in small_all:
        if TYPE[r] != 'C' or len(pads_by_ref[r]) != 2: continue
        ns = [B['pads'][i]['net'] for i in pads_by_ref[r]]
        if len([n for n in ns if n in GNDS]) == 1:
            sup = [n for n in ns if n not in GNDS and n in NET and NET[n]['category'] in ('supply', 'current_val')]
            if sup:
                c = capacity(fps[r].get('value', '')) or 1e-7 * package(fps[r].get('fp', ''))
                DCP[r] = dict(net_name=sup[0], C=c)
    def supply_pins(a, n):
        ps = [i for i in pads_by_ref[a] if B['pads'][i]['net'] == n]
        typed = [i for i in ps if B['pads'][i].get('pintype') in ('power_in', 'power_out')]
        return typed or ps
    def clusters(idx, d=1.5):
        cl = []
        for i in idx:
            p = B['pads'][i]
            for c in cl:
                if any(math.hypot(p['x'] - B['pads'][j]['x'], p['y'] - B['pads'][j]['y']) < d for j in c): c.append(i); break
            else: cl.append([i])
        return cl



    BOOST = {}
    for u in sorted(sw_chips):
        ns = nets_of_all(u)
        sws = sorted(n for n in ns if n in NET and NET[n]['category']=='sw')
        for sw in sws:
            inside = [n for r in sorted(refs_on[sw]) if is_ind(r) for n in sorted(nets_of_all(r))
                   if n != sw and n in ns and NET.get(n,{}).get('category') in ('current_val','supply')
                   and any(re.fullmatch(r'P?VIN|VCC|VBAT|VBUS', str(B['pads'][i].get('pinfunc','')), re.I)
                           for i in pads_by_ref[u] if B['pads'][i]['net']==n)]
            outs = [(r,n) for r in sorted(refs_on[sw]) if TYPE[r] in ('D','Q')
                    for n in sorted(nets_of_all(r)) if n != sw and NET.get(n,{}).get('category') in ('current_val','supply')]
            if inside and outs: BOOST[u] = dict(vin=inside[0], sw=sw, out_port=outs[0][1], rectifier=outs[0][0])
    PRODUCERS = collections.defaultdict(set)
    for n in NET:
        for i in pads_by_net[n]:
            p = B['pads'][i]
            if p.get('pintype') == 'power_out' and ROLE[p['ref']] == 'chip': PRODUCERS[n].add(p['ref'])
        for r in refs_on[n]:
            if not is_ind(r): continue
            for m in nets_of_all(r):
                if m != n and m in NET and NET[m]['category'] == 'sw':
                    PRODUCERS[n] |= {q for q in refs_on[m] if ROLE[q] == 'chip' and not (q in BOOST and BOOST[q]['vin']==n)}
    for u,b in BOOST.items(): PRODUCERS[b['out_port']].add(u)
    assigned = collections.Counter()
    for r in sorted(DCP, key=lambda r: DCP[r]['C']):
        if r not in ISLAND: continue
        n = DCP[r]['net_name']; a = ISLAND[r]; DCP[r]['spec'] = True
        if a: assigned[(a, n)] += 1
        DCP[r]['generator'] = not (a in pads_by_ref and supply_pins(a, n))
    for r in sorted(DCP, key=lambda r: DCP[r]['C']):
        if r in ISLAND: continue
        n = DCP[r]['net_name']
        cand = [a for a in anchor_on[n] if ROLE[a] == 'chip'] + sorted(PRODUCERS[n] - set(anchor_on[n]))
        cand = cand or anchor_on[n]
        if not cand: continue
        def score(a):
            sp = supply_pins(a, n)
            need = max(1, len(clusters(sp))) if sp else 1
            s = sheet_bonus(r, a) + hint(r, a) + 1.0 * (need - assigned[(a, n)]) / need - 0.02 * dist(r, a)
            if ROLE[a] == 'connector': s -= 1.0
            if a in PRODUCERS[n] and DCP[r]['C'] >= 4.7e-6: s += 1.5 - 1.0 * (need - assigned[(a, n)]) / need
            elif a in PRODUCERS[n]: s += 0.3
            if not sp: s -= 0.5
            return s
        a = max(cand, key=score); ISLAND[r] = a; assigned[(a, n)] += 1
        DCP[r]['generator'] = a in PRODUCERS[n] and not supply_pins(a, n)




    def w_net(n, a=None):
        k = NET[n]; w = WT['current_val'] if k['category'] == 'supply' else WT.get(k['category'], 1.0) * (1 + 0.15 * k.get('rank', 0))
        if k['category'] == 'signal' and a is not None and fps[a].get('n_pads', 0) > 20: w *= 0.3 if k['rank'] == 0 else 0.5
        if a is not None and ROLE.get(a) == 'connector': w *= 0.6
        return w
    import heapq
    nbr = collections.defaultdict(list)
    for n in NET:
        if n in GNDS or NET[n]['category'] == 'gnd': continue
        rs = [r for r in refs_on[n] if ROLE[r] in ('small', 'chip', 'connector')]
        if len(rs) > (6 if NET[n]['category'] == 'supply' else 12): continue
        for p in rs:
            if ROLE[p] != 'small': continue
            for q in rs:
                if q != p: nbr[p].append((q, 1.0 / max(1e-3, w_net(n, q if q in anchor else None))))
    def next_(r0):
        dist0 = {r0: 0.0}; best = {}; h = [(0.0, r0)]
        while h:
            d, r = heapq.heappop(h)
            if d > dist0.get(r, 1e9): continue
            if r in anchor or (r != r0 and ISLAND.get(r)):
                a = r if r in anchor else ISLAND[r]; dd = d + (0.0 if r in anchor else 0.2)
                if dd < best.get(a, 1e9): best[a] = dd
                continue
            for q, c in nbr[r]:
                if d + c < dist0.get(q, 1e9): dist0[q] = d + c; heapq.heappush(h, (d + c, q))
        return best
    CLOSE = {}
    for r in small:
        if r in ISLAND: continue
        best = next_(r)
        sc = {a: -d for a, d in best.items()}
        for a in anchor:
            b = 0.3 * sheet_bonus(r, a) + 0.3 * hint(r, a)
            if b > 0: sc[a] = sc.get(a, -6.0) + b
        CLOSE[r] = sc
    for r, sc in CLOSE.items():
        ISLAND[r] = max(sc, key=lambda a: (sc[a], -dist(r, a))) if sc else None

    LOOSE = loose_connector(B, fps, ROLE, CONS, OPERATE)
    islands = {}
    for a in anchor:
        mem = [r for r in fps if ISLAND.get(r) == a]
        dec = sorted([r for r in mem if r in DCP], key=lambda r: DCP[r]['C'])
        rest = [r for r in mem if r not in DCP]
        def important(r):
            ns = [n for n in nets_of[r] if n not in GNDS]
            best = max([(NET[n]['group'] if NET[n]['category'] != 'fast' else 0, -NET[n].get('rank', 0)) for n in ns] or [(9, 0)])
            return (min(((NET[n]['group'] if NET[n]['category'] != 'fast' else 0), -NET[n].get('rank', 0)) for n in ns) if ns else (9, 0),
                    -len(nets_of[r] & nets_of[a]))
        rest.sort(key=important)
        cnt = collections.Counter()
        for r in [a] + mem:
            for n in sorted(nets_of[r]):
                if n in GNDS: continue
                cnt[NET[n]['category']] += 1 + (NET[n].get('rank', 0) if NET[n]['category'] == 'fast' else 0)
        if cnt['sw']: category = 'power'
        elif cnt['fast'] >= 2 and cnt['analog'] >= 2: category = 'mixed'
        elif cnt['fast'] >= 2: category = 'digital'
        elif cnt['analog'] >= 2: category = 'analog'
        else: category = 'neutral'
        islands[a] = dict(category=category, role=ROLE[a], pinned=(ROLE[a] == 'connector' or a in OPERATE) and a not in LOOSE or fps[a].get('locked') or glob_any(a, CONS.get('pinned')),
                         decoupling=[dict(ref=r, net_name=DCP[r]['net_name'], C=DCP[r]['C'], over_coil=DCP[r].get('generator', False),
                                           pins=[B['pads'][i]['num'] for i in supply_pins(a, DCP[r]['net_name'])]) for r in dec],
                         members=dec + rest, counter=dict(cnt))




    PNET = {n for n, k in NET.items() if k['category'] in ('supply', 'current_val', 'sw')}
    def key(p): return '%s:%s' % (p['ref'], p['num'])
    def pads_on(r, n): return [B['pads'][i] for i in pads_by_ref[r] if B['pads'][i]['net'] == n]
    bridge = collections.defaultdict(list)
    for r in fps:
        if ROLE[r] in ('connector', 'mechanics', 'without') or TYPE[r] == 'C': continue
        ns = {B['pads'][i]['net'] for i in pads_by_ref[r]} & PNET
        if len(ns) < 2 or (TYPE[r] == 'R' and not shunt(r)): continue
        large = ROLE[r] == 'chip' and fps[r].get('n_pads', 0) > 8 and not any(r in v for v in PRODUCERS.values())
        for a in ns:
            for b in ns:
                if a != b and not (r in BOOST and {a,b}=={BOOST[r]['vin'],BOOST[r]['sw']}): bridge[a].append((r, b, large))
    roots = [n for n in sorted(PNET) if not PRODUCERS.get(n) and NET[n]['category'] != 'sw' and any(ROLE[r] == 'connector' for r in refs_on[n])]
    LEVEL = {}; SOURCE = {}; OVER = {}
    for n in roots:
        LEVEL[n] = 0; SOURCE[n] = [key(p) for r in sorted(refs_on[n]) if ROLE[r] == 'connector' for p in pads_on(r, n)]
    for allow_large in (False, True):
        front = sorted(LEVEL, key=lambda n: LEVEL[n])
        while front:
            nxt = []
            for n in front:
                for r, m, large in sorted(bridge[n]):
                    if m in LEVEL or (large and not allow_large): continue
                    LEVEL[m] = LEVEL[n] + 1; SOURCE[m] = [key(p) for p in pads_on(r, m)]; OVER[m] = (r, n); nxt.append(m)
            front = nxt




    def ancestors(n):
        out = set()
        while OVER.get(n): n = OVER[n][1]; out.add(n)
        return out
    PREV = []
    for a in sorted(LEVEL):
        for r, b, large in sorted(bridge[a]):
            if b not in LEVEL or a >= b or ROLE[r] == 'chip' or (NET[a]['category'], NET[b]['category']) != ('supply', 'supply'): continue
            if TYPE[r] not in ('Q', 'D', 'F', 'PTC') or (TYPE[r] != 'Q' and len(pads_by_ref[r]) != 2): continue
            if r in (OVER.get(a, (None,))[0], OVER.get(b, (None,))[0]) or a in ancestors(b) or b in ancestors(a): continue
            PREV.append((r, a, b))





    def _count(fld, r, v, positive=False):
        try: x = float(v)
        except (TypeError, ValueError): raise ValueError('specs.%s[%s]: not a number: %r' % (fld, r, v))
        if not math.isfinite(x) or x < 0 or (positive and x <= 0):
            raise ValueError('specs.%s[%s] = %r: must be finite and %s' % (fld, r, v, '> 0' if positive else '>= 0'))
        return x
    CURRENT_CONS = {r: _count('current_val', r, v) for r, v in (CONS.get('current_val') or {}).items()}
    CONVERSION_CONS = {r: _count('implementation', r, v) for r, v in (CONS.get('implementation') or {}).items()}
    SPANN_CONS = {p: _count('voltage', p, v, True) for p, v in (CONS.get('voltage') or {}).items()}
    def voltage(n):
        for pat, v in SPANN_CONS.items():
            if re.fullmatch(pat, base_name(n), re.I): return v
        m = re.fullmatch(r'\+?(\d+)V(\d*)', base_name(n), re.I)
        return float(m.group(1) + '.' + (m.group(2) or '0')) if m else None
    def base_load(r, n_pads_on):
        t = TYPE[r]
        if r in CURRENT_CONS: return CURRENT_CONS[r], 'spec_A'
        if ROLE[r] == 'connector': return 1.0 * n_pads_on, 'assumption'
        if ROLE[r] == 'chip':
            np_ = fps[r].get('n_pads', 0)
            return (1.0 if np_ > 20 else (0.5 if np_ > 8 else 0.3)), 'assumption'
        if t == 'C': return 0.0, 'C_direct_current_0'
        if t == 'R': return (1.0, 'assumption') if shunt(r) else (0.05, 'assumption')
        if t == 'D': return 0.02, 'assumption_diode'
        return 0.5, 'assumption'
    def actual_bootstrap(m):
        return NET[m]['category'] == 'sw' and not any(is_ind(q) for q in refs_on[m])
    def regulated(m, depth=0):
        if NET[m]['category'] != 'sw' or depth > 3: return [m]
        children = [k for k, (rr, nn) in OVER.items() if nn == m]

        return [x for k in children for x in regulated(k, depth + 1)]
    EXACT = ('spec_A', 'spec', 'series_element', 'linear_regulator')
    LOAD = {}; ORIGIN = {}
    for n in sorted(LEVEL, key=lambda n: -LEVEL[n]):
        raw_data = {}; provenance = {}
        src = set(SOURCE[n])
        for r in sorted(refs_on[n]):
            ps = [p for p in pads_on(r, n) if key(p) not in src]
            if not ps: continue
            bottom = [m for m, (rr, nn) in OVER.items() if rr == r and nn == n]
            if bottom:
                off = [m for m in bottom if not actual_bootstrap(m)] or bottom
                if r in CONVERSION_CONS:
                    w = CONVERSION_CONS[r] * sum(LOAD.get(m, 0.0) for m in off); h = 'spec'
                elif any(b['vin']==n and b['sw'] in off for b in BOOST.values()):
                    targets = [b['out_port'] for b in BOOST.values() if b['vin']==n and b['sw'] in off]
                    w = sum((voltage(x) or 0)*LOAD.get(x,0) for x in targets)/(voltage(n)*0.9) if voltage(n) else sum(LOAD.get(x,0) for x in targets)
                    h = 'assumption_boost_eta0.9'
                elif ROLE[r] != 'chip':
                    w = sum(LOAD.get(m, 0.0) for m in off); h = 'series_element'
                elif r not in sw_chips:
                    w = sum(LOAD.get(m, 0.0) for m in off); h = 'linear_regulator'
                else:
                    ve = voltage(n); targets = [x for m in off for x in regulated(m)]
                    va = {x: voltage(x) for x in targets}
                    if ve and targets and all(va.values()):
                        w = sum(va[x] * LOAD.get(x, 0.0) for x in targets) / (ve * 0.9); h = 'assumption_voltage_eta0.9'
                    else:
                        w = 0.5 * sum(LOAD.get(m, 0.0) for m in off); h = 'assumption_switching_regulator'
                if h in EXACT and not all(ORIGIN.get(m) == 'A' for m in off): h += '+relative'
            elif any(key(p) in CURRENT_CONS for p in ps):
                for p in ps:
                    if key(p) in CURRENT_CONS: raw_data[key(p)] = CURRENT_CONS[key(p)]; provenance[key(p)] = 'spec_A'
                    else: raw_data[key(p)] = 0.0; provenance[key(p)] = 'assumption_pad_without_spec'
                continue
            elif r in BOOST and n==BOOST[r]['sw']: w,h = 0., 'switch_pin_no_DC_Consumer'
            else: w, h = base_load(r, len(ps))
            for p in ps: raw_data[key(p)] = w / len(ps); provenance[key(p)] = h
        LOAD[n] = sum(raw_data.values())
        pos = [provenance[k] for k, v in raw_data.items() if v > 0]
        ORIGIN[n] = 'A' if pos and all(h in EXACT for h in pos) else 'relative'
        if any(b['sw']==n for b in BOOST.values()):ORIGIN[n]='relative'
        NET[n]['flow'] = dict(level=LEVEL[n], source=SOURCE[n], over=OVER.get(n, (None, None))[0],
                                from_=OVER.get(n, (None, None))[1], load={k: round(v, 7) for k, v in raw_data.items()},
                                total=round(LOAD[n], 7), unit=ORIGIN[n], provenance=provenance)
    for u,b in BOOST.items(): NET[b['sw']]['converter'] = dict(category='boost', chip=u, **b)
    VOLTAGE = voltages(B, fps, pads_by_ref, pads_by_net, NET, ROLE, TYPE, LEVEL, SOURCE, OVER, PRODUCERS, GNDS, SPANN_CONS, refs_on, shunt, load_pad, PREV)
    def voltage_s(n): return VOLTAGE[n][0] if n in VOLTAGE else None
    CURRENT_A, PART_A, LOAD_A = current_estimate(B, fps, pads_by_ref, NET, ROLE, TYPE, LEVEL, SOURCE, OVER, LOAD, ORIGIN, CURRENT_CONS, CONVERSION_CONS,
                                              refs_on, pads_on, key, shunt, sw_chips, GNDS, voltage_s, actual_bootstrap, regulated, analog_chip, load_pad, PREV)
    for n, (a, q) in CURRENT_A.items(): NET[n]['current_A'] = a; NET[n]['current_A_source'] = q
    for n, (a, q) in signal_currents(fps, refs_on, TYPE, NET, GNDS).items():
        NET[n]['current_A'] = a; NET[n]['current_A_source'] = 'signal_' + q
    for n, (v, q) in VOLTAGE.items(): NET[n]['voltage_V'] = v; NET[n]['voltage_source'] = q





    ESTIMATE = CONS.get('current_estimate', True) is not False
    for n in sorted(LEVEL):
        fl = NET[n]['flow']
        if ORIGIN[n] == 'A': fl['unit_source'] = 'spec'; continue
        if not ESTIMATE or not LOAD_A.get(n, ({},))[0]: continue
        la, ha = LOAD_A[n]
        fl['relative'] = dict(load=fl['load'], total=fl['total'], provenance=fl['provenance'])
        fl.update(load={k: round(v, 7) for k, v in sorted(la.items())}, total=round(sum(la.values()), 7), unit='A',
                  unit_source='estimate', provenance=dict(sorted(ha.items())))
    if sw_compact:
        from analysis.switch_node import correct
        correct(B, dict(net_list=NET, specs=CONS))
    width_after_current(B, NET, CONS, pads_by_net)
    for n, k in sorted(NET.items()):
        bga = any('BGA' in fps[r].get('fp', '').upper() for r in refs_on[n])
        k.update(net_shape(n, k, bga))
    part_list = {}
    for r, f in fps.items():
        part_list[r] = dict(role=ROLE[r], island=ISLAND.get(r) if ROLE[r] == 'small' else (r if r in islands else None),
                        decoupling=r in DCP, C=DCP[r]['C'] if r in DCP else None,
                        pinned=bool(((ROLE[r] in ('connector', 'mechanics') or r in OPERATE) and r not in LOOSE) or f.get('locked') or glob_any(r, CONS.get('pinned'))), operate=r in OPERATE,
                        board_side=f['side'])
        if r in LOOSE: part_list[r]['loose'] = True

    FUNC = functions(B, fps, pads_by_ref, pads_by_net, NET, ROLE, TYPE, DCP, ISLAND, PRODUCERS, shunt, supply_pins)
    for r in part_list: part_list[r].update(FUNC[r])
    for r, (a, q) in PART_A.items(): part_list[r]['current_A'] = a; part_list[r]['current_A_source'] = q


    for r, g in programming_parts(B, fps, pads_by_ref, pads_by_net, NET, ROLE, TYPE, GNDS).items(): part_list[r]['programming'] = g
    for r, f in fps.items():
        if f.get('dnp'): part_list[r]['dnp'] = True; part_list[r]['courtyard_obstacle'] = False
    TAP =measurement_taps(B, fps, pads_by_ref, pads_by_net, NET, ROLE, TYPE, part_list, shunt)
    expensive =[r for r in fps if glob_any(r, CONS.get('expensive_under'))] or \
        [r for r in fps if ROLE[r] == 'chip' and any(k['ref'] == r for k in B.get('keepouts', []))]
    K = dict(source=B.get('src'), cu_layers=L, sigw=SIGW, gnd=gnd, gnd_main=gnd[0] if gnd else None, net_list=NET, part_list=part_list,
                islands=islands, expensive_under=expensive, net_ties=net_ties, taps=TAP, specs=CONS, **({'structure': CONS.get('structure') or (RG or {}).get('structure')} if CONS.get('structure') or (RG or {}).get('structure') else {}), design_rules=(RG or {}).get('router'))
    from analysis.spacings import contract_spec
    K['spacings'] = contract_spec(B, RG)
    if B.get('p3_pin_groups') is not None:
        from placement.p3_groups import apply
        apply(B, K)
    if disturb_flank or CONS.get('disturb_flank'):
        from analysis.aggressor import create
        K['aggressor'] = create(K)
        if disturb_flank: K['disturb_flank'] = True
    return K


def mst_length(pts):
    pts = sorted(set((round(x, 4), round(y, 4)) for x, y in pts))
    if len(pts) < 2: return 0.0
    d = {p: math.dist(pts[0], p) for p in pts[1:]}; s = 0.0
    while d:
        p = min(d, key=lambda q: (d[q], q)); s += d.pop(p)
        for q in d: d[q] = min(d[q], math.dist(p, q))
    return s


def width_after_current(B, NET, CONS, pads_by_net):
    if CONS.get('current_width') is False: return
    dT = float(CONS.get('heating_K') or IPC_DT)
    from analysis.current_width import copper_thickness
    review = CONS.get('ipc_inner_model') == '2221' or bool(CONS.get('review_fix'))
    ta, ti = (copper_thickness(CONS), copper_thickness(CONS, inner=True)) if review else (COPPER_OUTER, COPPER_INNER)
    over = [(re.compile(k), v) for k, v in (CONS.get('net_list') or {}).items()]
    def w_spec(n):
        b = base_name(n)
        for rx, v in over:
            if rx.fullmatch(b) or rx.fullmatch(n): return 'w' in v
        return False
    P = B['pads']
    def current_val(n):
        fl = NET[n].get('flow') or {}
        return float(fl.get('total') or 0.0) if fl.get('unit') == 'A' else 0.0
    def total_length(n): return BUDGET_DETOUR * mst_length([(P[i]['x'], P[i]['y']) for i in pads_by_net.get(n, [])])
    budget = {}
    for n in NET:
        if NET[n]['category'] not in ('current_val', 'supply', 'sw'): continue
        for rx, mv in (CONS.get('budget_mV') or {}).items():
            if n == rx or re.fullmatch(rx, n) or re.fullmatch(rx, base_name(n)):
                I = current_val(n)
                if I > 0 and float(mv) > 0:
                    w = RHO_CU * total_length(n) * I / (ta * float(mv) / 1000)
                    budget[n] = (w, w * ta / ti, 'budget %g mV' % float(mv))
                break
    for name, g in sorted((CONS.get('channel_budget_mV') or {}).items()):
        nn = [n for n in g.get('net_list', []) if n in NET and NET[n]['category'] in ('current_val', 'supply', 'sw')]
        S = sum(total_length(n) * current_val(n) for n in nn)
        if not nn or S <= 0 or float(g.get('mV') or 0) <= 0: continue
        w = RHO_CU * S / (ta * float(g['mV']) / 1000)
        for n in nn:
            if w > budget.get(n, (0,))[0]: budget[n] = (w, w * ta / ti, 'channel %s %g mV' % (name, float(g['mV'])))
    for n, k in sorted(NET.items()):
        if k['category'] not in ('current_val', 'supply', 'sw'): continue
        I = current_val(n)
        if I <= 0: continue
        w_ipc = ipc_width(I, dT, ta); wb, wb_i, bq = budget.get(n, (0.0, 0.0, None))
        k['w_ipc'] = round(w_ipc, 3)
        if bq: k['w_budget'] = round(wb, 3); k['w_budget_source'] = bq
        nominal = max(w_ipc, wb)
        if nominal > W_CURRENT_MAX: k['area_required'] = True
        if not w_spec(n) and min(nominal, W_CURRENT_MAX) > k['w'] + 1e-9:
            k['w'] = round(min(nominal, W_CURRENT_MAX), 3); k['w_source'] = 'budget' if wb > w_ipc else ('ipc2221' if review else 'ipc2152')
        k['w_inner'] = round(max(k['w'], min(W_CURRENT_MAX, max(INNER_FACTOR * w_ipc, wb_i))), 3)
        if not review: continue
        k['w_ipc_source'] = 'ipc2221_outer'; k['w_inner_source'] = 'heuristic_2.3 (no IPC-2152 correction)'
        if CONS.get('ipc_inner_model') == '2221':
            from analysis.current_width import ipc2221, copper_thickness
            ta, ti = copper_thickness(CONS), copper_thickness(CONS, inner=True)
            wi = ipc2221(I, dT, ti, inner=True)
            wa = ipc2221(I, dT, ta)
            k['w_ipc'] = round(wa, 3); k['w_ipc_inner'] = round(wi, 3)
            k['w_inner'] = round(max(k['w'], min(W_CURRENT_MAX, max(wi, wb * ta / ti))), 3)
            k['w_inner_source'] = 'ipc2221_inner, copper_thickness=%g mm' % ti
            if not w_spec(n): k['w'] = round(max(k['w'], min(W_CURRENT_MAX, wa)), 3)
        inner_nominal = max(wi, wb * ta / ti) if CONS.get('ipc_inner_model') == '2221' else max(INNER_FACTOR * w_ipc, wb_i)
        outer_nominal = max(nominal, wa) if CONS.get('ipc_inner_model') == '2221' else nominal
        k['w_demand_outer'] = round(outer_nominal, 3)
        k['w_demand_inner'] = round(max(k['w'], inner_nominal), 3)
        k['w_deficit_outer'] = round(max(0.0, outer_nominal - k['w']), 3)
        k['w_deficit_inner'] = round(max(0.0, inner_nominal - k['w_inner']), 3)
        if max(outer_nominal, inner_nominal) > W_CURRENT_MAX: k['area_required'] = True


def voltages(B, fps, pads_by_ref, pads_by_net, NET, ROLE, TYPE, LEVEL, SOURCE, OVER, PRODUCERS, GNDS, SPANN_CONS, refs_on, shunt, load_pad, PREV=()):
    P = B['pads']; out = {}
    def name_v(n):
        b = base_name(n)
        for pat, v in SPANN_CONS.items():
            if re.fullmatch(pat, b, re.I): return v, 'spec'
        m = re.fullmatch(r'\+?(\d+)V(\d*)', b, re.I)
        if m: return float(m.group(1) + '.' + (m.group(2) or '0')), 'name'
        m = re.search(r'(?<![A-Z0-9.])\+?(\d{1,2})[.,](\d{1,2})\s?V(?![A-Z0-9])', b.upper())
        if m: return float(m.group(1) + '.' + m.group(2)), 'name'
        for t in tokens(n):
            m = re.fullmatch(r'\+?(\d{1,2})V(\d{0,2})', t)
            if m: return float(m.group(1) + '.' + (m.group(2) or '0')), 'name'
        return None
    def regulator(n):
        if OVER.get(n):
            r, m = OVER[n]
            if ROLE[r] == 'chip': return r
            if TYPE[r] == 'L' and NET[m]['category'] == 'sw' and OVER.get(m) and ROLE[OVER[m][0]] == 'chip': return OVER[m][0]
            return None
        e = sorted(PRODUCERS.get(n, ()))
        return e[0] if e else None
    def fb_divider(n, u):
        vref = table(REGULATOR_VREF, (fps[u].get('value') or '').upper())
        if not vref: return None
        for f in sorted({P[i]['net'] for i in pads_by_ref[u]}):
            if not f or f == n or f in GNDS or f not in pads_by_net: continue
            upper, bottom = [], []
            for j in pads_by_net[f]:
                r = P[j]['ref']
                if TYPE.get(r) != 'R' or len(pads_by_ref[r]) != 2: continue
                different = [P[k]['net'] for k in pads_by_ref[r] if k != j]
                R = resistance(fps[r].get('value', ''))
                if not R or not different: continue
                if different[0] == n: upper.append(R)
                elif different[0] in GNDS: bottom.append(R)
            if upper and bottom:
                Ro = 1 / sum(1 / x for x in upper); Ru = 1 / sum(1 / x for x in bottom)
                return round(vref * (1 + Ro / Ru), 3), 'fb_divider %s (Vref %g V, %s)' % (u, vref, base_name(f))
        return None
    def diode_v(n):
        best = None
        for i in pads_by_net.get(n, []):
            r = P[i]['ref']
            if TYPE.get(r) not in ('D', 'TVS', 'Z', 'ESD') or len(pads_by_ref[r]) != 2: continue
            if not any(P[k]['net'] in GNDS for k in pads_by_ref[r] if k != i): continue
            w = (fps[r].get('value') or '').upper(); v = None
            for pat, f in TVS_V:
                m = re.search(pat, w)
                if m: v = float(m.group(1)) * f; break
            if v is None:
                m = re.search(ZENER_V, w)
                if m: v = float(m.group(1) + '.' + (m.group(2) or '0'))
            if v is None and re.search(r'TVS', '%s %s %s' % (r, w, fps[r].get('fp') or ''), re.I):
                m = re.search(TVS_OTHERWISE_V, w)
                if m: v = float(m.group(1))
            s = rail(v) if v else None
            if s and (best is None or s < best[0]): best = (s, 'protection diode %s %s' % (r, fps[r].get('value')))
        return best
    def usb_v(n):
        if LEVEL.get(n) != 0: return None
        if any(PD_CHIP.search(fps[r].get('value') or '') for r in refs_on[n]): return None
        for r in sorted(refs_on[n]):
            f = fps[r]
            if ROLE[r] == 'connector' and re.search(r'USB', '%s %s %s' % (f.get('fp', ''), f.get('value', ''), f.get('lib', '')), re.I) \
                    and any('%s:%s' % (P[i]['ref'], P[i]['num']) in SOURCE[n] for i in pads_by_ref[r] if P[i]['net'] == n):
                return 5.0, 'usb %s' % r
        return None
    for n in sorted(LEVEL):
        if NET[n]['category'] == 'sw': continue
        v = name_v(n)
        u = regulator(n) if v is None else None
        if u: v = (lambda x: (x, 'fixed voltage %s' % u) if x else None)(fixed_voltage(fps[u].get('value'))) or fb_divider(n, u)
        v = v or diode_v(n) or usb_v(n)
        if v: out[n] = v
    def series(r, a, b):
        if ROLE[r] == 'chip' or NET[a]['category'] == 'sw' or NET[b]['category'] == 'sw': return False
        return TYPE[r] in ('F', 'PTC', 'FB', 'L', 'Q') or (TYPE[r] == 'R' and shunt(r))
    edges = sorted([(OVER[n][0], OVER[n][1], n) for n in LEVEL if OVER.get(n)] + list(PREV))
    for _ in range(8):
        new = False
        for r, m, n in edges:
            if m not in LEVEL or n not in LEVEL or not series(r, m, n): continue
            if n not in out and m in out: out[n] = (out[m][0], 'like %s via %s' % (base_name(m), r)); new = True
            elif m not in out and n in out: out[m] = (out[n][0], 'like %s via %s' % (base_name(n), r)); new = True
        if not new: break
    return out


def current_estimate(B, fps, pads_by_ref, NET, ROLE, TYPE, LEVEL, SOURCE, OVER, LOAD, ORIGIN, CURRENT_CONS, CONVERSION_CONS,
                    refs_on, pads_on, key, shunt, sw_chips, GNDS, voltage, actual_bootstrap, regulated, analog_chip, load_pad, PREV=()):
    def txt(r): f = fps[r]; return '%s %s %s' % (f.get('value', '') or '', f.get('fp', '') or '', f.get('lib', '') or '')
    def meas_shunt(r):
        return any(analog_chip(q) for i in pads_by_ref[r] for q in refs_on.get(B['pads'][i]['net'], ()) if B['pads'][i]['net'] not in GNDS)
    def shunt_A(r):
        R = resistance(fps[r].get('value', ''))
        if not R: return None
        m = re.search(r'(0201|0402|0603|0805|1206|1210|1812|2010|2512|2725|2728|3920)', fps[r].get('fp', '') or '')
        Imax = math.sqrt((SHUNT_P[m.group(1)] if m else SHUNT_P_OTHERWISE) / R)
        if R < SHUNT_R_MIN: return Imax if meas_shunt(r) else None
        return min(SHUNT_U / R, Imax)
    def connector_A(r, n_pins, only_table=False):
        w = table(CONNECTOR_A, txt(r))
        if not w and only_table: return None
        per, ges = w if w else (CONNECTOR_OTHERWISE_A, None)
        return min(per * n_pins, ges) if ges else per * n_pins
    def regulator_A(r, only_table=False):
        a = table(REGULATOR_A, fps[r].get('value', '')) or table(REGULATOR_HOUSING_A, fps[r].get('fp', ''))
        return a if a or only_table else (2.0 if r in sw_chips else 0.5)
    def actual_led(r):
        return TYPE[r] == 'LED' or prefix(r) == 'LED' or bool(re.search(r'(^|[^A-Z])LED', '%s %s' % (fps[r].get('fp', ''), fps[r].get('value', '')), re.I))
    def consumer(r, n):
        t = TYPE[r]
        if r in CURRENT_CONS: return CURRENT_CONS[r], 'spec'
        if ROLE[r] == 'chip':
            a = table(CONSUMER_A, txt(r))
            if a: return a, 'family'
            if analog_chip(r): return OPV_A, 'opv'
            np_ = fps[r].get('n_pads', 0)
            return next(a for until, a in CHIP_PADS_A if np_ <= until), 'package_%dpads' % np_
        if t == 'C': return 0.0, 'C_0'
        if t == 'R':
            if shunt(r):
                a = shunt_A(r)
                return (a, 'shunt') if a is not None else (0.0, 'bridge_under_1mOhm')
            R = resistance(fps[r].get('value', ''))
            return (min((voltage(n) or 3.3) / R, R_MAX_A), 'ohm') if R else (1e-4, 'R_unknown')
        if actual_led(r) and ROLE[r] != 'chip': return table(CONSUMER_A, txt(r)) or LED_A, 'led'
        if t == 'D': return 0.0, 'diode_0'
        if t in ('F', 'PTC'):
            nominal = fuse_ampere(fps[r].get('value', ''))
            return (0.5 * nominal, 'fuse_half') if nominal else (1e-3, 'fuse_without_value')
        if t == 'Q': return (table(TRANSISTOR_A, fps[r].get('fp', ''), 0.5), 'transistor_package') if load_pad(r, n) else (0.0, 'gate')
        return 1e-3, 'otherwise'

    load_nets = collections.Counter()
    for n in LEVEL:
        src = set(SOURCE[n])
        for r in refs_on[n]:
            if ROLE[r] in ('chip', 'small') and any(key(p) not in src for p in pads_on(r, n)) \
                    and not any(rr == r and nn == n for rr, nn in OVER.values()):
                load_nets[r] += 1



    required = set()
    for n in LEVEL:
        if any(key(p) in CURRENT_CONS or p['ref'] in CURRENT_CONS for r in refs_on[n] for p in pads_on(r,n) if key(p) not in set(SOURCE[n])): required.add(n)
    for n in sorted(LEVEL, key=lambda n:-LEVEL[n]):
        if n in required and OVER.get(n): required.add(OVER[n][1])
    for n in required: NET[n]['current_required'] = True
    HD = {}
    for n in sorted(LEVEL, key=lambda n: (LEVEL[n], n)):
        d = []; src = set(SOURCE[n])
        if LEVEL[n] == 0:
            for r in sorted(refs_on[n]):
                k = sum(1 for p in pads_on(r, n) if key(p) in src)
                a = connector_A(r, k, only_table=True) if ROLE[r] == 'connector' and k else None
                if a: d.append((a, 'connector %s' % r))
        for r0 in sorted(refs_on[n]):
            if TYPE[r0] in ('F', 'PTC') and fuse_ampere(fps[r0].get('value', '')):
                nominal = fuse_ampere(fps[r0].get('value', '')); d.append((FUSE_OPERATION * nominal, 'fuse %s %gA' % (r0, nominal)))
        if OVER.get(n):
            r0, m0 = OVER[n]
            if ROLE[r0] == 'chip':
                a = regulator_A(r0, only_table=True)
                if a: d.append((a, 'regulator %s' % r0))
            elif HD.get(m0): d.append(HD[m0])
        HD[n] = min(d) if d else None
    def pass_(off_connector, one={}, sink={}, before={}):
        IA = {}; Q = {}; SH = {}; TA = collections.defaultdict(float); TQ = {}; PL = {}
        TAUS = collections.defaultdict(float); T_IN = collections.defaultdict(float); RETURN_TIME = collections.defaultdict(float)
        for n in sorted(LEVEL, key=lambda n: (-LEVEL[n], n)):
            src = set(SOURCE[n]); contrib = []; shunts = []; pl = {}
            if OVER.get(n) and TYPE[OVER[n][0]] == 'R' and shunt(OVER[n][0]):
                a = shunt_A(OVER[n][0])
                if a is not None: shunts.append(a)
            for r in sorted(refs_on[n]):
                ps = [p for p in pads_on(r, n) if key(p) not in src]
                if not ps: continue
                bottom = [m for m, (rr, nn) in OVER.items() if rr == r and nn == n]
                sh = False; per_pad = None
                if (r, n) in one:
                    w = before.get(one[(r, n)], 0.0); h = 'second_feed_after_%s' % base_name(one[(r, n)])
                    TA[r] = max(TA[r], w); TQ[r] = 'throughflow_' + h
                elif (r, n) in sink: w, h = 0.0, 'second_feed'
                elif bottom:
                    off = [m for m in bottom if not actual_bootstrap(m)] or bottom
                    ia = sum(IA.get(m, 0.0) for m in off); sh = any(SH.get(m) for m in off)
                    if r in CONVERSION_CONS: w = CONVERSION_CONS[r] * ia; h = 'implementation_spec'
                    elif any((NET.get(m,{}).get('converter') or {}).get('category')=='boost' and (NET[m]['converter']['vin']==n) for m in off):
                        targets = [NET[m]['converter']['out_port'] for m in off if (NET.get(m,{}).get('converter') or {}).get('category')=='boost']
                        ve=voltage(n)
                        w = sum((voltage(x) or 0)*IA.get(x,0) for x in targets)/(ve*.9) if ve else ia
                        h = 'boost_voltage_eta0.9'
                    elif ROLE[r] != 'chip': w = ia; h = 'series_element'
                    elif r not in sw_chips: w = ia; h = 'linear_regulator'
                    else:
                        ve = voltage(n); targets = [x for m in off for x in regulated(m)]; va = {x: voltage(x) for x in targets}
                        if ve and targets and all(va.values()): w = sum(va[x] * IA.get(x, 0.0) for x in targets) / (ve * 0.9); h = 'switching_regulator_voltage_eta0.9'
                        else: w = 0.5 * ia; h = 'switching_regulator_0.5'
                    TA[r] = max(TA[r], ia); TQ[r] = 'throughflow_' + h
                elif (NET[n].get('converter') or {}).get('chip')==r:
                    w,h=0.,'switch_pin_no_DC_Consumer'
                elif any(key(p) in CURRENT_CONS for p in ps):
                    per_pad = {key(p): CURRENT_CONS.get(key(p), 0.0) for p in ps}
                    w = sum(per_pad.values()); h = 'spec_pad'; TA[r] += w; TQ[r] = h
                elif r in CURRENT_CONS:
                    w,h=CURRENT_CONS[r],'spec';TA[r]+=w;TQ[r]=h
                elif ROLE[r] == 'connector':
                    if r in off_connector: w = off_connector[r][0] * len(ps) / off_connector[r][1]; h = 'connector_kirchhoff'
                    else: w = connector_A(r, len(ps)); h = 'connector_pin'
                    TAUS[r] += w; TQ[r] = h
                else:
                    w, h = consumer(r, n)
                    if h == 'shunt': shunts.append(w)
                    TA[r] = w; TQ[r] = h
                    w = w / max(1, load_nets[r]) if ROLE[r] == 'chip' and h != 'spec' else w
                contrib.append((w, h, r, sh))
                for p in ps: pl[key(p)] = ((per_pad[key(p)] if per_pad is not None else w / len(ps)), h)
            PL[n] = pl
            if ORIGIN.get(n) == 'A':
                IA[n] = LOAD[n]; Q[n] = 'spec'; SH[n] = False; continue
            s = sum(b[0] for b in contrib)
            main = max(contrib, key=lambda b: (b[0], b[2])) if contrib else None
            if shunts:
                a, q = sum(shunts), 'shunt'
                if HD.get(n) and a > HD[n][0]: a, q = HD[n][0], 'shunt_capped %s' % HD[n][1]
                IA[n] = a; Q[n] = q; SH[n] = True; continue
            q = 'sum(%s %s)' % (main[1], main[2]) if main and main[0] > 0 else 'no_load'
            SH[n] = bool(main and main[3])
            lid = []
            if LEVEL[n] == 0:
                for r in sorted(refs_on[n]):
                    k = sum(1 for p in pads_on(r, n) if key(p) in src)
                    if ROLE[r] == 'connector' and k: lid.append((connector_A(r, k), 'connector %s' % r))
            for r0 in sorted(refs_on[n]):
                if TYPE[r0] in ('F', 'PTC') and fuse_ampere(fps[r0].get('value', '')):
                    nominal = fuse_ampere(fps[r0].get('value', '')); lid.append((FUSE_OPERATION * nominal, 'fuse %s %gA' % (r0, nominal)))
            if OVER.get(n) and ROLE[OVER[n][0]] == 'chip': lid.append((regulator_A(OVER[n][0]), 'regulator %s' % OVER[n][0]))
            if lid and n not in required and not actual_bootstrap(n):
                d, dq = min(lid)
                if s > d: s = d; q = 'cap %s' % dq
                elif s < 1e-3: s = 0.5 * d; q = 'only_source %s' % dq
            boost=NET[n].get('converter') or {}
            if boost.get('category')=='boost':
                ve,va=voltage(boost['vin']),voltage(boost['out_port'])
                io=IA.get(boost['out_port'],s)
                if ve and va:
                    s=io*va/(ve*.9);q='boost_inductor_mean_current_eta0.9'
                    ps=pads_on(boost['chip'],n)
                    for p in ps: PL[n][key(p)]=(max(0.,s-io)/max(1,len(ps)),'boost_Switch_MeanCurrent')
            IA[n] = s; Q[n] = q



        for r in sorted(fps):
            ns=sorted(refs_on_n for refs_on_n in refs_on if r in refs_on[refs_on_n])
            if TYPE[r]!='D' or len(ns)!=4 or not any(x in GNDS for x in ns):continue
            dc=[m for m,(rr,_) in OVER.items() if rr==r and m in IA and NET[m]['category'] in ('current_val','supply')]
            ac=[m for m in ns if m in LEVEL and LEVEL[m]==0 and m not in GNDS and m not in dc]
            if len(dc)!=1 or len(ac)!=2:continue
            for m in ac:
                IA[m]=IA[dc[0]];Q[m]='rectifier_MeanCurrent_'+dc[0]
                NET[m]['current_model']='rectifier_Mean_RMS_unknown'
                for p in pads_on(r,m):PL[m][key(p)]=(IA[m]/len(pads_on(r,m)),'rectifier_Mean_RMS_unknown')
        for n in sorted(LEVEL, key=lambda n: (LEVEL[n], n)):
            if OVER.get(n) and NET.get(OVER[n][1],{}).get('converter',{}).get('category') != 'boost' and ROLE[OVER[n][0]] != 'chip' and n not in required and not SH.get(n) and IA[n] > IA.get(OVER[n][1], IA[n]):
                IA[n] = IA[OVER[n][1]]; Q[n] = 'lid_upper %s' % OVER[n][1]
        for n in LEVEL:
            for r in sorted(refs_on[n]):
                k = sum(1 for p in pads_on(r, n) if key(p) in set(SOURCE[n]))
                if ROLE[r] == 'connector' and k:
                    ret = NET[n]['category'] == 'current_val' and SH.get(n)
                    (RETURN_TIME if ret else T_IN)[r] += IA[n] * k / len(SOURCE[n]); TQ.setdefault(r, 'feed')
        for r in set(TAUS) | set(T_IN) | set(RETURN_TIME): TA[r] = max(TAUS[r], T_IN[r], RETURN_TIME[r])
        return IA, Q, SH, TA, TQ, PL
    all_items = {(r, x): True for r, a, b in PREV for x in (a, b)}
    IA, Q, SH, TA, TQ, PL = pass_({}, {}, all_items)


    off = {}
    for r in sorted(fps):
        if ROLE[r] != 'connector': continue
        ret = sum(IA[n] for n in LEVEL if LEVEL[n] == 0 and NET[n]['category'] == 'current_val' and SH.get(n)
                    and any(key(p) in set(SOURCE[n]) for p in pads_on(r, n)))
        pins = sum(1 for n in LEVEL if NET[n]['category'] in ('supply', 'current_val') for p in pads_on(r, n) if key(p) not in set(SOURCE[n]))
        if ret > 0 and pins: off[r] = (ret, pins)
    if off: IA, Q, SH, TA, TQ, PL = pass_(off, {}, all_items)


    if PREV:
        one = {}
        def owned(n): return 0.0 if Q.get(n, '').startswith(('only_source', 'no_load')) else IA.get(n, 0.0)
        for r, a, b in PREV:
            v, z = (a, b) if owned(a) >= owned(b) else (b, a)

            if owned(v) <= 0 or owned(z) > 0.1 * owned(v): continue
            one[(r, z)] = v
        sink = {k: True for k in all_items if k not in one}
        for _ in range(2):
            if one: IA, Q, SH, TA, TQ, PL = pass_(off, one, sink, IA)


    pads = {}
    for n in sorted(PL):
        s = sum(w for w, _ in PL[n].values())
        f = IA[n] / s if s > 1e-12 else 0.0
        pads[n] = ({k: w * f for k, (w, _) in PL[n].items()}, {k: 'estimate_' + h for k, (_, h) in PL[n].items()})
    net_list = {n: (round(IA[n], 9), Q[n]) for n in sorted(IA)}
    part_list = {r: (round(TA[r], 9), TQ[r]) for r in sorted(TA) if r in TQ}
    return net_list, part_list, pads



MEAS_PIN = re.compile(r'SENSE|SNS|ISNS|ISEN|IMON|ADC|AIN|^CS[PN+-]?$|^V?FB$|^IN[+-]$|^[+-]IN|^[+-]$|NINV|^INV', re.I)

MEAS_NET = re.compile(r'CURR|SENS|SNS|ISNS|IMON|SHUNT|MEAS|ADC|AIN\d|CSP|CSN', re.I)


def measurement_taps(B, fps, pads_by_ref, pads_by_net, NET, ROLE, TYPE, part_list, shunt):
    P = B['pads']
    def key(i): return '%s:%s' % (P[i]['ref'], P[i]['num'])
    def category(n): return NET[n]['category'] if n in NET else None
    def two_pole(r, t): return ROLE.get(r) == 'small' and TYPE.get(r) in t and len(pads_by_ref[r]) == 2
    def actual_shunt(r):
        m = re.match(r'\s*R(\d+)\b', fps[r].get('value', '') or '')
        return shunt(r) or (TYPE.get(r) == 'R' and bool(m) and 0 < float('0.' + m.group(1)) <= 1.0)
    def meas(m, n, depth=0):
        if m not in NET or m == n or NET[m].get('kelvin') or category(m) in ('gnd', 'current_val', 'supply', 'sw'): return False
        if category(m) == 'analog' or MEAS_NET.search(base_name(m)): return True
        for i in pads_by_net.get(m, []):
            if ROLE.get(P[i]['ref']) == 'chip' and MEAS_PIN.search((P[i].get('pinfunc') or '').strip()): return True
        if depth == 0:
            for i in pads_by_net.get(m, []):
                r = P[i]['ref']
                if two_pole(r, ('R',)):
                    m2 = next(P[j]['net'] for j in pads_by_ref[r] if j != i)
                    if m2 != n and meas(m2, n, 1): return True
        return False
    out = {}
    for n in sorted(pads_by_net):
        k = NET.get(n) or {}
        if not (k.get('kelvin') or k.get('category') in ('current_val', 'supply')): continue
        ps = pads_by_net[n]
        if k.get('kelvin'): st = [i for i in ps if P[i]['ref'] == k['kelvin']]; wo = 'Net-Tie %s' % k['kelvin']
        else:
            st = [i for i in ps if part_list.get(P[i]['ref'], {}).get('function') == 'shunt' or actual_shunt(P[i]['ref'])]; wo = 'Shunt'
            if not st and k.get('category') == 'current_val':
                st = [i for i in ps if TYPE.get(P[i]['ref']) == 'Q']; wo = 'Load terminal transistor'
        if not st: continue
        taps = []
        for i in ps:
            r = P[i]['ref']
            if i in st or ROLE.get(r) != 'small' or TYPE.get(r) not in ('R', 'C', 'D') or actual_shunt(r) \
                    or part_list.get(r, {}).get('function') in ('shunt', 'decoupling_small', 'decoupling_medium', 'decoupling_large'): continue
            other = [P[j]['net'] for j in pads_by_ref[r] if j != i and P[j]['net'] != n]
            if not other or any(category(m) in ('gnd', 'supply') or (NET.get(m) or {}).get('kelvin') for m in other): continue
            goal = [m for m in other if meas(m, n)]
            if TYPE[r] in ('R', 'C') and (len(other) != 1 or not goal): continue
            if TYPE[r] == 'D' and not goal: continue
            s = min(st, key=lambda j: (math.hypot(P[j]['x'] - P[i]['x'], P[j]['y'] - P[i]['y']), key(j)))
            taps.append(dict(pad=key(i), start=key(s), reason='%s %s -> %s, Start at %s' % (TYPE[r], fps[r].get('value', ''), base_name(goal[0]), wo)))
        if not taps: continue
        for i in ps:
            r = P[i]['ref']
            if i in st or not two_pole(r, ('R',)) or any(t['pad'] == key(i) for t in taps): continue
            v = resistance(fps[r].get('value', '')); m = next(P[j]['net'] for j in pads_by_ref[r] if j != i)
            if v and v >= 1e5 and category(m) == 'supply':
                taps.append(dict(pad=key(i), start=None, reason='R %s at %s: currentless pull-up, no sense line' % (fps[r].get('value', ''), base_name(m))))
        out[n] = sorted(taps, key=lambda t: t['pad'])
    return out
PROG_CONNECTOR = re.compile(r'TAG.?CONNECT|TC20[35]0|SWD|JTAG|ICSP|\bISP\b|_ISP|UPDI|CORTEX.?DEBUG|DEBUG|PROG', re.I)
PROG_DEBUG = SWD_T | {'UPDI', 'SWIM', 'PDI', 'MCLR'}
PROG_RESET = {'NRST', 'RST', 'RESET', 'NRESET', 'MCLR'}
PROG_ICSP = {'MISO', 'MOSI', 'SCK'}
PROG_UART = {'TX', 'RX', 'TXD', 'RXD'}
TASTER = re.compile(r'BUTTON|SWITCH|TASTER|SW_PUSH|TACT|PUSH', re.I)


def prog_hits(T):
    d = T & PROG_DEBUG; rs = T & PROG_RESET; ic = T & PROG_ICSP; ua = T & PROG_UART
    if len(d) >= 2 or (d and rs): return d | rs
    if len(ic) >= 2 and rs: return ic | rs
    if ua and T & {'DTR', 'RTS', 'BOOT', 'BOOT0'}: return ua | (T & {'DTR', 'RTS', 'BOOT', 'BOOT0'})
    if ua and T & {'IO0', 'GPIO0'} and T & (PROG_RESET | {'EN', 'CHIP_PU'}): return ua | (T & {'IO0', 'GPIO0', 'EN', 'CHIP_PU'}) | rs
    return set()


def programming_parts(B, fps, pads_by_ref, pads_by_net, NET, ROLE, TYPE, GNDS):
    P = B['pads']; out = {}
    def net_list(r): return [P[i]['net'] for i in pads_by_ref[r]]
    def names(n, r=None):
        s = set(tokens(n)) if n else set()
        for i in pads_by_net.get(n, []):
            if r is None or P[i]['ref'] == r: s |= set(tokens(P[i].get('pinfunc') or ''))
        return s
    def signal(n): return n in NET and NET[n]['category'] not in ('gnd', 'supply', 'current_val', 'sw')
    def chip_on(n): return any(ROLE.get(P[i]['ref']) == 'chip' for i in pads_by_net.get(n, []))
    for r in sorted(fps):
        f = fps[r]
        if ROLE[r] not in ('connector', 'without', 'small', 'mechanics'): continue
        txt = '%s %s %s' % (f.get('fp', ''), f.get('value', ''), f.get('lib', ''))
        sig = sorted({n for n in net_list(r) if signal(n)})
        hits = sorted(prog_hits({t for n in sig for t in names(n, r)}))
        if re.search(r'TAG.?CONNECT|TC20[35]0', txt, re.I): out[r] = 'Tag-Connect'
        elif (ROLE[r] == 'connector' or f.get('dnp')) and sig and (PROG_CONNECTOR.search(txt) or hits):
            out[r] = '%sprogramming connector (%s)' % ('unpopulated ' if f.get('dnp') else '', ', '.join(hits) or PROG_CONNECTOR.search(txt).group(0))
    p_nets = {n for r in out for n in net_list(r) if signal(n)}
    for r in sorted(fps):
        if r in out or len(pads_by_ref[r]) < 2: continue
        ns = net_list(r); an = [n for n in ns if n in p_nets]
        if not an: continue
        rest = [n for n in ns if n not in p_nets]
        if TYPE[r] == 'R' and len(ns) == 2 and all(signal(n) for n in ns) and any(not chip_on(n) for n in an) and any(chip_on(n) for n in ns):


            out[r] = 'Series resistor at programming connector (%s)' % ', '.join(sorted({base_name(n) for n in an}))
        elif TYPE[r] in ('D', 'TVS', 'ESD') and rest and all(n in GNDS for n in rest):
            out[r] = 'Protection diode at programming connector (%s)' % ', '.join(sorted({base_name(n) for n in an}))
        elif (TASTER.search('%s %s' % (fps[r].get('fp', ''), fps[r].get('value', ''))) or prefix(r) in ('SW', 'BTN', 'S', 'KEY')) \
                and rest and all(n in GNDS for n in rest):
            out[r] = 'Button at programming connector (%s)' % ', '.join(sorted({base_name(n) for n in an}))
    return out


def net_shape(n, k, bga=False):
    if 'shape_rmax_mm' in k and (not math.isfinite(float(k['shape_rmax_mm'])) or float(k['shape_rmax_mm']) <= 0):
        raise ValueError('invalid shaping radius for '+n)
    if k.get('shaping'):
        if k['shaping'] not in ('45r', 'arcs', 'organic'):
            raise ValueError('unknown net shaping for %s: %s' % (n, k['shaping']))
        radius=float(k.get('shape_rmax_mm', .5 if k['shaping']=='45r' else 3.))
        if not math.isfinite(radius) or radius<=0:
            raise ValueError('invalid shaping radius for '+n)
        return dict(shaping=k['shaping'], shape_reason=k.get('shape_reason','Spec'), shape_rmax_mm=radius)
    if k.get('category') in ('current_val', 'supply', 'sw'):
        f, g = 'organic', 'Current path (width anchors remain)'
    elif k.get('pair'):
        f, g = 'arcs', 'Differential pair (together, concentric)'
    elif re.search(r'(?:^|[/_])(RF|ANT|ANTENNA|LNA)(?:$|[_0-9])', n, re.I) or 'RF matching' in k.get('reason', '') or k.get('rf'):
        f, g = 'arcs', 'RF/AntennaLine'
    elif k.get('length_group') or k.get('interface') in ('ddr3', 'sdram', 'qspi', 'spi', 'sdio') :
        f, g = '45r', 'Bus/BGA (packing and length matching)'
    elif (k.get('rank') or 0) >= 5:
        f, g = 'arcs', 'Rank-5-SingleNet'
    else:
        f, g = '45r', 'GPIO/analog/slow'
    return dict(shaping=f, shape_reason=g, shape_rmax_mm=float(k.get('shape_rmax_mm', .5 if f == '45r' else 3.)),
                **({'shape_fan_out': '45r'} if bga else {}))


def report(K):
    out = []; N = K['net_list']
    c = collections.Counter((k['category'], k.get('current_val') or k.get('rank')) for k in N.values())
    out.append('Layers %d, signal width %.2f mm, GND %s' % (K['cu_layers'], K['sigw'], K['gnd']))
    for category in ('sw', 'fast', 'supply', 'current_val', 'analog', 'signal', 'gnd'):
        ns = sorted([n for n, k in N.items() if k['category'] == category], key=lambda n: (-(N[n].get('rank') or 0), n))
        if not ns: continue
        out.append('%-11s %3d  %s' % (category, len(ns), ', '.join('%s%s' % (base_name(n), '' if category in ('gnd',) else
                                      '[%s]' % (N[n]['current_val'] if N[n].get('current_val') else N[n]['rank'])) for n in ns[:40]) + (' ...' if len(ns) > 40 else '')))
    fl = {n: k['flow'] for n, k in N.items() if k.get('flow')}
    if fl:
        children = collections.defaultdict(list)
        for n, f in fl.items():
            if f['from_']: children[f['from_']].append(n)
        lines = []
        def tree_idx(n, depth):
            f = fl[n]; k = N[n]
            sa = k.get('current_A')
            if f['unit'] == 'A': load = '%.3g A (%s)' % (f['total'], f.get('unit_source') or 'A')
            else: load = 'Load %.2f relative' % f['total']
            extra = '' if sa is None or f.get('unit_source') == 'spec' else ' from %s' % k.get('current_A_source')
            if k.get('voltage_V') is not None: extra += ', %g V (%s)' % (k['voltage_V'], k.get('voltage_source'))
            if k.get('w_source'): extra += ', w %.2f mm (%s)' % (k['w'], k['w_source'])
            if k.get('area_required'): extra += ', area needed'
            if k.get('w_deficit_inner'): extra += ', inside missing %g mm (need %g mm)' % (k['w_deficit_inner'], k['w_demand_inner'])
            if k.get('w_deficit_outer'): extra += ', outside missing %g mm (need %g mm)' % (k['w_deficit_outer'], k['w_demand_outer'])
            lines.append('  ' * depth + '%s  [Source %s, %s%s]' % (base_name(n), (f['over'] or ','.join(f['source'][:2])), load, extra))
            for m in sorted(children[n], key=lambda m: -fl[m]['total']): tree_idx(m, depth + 1)
        for n in sorted([n for n, f in fl.items() if not f['from_']], key=lambda n: -fl[n]['total']): tree_idx(n, 0)
        out.append('Current flow (source -> consumer; A = constraint or estimate from the schematic, voltage, width by current):')
        out += ['  ' + z for z in lines]
    R = collections.Counter(t['role'] for t in K['part_list'].values())
    out.append('Parts: ' + ', '.join('%s %d' % kv for kv in sorted(R.items())))
    for a, I in sorted(K['islands'].items(), key=lambda kv: (-len(kv[1]['members']), kv[0])):
        if not I['members'] and I['role'] == 'connector': continue
        dec = ' '.join('%s(%s@%s)' % (e['ref'], si(e['C']), base_name(e['net_name'])) for e in I['decoupling'])
        rest = [r for r in I['members'] if not K['part_list'][r]['decoupling']]
        out.append('Island %-14s %-9s %2d parts | decoupling: %s | others: %s' % (a, I['category'], len(I['members']), dec or '-', ' '.join(rest)))
    for n, ts in sorted((K.get('taps') or {}).items()):
        out.append('Sense tap at %-10s %s' % (base_name(n), ' '.join('%s(from %s)' % (t['pad'], t['start'] or 'currentless') for t in ts)))
    prog = sorted((r, t['programming']) for r, t in K['part_list'].items() if t.get('programming'))
    if prog: out.append('Programming parts (P25): ' + '; '.join('%s %s' % rt for rt in prog))
    dnp = sorted(r for r, t in K['part_list'].items() if t.get('dnp'))
    if dnp: out.append('unpopulated (courtyard no obstacle): ' + ' '.join(dnp))
    vacant = [r for r, t in K['part_list'].items() if t['role'] == 'small' and not t['island']]
    if vacant: out.append('without island: ' + ' '.join(sorted(vacant)))
    post = collections.defaultdict(list)
    for r, t in sorted(K['part_list'].items()):
        if t['role'] == 'small' and t.get('function'): post[t['function']].append(r)
    if post:
        out.append('Function of the small parts (from context in the net graph; part -> supported pin):')
        def short(r):
            t = K['part_list'][r]; s = t.get('supports') or {}
            goal = '%s:%s' % (s['ref'], s['pin']) if s.get('ref') and s.get('pin') else ''
            if s.get('over_coil'): goal += ' Output %s' % s.get('regulator')
            w = '%s ' % si(t['C']) if t.get('C') else ''
            return '%s(%s%s)' % (r, w, goal) if (w or goal) else r
        for f in list(FUNCTIONS) + sorted(set(post) - set(FUNCTIONS)):
            if post.get(f): out.append('  %-18s %3d  %s' % (f, len(post[f]), ' '.join(short(r) for r in post[f])))
        for f in ('rc_element', 'misc'):
            for r in post.get(f, []):
                if K['part_list'][r].get('function_reason'): out.append('    %-10s %-9s %s' % (r, f, K['part_list'][r]['function_reason']))
    return '\n'.join(out)


def si(c):
    for f, u in ((1e-6, 'u'), (1e-9, 'n'), (1e-12, 'p')):
        if c >= f: return ('%g' % round(c / f, 2)) + u
    return '%g' % c

if __name__ == '__main__':
    B, RG, CONS, kw = load(sys.argv)
    K = analyze(B, RG, CONS, impedance=kw.get('impedance') == '1', disturb_flank=kw.get('disturb_flank') == '1', pairs=kw.get('pairs', kw.get('pair')) == '1', sw_compact=kw.get('sw_compact') == '1')
    json.dump(K, open(sys.argv[2], 'w'), indent=1)
    txt = report(K)
    open(sys.argv[2].replace('.json', '.txt'), 'w').write(txt + '\n')
    print(txt)
