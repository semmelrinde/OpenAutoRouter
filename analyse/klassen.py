""
import json, sys, re, os, fnmatch, collections, math

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gemeinsam.werte import kapazitaet, widerstand, praefix, tokens, basisname, bauform, typ

STECKER_PFX = {'J', 'CN', 'CON', 'P', 'USB', 'HDR', 'X', 'XS'}
STECKER_HINT = re.compile(r'conn|jst|header|fpc|ffc|usb|tag.?connect|tc20[35]0|molex|hirose|terminal|socket|pinhead|pin_head|_ph_|_sh_|\bph\b', re.I)
MECH_FP = re.compile(r'MountingHole|Mounting_Hole|Fiducial|LOGO|Logo|OSHW|TestPoint|Measurement_Point', re.I)


BEDIEN_HINT = re.compile(r'(^|[^A-Z])LED|WS281|SK68|APA10|BUTTON|SWITCH|TASTER|BUZZER|BEEPER|SPEAKER|DISPLAY|OLED|LCD|POTENTIOMETER|ENCODER', re.I)
MECH_PFX = {'H', 'MH', 'FID', 'REF', 'LOGO', 'TP', 'SW', 'BTN', 'S', 'BT', 'MK', 'LS', 'K', 'G', 'KIBUZZARD'}
PASSIV_PFX = {'R', 'C', 'L', 'FB', 'D', 'F', 'Y', 'Q', 'TVS', 'RN', 'RV', 'VR', 'NTC', 'PTC', 'FL', 'Z', 'ESD', 'DS', 'NT', 'JP', 'SJ', 'RA', 'CA'}
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
AN_T3 = {'AIN', 'VREF', 'REF', 'VCM', 'BIAS', 'MICBIAS', 'MIC', 'NTC', 'INP', 'INN', 'INM'}
AN_T2 = {'FB', 'SENSE', 'SNS', 'ISENSE', 'VSENSE', 'ISNS', 'VSNS', 'TRIM', 'HR', 'CTF', 'ANALOG', 'AUDIO'}
PWR_START = ('VIN', 'VBUS', 'VBAT', 'VSYS', 'VCC', 'VDD', 'AVDD', 'DVDD', 'IOVDD', 'VCORE', 'VLED', 'PVIN', 'VMOT', 'VOUT', 'VPP',
             'DREG', 'AREG', 'VREG', 'PWR', 'POWER', 'VSUP', 'SUPPLY')
PWR_HOCH = ('VIN', 'VBUS', 'VBAT', 'VSYS', 'PVIN', 'VLED', 'VMOT', 'PWR', 'POWER')
PWR_KLEIN = ('AVDD', 'VREF', 'DREG', 'AREG', 'OPV', 'ADC', 'DAC', 'ANALOG', 'AUDIO', 'BIAS')

GEW = {'sw': 3.0, 'schnell': 2.0, 'analog': 2.0, 'strom': 1.5, 'signal': 1.0, 'versorgung': 0.3}
GPIO_NR = re.compile(r'GPIO\d+|IO\d{1,2}|P[A-K]\d{1,2}')
HF_SPULE_BIS = 0.2e-6
STEUER_R_AB = 1000.0
KLEINSIGNAL_Q = re.compile(r'SOT-?23(?!-?[5-8])|SC-?70|SOT-?323|SOT-?523|SOT-?416|TO-?92', re.I)
ZWEIPOL_T = ('L', 'R', 'C', 'F', 'Y')
TREIBER_CHIP = re.compile(r'DRV8|DRV10|TB6612|TB67|A4988|A3967|TMC2|L29[38]|BTS7|BTN79|VNH\d|MP6500|ULN200|ULN280|TPA31|TPA20|PAM84|MAX9835|MAX9735|MAX9744', re.I)
USB_FS = re.compile(r'ESP32-?S[23]|ESP32-?C[36]|ESP32-?H2|RP2040|RP2350|ATMEGA\d*U\d|AT90USB|SAMD\d|ATSAMD|SAML2|STM32[FLG][0134]|STM32WB|'
                    r'CH32V|CH55\d|CH34[013]|CH910\d|CP210\d|FT23[01]X|FT232R|PL2303|NRF52840|NRF5340|PIC32MX|PIC18F\d+K50|EFM32|LPC11U|GD32F[13]|GD32VF', re.I)
USB_HS = re.compile(r'USB25[01]\d|USB2422|USB2640|USB33[0-4]\d|FT232H|FT2232H|FT4232H|FT60[01]|CY7C68013|CYUSB|FE1\.1|GL85[02]|GL3520|VL81\d|'
                    r'TUSB\d|LAN95\d\d|LAN78\d\d|AX88179|RTL8153|CY7C65', re.I)
TREIBER_AUS = re.compile(r'OUT|^[AB]O\d|^[AB]\d?OUT|MOTOR|^M\d|SPK|^[LR]OUT|^SW', re.I)

def typ_teil(ref, f):
    ""
    t = typ(ref, f.get('fp', ''))
    if t in ZWEIPOL_T and (f.get('n_pads') or 0) > 6 and typ('X', f.get('fp', '')) != t: return 'U'
    return t








SHUNT_U = 0.1
SHUNT_P = {'0201': 0.05, '0402': 0.063, '0603': 0.1, '0805': 0.125, '1206': 0.25, '1210': 0.5, '1812': 0.5, '2010': 0.75, '2512': 1.0,
           '2725': 2.0, '2728': 2.0, '3920': 3.0}
SHUNT_P_SONST = 2.0
SHUNT_R_MIN = 1e-3
SICHERUNG_BETRIEB = 0.6
STECKER_A = (
    (r'XT90', 45.0, None), (r'XT60', 30.0, None), (r'XT30', 15.0, None), (r'USB.?C|TYPE.?C', 0.75, 3.0),
    (r'MICRO.?USB|USB.?MICRO|MICROB', 1.0, 1.8), (r'MINI.?USB|USB.?MINI', 0.5, 0.5), (r'USB', 0.5, 1.5),
    (r'BARREL|DC.?JACK|JACK.?DC|HOHL|PJ-0\d\d', 3.0, 3.0), (r'TERMINAL|SCREW|KLEMME|PHOENIX|WAGO|KF\d', 10.0, None),
    (r'MINI.?FIT|5566', 9.0, None), (r'MICRO.?FIT|43045|43650', 5.0, None), (r'JST.?VH|_VH_|\bVH\b', 10.0, None),
    (r'JST.?XH|_XH_|\bXH\b|B\d+B-XH', 3.0, None), (r'JST.?EH|_EH_|\bEH\b', 3.0, None), (r'JST.?PH|_PH_|\bPH\b|-PH-', 2.0, None),
    (r'JST.?PA\b|_PA_|\dB-PASS', 3.0, None), (r'JST.?ZH|_ZH_|\bZH\b', 1.0, None), (r'JST.?SH|_SH_|\bSH\b|-SH-', 1.0, None), (r'JST.?GH|_GH_|\bGH\b', 1.0, None),
    (r'FPC|FFC', 0.5, None), (r'TAG.?CONNECT|TC20[35]0', 0.1, None), (r'1\.27|1,27|P1\.27', 1.0, None),
    (r'PINHEADER|PIN_HEADER|HEADER|2\.54|PINSOCKET', 3.0, None))
STECKER_SONST_A = 1.0
REGLER_A = (
    (r'TPS548A28|TPS548B22', 15.0), (r'LTC3833|LTC3866', 20.0), (r'TPS56C215|TPS56C230', 12.0), (r'TPS568230', 8.0), (r'TPS56637', 6.0),
    (r'TPS6293\d|TPS5430|TPS5450|LM2596|MP1584|TPS54331|TPS563\d{3}|TPS62130|MP2307|LMR3361|LMR33630', 3.0),
    (r'AP6320\d|SY8089|TPS6208\d|TPS62A0\d|MP2359|RT8059|MP2451', 2.0), (r'LM2576|LM2575|XL1509', 2.0),
    (r'AMS1117|LM1117|NCP1117|LD1117|AZ1117', 1.0), (r'TLV757|AP7361|LP38692', 1.0), (r'^L?78M', 0.5), (r'^L?78L|^79L', 0.1),
    (r'^L?78\d|^79\d', 1.0), (r'TLV755|TLV733|AP2112|RT9013|ME6211|LP5907|TLV702', 0.5), (r'AP2210|LP2985|MIC5219', 0.3),
    (r'XC6206|XC6220|HT7[35]\d\d|MCP1700|MCP1702|MCP1703|TPS7A02|TLV713', 0.25))
REGLER_GEH_A = ((r'SOT-?223|DPAK|TO-?252|TO-?263|D2PAK|TO-?220', 1.0), (r'SOT-?23|SC-?70|SOT-?353|DFN|SON|X2SON', 0.3))
VERBRAUCHER_A = (
    (r'ESP32|ESP-?WROOM|ESP-?WROVER|ESP32-?S3|ESP32-?C\d', 0.35), (r'ESP8266|ESP-?12', 0.25), (r'RP2040|RP2350', 0.05),
    (r'STM32H7|STM32F7|IMXRT|MIMXRT', 0.3), (r'STM32|GD32|CH32|SAMD|ATSAM|LPC\d', 0.1), (r'NRF52|NRF53|CC26|CC13|EFR32', 0.02),
    (r'ATMEGA|ATTINY|PIC\d', 0.02), (r'W5500|LAN87|KSZ|DP83', 0.15), (r'SX12\d\d|RFM9|LLCC68', 0.12),
    (r'WS281|SK68|APA10|SK98', 0.04), (r'DRV8|TB6612|A4988|TMC2|L298|BTS7', 1.0), (r'TPA31|PAM84|MAX9835|MAX9735', 0.5))
CHIP_PADS_A = ((8, 0.005), (20, 0.01), (64, 0.05), (10 ** 6, 0.1))
OPV_A = 0.002
LED_A = 0.02
LED_LEISTUNG_A = ((r'XHP|COB', 1.0), (r'3535|XP-?[EGLM]\d?|XM-?L|OSLON|LUXEON|SST-?\d', 0.35), (r'3030|3020', 0.15))
LUEFTER = re.compile(r'FAN|L(UE|Ü)FTER|MOTOR|PUMP', re.I)
LUEFTER_A = 0.25
SIGNAL_I_MIN = 0.1


def signal_stroeme(fps, refs_on, TYP, NETZ, GNDS):
    ""
    out = {}
    for n in sorted(refs_on):
        k = NETZ.get(n) or {}
        if n in GNDS or k.get('art') in ('strom', 'versorgung', 'sw') or k.get('strom_A') is not None: continue
        kand = []
        for r in sorted(refs_on[n]):
            f = fps.get(r) or {}; txt = '%s %s' % (f.get('value', '') or '', f.get('fp', '') or '')
            if TYP.get(r) == 'LED' or praefix(r) == 'LED' or re.search(r'(^|[^A-Z])LED', txt, re.I):
                a = tabelle(LED_LEISTUNG_A, txt)
                if a: kand.append((a, 'led_leistung %s' % r))
            elif TYP.get(r) == 'R':
                R = widerstand(f.get('value', ''))
                if R and SHUNT_R_MIN <= R <= 1.0:
                    m = re.search(r'(0201|0402|0603|0805|1206|1210|1812|2010|2512)', f.get('fp', '') or '')
                    kand.append((min(SHUNT_U / R, math.sqrt((SHUNT_P[m.group(1)] if m else SHUNT_P_SONST) / R)), 'shunt %s' % r))
            elif (praefix(r) in ('J', 'P', 'CN', 'X') and LUEFTER.search(txt)) or (LUEFTER.search(n) and praefix(r) in ('J', 'P', 'CN', 'X', 'M')):
                kand.append((LUEFTER_A, 'luefter %s' % r))
        if kand:
            a, q = max(kand)
            if a >= SIGNAL_I_MIN: out[n] = (round(a, 4), q)
    return out
R_MAX_A = 0.01
TRANSISTOR_A = ((r'SOT-?23|SC-?70|SOT-?323|SOT-?523', 0.5), (r'SOT-?223|SO-?8|SOIC-?8|DFN|PowerPAK|TDSON|SON', 2.0),
                (r'DPAK|TO-?252|TO-?263|D2PAK|TO-?220|TO-?247', 5.0))



FEST_V = ((r'^(?:L|LM|MC|UA|KA|AN|NJM)?78(?:L|M)?(\d\d)', '78'), (r'AP632\d([35])', 'ap'),
          (r'(?:LM2596|LM2576|LM2575|LM2574|LM2594|XL1509|LM2678|LM2679)[A-Z]*-(\d{1,2}(?:\.\d)?)', 'volt'),
          (r'(?:AMS1117|LM1117|AZ1117|NCP1117|LDL?1117|SPX1117|TLV1117|AP2112|AP2210|AP7361|AP7333|RT9013|RT9193|MIC5219|MIC5205|LP5907|'
           r'LP2985|SPX3819|MCP170[0-3]|MCP1711|MCP1825|ME6211|ME6206|XC6206|XC6210|XC6220|HT7[1357]|TLV7\d\d|TPS7\d\d|ADP\d{3,4})'
           r'[A-Z]{0,5}-?(\d{1,2}\.\d{1,2}|\d{2}|\d(?![\d.]))', 'ldo'))
REGLER_VREF = ((r'TPS6293\d|TPS54331|TPS6213\d|TPS6214\d|TPS6215\d|TPS56[23]\d{3}|MP1584|MP2451|MP2315|AP632\d0|AP3211', 0.8),
               (r'TPS5430|TPS5450|TPS5410', 1.221), (r'LM2596|LM2576|LM2575|LM2574|LM2594|XL1509|XL401[56]|LM267[89]', 1.23),
               (r'MP2307', 0.925), (r'MP2359', 0.81), (r'SY8089|SY8088|SY8113|SY8120|RT8059|RT8008|MT3608|TLV6256\d|TPS62A0\d', 0.6),
               (r'LMR3363\d|LMR3361\d|LMR1430', 1.0), (r'AMS1117|LM1117|NCP1117|AZ1117|LDL?1117|LM317|LM108[56]', 1.25),
               (r'MCP1640|MCP16251', 1.21), (r'TPS6102[23]', 0.595), (r'TPS630[02]\d', 0.5))
TVS_V = ((r'(?:SMAJ|SMBJ|SMCJ|SMDJ|SMF|SMLJ)(\d+(?:\.\d+)?)', 1.0),
         (r'(?:P4SMA|P6SMB|1\.5SMC|P6KE|1\.5KE|P4KE)(\d+(?:\.\d+)?)', 0.85))
ZENER_V = r'(?:BZX|BZT|BZV|MMSZ|MMBZ|ZMM|ZPD)\w*?(\d+)V(\d)?'
TVS_SONST_V = r'(?<![\dA-Z.])(\d{1,2}(?:\.\d)?)V\b'
PD_CHIP = re.compile(r'CH22[14]|IP272\d|IP2716|HUSB238|STUSB45|FUSB302|TPS6598|AP33772|CYPD3177|SW2303', re.I)
SCHIENEN_V = (1.8, 2.5, 3.3, 5.0, 9.0, 12.0, 19.0, 24.0, 36.0, 48.0)


IPC_DT = 10.0
IPC_K_AUSSEN = 0.048
KUPFER_AUSSEN = 0.035
KUPFER_INNEN = 0.0175
INNEN_FAKTOR = 2.3
W_STROM_MAX = 2.5
RHO_CU = 1.72e-5
BUDGET_UMWEG = 1.3


def ipc_breite(I, dT=IPC_DT, t=KUPFER_AUSSEN):
    ""
    from analyse.strombreite import ipc2221
    return ipc2221(I, dT, t)


def festspannung(wert):
    ""
    w = (wert or '').upper()
    if 'ADJ' in w: return None
    for pat, art in FEST_V:
        m = re.search(pat, w)
        if not m: continue
        c = m.group(1)
        if art == '78': return 3.3 if c == '33' else (2.5 if c == '25' else float(int(c)))
        if art == 'ap': return {'3': 3.3, '5': 5.0}[c]
        v = float(c) if art == 'volt' or '.' in c or len(c) == 1 else int(c[0]) + int(c[1]) / 10.0
        return v if v >= 0.8 else None
    return None


def schiene(v):
    ""
    s = [x for x in SCHIENEN_V if x <= v * 1.02]
    return s[-1] if s else None


def wert_ampere(txt):
    ""
    m = re.search(r'(\d+(?:[.,]\d+)?)\s*(m?)A(?![a-z])', txt or '')
    if not m: return None
    v = float(m.group(1).replace(',', '.')) * (1e-3 if m.group(2) else 1.0)
    return v if v > 0 else None


def sicherung_ampere(txt):
    ""
    a = wert_ampere(txt)
    if a: return a
    m = re.search(r'\d{4}L(\d{3})|MF-?[A-Z]{1,5}(\d{3})|SMD[A-Z]{0,2}(\d{3})', (txt or '').upper())
    if not m: return None
    v = int(next(g for g in m.groups() if g)) / 100.0
    return v if v > 0 else None


def tabelle(muster, txt, standard=None):
    for pat, *w in muster:
        if re.search(pat, txt or '', re.I): return w[0] if len(w) == 1 else tuple(w)
    return standard





KLEIN_BIS = 220e-9
MITTEL_BIS = 4.7e-6
QUARZ_LAST_BIS = 100e-12
SERIEN_R_BIS = 470.0
SHUNT_PFAD_BIS = 100.0
TEILER_VERH = 0.1
EINSTELL_T = {'RT', 'RSET', 'ISET', 'ILIM', 'RLIM', 'IREF', 'RREF', 'RBIAS', 'IBIAS', 'FREQ', 'FSET', 'ROSC', 'PROG', 'RPROG'}
FB_T = {'FB', 'VFB', 'FBK', 'ADJ'}
KEIN_QUARZ = {'R', 'C', 'L', 'D', 'FB', 'F', 'Q', 'TVS', 'LED', 'NT', 'RN', 'ESD', 'NTC', 'PTC', 'JP', 'SJ', 'U', 'J', 'TP', 'SW', 'IC'}
FUNKTIONEN = ('entkopplung_klein', 'quarz', 'quarz_last', 'serien_r', 'spule_sw', 'bootstrap', 'entkopplung_mittel', 'entkopplung_gross',
              'rueckkopplung', 'rueckkopplung_c', 'einstell_r', 'shunt', 'rc_glied', 'teiler', 'pullup', 'pulldown', 'sonstig')


def funktionen(B, fps, pads_by_ref, pads_by_net, NETZ, ROLLE, TYP, ENTK, INSEL, ERZ, shunt, supply_pins):
    ""
    P = B['pads']
    def art(n): return NETZ[n]['art'] if n in NETZ else None
    def kat(n):
        k = NETZ.get(n)
        if k is None: return None
        if k['art'] == 'gnd' or k.get('kelvin'): return 'G'
        if k['art'] == 'versorgung': return 'V'
        return 'P' if k['art'] in ('strom', 'sw') else 'S'
    def netze(r): return [P[i]['net'] if P[i]['net'] in NETZ else None for i in pads_by_ref[r]]
    def zweipole(t): return [r for r in sorted(fps) if ROLLE[r] == 'klein' and TYP[r] == t and len(pads_by_ref[r]) == 2]
    ZP = {t: set(zweipole(t)) for t in ('R', 'C', 'L')}
    def teile_an(n, t): return sorted({P[i]['ref'] for i in pads_by_net.get(n, [])} & ZP[t])
    def anderes(r, n):
        ns = netze(r); return ns[1] if ns[0] == n else ns[0]
    def chip_pins(n): return [(P[i]['ref'], i) for i in pads_by_net.get(n, []) if ROLLE[P[i]['ref']] == 'chip']
    def nah(r, n, kand):
        me = [i for i in pads_by_ref[r] if P[i]['net'] == n] or pads_by_ref[r]
        return min(kand, key=lambda j: (min(math.hypot(P[i]['x'] - P[j]['x'], P[i]['y'] - P[j]['y']) for i in me), j))
    def pin(j, **kw): return dict(ref=P[j]['ref'], pin=P[j]['num'], netz=P[j]['net'], **kw)
    def ein_pin(n): cp = chip_pins(n); return pin(cp[0][1]) if len(cp) == 1 else None
    def wert_r(r): return widerstand(fps[r].get('value', ''))
    def wert_c(r): return kapazitaet(fps[r].get('value', ''))
    def bn(n): return basisname(n) if n else 'offen'
    def namen(n): return set(tokens(n)) | {t for i in pads_by_net.get(n, []) for t in tokens(P[i].get('pinfunc') or '')}
    def wtxt(r): return (fps[r].get('value') or '?').split()[0]
    F = {}
    def setze(r, f, grund=None, stuetzt=None):
        if r in F: return
        F[r] = dict(funktion=f)
        if grund: F[r]['funktion_grund'] = grund
        if stuetzt: F[r]['stuetzt'] = stuetzt


    QNETZ = {}
    for r in sorted(fps):
        if ROLLE[r] in ('ohne', 'mechanik', 'chip') or TYP[r] in KEIN_QUARZ or len(pads_by_ref[r]) > 4: continue
        ns = sorted({n for n in netze(r) if n and kat(n) != 'G'})
        if len(ns) != 2 or any(kat(n) != 'S' for n in ns): continue
        txt = '%s %s' % (fps[r].get('fp', ''), fps[r].get('value', ''))
        hinweis = TYP[r] == 'Y' or praefix(r) in ('Y', 'XTAL', 'XT') or bool(re.search(r'crystal|xtal|quar[zt]|resonator|\d\s*[kM]?Hz', txt, re.I))
        last = [[c for c in teile_an(n, 'C') if kat(anderes(c, n)) == 'G' and (wert_c(c) is None or wert_c(c) <= QUARZ_LAST_BIS * 1.001)] for n in ns]
        if not hinweis and not all(last): continue
        weg = []
        for n in ns:
            w = {c: (j, None) for c, j in chip_pins(n)}
            for rr in teile_an(n, 'R'):
                m = anderes(rr, n)
                if m and m != n and kat(m) == 'S':
                    for c, j in chip_pins(m): w.setdefault(c, (j, rr))
            weg.append(w)
        gem = sorted(set(weg[0]) & set(weg[1]))
        if not gem: continue
        chip = gem[0]; js = [weg[0][chip], weg[1][chip]]
        setze(r, 'quarz', 'an %s:%s und %s:%s' % (chip, P[js[0][0]]['num'], chip, P[js[1][0]]['num']),
              dict(ref=chip, pin=P[js[0][0]]['num'], pins=[P[j]['num'] for j, _ in js], netz=ns[0], netze=ns))
        for n, (j, rr), ls in zip(ns, js, last):
            QNETZ[n] = (r, j)
            if rr: setze(rr, 'serien_r', 'Reihenwiderstand zwischen Quarz %s und %s:%s' % (r, chip, P[j]['num']), pin(j, quarz=r))
            for c in ls: setze(c, 'quarz_last', 'Lastkondensator von %s an %s (%s)' % (r, bn(n), wtxt(c)), pin(j, quarz=r))


    for r in sorted(ZP['R']):
        a, b = netze(r); v = wert_r(r)
        if shunt(r): setze(r, 'shunt', 'Messwiderstand %s zwischen %s und %s' % (wtxt(r), bn(a), bn(b)))
        elif v is not None and v <= SHUNT_PFAD_BIS and a != b and 'strom' in (art(a), art(b)) and all(art(n) == 'strom' or kat(n) == 'G' for n in (a, b)):
            setze(r, 'shunt', 'Widerstand %s im Strompfad zwischen %s und %s' % (wtxt(r), bn(a), bn(b)))
    RN = ZP['R'] - set(F)
    def rs_an(n): return [r for r in teile_an(n, 'R') if r in RN]


    FBN = {}
    for n in sorted(NETZ):
        if kat(n) != 'S' or n in QNETZ: continue
        rr = rs_an(n)
        if not rr: continue
        cp = chip_pins(n)
        zuG = [r for r in rr if kat(anderes(r, n)) == 'G']
        zuV = [r for r in rr if kat(anderes(r, n)) == 'V']
        zuS = [r for r in rr if kat(anderes(r, n)) in ('S', 'P') and anderes(r, n) != n]
        oben = [r for r in rr if r not in zuG and anderes(r, n) not in (n, None) and any(c in ERZ.get(anderes(r, n), ()) for c, _ in cp)]
        if zuG and (oben or (zuV and cp and namen(n) & FB_T)):
            top = oben or zuV
            j = next((j for c, j in cp if any(c in ERZ.get(anderes(r, n), ()) for r in top)), cp[0][1])
            FBN[n] = j
            for r in top + zuG:
                setze(r, 'rueckkopplung', 'Rueckkopplungsteiler %s -> %s -> Masse an %s:%s' % (bn(anderes(top[0], n)), bn(n), P[j]['ref'], P[j]['num']), pin(j))
        elif zuG and zuV:
            for r in zuV + zuG: setze(r, 'teiler', 'Spannungsteiler %s -> %s -> Masse' % (bn(anderes(zuV[0], n)), bn(n)), ein_pin(n))
        elif zuG and zuS:
            for rg in zuG:
                vg = wert_r(rg)
                hoch = [r for r in zuS if vg and wert_r(r) is not None and wert_r(r) >= TEILER_VERH * vg]
                for r in ([rg] + hoch if hoch else []):
                    setze(r, 'teiler', 'Spannungsteiler %s -> %s -> Masse' % (bn(anderes(hoch[0], n)), bn(n)), ein_pin(n))
        def vorwiderstand(r):
            rest = {P[i]['ref'] for i in pads_by_net[n]} - {r}
            return bool(rest) and all(TYP[x] in ('D', 'LED') for x in rest)
        for r in zuV + zuG:
            if r in F: continue
            ziel = 'Versorgung %s' % bn(anderes(r, n)) if r in zuV else 'Masse'
            if vorwiderstand(r): setze(r, 'sonstig', 'Vorwiderstand einer Diode/LED (%s nach %s)' % (bn(n), ziel))
            elif r in zuV: setze(r, 'pullup', '%s von %s nach %s' % (wtxt(r), bn(n), bn(anderes(r, n))), ein_pin(n))
            elif len(pads_by_net[n]) == 2 and cp and namen(n) & EINSTELL_T:
                setze(r, 'einstell_r', 'Einstellwiderstand %s an %s:%s' % (wtxt(r), cp[0][0], P[cp[0][1]]['num']), pin(cp[0][1]))
            else: setze(r, 'pulldown', '%s von %s nach Masse' % (wtxt(r), bn(n)), ein_pin(n))
    for r in sorted(RN - set(F)):
        n = next((n for n in netze(r) if n in FBN), None)
        if n: setze(r, 'rueckkopplung', 'am Rueckkopplungsknoten %s (von %s)' % (bn(n), bn(anderes(r, n))), pin(FBN[n]))


    for c in sorted(ZP['C']):
        if c in F: continue
        a, b = netze(c); ka, kb = kat(a), kat(b)
        if c in ENTK:
            n = ENTK[c]['netz']; C = ENTK[c]['C']; an = INSEL.get(c)
            stufe = 'klein' if C <= KLEIN_BIS * 1.001 else ('mittel' if C <= MITTEL_BIS * 1.001 else 'gross')
            g = '%s an %s' % (si(C), bn(n)) + ('' if wert_c(c) else ' (Wert nicht lesbar, nach Bauform geschaetzt)'); st = None
            if an in fps:
                sp = supply_pins(an, n)
                spule = [i for l in teile_an(n, 'L') for i in pads_by_ref[l] if P[i]['net'] == n and art(anderes(l, n)) == 'sw'
                         and any(ch == an for ch, _ in chip_pins(anderes(l, n)))]
                if sp:
                    j = nah(c, n, sp); st = pin(j); g += ', stuetzt %s:%s' % (an, P[j]['num'])
                    if len(sp) > 1: st['pins'] = [P[i]['num'] for i in sp]
                elif spule:
                    j = nah(c, n, spule); st = pin(j, regler=an, ueber_spule=True); g += ', Ausgangskondensator von %s hinter %s' % (an, P[j]['ref'])
                else: g += ', Insel %s ohne Pin an diesem Netz' % an
            setze(c, 'entkopplung_' + stufe, g, st); continue
        if any(n in FBN for n in (a, b)):
            n = a if a in FBN else b; par = [r for r in rs_an(n) if anderes(r, n) == anderes(c, n)]
            setze(c, 'rueckkopplung_c', ('parallel zu %s am Rueckkopplungsknoten %s' % (par[0], bn(n))) if par else 'am Rueckkopplungsknoten %s' % bn(n), pin(FBN[n]))
            continue
        if a is None or b is None or a == b: setze(c, 'sonstig', 'Kondensator mit offenem oder kurzgeschlossenem Anschluss'); continue
        if 'G' not in (ka, kb):
            for x, y in ((a, b), (b, a)):
                bj = [j for ch, j in chip_pins(y) if any(ch == c2 for c2, _ in chip_pins(x))]
                if art(x) == 'sw' and teile_an(x, 'L') and len(pads_by_net[y]) == 2 and bj:
                    setze(c, 'bootstrap', 'zwischen Schaltknoten %s und %s:%s' % (bn(x), P[bj[0]]['ref'], P[bj[0]]['num']), pin(bj[0])); break
            if c in F: continue
            gem = sorted({ch for ch, _ in chip_pins(a)} & {ch for ch, _ in chip_pins(b)})
            if gem and 'V' not in (ka, kb) and (ka, kb) != ('P', 'P') and (rs_an(a) or rs_an(b)):
                setze(c, 'rueckkopplung_c', 'zwischen zwei Pins von %s (%s, %s): Gegenkopplung/Kompensation' % (gem[0], bn(a), bn(b)),
                      pin(next(j for ch, j in chip_pins(a) if ch == gem[0])))
            else: setze(c, 'sonstig', 'Kondensator zwischen %s und %s (Kopplung o. ae.)' % (bn(a), bn(b)))
            continue
        n = b if ka == 'G' else a
        if kat(n) == 'S':
            speist = [r for r in rs_an(n) if anderes(r, n) not in (n, None) and kat(anderes(r, n)) != 'G']
            if speist: setze(c, 'rc_glied', '%s an %s, gespeist ueber %s; keine Entkopplung' % (wtxt(c), bn(n), ', '.join(speist)), ein_pin(n))
            else: setze(c, 'sonstig', 'Kondensator %s an %s nach Masse ohne Vorwiderstand (Filter oder Zeitglied am Pin)' % (wtxt(c), bn(n)), ein_pin(n))
        else: setze(c, 'sonstig', 'Kondensator zwischen %s und %s' % (bn(a), bn(b)))


    for r in sorted(RN - set(F)):
        a, b = netze(r); v = wert_r(r)
        if a is None or b is None or a == b: setze(r, 'sonstig', 'Widerstand mit offenem oder kurzgeschlossenem Anschluss'); continue
        rc = [n for n in (a, b) if any(F.get(c, {}).get('funktion') == 'rc_glied' and kat(anderes(c, n)) == 'G' for c in teile_an(n, 'C'))]
        if kat(a) == 'S' and kat(b) == 'S' and 'schnell' in (art(a), art(b)) and (v is None or v <= SERIEN_R_BIS):
            kurz = min((a, b), key=lambda n: (len(pads_by_net[n]), 0 if any(P[j].get('pintype') in ('output', 'tri_state') for _, j in chip_pins(n)) else 1, n))
            cp = chip_pins(kurz) or chip_pins(anderes(r, kurz))
            setze(r, 'serien_r', '%s in Reihe zwischen %s und %s (schnell)' % (wtxt(r), bn(a), bn(b)), pin(nah(r, P[cp[0][1]]['net'], [j for _, j in cp])) if cp else None)
        elif rc and all(kat(n) in ('S', 'P') for n in (a, b)):
            setze(r, 'rc_glied', 'Widerstand %s des RC-Glieds an %s (von %s)' % (wtxt(r), bn(rc[0]), bn(anderes(r, rc[0]))), ein_pin(rc[0]))
        elif all(kat(n) in ('S', 'P') for n in (a, b)):
            setze(r, 'sonstig', 'Reihenwiderstand %s zwischen %s und %s (kein schnelles Netz)' % (wtxt(r), bn(a), bn(b)))
        else: setze(r, 'sonstig', 'Widerstand %s zwischen %s und %s' % (wtxt(r), bn(a), bn(b)))


    for l in sorted(ZP['L']):
        sw = [n for n in netze(l) if art(n) == 'sw']
        cp = chip_pins(sw[0]) if sw else []
        if sw: setze(l, 'spule_sw', 'Spule am Schaltknoten %s' % bn(sw[0]), pin(nah(l, sw[0], [j for _, j in cp])) if cp else None)
        else: setze(l, 'sonstig', 'Spule/Drossel zwischen %s und %s (kein Schaltknoten)' % tuple(bn(n) for n in netze(l)))
    for r in fps:
        if r not in F: F[r] = dict(funktion='sonstig' if ROLLE[r] == 'klein' else ROLLE[r])
    return F


def lade(argv):
    B = json.load(open(argv[1]))
    kw = dict(a.split('=', 1) for a in argv[3:] if '=' in a)
    RG = json.load(open(kw['regeln'])) if 'regeln' in kw else None
    VG = json.load(open(kw['vorgaben'])) if 'vorgaben' in kw and os.path.exists(kw['vorgaben']) else {}
    return B, RG, VG, kw


def lose_stecker(B, fps, ROLLE, VG, bedien=()):
    ""
    try:
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
    except ImportError:
        return set()
    kand = {r for r in fps if ROLLE.get(r) in ('stecker', 'mechanik') or r in bedien}
    hart = {r for r in fps if fps[r].get('locked') or glob_any(r, VG.get('fest'))}



    if not str(B.get('src', '')).startswith('aufgabe'):
        return set()
    cy = {}
    for c in B.get('courtyards') or []:
        r = c.get('ref')
        if r in kand or r in hart:
            try: cy[r] = unary_union([Polygon(q) for q in c.get('polys') or [] if len(q) >= 3])
            except (ValueError, TypeError): pass
    lose = set()
    for r in sorted(kand - hart):
        g = cy.get(r)
        if g is None or g.is_empty: continue
        for q in sorted((kand | hart) - {r}):
            h = cy.get(q)
            if h is None or h.is_empty or fps[q]['side'] != fps[r]['side']: continue
            if g.intersection(h).area > 0.01:
                lose.add(r); break
    return lose


def glob_any(ref, pats): return any(fnmatch.fnmatchcase(ref, p) for p in pats or [])


def ist_masse_name(name):
    ""
    return basisname(name).upper() in ('0V', 'GNDPWR') or any(
        re.fullmatch(r'(A|D|P|S|C)?GND[A-Z]?\d*', t) or t.startswith('VSS')
        for t in tokens(name))


def paarkennung(name):
    ""
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


def analysiere(B, RG=None, VG=None, impedanz=False, stoer_flanke=False, paare=False, sw_kompakt=False):
    VG = VG or {}
    L = int(B.get('lagen') or (RG or {}).get('router', {}).get('lagen') or 4)
    SIGW = (RG or {}).get('router', {}).get('sigw') or (0.10 if L >= 6 else 0.15)


    SIGMIN = min(SIGW, (RG or {}).get('router', {}).get('sigmin') or SIGW)
    fps = {f['ref']: f for f in B['fps']}
    pads_by_ref = collections.defaultdict(list); pads_by_net = collections.defaultdict(list)
    for i, p in enumerate(B['pads']):
        if p['ref'] not in fps: continue
        pads_by_ref[p['ref']].append(i)
        if p['net'] and not p['net'].startswith('unconnected'): pads_by_net[p['net']].append(i)
    refs_on = {n: {B['pads'][i]['ref'] for i in v} for n, v in pads_by_net.items()}
    TYP = {r: typ_teil(r, f) for r, f in fps.items()}


    def rolle(r):
        f = fps[r]
        if f.get('dnp') or f.get('board_only') and not pads_by_ref[r]: return 'ohne'
        if glob_any(r, VG.get('anker')): return 'chip'
        pf = TYP[r]; txt = '%s %s %s' % (f.get('fp', ''), f.get('lib', ''), f.get('value', ''))
        if MECH_FP.search(f.get('fp', '')): return 'mechanik'
        if pf in STECKER_PFX or (pf not in PASSIV_PFX and pf not in CHIP_PFX and STECKER_HINT.search(txt)): return 'stecker'
        if pf in MECH_PFX: return 'mechanik'
        if (praefix(r) == 'LED' or pf == 'LED') and f.get('n_pads', 0) >= 4: return 'chip'
        if pf in PASSIV_PFX and praefix(r) not in PASSIV_PFX and f.get('n_pads', 0) >= 5: return 'chip'
        if pf in PASSIV_PFX or pf == 'LED': return 'klein'
        if pf in CHIP_PFX or f.get('n_pads', 0) >= 5: return 'chip'
        return 'klein'
    ROLLE = {r: rolle(r) for r in fps}
    def bedien(r):
        f = fps[r]
        if VG.get('bedien_fest') is False or ROLLE[r] not in ('klein', 'chip'): return False
        return praefix(r) in ('LED', 'DS', 'RV', 'VR', 'BZ') or bool(BEDIEN_HINT.search('%s %s' % (f.get('fp', ''), f.get('value', ''))))
    BEDIEN = {r for r in fps if bedien(r)}
    has_nets = {r: any(B['pads'][i]['net'] and not B['pads'][i]['net'].startswith('unconnected') for i in pads_by_ref[r]) for r in fps}
    for r in fps:
        if not has_nets[r] and ROLLE[r] in ('klein', 'chip'): ROLLE[r] = 'ohne'
    def analog_chip(r):
        f = fps[r]; return ROLLE[r] == 'chip' and bool(ANALOG_CHIP.search('%s %s' % (f.get('value', ''), f.get('fp', ''))))
    def shunt(r):
        if TYP[r] != 'R': return False
        v = widerstand(fps[r].get('value', '')); return v is not None and 0 < v <= 1.0
    def hf_spule(r):
        ""
        if TYP[r] != 'L': return False
        v = kapazitaet(fps[r].get('value', ''))
        if v is not None: return v < HF_SPULE_BIS
        if bauform(fps[r].get('fp', '')) > 3: return False
        for i in pads_by_ref[r]:
            for j in pads_by_net.get(B['pads'][i]['net'], []):
                c = B['pads'][j]['ref']
                if TYP.get(c) == 'C' and (kapazitaet(fps[c].get('value', '')) or 1.0) <= QUARZ_LAST_BIS * 1.001: return True
        return False
    def is_ind(r): return TYP[r] == 'L' and not hf_spule(r)


    over = [(re.compile(k), v) for k, v in (VG.get('netze') or {}).items()]
    gnd_forced = VG.get('gnd') or []
    def pintypes(n): return {B['pads'][i].get('pintype', '') for i in pads_by_net[n]}
    def lastpad(r, n):
        ""
        ps = [B['pads'][i] for i in pads_by_net[n] if B['pads'][i]['ref'] == r]
        return len(ps) >= 2 or any(p.get('pintype') in ('passive', 'power_in', 'power_out') for p in ps)

    gnd_names = {n for n in pads_by_net if n in gnd_forced or ist_masse_name(n)}

    def schnell_pin(n):
        ""
        for i in sorted(pads_by_net[n]):
            p = B['pads'][i]; f = (p.get('pinfunc') or '').strip().upper()
            if not f or '/' in f.strip('/~{}') or ROLLE[p['ref']] != 'chip': continue
            T = [t for t in re.split(r'[_\-()/{}~\s.,]+', f) if t]
            if not T: continue
            if re.fullmatch(r'USB_?D[PMN+\-]|D[+\-]|DPLUS|DMINUS', f) or (T[0] == 'USB' and T[-1] in ('DP', 'DM', 'DN', 'P', 'N')):
                return dict(art='schnell', rang=5, strom=None, grund='USB-Paar (Pinname %s:%s)' % (p['ref'], f))
            if set(T) & XTAL_T: return dict(art='schnell', rang=4, strom=None, grund='Quarz/Oszillator (Pinname %s:%s)' % (p['ref'], f), quarz=True)
            if any(t.startswith(VHS_IF) for t in T): return dict(art='schnell', rang=5, strom=None, grund='Hochgeschwindigkeit (Pinname %s:%s)' % (p['ref'], f))
            if set(T) & CLK_T or any(t.endswith('CLK') for t in T):
                return dict(art='schnell', rang=4, strom=None, grund='Takt (Pinname %s:%s)' % (p['ref'], f))
            if any(t.startswith(HS_IF) for t in T): return dict(art='schnell', rang=4, strom=None, grund='xSPI/SDIO (Pinname %s:%s)' % (p['ref'], f))
            if set(T) & {'MOSI', 'MISO', 'COPI', 'CIPO'}: return dict(art='schnell', rang=3, strom=None, grund='SPI (Pinname %s:%s)' % (p['ref'], f))
        return None

    def klasse(n):
        base = basisname(n); T = tokens(n); u = base.upper(); refs = refs_on[n]
        for rx, v in over:
            if rx.fullmatch(base) or rx.fullmatch(n): return dict(dict(art='signal', rang=0, strom=None), **v, grund='Vorgabe')
        if n in gnd_forced or ist_masse_name(n):
            return dict(art='gnd', rang=0, strom=None, grund='Name')
        if any(hf_spule(r) for r in refs) and not any(is_ind(r) for r in refs):
            return dict(art='schnell', rang=4, strom=None, grund='HF-Anpassung (Spule < 0,2 uH bzw. Platzhalter mit pF-Kondensator)')
        if any(re.fullmatch(r'(SW|LX|SWN|PH)\d?', t) for t in T) and any(is_ind(r) for r in refs):
            return dict(art='sw', rang=5, strom='hoch', grund='Schaltknoten (Name + Spule)')
        chips_n = [r for r in refs if ROLLE[r] == 'chip']
        c_gnd = [r for r in refs if TYP[r] == 'C' and any(B['pads'][i]['net'] in gnd_names for i in pads_by_ref[r])]
        if any(is_ind(r) for r in refs) and chips_n and not c_gnd and not any(t.startswith(PWR_START) for t in T) \
                and not any(re.fullmatch(r'\+?\d+V\d*', t) for t in T):
            return dict(art='sw', rang=5, strom='hoch', grund='Schaltknoten (Chip-Pin + Spule, ohne C nach GND)')
        if any(t in ('BST', 'CBST', 'BOOT', 'BS') for t in T) and len(pads_by_net[n]) == 2 and any(TYP[r] == 'C' for r in refs):
            return dict(art='sw', rang=4, strom='mittel', grund='Bootstrap')

        Ts = set(T); anyst = lambda pre: any(t.startswith(pre) for t in T)
        if anyst('USB') and (T[-1] in ('P', 'N', 'DP', 'DM', 'DN') or Ts & {'DP', 'DN', 'DM', 'DPLUS', 'DMINUS'}):
            return dict(art='schnell', rang=5, strom=None, grund='USB-Paar')
        if any(anyst(x) for x in VHS_IF): return dict(art='schnell', rang=5, strom=None, grund='Hochgeschwindigkeits-Schnittstelle')
        if any(anyst(x) for x in HS_IF):
            if Ts & CLK_T or any(t.endswith('CLK') for t in T): return dict(art='schnell', rang=5, strom=None, grund='xSPI/SDIO-Takt')
            return dict(art='schnell', rang=4, strom=None, grund='xSPI/SDIO-Daten')
        if (Ts & GPIO_T or (any(GPIO_NR.fullmatch(t) for t in T) and not any(t.startswith(PWR_START) for t in T))) and not Ts & CLK_T:
            return dict(art='signal', rang=0, strom=None, grund='Steuersignal')
        if Ts & XTAL_T: return dict(art='schnell', rang=4, strom=None, grund='Quarz/Oszillator', quarz=True)
        if Ts & SWD_T: return dict(art='schnell', rang=2, strom=None, grund='SWD/JTAG')
        spi = anyst('SPI'); aud = any(anyst(x) for x in AUD_IF)
        if Ts & CLK_T or any(t.endswith('CLK') for t in T):
            return dict(art='schnell', rang=4, strom=None, grund='Takt (%s)' % ('SPI' if spi else 'Audio' if aud else 'CLK'))
        if spi and Ts & {'MOSI', 'MISO', 'SDI', 'SDO', 'SI', 'SO', 'CS', 'NSS', 'SS', 'NCS', 'CE'}:
            return dict(art='schnell', rang=3, strom=None, grund='SPI')
        if spi or any(re.fullmatch(r'N?CS\d*', t) for t in T): return dict(art='schnell', rang=3, strom=None, grund='SPI/CS')
        if aud or Ts & {'LRCK', 'LRCLK', 'FSYNC', 'WS', 'FS', 'SDOUT', 'SDIN', 'SDATA'}:
            return dict(art='schnell', rang=3, strom=None, grund='Audio-Digital (I2S/SAI)')
        if Ts & SLOW_T or anyst('I2C') or anyst('UART'):
            return dict(art='signal', rang=1, strom=None, grund='langsame Schnittstelle (I2C/UART/PWM)')
        if Ts & GPIO_T: return dict(art='signal', rang=0, strom=None, grund='Steuersignal')

        opin = bool(re.search(r'(-|\+)IN\)?$|--\)$|-\+\)$', base))
        if opin or Ts & AN_T3 or (Ts & {'IN'} and (anyst('ADC') or anyst('AIN') or anyst('OPV'))) or any(re.fullmatch(r'IN\d+[PNM]?', t) for t in T):
            return dict(art='analog', rang=3, strom=None, grund='Analog-Eingang/Referenz')
        if Ts & AN_T2: return dict(art='analog', rang=2, strom=None, grund='Rueckkopplung/Messung')

        pwr_name = any(re.fullmatch(r'\+?\d+V\d*', t) or t.startswith(PWR_START) for t in T) or u.startswith('+')
        pt = pintypes(n)
        if pwr_name or pt & {'power_in', 'power_out'} or re.search(r'power|pwr|supply', B.get('netclass', {}).get(n, ''), re.I):
            if any(t.startswith(PWR_HOCH) for t in T) or any(is_ind(r) or shunt(r) or TYP[r] == 'F' for r in refs): lvl = 'hoch'
            elif any(t.startswith(PWR_KLEIN) or t in PWR_KLEIN for t in T): lvl = 'klein'
            else: lvl = 'mittel'
            return dict(art='versorgung', rang=0, strom=lvl, grund='Versorgung (%s)' % ('Name' if pwr_name else 'Pin-Typ'))


        if any(is_ind(r) for r in refs): return dict(art='strom', rang=0, strom='hoch', grund='Strompfad (Spule)')
        if any(TYP[r] in ('F', 'PTC') for r in refs): return dict(art='strom', rang=0, strom='hoch', grund='Strompfad (Sicherung)')
        if any(shunt(r) for r in refs): return dict(art='strom', rang=0, strom='mittel', grund='Strompfad (Shunt)')




        steuer = any(TYP[r] == 'R' and not shunt(r) and (widerstand(fps[r].get('value', '')) or 0) >= STEUER_R_AB for r in refs) \
            and not any(r in c_gnd and (kapazitaet(fps[r].get('value', '')) or 0) >= 1e-6 for r in refs) \
            and all(KLEINSIGNAL_Q.search(fps[r].get('fp', '') or '') for r in refs if TYP[r] == 'Q')
        if any(TYP[r] == 'Q' and lastpad(r, n) for r in refs) and not steuer:
            return dict(art='strom', rang=0, strom='mittel', grund='Strompfad (Lastanschluss eines Transistors)')
        if any(TYP[r] == 'Q' for r in refs) and any(ROLLE[r] == 'stecker' for r in refs) and not steuer:
            return dict(art='strom', rang=0, strom='mittel', grund='Strompfad (Transistor am Stecker)')

        if not steuer and any(ROLLE[r] == 'stecker' for r in refs) and any(
                ROLLE[B['pads'][i]['ref']] == 'chip' and TREIBER_CHIP.search(fps[B['pads'][i]['ref']].get('value') or '')
                and (not (B['pads'][i].get('pinfunc') or '').strip() or TREIBER_AUS.search(B['pads'][i]['pinfunc'])) for i in pads_by_net[n]):
            return dict(art='strom', rang=0, strom='mittel', grund='Strompfad (Treiberausgang am Stecker)')
        sp = schnell_pin(n)
        if sp: return sp
        if any(analog_chip(r) for r in refs):
            return dict(art='analog', rang=2, strom=None, grund='Pin eines Analog-Chips')
        return dict(art='signal', rang=0, strom=None, grund='Rest (GPIO)')

    NETZ = {n: klasse(n) for n in pads_by_net}



    STUFEN = {'klein': 1, 'mittel': 2, 'hoch': 3}
    for _ in range(6):
        neu = False
        for r in sorted(fps):
            if TYP[r] in ('F', 'PTC') or shunt(r): seiten = sorted({B['pads'][i]['net'] for i in pads_by_ref[r]} & set(NETZ))
            elif TYP[r] == 'Q': seiten = sorted(n for n in {B['pads'][i]['net'] for i in pads_by_ref[r]} & set(NETZ) if lastpad(r, n))
            else: continue
            if len(seiten) != 2: continue
            for a, b in ((seiten[0], seiten[1]), (seiten[1], seiten[0])):
                ka, kb = NETZ[a], NETZ[b]
                if ka['art'] not in ('versorgung', 'strom') or not ka.get('strom'): continue
                if kb['grund'] == 'Rest (GPIO)' or (kb['art'] == 'strom' and STUFEN.get(kb.get('strom'), 0) < STUFEN[ka['strom']]
                                                   and kb['grund'].startswith('Strompfad (')):
                    NETZ[b] = dict(art='strom', rang=0, strom=ka['strom'], grund='Strompfad (hinter %s von %s)' % (r, a)); neu = True
        if not neu: break
    gnd = [n for n in pads_by_net if NETZ[n]['art'] == 'gnd']
    gnd = [n for n in gnd_forced if n in pads_by_net] + sorted([n for n in gnd if n not in gnd_forced], key=lambda n: -len(pads_by_net[n]))


    net_ties = sorted(r for r in fps if TYP[r] == 'NT' or 'nettie' in re.sub(r'[^a-z]', '', fps[r].get('fp', '').lower()))
    for r in net_ties:
        ns = sorted({B['pads'][i]['net'] for i in pads_by_ref[r] if B['pads'][i]['net'] in NETZ})
        for n in ns:
            if n == (gnd[0] if gnd else None) or n in gnd_forced or NETZ[n]['art'] != 'gnd' or NETZ[n]['grund'] == 'Vorgabe': continue
            an = [m for m in ns if m != n and NETZ[m]['art'] == 'gnd']
            if an: NETZ[n] = dict(art='analog', rang=3, strom=None, kelvin=r, grund='Kelvin-Messmasse (Net-Tie %s an %s)' % (r, basisname(an[0])))
    gnd = [n for n in gnd if NETZ[n]['art'] == 'gnd']
    GNDS = set(gnd)

    sw_chips = {r for n, k in NETZ.items() if k['art'] == 'sw' for r in refs_on[n] if ROLLE[r] == 'chip'}
    for n, k in NETZ.items():
        if k['art'] in ('signal', 'analog') and refs_on[n] & sw_chips: k['empfindlich_sw'] = True
        if k['art'] == 'analog': k['empfindlich_sw'] = True

    if paare or VG.get('paare') is True or impedanz or VG.get('impedanz') is True:


        by_base = collections.defaultdict(lambda: collections.defaultdict(set))
        for n, k in sorted(NETZ.items()):
            if k['art'] not in ('schnell', 'analog', 'signal'): continue
            key = paarkennung(n)
            if key: by_base[key[:2]][key[2]].add(n)
            elif n.startswith(('Net-(', 'N-')):
                for i in pads_by_net[n]:
                    p = B['pads'][i]; key = paarkennung(p.get('pinfunc', ''))
                    if key: by_base[('pin:' + p['ref'], key[1])][key[2]].add(n)

            for i in pads_by_net[n]:
                p = B['pads'][i]; f = fps[p['ref']]
                if ROLLE[p['ref']] != 'stecker' or 'USB' not in (f.get('fp','')+' '+f.get('value','')).upper(): continue
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


            if any(TYP[r] == 'C' and {n, pn}.issubset({B['pads'][i]['net'] for i in pads_by_ref[r]}) for r in refs_on[n] & refs_on[pn]) and any(re.search(r'MAX(?:232|3232)|SP(?:232|3232)', fps[r].get('value',''), re.I) for r in refs_on[n] & refs_on[pn]): continue
            analog = all(NETZ[q]['art'] == 'analog' or re.search(r'(?:^|[_/])(?:IN\d*|AIN\d*|SENSE\d*|AUDIO|MIC|LINE)', q.upper()) for q in (n, pn))
            usb = n in usb_names or (paarkennung(n) and 'USB' in paarkennung(n)[1])
            usb = usb or any(USB_FS.search(fps[r].get('value') or '') or USB_HS.search(fps[r].get('value') or '') for r in refs_on[n] & refs_on[pn] if ROLLE[r] == 'chip')
            art, rang = ('analog', 3) if analog else ('schnell', max(5 if usb else 3, NETZ[n]['rang'], NETZ[pn]['rang']))
            k = NETZ[n]; k.update(paar=pn, paar_art=art, paar_rang=rang)
            if k['grund'] != 'Vorgabe': k.update(art=art, rang=rang, strom=None)
    else:
        by_base = {}
        for n in NETZ:
            m = re.match(r'(.*?)(_?)(P|N|DP|DN|DM|\+|-)$', basisname(n))
            if m and NETZ[n]['art'] == 'schnell':



                scope = n.rpartition('/')[0] if NETZ[n].get('z_diff_ohm') and (impedanz or VG.get('impedanz') is True) else n.rsplit('/', 1)[0]
                by_base.setdefault((scope, m.group(1)), []).append(n)
        for k, v in by_base.items():
            if len(v) == 2:
                NETZ[v[0]]['paar'] = v[1]; NETZ[v[1]]['paar'] = v[0]



    for n, k in sorted(NETZ.items()):
        if k['art'] != 'schnell' or not k['grund'].startswith('USB-Paar'): continue
        chips = {r for r in refs_on[n] if ROLLE[r] == 'chip'}
        for r in refs_on[n]:
            if TYP[r] == 'R' and len(pads_by_ref[r]) == 2:
                chips |= {q for i in pads_by_ref[r] for q in refs_on.get(B['pads'][i]['net'], ()) if ROLLE[q] == 'chip'}
        hs = sorted(c for c in chips if USB_HS.search(fps[c].get('value') or ''))
        fs = sorted(c for c in chips if USB_FS.search(fps[c].get('value') or ''))
        if hs: k['tr_ns'] = 0.5; k['tr_quelle'] = 'USB High-Speed (%s)' % hs[0]
        elif fs: k['tr_ns'] = 4.0; k['tr_quelle'] = 'USB Full-Speed (%s %s)' % (fs[0], fps[fs[0]].get('value'))

    def router_params(k):
        art = k['art']
        if art == 'sw': return dict(w=0.5, wmin=0.25, pen=2.0, via=1.5, inner=1.8, prio=0 if k['rang'] >= 5 else 1, gruppe=0)
        if art in ('strom', 'versorgung'):
            w, wmin, prio = {'hoch': (1.0, 0.3, 2), 'mittel': (0.5, 0.2, 4), 'klein': (0.3, 0.15, 5)}[k['strom']]
            return dict(w=w, wmin=wmin, pen=1.0, via=1.0, inner=1.25, prio=prio, gruppe=2)
        if art == 'schnell': return dict(w=SIGW, wmin=SIGMIN, pen=0.0, via=round(1.2 + 0.15 * k['rang'], 2), inner=1.0, prio=8, gruppe=1)
        if art == 'analog': return dict(w=SIGW, wmin=SIGMIN, pen=0.0, via=1.0, inner=0.95, prio=8, gruppe=3)
        if art == 'gnd':
            if L <= 2:
                return dict(w=max(0.35, SIGW), wmin=SIGMIN, pen=0.5, via=1.0, inner=1.0, prio=5, gruppe=5)
            return dict(w=0.35, wmin=0.15, pen=0.5, via=0.0, inner=99.0, prio=7, gruppe=5)
        return dict(w=SIGW, wmin=SIGMIN, pen=0.0, via=0.9, inner=0.9, prio=8, gruppe=4)




    VIA_F = float(VG.get('via_faktor') or {2: 4.0}.get(L, 3.0))
    for n, k in NETZ.items():
        rp = router_params(k)
        for key in ('w', 'wmin'):
            if key in k: rp[key] = k[key]
        rp['via'] = round(rp['via'] * VIA_F, 3)
        k.update(rp)
    BMIN = float((RG or {}).get('router', {}).get('bahn_min') or 0.0)
    if BMIN:
        for k in NETZ.values(): k['w'] = max(k['w'], BMIN); k['wmin'] = max(k['wmin'], BMIN)
    for n, b in (VG.get('breite') or {}).items():
        if n in NETZ:
            b = float(b); NETZ[n]['w'] = max(NETZ[n]['w'], b); NETZ[n]['breite_vorgabe'] = b


    anker = [r for r in fps if ROLLE[r] in ('chip', 'stecker')]
    nets_of = {r: {B['pads'][i]['net'] for i in pads_by_ref[r] if B['pads'][i]['net'] in NETZ} for r in fps}
    anker_on = {n: [r for r in refs_on[n] if r in anker] for n in NETZ}
    def kern(a):
        k = re.sub(r'\d+$', '', re.sub(r'^[A-Za-z]+_', '', a))
        return k.upper() if len(re.sub(r'[^A-Za-z]', '', k)) >= 2 else ''
    def hint(p, a):
        if len(a) >= 4 and re.search(r'(^|[^0-9A-Za-z])' + re.escape(a) + r'(?![0-9A-Za-z])', p): return 4.0
        k = kern(a)
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
    INSEL = {}
    for r, a in (VG.get('zuordnung') or {}).items():
        if r in fps: INSEL[r] = a if a != 'frei' else None
    klein_alle = [r for r in fps if ROLLE[r] == 'klein']
    klein = [r for r in klein_alle if r not in INSEL]



    ENTK = {}
    for r in klein_alle:
        if TYP[r] != 'C' or len(pads_by_ref[r]) != 2: continue
        ns = [B['pads'][i]['net'] for i in pads_by_ref[r]]
        if len([n for n in ns if n in GNDS]) == 1:
            sup = [n for n in ns if n not in GNDS and n in NETZ and NETZ[n]['art'] in ('versorgung', 'strom')]
            if sup:
                c = kapazitaet(fps[r].get('value', '')) or 1e-7 * bauform(fps[r].get('fp', ''))
                ENTK[r] = dict(netz=sup[0], C=c)
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
        sws = sorted(n for n in ns if n in NETZ and NETZ[n]['art']=='sw')
        for sw in sws:
            ins = [n for r in sorted(refs_on[sw]) if is_ind(r) for n in sorted(nets_of_all(r))
                   if n != sw and n in ns and NETZ.get(n,{}).get('art') in ('strom','versorgung')
                   and any(re.fullmatch(r'P?VIN|VCC|VBAT|VBUS', str(B['pads'][i].get('pinfunc','')), re.I)
                           for i in pads_by_ref[u] if B['pads'][i]['net']==n)]
            outs = [(r,n) for r in sorted(refs_on[sw]) if TYP[r] in ('D','Q')
                    for n in sorted(nets_of_all(r)) if n != sw and NETZ.get(n,{}).get('art') in ('strom','versorgung')]
            if ins and outs: BOOST[u] = dict(vin=ins[0], sw=sw, ausgang=outs[0][1], gleichrichter=outs[0][0])
    ERZ = collections.defaultdict(set)
    for n in NETZ:
        for i in pads_by_net[n]:
            p = B['pads'][i]
            if p.get('pintype') == 'power_out' and ROLLE[p['ref']] == 'chip': ERZ[n].add(p['ref'])
        for r in refs_on[n]:
            if not is_ind(r): continue
            for m in nets_of_all(r):
                if m != n and m in NETZ and NETZ[m]['art'] == 'sw':
                    ERZ[n] |= {q for q in refs_on[m] if ROLLE[q] == 'chip' and not (q in BOOST and BOOST[q]['vin']==n)}
    for u,b in BOOST.items(): ERZ[b['ausgang']].add(u)
    assigned = collections.Counter()
    for r in sorted(ENTK, key=lambda r: ENTK[r]['C']):
        if r not in INSEL: continue
        n = ENTK[r]['netz']; a = INSEL[r]; ENTK[r]['vorgabe'] = True
        if a: assigned[(a, n)] += 1
        ENTK[r]['erzeuger'] = not (a in pads_by_ref and supply_pins(a, n))
    for r in sorted(ENTK, key=lambda r: ENTK[r]['C']):
        if r in INSEL: continue
        n = ENTK[r]['netz']
        cand = [a for a in anker_on[n] if ROLLE[a] == 'chip'] + sorted(ERZ[n] - set(anker_on[n]))
        cand = cand or anker_on[n]
        if not cand: continue
        def score(a):
            sp = supply_pins(a, n)
            need = max(1, len(clusters(sp))) if sp else 1
            s = sheet_bonus(r, a) + hint(r, a) + 1.0 * (need - assigned[(a, n)]) / need - 0.02 * dist(r, a)
            if ROLLE[a] == 'stecker': s -= 1.0
            if a in ERZ[n] and ENTK[r]['C'] >= 4.7e-6: s += 1.5 - 1.0 * (need - assigned[(a, n)]) / need
            elif a in ERZ[n]: s += 0.3
            if not sp: s -= 0.5
            return s
        a = max(cand, key=score); INSEL[r] = a; assigned[(a, n)] += 1
        ENTK[r]['erzeuger'] = a in ERZ[n] and not supply_pins(a, n)




    def w_net(n, a=None):
        k = NETZ[n]; w = GEW['strom'] if k['art'] == 'versorgung' else GEW.get(k['art'], 1.0) * (1 + 0.15 * k.get('rang', 0))
        if k['art'] == 'signal' and a is not None and fps[a].get('n_pads', 0) > 20: w *= 0.3 if k['rang'] == 0 else 0.5
        if a is not None and ROLLE.get(a) == 'stecker': w *= 0.6
        return w
    import heapq
    nbr = collections.defaultdict(list)
    for n in NETZ:
        if n in GNDS or NETZ[n]['art'] == 'gnd': continue
        rs = [r for r in refs_on[n] if ROLLE[r] in ('klein', 'chip', 'stecker')]
        if len(rs) > (6 if NETZ[n]['art'] == 'versorgung' else 12): continue
        for p in rs:
            if ROLLE[p] != 'klein': continue
            for q in rs:
                if q != p: nbr[p].append((q, 1.0 / max(1e-3, w_net(n, q if q in anker else None))))
    def naechster(r0):
        dist0 = {r0: 0.0}; best = {}; h = [(0.0, r0)]
        while h:
            d, r = heapq.heappop(h)
            if d > dist0.get(r, 1e9): continue
            if r in anker or (r != r0 and INSEL.get(r)):
                a = r if r in anker else INSEL[r]; dd = d + (0.0 if r in anker else 0.2)
                if dd < best.get(a, 1e9): best[a] = dd
                continue
            for q, c in nbr[r]:
                if d + c < dist0.get(q, 1e9): dist0[q] = d + c; heapq.heappush(h, (d + c, q))
        return best
    NAH = {}
    for r in klein:
        if r in INSEL: continue
        best = naechster(r)
        sc = {a: -d for a, d in best.items()}
        for a in anker:
            b = 0.3 * sheet_bonus(r, a) + 0.3 * hint(r, a)
            if b > 0: sc[a] = sc.get(a, -6.0) + b
        NAH[r] = sc
    for r, sc in NAH.items():
        INSEL[r] = max(sc, key=lambda a: (sc[a], -dist(r, a))) if sc else None

    LOSE = lose_stecker(B, fps, ROLLE, VG, BEDIEN)
    inseln = {}
    for a in anker:
        mem = [r for r in fps if INSEL.get(r) == a]
        ent = sorted([r for r in mem if r in ENTK], key=lambda r: ENTK[r]['C'])
        rest = [r for r in mem if r not in ENTK]
        def wichtig(r):
            ns = [n for n in nets_of[r] if n not in GNDS]
            best = max([(NETZ[n]['gruppe'] if NETZ[n]['art'] != 'schnell' else 0, -NETZ[n].get('rang', 0)) for n in ns] or [(9, 0)])
            return (min(((NETZ[n]['gruppe'] if NETZ[n]['art'] != 'schnell' else 0), -NETZ[n].get('rang', 0)) for n in ns) if ns else (9, 0),
                    -len(nets_of[r] & nets_of[a]))
        rest.sort(key=wichtig)
        cnt = collections.Counter()
        for r in [a] + mem:
            for n in sorted(nets_of[r]):
                if n in GNDS: continue
                cnt[NETZ[n]['art']] += 1 + (NETZ[n].get('rang', 0) if NETZ[n]['art'] == 'schnell' else 0)
        if cnt['sw']: art = 'leistung'
        elif cnt['schnell'] >= 2 and cnt['analog'] >= 2: art = 'gemischt'
        elif cnt['schnell'] >= 2: art = 'digital'
        elif cnt['analog'] >= 2: art = 'analog'
        else: art = 'neutral'
        inseln[a] = dict(art=art, rolle=ROLLE[a], fest=(ROLLE[a] == 'stecker' or a in BEDIEN) and a not in LOSE or fps[a].get('locked') or glob_any(a, VG.get('fest')),
                         entkopplung=[dict(ref=r, netz=ENTK[r]['netz'], C=ENTK[r]['C'], ueber_spule=ENTK[r].get('erzeuger', False),
                                           pins=[B['pads'][i]['num'] for i in supply_pins(a, ENTK[r]['netz'])]) for r in ent],
                         mitglieder=ent + rest, zaehler=dict(cnt))




    PNETZ = {n for n, k in NETZ.items() if k['art'] in ('versorgung', 'strom', 'sw')}
    def key(p): return '%s:%s' % (p['ref'], p['num'])
    def pads_auf(r, n): return [B['pads'][i] for i in pads_by_ref[r] if B['pads'][i]['net'] == n]
    bruecke = collections.defaultdict(list)
    for r in fps:
        if ROLLE[r] in ('stecker', 'mechanik', 'ohne') or TYP[r] == 'C': continue
        ns = {B['pads'][i]['net'] for i in pads_by_ref[r]} & PNETZ
        if len(ns) < 2 or (TYP[r] == 'R' and not shunt(r)): continue
        gross = ROLLE[r] == 'chip' and fps[r].get('n_pads', 0) > 8 and not any(r in v for v in ERZ.values())
        for a in ns:
            for b in ns:
                if a != b and not (r in BOOST and {a,b}=={BOOST[r]['vin'],BOOST[r]['sw']}): bruecke[a].append((r, b, gross))
    roots = [n for n in sorted(PNETZ) if not ERZ.get(n) and NETZ[n]['art'] != 'sw' and any(ROLLE[r] == 'stecker' for r in refs_on[n])]
    STUFE = {}; QUELLE = {}; UEBER = {}
    for n in roots:
        STUFE[n] = 0; QUELLE[n] = [key(p) for r in sorted(refs_on[n]) if ROLLE[r] == 'stecker' for p in pads_auf(r, n)]
    for erlaube_gross in (False, True):
        front = sorted(STUFE, key=lambda n: STUFE[n])
        while front:
            nxt = []
            for n in front:
                for r, m, gross in sorted(bruecke[n]):
                    if m in STUFE or (gross and not erlaube_gross): continue
                    STUFE[m] = STUFE[n] + 1; QUELLE[m] = [key(p) for p in pads_auf(r, m)]; UEBER[m] = (r, n); nxt.append(m)
            front = nxt




    def vorfahren(n):
        out = set()
        while UEBER.get(n): n = UEBER[n][1]; out.add(n)
        return out
    ALT = []
    for a in sorted(STUFE):
        for r, b, gross in sorted(bruecke[a]):
            if b not in STUFE or a >= b or ROLLE[r] == 'chip' or (NETZ[a]['art'], NETZ[b]['art']) != ('versorgung', 'versorgung'): continue
            if TYP[r] not in ('Q', 'D', 'F', 'PTC') or (TYP[r] != 'Q' and len(pads_by_ref[r]) != 2): continue
            if r in (UEBER.get(a, (None,))[0], UEBER.get(b, (None,))[0]) or a in vorfahren(b) or b in vorfahren(a): continue
            ALT.append((r, a, b))





    def _zahl(feld, r, v, positiv=False):
        try: x = float(v)
        except (TypeError, ValueError): raise ValueError('vorgaben.%s[%s]: keine Zahl: %r' % (feld, r, v))
        if not math.isfinite(x) or x < 0 or (positiv and x <= 0):
            raise ValueError('vorgaben.%s[%s] = %r: muss endlich und %s sein' % (feld, r, v, '> 0' if positiv else '>= 0'))
        return x
    STROM_VG = {r: _zahl('strom', r, v) for r, v in (VG.get('strom') or {}).items()}
    UMSETZ_VG = {r: _zahl('umsetzung', r, v) for r, v in (VG.get('umsetzung') or {}).items()}
    SPANN_VG = {p: _zahl('spannung', p, v, True) for p, v in (VG.get('spannung') or {}).items()}
    def spannung(n):
        for pat, v in SPANN_VG.items():
            if re.fullmatch(pat, basisname(n), re.I): return v
        m = re.fullmatch(r'\+?(\d+)V(\d*)', basisname(n), re.I)
        return float(m.group(1) + '.' + (m.group(2) or '0')) if m else None
    def grundlast(r, n_pads_auf):
        t = TYP[r]
        if r in STROM_VG: return STROM_VG[r], 'vorgabe_A'
        if ROLLE[r] == 'stecker': return 1.0 * n_pads_auf, 'annahme'
        if ROLLE[r] == 'chip':
            np_ = fps[r].get('n_pads', 0)
            return (1.0 if np_ > 20 else (0.5 if np_ > 8 else 0.3)), 'annahme'
        if t == 'C': return 0.0, 'C_gleichstrom_0'
        if t == 'R': return (1.0, 'annahme') if shunt(r) else (0.05, 'annahme')
        if t == 'D': return 0.02, 'annahme_diode'
        return 0.5, 'annahme'
    def ist_bootstrap(m):
        return NETZ[m]['art'] == 'sw' and not any(is_ind(q) for q in refs_on[m])
    def geregelt(m, tiefe=0):
        ""
        if NETZ[m]['art'] != 'sw' or tiefe > 3: return [m]
        kinder = [k for k, (rr, nn) in UEBER.items() if nn == m]

        return [x for k in kinder for x in geregelt(k, tiefe + 1)]
    EXAKT = ('vorgabe_A', 'vorgabe', 'reihenglied', 'linearregler')
    LAST = {}; HERK = {}
    for n in sorted(STUFE, key=lambda n: -STUFE[n]):
        roh = {}; herkunft = {}
        src = set(QUELLE[n])
        for r in sorted(refs_on[n]):
            ps = [p for p in pads_auf(r, n) if key(p) not in src]
            if not ps: continue
            unten = [m for m, (rr, nn) in UEBER.items() if rr == r and nn == n]
            if unten:
                aus = [m for m in unten if not ist_bootstrap(m)] or unten
                if r in UMSETZ_VG:
                    w = UMSETZ_VG[r] * sum(LAST.get(m, 0.0) for m in aus); h = 'vorgabe'
                elif any(b['vin']==n and b['sw'] in aus for b in BOOST.values()):
                    ziele = [b['ausgang'] for b in BOOST.values() if b['vin']==n and b['sw'] in aus]
                    w = sum((spannung(x) or 0)*LAST.get(x,0) for x in ziele)/(spannung(n)*0.9) if spannung(n) else sum(LAST.get(x,0) for x in ziele)
                    h = 'annahme_boost_eta0.9'
                elif ROLLE[r] != 'chip':
                    w = sum(LAST.get(m, 0.0) for m in aus); h = 'reihenglied'
                elif r not in sw_chips:
                    w = sum(LAST.get(m, 0.0) for m in aus); h = 'linearregler'
                else:
                    ve = spannung(n); ziele = [x for m in aus for x in geregelt(m)]
                    va = {x: spannung(x) for x in ziele}
                    if ve and ziele and all(va.values()):
                        w = sum(va[x] * LAST.get(x, 0.0) for x in ziele) / (ve * 0.9); h = 'annahme_spannung_eta0.9'
                    else:
                        w = 0.5 * sum(LAST.get(m, 0.0) for m in aus); h = 'annahme_schaltregler'
                if h in EXAKT and not all(HERK.get(m) == 'A' for m in aus): h += '+relativ'
            elif any(key(p) in STROM_VG for p in ps):
                for p in ps:
                    if key(p) in STROM_VG: roh[key(p)] = STROM_VG[key(p)]; herkunft[key(p)] = 'vorgabe_A'
                    else: roh[key(p)] = 0.0; herkunft[key(p)] = 'annahme_pad_ohne_vorgabe'
                continue
            elif r in BOOST and n==BOOST[r]['sw']: w,h = 0., 'schaltpin_kein_DC_Verbraucher'
            else: w, h = grundlast(r, len(ps))
            for p in ps: roh[key(p)] = w / len(ps); herkunft[key(p)] = h
        LAST[n] = sum(roh.values())
        pos = [herkunft[k] for k, v in roh.items() if v > 0]
        HERK[n] = 'A' if pos and all(h in EXAKT for h in pos) else 'relativ'
        if any(b['sw']==n for b in BOOST.values()):HERK[n]='relativ'
        NETZ[n]['fluss'] = dict(stufe=STUFE[n], quelle=QUELLE[n], ueber=UEBER.get(n, (None, None))[0],
                                von=UEBER.get(n, (None, None))[1], last={k: round(v, 7) for k, v in roh.items()},
                                gesamt=round(LAST[n], 7), einheit=HERK[n], herkunft=herkunft)
    for u,b in BOOST.items(): NETZ[b['sw']]['wandler'] = dict(art='boost', chip=u, **b)
    SPANNUNG = spannungen(B, fps, pads_by_ref, pads_by_net, NETZ, ROLLE, TYP, STUFE, QUELLE, UEBER, ERZ, GNDS, SPANN_VG, refs_on, shunt, lastpad, ALT)
    def spannung_s(n): return SPANNUNG[n][0] if n in SPANNUNG else None
    STROM_A, TEIL_A, LAST_A = stromschaetzung(B, fps, pads_by_ref, NETZ, ROLLE, TYP, STUFE, QUELLE, UEBER, LAST, HERK, STROM_VG, UMSETZ_VG,
                                              refs_on, pads_auf, key, shunt, sw_chips, GNDS, spannung_s, ist_bootstrap, geregelt, analog_chip, lastpad, ALT)
    for n, (a, q) in STROM_A.items(): NETZ[n]['strom_A'] = a; NETZ[n]['strom_A_quelle'] = q
    for n, (a, q) in signal_stroeme(fps, refs_on, TYP, NETZ, GNDS).items():
        NETZ[n]['strom_A'] = a; NETZ[n]['strom_A_quelle'] = 'signal_' + q
    for n, (v, q) in SPANNUNG.items(): NETZ[n]['spannung_V'] = v; NETZ[n]['spannung_quelle'] = q





    SCHAETZ = VG.get('strom_schaetzung', True) is not False
    for n in sorted(STUFE):
        fl = NETZ[n]['fluss']
        if HERK[n] == 'A': fl['einheit_quelle'] = 'vorgabe'; continue
        if not SCHAETZ or not LAST_A.get(n, ({},))[0]: continue
        la, ha = LAST_A[n]
        fl['relativ'] = dict(last=fl['last'], gesamt=fl['gesamt'], herkunft=fl['herkunft'])
        fl.update(last={k: round(v, 7) for k, v in sorted(la.items())}, gesamt=round(sum(la.values()), 7), einheit='A',
                  einheit_quelle='schaetzung', herkunft=dict(sorted(ha.items())))
    if sw_kompakt:
        from analyse.schaltknoten import korrigiere
        korrigiere(B, dict(netze=NETZ, vorgaben=VG))
    breite_nach_strom(B, NETZ, VG, pads_by_net)
    for n, k in sorted(NETZ.items()):
        bga = any('BGA' in fps[r].get('fp', '').upper() for r in refs_on[n])
        k.update(netzform(n, k, bga))
    teile = {}
    for r, f in fps.items():
        teile[r] = dict(rolle=ROLLE[r], insel=INSEL.get(r) if ROLLE[r] == 'klein' else (r if r in inseln else None),
                        entkopplung=r in ENTK, C=ENTK[r]['C'] if r in ENTK else None,
                        fest=bool(((ROLLE[r] in ('stecker', 'mechanik') or r in BEDIEN) and r not in LOSE) or f.get('locked') or glob_any(r, VG.get('fest'))), bedien=r in BEDIEN,
                        seite=f['side'])
        if r in LOSE: teile[r]['lose'] = True

    FKT = funktionen(B, fps, pads_by_ref, pads_by_net, NETZ, ROLLE, TYP, ENTK, INSEL, ERZ, shunt, supply_pins)
    for r in teile: teile[r].update(FKT[r])
    for r, (a, q) in TEIL_A.items(): teile[r]['strom_A'] = a; teile[r]['strom_A_quelle'] = q


    for r, g in programmierteile(B, fps, pads_by_ref, pads_by_net, NETZ, ROLLE, TYP, GNDS).items(): teile[r]['programmier'] = g
    for r, f in fps.items():
        if f.get('dnp'): teile[r]['dnp'] = True; teile[r]['courtyard_hindernis'] = False
    ABG =messabgriffe(B, fps, pads_by_ref, pads_by_net, NETZ, ROLLE, TYP, teile, shunt)
    teuer =[r for r in fps if glob_any(r, VG.get('teuer_unter'))] or \
        [r for r in fps if ROLLE[r] == 'chip' and any(k['ref'] == r for k in B.get('keepouts', []))]
    K = dict(quelle=B.get('src'), lagen=L, sigw=SIGW, gnd=gnd, gnd_haupt=gnd[0] if gnd else None, netze=NETZ, teile=teile,
                inseln=inseln, teuer_unter=teuer, net_ties=net_ties, abgriffe=ABG, vorgaben=VG, **({'aufbau': VG.get('aufbau') or (RG or {}).get('aufbau')} if VG.get('aufbau') or (RG or {}).get('aufbau') else {}), regeln=(RG or {}).get('router'))
    from analyse.abstaende import vertrag
    K['abstaende'] = vertrag(B, RG)
    if B.get('p3_pin_gruppen') is not None:
        from platzierung.p3_gruppen import anwenden
        anwenden(B, K)
    if stoer_flanke or VG.get('stoer_flanke'):
        from analyse.stoerer import erstellen
        K['stoerer'] = erstellen(K)
        if stoer_flanke: K['stoer_flanke'] = True
    return K


def mst_laenge(pts):
    ""
    pts = sorted(set((round(x, 4), round(y, 4)) for x, y in pts))
    if len(pts) < 2: return 0.0
    d = {p: math.dist(pts[0], p) for p in pts[1:]}; s = 0.0
    while d:
        p = min(d, key=lambda q: (d[q], q)); s += d.pop(p)
        for q in d: d[q] = min(d[q], math.dist(p, q))
    return s


def breite_nach_strom(B, NETZ, VG, pads_by_net):
    ""
    if VG.get('strom_breite') is False: return
    dT = float(VG.get('erwaermung_K') or IPC_DT)
    from analyse.strombreite import kupferdicke
    review = VG.get('ipc_innen_modell') == '2221' or bool(VG.get('review_fix'))
    ta, ti = (kupferdicke(VG), kupferdicke(VG, innen=True)) if review else (KUPFER_AUSSEN, KUPFER_INNEN)
    over = [(re.compile(k), v) for k, v in (VG.get('netze') or {}).items()]
    def w_vorgabe(n):
        b = basisname(n)
        for rx, v in over:
            if rx.fullmatch(b) or rx.fullmatch(n): return 'w' in v
        return False
    P = B['pads']
    def strom(n):
        fl = NETZ[n].get('fluss') or {}
        return float(fl.get('gesamt') or 0.0) if fl.get('einheit') == 'A' else 0.0
    def laenge(n): return BUDGET_UMWEG * mst_laenge([(P[i]['x'], P[i]['y']) for i in pads_by_net.get(n, [])])
    budget = {}
    for n in NETZ:
        if NETZ[n]['art'] not in ('strom', 'versorgung', 'sw'): continue
        for rx, mv in (VG.get('budget_mV') or {}).items():
            if n == rx or re.fullmatch(rx, n) or re.fullmatch(rx, basisname(n)):
                I = strom(n)
                if I > 0 and float(mv) > 0:
                    w = RHO_CU * laenge(n) * I / (ta * float(mv) / 1000)
                    budget[n] = (w, w * ta / ti, 'budget %g mV' % float(mv))
                break
    for name, g in sorted((VG.get('kanal_budget_mV') or {}).items()):
        nn = [n for n in g.get('netze', []) if n in NETZ and NETZ[n]['art'] in ('strom', 'versorgung', 'sw')]
        S = sum(laenge(n) * strom(n) for n in nn)
        if not nn or S <= 0 or float(g.get('mV') or 0) <= 0: continue
        w = RHO_CU * S / (ta * float(g['mV']) / 1000)
        for n in nn:
            if w > budget.get(n, (0,))[0]: budget[n] = (w, w * ta / ti, 'kanal %s %g mV' % (name, float(g['mV'])))
    for n, k in sorted(NETZ.items()):
        if k['art'] not in ('strom', 'versorgung', 'sw'): continue
        I = strom(n)
        if I <= 0: continue
        w_ipc = ipc_breite(I, dT, ta); wb, wb_i, bq = budget.get(n, (0.0, 0.0, None))
        k['w_ipc'] = round(w_ipc, 3)
        if bq: k['w_budget'] = round(wb, 3); k['w_budget_quelle'] = bq
        soll = max(w_ipc, wb)
        if soll > W_STROM_MAX: k['flaeche_noetig'] = True
        if not w_vorgabe(n) and min(soll, W_STROM_MAX) > k['w'] + 1e-9:
            k['w'] = round(min(soll, W_STROM_MAX), 3); k['w_quelle'] = 'budget' if wb > w_ipc else ('ipc2221' if review else 'ipc2152')
        k['w_innen'] = round(max(k['w'], min(W_STROM_MAX, max(INNEN_FAKTOR * w_ipc, wb_i))), 3)
        if not review: continue
        k['w_ipc_quelle'] = 'ipc2221_aussen'; k['w_innen_quelle'] = 'heuristik_2.3 (keine IPC-2152-Korrektur)'
        if VG.get('ipc_innen_modell') == '2221':
            from analyse.strombreite import ipc2221, kupferdicke
            ta, ti = kupferdicke(VG), kupferdicke(VG, innen=True)
            wi = ipc2221(I, dT, ti, innen=True)
            wa = ipc2221(I, dT, ta)
            k['w_ipc'] = round(wa, 3); k['w_ipc_innen'] = round(wi, 3)
            k['w_innen'] = round(max(k['w'], min(W_STROM_MAX, max(wi, wb * ta / ti))), 3)
            k['w_innen_quelle'] = 'ipc2221_innen, kupferdicke=%g mm' % ti
            if not w_vorgabe(n): k['w'] = round(max(k['w'], min(W_STROM_MAX, wa)), 3)
        innen_soll = max(wi, wb * ta / ti) if VG.get('ipc_innen_modell') == '2221' else max(INNEN_FAKTOR * w_ipc, wb_i)
        aussen_soll = max(soll, wa) if VG.get('ipc_innen_modell') == '2221' else soll
        k['w_bedarf_aussen'] = round(aussen_soll, 3)
        k['w_bedarf_innen'] = round(max(k['w'], innen_soll), 3)
        k['w_defizit_aussen'] = round(max(0.0, aussen_soll - k['w']), 3)
        k['w_defizit_innen'] = round(max(0.0, innen_soll - k['w_innen']), 3)
        if max(aussen_soll, innen_soll) > W_STROM_MAX: k['flaeche_noetig'] = True


def spannungen(B, fps, pads_by_ref, pads_by_net, NETZ, ROLLE, TYP, STUFE, QUELLE, UEBER, ERZ, GNDS, SPANN_VG, refs_on, shunt, lastpad, ALT=()):
    ""
    P = B['pads']; out = {}
    def name_v(n):
        b = basisname(n)
        for pat, v in SPANN_VG.items():
            if re.fullmatch(pat, b, re.I): return v, 'vorgabe'
        m = re.fullmatch(r'\+?(\d+)V(\d*)', b, re.I)
        if m: return float(m.group(1) + '.' + (m.group(2) or '0')), 'name'
        m = re.search(r'(?<![A-Z0-9.])\+?(\d{1,2})[.,](\d{1,2})\s?V(?![A-Z0-9])', b.upper())
        if m: return float(m.group(1) + '.' + m.group(2)), 'name'
        for t in tokens(n):
            m = re.fullmatch(r'\+?(\d{1,2})V(\d{0,2})', t)
            if m: return float(m.group(1) + '.' + (m.group(2) or '0')), 'name'
        return None
    def regler(n):
        if UEBER.get(n):
            r, m = UEBER[n]
            if ROLLE[r] == 'chip': return r
            if TYP[r] == 'L' and NETZ[m]['art'] == 'sw' and UEBER.get(m) and ROLLE[UEBER[m][0]] == 'chip': return UEBER[m][0]
            return None
        e = sorted(ERZ.get(n, ()))
        return e[0] if e else None
    def fb_teiler(n, u):
        vref = tabelle(REGLER_VREF, (fps[u].get('value') or '').upper())
        if not vref: return None
        for f in sorted({P[i]['net'] for i in pads_by_ref[u]}):
            if not f or f == n or f in GNDS or f not in pads_by_net: continue
            oben, unten = [], []
            for j in pads_by_net[f]:
                r = P[j]['ref']
                if TYP.get(r) != 'R' or len(pads_by_ref[r]) != 2: continue
                anders = [P[k]['net'] for k in pads_by_ref[r] if k != j]
                R = widerstand(fps[r].get('value', ''))
                if not R or not anders: continue
                if anders[0] == n: oben.append(R)
                elif anders[0] in GNDS: unten.append(R)
            if oben and unten:
                Ro = 1 / sum(1 / x for x in oben); Ru = 1 / sum(1 / x for x in unten)
                return round(vref * (1 + Ro / Ru), 3), 'fb_teiler %s (Vref %g V, %s)' % (u, vref, basisname(f))
        return None
    def diode_v(n):
        best = None
        for i in pads_by_net.get(n, []):
            r = P[i]['ref']
            if TYP.get(r) not in ('D', 'TVS', 'Z', 'ESD') or len(pads_by_ref[r]) != 2: continue
            if not any(P[k]['net'] in GNDS for k in pads_by_ref[r] if k != i): continue
            w = (fps[r].get('value') or '').upper(); v = None
            for pat, f in TVS_V:
                m = re.search(pat, w)
                if m: v = float(m.group(1)) * f; break
            if v is None:
                m = re.search(ZENER_V, w)
                if m: v = float(m.group(1) + '.' + (m.group(2) or '0'))
            if v is None and re.search(r'TVS', '%s %s %s' % (r, w, fps[r].get('fp') or ''), re.I):
                m = re.search(TVS_SONST_V, w)
                if m: v = float(m.group(1))
            s = schiene(v) if v else None
            if s and (best is None or s < best[0]): best = (s, 'schutzdiode %s %s' % (r, fps[r].get('value')))
        return best
    def usb_v(n):
        if STUFE.get(n) != 0: return None
        if any(PD_CHIP.search(fps[r].get('value') or '') for r in refs_on[n]): return None
        for r in sorted(refs_on[n]):
            f = fps[r]
            if ROLLE[r] == 'stecker' and re.search(r'USB', '%s %s %s' % (f.get('fp', ''), f.get('value', ''), f.get('lib', '')), re.I) \
                    and any('%s:%s' % (P[i]['ref'], P[i]['num']) in QUELLE[n] for i in pads_by_ref[r] if P[i]['net'] == n):
                return 5.0, 'usb %s' % r
        return None
    for n in sorted(STUFE):
        if NETZ[n]['art'] == 'sw': continue
        v = name_v(n)
        u = regler(n) if v is None else None
        if u: v = (lambda x: (x, 'festspannung %s' % u) if x else None)(festspannung(fps[u].get('value'))) or fb_teiler(n, u)
        v = v or diode_v(n) or usb_v(n)
        if v: out[n] = v
    def reihe(r, a, b):
        if ROLLE[r] == 'chip' or NETZ[a]['art'] == 'sw' or NETZ[b]['art'] == 'sw': return False
        return TYP[r] in ('F', 'PTC', 'FB', 'L', 'Q') or (TYP[r] == 'R' and shunt(r))
    kanten = sorted([(UEBER[n][0], UEBER[n][1], n) for n in STUFE if UEBER.get(n)] + list(ALT))
    for _ in range(8):
        neu = False
        for r, m, n in kanten:
            if m not in STUFE or n not in STUFE or not reihe(r, m, n): continue
            if n not in out and m in out: out[n] = (out[m][0], 'wie %s ueber %s' % (basisname(m), r)); neu = True
            elif m not in out and n in out: out[m] = (out[n][0], 'wie %s ueber %s' % (basisname(n), r)); neu = True
        if not neu: break
    return out


def stromschaetzung(B, fps, pads_by_ref, NETZ, ROLLE, TYP, STUFE, QUELLE, UEBER, LAST, HERK, STROM_VG, UMSETZ_VG,
                    refs_on, pads_auf, key, shunt, sw_chips, GNDS, spannung, ist_bootstrap, geregelt, analog_chip, lastpad, ALT=()):
    ""
    def txt(r): f = fps[r]; return '%s %s %s' % (f.get('value', '') or '', f.get('fp', '') or '', f.get('lib', '') or '')
    def mess_shunt(r):
        return any(analog_chip(q) for i in pads_by_ref[r] for q in refs_on.get(B['pads'][i]['net'], ()) if B['pads'][i]['net'] not in GNDS)
    def shunt_A(r):
        ""
        R = widerstand(fps[r].get('value', ''))
        if not R: return None
        m = re.search(r'(0201|0402|0603|0805|1206|1210|1812|2010|2512|2725|2728|3920)', fps[r].get('fp', '') or '')
        Imax = math.sqrt((SHUNT_P[m.group(1)] if m else SHUNT_P_SONST) / R)
        if R < SHUNT_R_MIN: return Imax if mess_shunt(r) else None
        return min(SHUNT_U / R, Imax)
    def stecker_A(r, n_pins, nur_tabelle=False):
        w = tabelle(STECKER_A, txt(r))
        if not w and nur_tabelle: return None
        je, ges = w if w else (STECKER_SONST_A, None)
        return min(je * n_pins, ges) if ges else je * n_pins
    def regler_A(r, nur_tabelle=False):
        a = tabelle(REGLER_A, fps[r].get('value', '')) or tabelle(REGLER_GEH_A, fps[r].get('fp', ''))
        return a if a or nur_tabelle else (2.0 if r in sw_chips else 0.5)
    def ist_led(r):
        return TYP[r] == 'LED' or praefix(r) == 'LED' or bool(re.search(r'(^|[^A-Z])LED', '%s %s' % (fps[r].get('fp', ''), fps[r].get('value', '')), re.I))
    def verbraucher(r, n):
        ""
        t = TYP[r]
        if r in STROM_VG: return STROM_VG[r], 'vorgabe'
        if ROLLE[r] == 'chip':
            a = tabelle(VERBRAUCHER_A, txt(r))
            if a: return a, 'familie'
            if analog_chip(r): return OPV_A, 'opv'
            np_ = fps[r].get('n_pads', 0)
            return next(a for bis, a in CHIP_PADS_A if np_ <= bis), 'gehaeuse_%dpads' % np_
        if t == 'C': return 0.0, 'C_0'
        if t == 'R':
            if shunt(r):
                a = shunt_A(r)
                return (a, 'shunt') if a is not None else (0.0, 'bruecke_unter_1mOhm')
            R = widerstand(fps[r].get('value', ''))
            return (min((spannung(n) or 3.3) / R, R_MAX_A), 'ohm') if R else (1e-4, 'R_unbekannt')
        if ist_led(r) and ROLLE[r] != 'chip': return tabelle(VERBRAUCHER_A, txt(r)) or LED_A, 'led'
        if t == 'D': return 0.0, 'diode_0'
        if t in ('F', 'PTC'):
            nenn = sicherung_ampere(fps[r].get('value', ''))
            return (0.5 * nenn, 'sicherung_halb') if nenn else (1e-3, 'sicherung_ohne_wert')
        if t == 'Q': return (tabelle(TRANSISTOR_A, fps[r].get('fp', ''), 0.5), 'transistor_gehaeuse') if lastpad(r, n) else (0.0, 'gate')
        return 1e-3, 'sonst'

    lastnetze = collections.Counter()
    for n in STUFE:
        src = set(QUELLE[n])
        for r in refs_on[n]:
            if ROLLE[r] in ('chip', 'klein') and any(key(p) not in src for p in pads_auf(r, n)) \
                    and not any(rr == r and nn == n for rr, nn in UEBER.values()):
                lastnetze[r] += 1



    verlangt = set()
    for n in STUFE:
        if any(key(p) in STROM_VG or p['ref'] in STROM_VG for r in refs_on[n] for p in pads_auf(r,n) if key(p) not in set(QUELLE[n])): verlangt.add(n)
    for n in sorted(STUFE, key=lambda n:-STUFE[n]):
        if n in verlangt and UEBER.get(n): verlangt.add(UEBER[n][1])
    for n in verlangt: NETZ[n]['strom_verlangt'] = True
    HD = {}
    for n in sorted(STUFE, key=lambda n: (STUFE[n], n)):
        d = []; src = set(QUELLE[n])
        if STUFE[n] == 0:
            for r in sorted(refs_on[n]):
                k = sum(1 for p in pads_auf(r, n) if key(p) in src)
                a = stecker_A(r, k, nur_tabelle=True) if ROLLE[r] == 'stecker' and k else None
                if a: d.append((a, 'stecker %s' % r))
        for r0 in sorted(refs_on[n]):
            if TYP[r0] in ('F', 'PTC') and sicherung_ampere(fps[r0].get('value', '')):
                nenn = sicherung_ampere(fps[r0].get('value', '')); d.append((SICHERUNG_BETRIEB * nenn, 'sicherung %s %gA' % (r0, nenn)))
        if UEBER.get(n):
            r0, m0 = UEBER[n]
            if ROLLE[r0] == 'chip':
                a = regler_A(r0, nur_tabelle=True)
                if a: d.append((a, 'regler %s' % r0))
            elif HD.get(m0): d.append(HD[m0])
        HD[n] = min(d) if d else None
    def durchlauf(aus_stecker, ein={}, senke={}, vorher={}):
        ""
        IA = {}; Q = {}; SH = {}; TA = collections.defaultdict(float); TQ = {}; PL = {}
        TAUS = collections.defaultdict(float); TEIN = collections.defaultdict(float); TRUECK = collections.defaultdict(float)
        for n in sorted(STUFE, key=lambda n: (-STUFE[n], n)):
            src = set(QUELLE[n]); beitr = []; shunts = []; pl = {}
            if UEBER.get(n) and TYP[UEBER[n][0]] == 'R' and shunt(UEBER[n][0]):
                a = shunt_A(UEBER[n][0])
                if a is not None: shunts.append(a)
            for r in sorted(refs_on[n]):
                ps = [p for p in pads_auf(r, n) if key(p) not in src]
                if not ps: continue
                unten = [m for m, (rr, nn) in UEBER.items() if rr == r and nn == n]
                sh = False; je_pad = None
                if (r, n) in ein:
                    w = vorher.get(ein[(r, n)], 0.0); h = 'zweite_einspeisung_nach_%s' % basisname(ein[(r, n)])
                    TA[r] = max(TA[r], w); TQ[r] = 'durchfluss_' + h
                elif (r, n) in senke: w, h = 0.0, 'zweite_einspeisung'
                elif unten:
                    aus = [m for m in unten if not ist_bootstrap(m)] or unten
                    ia = sum(IA.get(m, 0.0) for m in aus); sh = any(SH.get(m) for m in aus)
                    if r in UMSETZ_VG: w = UMSETZ_VG[r] * ia; h = 'umsetzung_vorgabe'
                    elif any((NETZ.get(m,{}).get('wandler') or {}).get('art')=='boost' and (NETZ[m]['wandler']['vin']==n) for m in aus):
                        ziele = [NETZ[m]['wandler']['ausgang'] for m in aus if (NETZ.get(m,{}).get('wandler') or {}).get('art')=='boost']
                        ve=spannung(n)
                        w = sum((spannung(x) or 0)*IA.get(x,0) for x in ziele)/(ve*.9) if ve else ia
                        h = 'boost_spannung_eta0.9'
                    elif ROLLE[r] != 'chip': w = ia; h = 'reihenglied'
                    elif r not in sw_chips: w = ia; h = 'linearregler'
                    else:
                        ve = spannung(n); ziele = [x for m in aus for x in geregelt(m)]; va = {x: spannung(x) for x in ziele}
                        if ve and ziele and all(va.values()): w = sum(va[x] * IA.get(x, 0.0) for x in ziele) / (ve * 0.9); h = 'schaltregler_spannung_eta0.9'
                        else: w = 0.5 * ia; h = 'schaltregler_0.5'
                    TA[r] = max(TA[r], ia); TQ[r] = 'durchfluss_' + h
                elif (NETZ[n].get('wandler') or {}).get('chip')==r:
                    w,h=0.,'schaltpin_kein_DC_Verbraucher'
                elif any(key(p) in STROM_VG for p in ps):
                    je_pad = {key(p): STROM_VG.get(key(p), 0.0) for p in ps}
                    w = sum(je_pad.values()); h = 'vorgabe_pad'; TA[r] += w; TQ[r] = h
                elif r in STROM_VG:
                    w,h=STROM_VG[r],'vorgabe';TA[r]+=w;TQ[r]=h
                elif ROLLE[r] == 'stecker':
                    if r in aus_stecker: w = aus_stecker[r][0] * len(ps) / aus_stecker[r][1]; h = 'stecker_kirchhoff'
                    else: w = stecker_A(r, len(ps)); h = 'stecker_pin'
                    TAUS[r] += w; TQ[r] = h
                else:
                    w, h = verbraucher(r, n)
                    if h == 'shunt': shunts.append(w)
                    TA[r] = w; TQ[r] = h
                    w = w / max(1, lastnetze[r]) if ROLLE[r] == 'chip' and h != 'vorgabe' else w
                beitr.append((w, h, r, sh))
                for p in ps: pl[key(p)] = ((je_pad[key(p)] if je_pad is not None else w / len(ps)), h)
            PL[n] = pl
            if HERK.get(n) == 'A':
                IA[n] = LAST[n]; Q[n] = 'vorgabe'; SH[n] = False; continue
            s = sum(b[0] for b in beitr)
            haupt = max(beitr, key=lambda b: (b[0], b[2])) if beitr else None
            if shunts:
                a, q = sum(shunts), 'shunt'
                if HD.get(n) and a > HD[n][0]: a, q = HD[n][0], 'shunt_gedeckelt %s' % HD[n][1]
                IA[n] = a; Q[n] = q; SH[n] = True; continue
            q = 'summe(%s %s)' % (haupt[1], haupt[2]) if haupt and haupt[0] > 0 else 'keine_last'
            SH[n] = bool(haupt and haupt[3])
            deckel = []
            if STUFE[n] == 0:
                for r in sorted(refs_on[n]):
                    k = sum(1 for p in pads_auf(r, n) if key(p) in src)
                    if ROLLE[r] == 'stecker' and k: deckel.append((stecker_A(r, k), 'stecker %s' % r))
            for r0 in sorted(refs_on[n]):
                if TYP[r0] in ('F', 'PTC') and sicherung_ampere(fps[r0].get('value', '')):
                    nenn = sicherung_ampere(fps[r0].get('value', '')); deckel.append((SICHERUNG_BETRIEB * nenn, 'sicherung %s %gA' % (r0, nenn)))
            if UEBER.get(n) and ROLLE[UEBER[n][0]] == 'chip': deckel.append((regler_A(UEBER[n][0]), 'regler %s' % UEBER[n][0]))
            if deckel and n not in verlangt and not ist_bootstrap(n):
                d, dq = min(deckel)
                if s > d: s = d; q = 'deckel %s' % dq
                elif s < 1e-3: s = 0.5 * d; q = 'nur_quelle %s' % dq
            boost=NETZ[n].get('wandler') or {}
            if boost.get('art')=='boost':
                ve,va=spannung(boost['vin']),spannung(boost['ausgang'])
                io=IA.get(boost['ausgang'],s)
                if ve and va:
                    s=io*va/(ve*.9);q='boost_induktor_mittelstrom_eta0.9'
                    ps=pads_auf(boost['chip'],n)
                    for p in ps: PL[n][key(p)]=(max(0.,s-io)/max(1,len(ps)),'boost_Schalter_Mittelstrom')
            IA[n] = s; Q[n] = q



        for r in sorted(fps):
            ns=sorted(refs_on_n for refs_on_n in refs_on if r in refs_on[refs_on_n])
            if TYP[r]!='D' or len(ns)!=4 or not any(x in GNDS for x in ns):continue
            dc=[m for m,(rr,_) in UEBER.items() if rr==r and m in IA and NETZ[m]['art'] in ('strom','versorgung')]
            ac=[m for m in ns if m in STUFE and STUFE[m]==0 and m not in GNDS and m not in dc]
            if len(dc)!=1 or len(ac)!=2:continue
            for m in ac:
                IA[m]=IA[dc[0]];Q[m]='gleichrichter_Mittelstrom_'+dc[0]
                NETZ[m]['strom_modell']='gleichricht_Mittelwert_RMS_unbekannt'
                for p in pads_auf(r,m):PL[m][key(p)]=(IA[m]/len(pads_auf(r,m)),'gleichricht_Mittelwert_RMS_unbekannt')
        for n in sorted(STUFE, key=lambda n: (STUFE[n], n)):
            if UEBER.get(n) and NETZ.get(UEBER[n][1],{}).get('wandler',{}).get('art') != 'boost' and ROLLE[UEBER[n][0]] != 'chip' and n not in verlangt and not SH.get(n) and IA[n] > IA.get(UEBER[n][1], IA[n]):
                IA[n] = IA[UEBER[n][1]]; Q[n] = 'deckel_oben %s' % UEBER[n][1]
        for n in STUFE:
            for r in sorted(refs_on[n]):
                k = sum(1 for p in pads_auf(r, n) if key(p) in set(QUELLE[n]))
                if ROLLE[r] == 'stecker' and k:
                    rueck = NETZ[n]['art'] == 'strom' and SH.get(n)
                    (TRUECK if rueck else TEIN)[r] += IA[n] * k / len(QUELLE[n]); TQ.setdefault(r, 'einspeisung')
        for r in set(TAUS) | set(TEIN) | set(TRUECK): TA[r] = max(TAUS[r], TEIN[r], TRUECK[r])
        return IA, Q, SH, TA, TQ, PL
    alle = {(r, x): True for r, a, b in ALT for x in (a, b)}
    IA, Q, SH, TA, TQ, PL = durchlauf({}, {}, alle)


    aus = {}
    for r in sorted(fps):
        if ROLLE[r] != 'stecker': continue
        rueck = sum(IA[n] for n in STUFE if STUFE[n] == 0 and NETZ[n]['art'] == 'strom' and SH.get(n)
                    and any(key(p) in set(QUELLE[n]) for p in pads_auf(r, n)))
        pins = sum(1 for n in STUFE if NETZ[n]['art'] in ('versorgung', 'strom') for p in pads_auf(r, n) if key(p) not in set(QUELLE[n]))
        if rueck > 0 and pins: aus[r] = (rueck, pins)
    if aus: IA, Q, SH, TA, TQ, PL = durchlauf(aus, {}, alle)


    if ALT:
        ein = {}
        def eigen(n): return 0.0 if Q.get(n, '').startswith(('nur_quelle', 'keine_last')) else IA.get(n, 0.0)
        for r, a, b in ALT:
            v, z = (a, b) if eigen(a) >= eigen(b) else (b, a)

            if eigen(v) <= 0 or eigen(z) > 0.1 * eigen(v): continue
            ein[(r, z)] = v
        senke = {k: True for k in alle if k not in ein}
        for _ in range(2):
            if ein: IA, Q, SH, TA, TQ, PL = durchlauf(aus, ein, senke, IA)


    pads = {}
    for n in sorted(PL):
        s = sum(w for w, _ in PL[n].values())
        f = IA[n] / s if s > 1e-12 else 0.0
        pads[n] = ({k: w * f for k, (w, _) in PL[n].items()}, {k: 'schaetzung_' + h for k, (_, h) in PL[n].items()})
    netze = {n: (round(IA[n], 9), Q[n]) for n in sorted(IA)}
    teile = {r: (round(TA[r], 9), TQ[r]) for r in sorted(TA) if r in TQ}
    return netze, teile, pads



MESS_PIN = re.compile(r'SENSE|SNS|ISNS|ISEN|IMON|ADC|AIN|^CS[PN+-]?$|^V?FB$|^IN[+-]$|^[+-]IN|^[+-]$|NINV|^INV', re.I)

MESS_NETZ = re.compile(r'CURR|SENS|SNS|ISNS|IMON|SHUNT|MEAS|ADC|AIN\d|CSP|CSN', re.I)


def messabgriffe(B, fps, pads_by_ref, pads_by_net, NETZ, ROLLE, TYP, teile, shunt):
    ""
    P = B['pads']
    def key(i): return '%s:%s' % (P[i]['ref'], P[i]['num'])
    def art(n): return NETZ[n]['art'] if n in NETZ else None
    def zweipol(r, t): return ROLLE.get(r) == 'klein' and TYP.get(r) in t and len(pads_by_ref[r]) == 2
    def ist_shunt(r):
        m = re.match(r'\s*R(\d+)\b', fps[r].get('value', '') or '')
        return shunt(r) or (TYP.get(r) == 'R' and bool(m) and 0 < float('0.' + m.group(1)) <= 1.0)
    def mess(m, n, tiefe=0):
        ""
        if m not in NETZ or m == n or NETZ[m].get('kelvin') or art(m) in ('gnd', 'strom', 'versorgung', 'sw'): return False
        if art(m) == 'analog' or MESS_NETZ.search(basisname(m)): return True
        for i in pads_by_net.get(m, []):
            if ROLLE.get(P[i]['ref']) == 'chip' and MESS_PIN.search((P[i].get('pinfunc') or '').strip()): return True
        if tiefe == 0:
            for i in pads_by_net.get(m, []):
                r = P[i]['ref']
                if zweipol(r, ('R',)):
                    m2 = next(P[j]['net'] for j in pads_by_ref[r] if j != i)
                    if m2 != n and mess(m2, n, 1): return True
        return False
    out = {}
    for n in sorted(pads_by_net):
        k = NETZ.get(n) or {}
        if not (k.get('kelvin') or k.get('art') in ('strom', 'versorgung')): continue
        ps = pads_by_net[n]
        if k.get('kelvin'): st = [i for i in ps if P[i]['ref'] == k['kelvin']]; wo = 'Net-Tie %s' % k['kelvin']
        else:
            st = [i for i in ps if teile.get(P[i]['ref'], {}).get('funktion') == 'shunt' or ist_shunt(P[i]['ref'])]; wo = 'Shunt'
            if not st and k.get('art') == 'strom':
                st = [i for i in ps if TYP.get(P[i]['ref']) == 'Q']; wo = 'Lastanschluss Transistor'
        if not st: continue
        taps = []
        for i in ps:
            r = P[i]['ref']
            if i in st or ROLLE.get(r) != 'klein' or TYP.get(r) not in ('R', 'C', 'D') or ist_shunt(r) \
                    or teile.get(r, {}).get('funktion') in ('shunt', 'entkopplung_klein', 'entkopplung_mittel', 'entkopplung_gross'): continue
            andere = [P[j]['net'] for j in pads_by_ref[r] if j != i and P[j]['net'] != n]
            if not andere or any(art(m) in ('gnd', 'versorgung') or (NETZ.get(m) or {}).get('kelvin') for m in andere): continue
            ziel = [m for m in andere if mess(m, n)]
            if TYP[r] in ('R', 'C') and (len(andere) != 1 or not ziel): continue
            if TYP[r] == 'D' and not ziel: continue
            s = min(st, key=lambda j: (math.hypot(P[j]['x'] - P[i]['x'], P[j]['y'] - P[i]['y']), key(j)))
            taps.append(dict(pad=key(i), start=key(s), grund='%s %s -> %s, Start am %s' % (TYP[r], fps[r].get('value', ''), basisname(ziel[0]), wo)))
        if not taps: continue
        for i in ps:
            r = P[i]['ref']
            if i in st or not zweipol(r, ('R',)) or any(t['pad'] == key(i) for t in taps): continue
            v = widerstand(fps[r].get('value', '')); m = next(P[j]['net'] for j in pads_by_ref[r] if j != i)
            if v and v >= 1e5 and art(m) == 'versorgung':
                taps.append(dict(pad=key(i), start=None, grund='R %s an %s: stromloser Pull-up, keine Messleitung' % (fps[r].get('value', ''), basisname(m))))
        out[n] = sorted(taps, key=lambda t: t['pad'])
    return out
PROG_STECKER = re.compile(r'TAG.?CONNECT|TC20[35]0|SWD|JTAG|ICSP|\bISP\b|_ISP|UPDI|CORTEX.?DEBUG|DEBUG|PROG', re.I)
PROG_DEBUG = SWD_T | {'UPDI', 'SWIM', 'PDI', 'MCLR'}
PROG_RESET = {'NRST', 'RST', 'RESET', 'NRESET', 'MCLR'}
PROG_ICSP = {'MISO', 'MOSI', 'SCK'}
PROG_UART = {'TX', 'RX', 'TXD', 'RXD'}
TASTER = re.compile(r'BUTTON|SWITCH|TASTER|SW_PUSH|TACT|PUSH', re.I)


def prog_treffer(T):
    ""
    d = T & PROG_DEBUG; rs = T & PROG_RESET; ic = T & PROG_ICSP; ua = T & PROG_UART
    if len(d) >= 2 or (d and rs): return d | rs
    if len(ic) >= 2 and rs: return ic | rs
    if ua and T & {'DTR', 'RTS', 'BOOT', 'BOOT0'}: return ua | (T & {'DTR', 'RTS', 'BOOT', 'BOOT0'})
    if ua and T & {'IO0', 'GPIO0'} and T & (PROG_RESET | {'EN', 'CHIP_PU'}): return ua | (T & {'IO0', 'GPIO0', 'EN', 'CHIP_PU'}) | rs
    return set()


def programmierteile(B, fps, pads_by_ref, pads_by_net, NETZ, ROLLE, TYP, GNDS):
    ""
    P = B['pads']; out = {}
    def netze(r): return [P[i]['net'] for i in pads_by_ref[r]]
    def namen(n, r=None):
        s = set(tokens(n)) if n else set()
        for i in pads_by_net.get(n, []):
            if r is None or P[i]['ref'] == r: s |= set(tokens(P[i].get('pinfunc') or ''))
        return s
    def signal(n): return n in NETZ and NETZ[n]['art'] not in ('gnd', 'versorgung', 'strom', 'sw')
    def chip_an(n): return any(ROLLE.get(P[i]['ref']) == 'chip' for i in pads_by_net.get(n, []))
    for r in sorted(fps):
        f = fps[r]
        if ROLLE[r] not in ('stecker', 'ohne', 'klein', 'mechanik'): continue
        txt = '%s %s %s' % (f.get('fp', ''), f.get('value', ''), f.get('lib', ''))
        sig = sorted({n for n in netze(r) if signal(n)})
        treffer = sorted(prog_treffer({t for n in sig for t in namen(n, r)}))
        if re.search(r'TAG.?CONNECT|TC20[35]0', txt, re.I): out[r] = 'Tag-Connect'
        elif (ROLLE[r] == 'stecker' or f.get('dnp')) and sig and (PROG_STECKER.search(txt) or treffer):
            out[r] = '%sProgrammierstecker (%s)' % ('unbestueckter ' if f.get('dnp') else '', ', '.join(treffer) or PROG_STECKER.search(txt).group(0))
    pnetze = {n for r in out for n in netze(r) if signal(n)}
    for r in sorted(fps):
        if r in out or len(pads_by_ref[r]) < 2: continue
        ns = netze(r); an = [n for n in ns if n in pnetze]
        if not an: continue
        rest = [n for n in ns if n not in pnetze]
        if TYP[r] == 'R' and len(ns) == 2 and all(signal(n) for n in ns) and any(not chip_an(n) for n in an) and any(chip_an(n) for n in ns):


            out[r] = 'Reihenwiderstand am Programmierstecker (%s)' % ', '.join(sorted({basisname(n) for n in an}))
        elif TYP[r] in ('D', 'TVS', 'ESD') and rest and all(n in GNDS for n in rest):
            out[r] = 'Schutzdiode am Programmierstecker (%s)' % ', '.join(sorted({basisname(n) for n in an}))
        elif (TASTER.search('%s %s' % (fps[r].get('fp', ''), fps[r].get('value', ''))) or praefix(r) in ('SW', 'BTN', 'S', 'KEY')) \
                and rest and all(n in GNDS for n in rest):
            out[r] = 'Taster am Programmierstecker (%s)' % ', '.join(sorted({basisname(n) for n in an}))
    return out


def netzform(n, k, bga=False):
    ""
    if 'form_rmax_mm' in k and (not math.isfinite(float(k['form_rmax_mm'])) or float(k['form_rmax_mm']) <= 0):
        raise ValueError('ungueltiger Formradius fuer '+n)
    if k.get('form'):
        if k['form'] not in ('45r', 'boegen', 'organisch'):
            raise ValueError('unbekannte Netzform fuer %s: %s' % (n, k['form']))
        radius=float(k.get('form_rmax_mm', .5 if k['form']=='45r' else 3.))
        if not math.isfinite(radius) or radius<=0:
            raise ValueError('ungueltiger Formradius fuer '+n)
        return dict(form=k['form'], form_grund=k.get('form_grund','Vorgabe'), form_rmax_mm=radius)
    if k.get('art') in ('strom', 'versorgung', 'sw'):
        f, g = 'organisch', 'Strompfad (Breitenanker bleiben)'
    elif k.get('paar'):
        f, g = 'boegen', 'Differenzpaar (gemeinsam, konzentrisch)'
    elif re.search(r'(?:^|[/_])(RF|ANT|ANTENNA|LNA)(?:$|[_0-9])', n, re.I) or 'HF-Anpassung' in k.get('grund', '') or k.get('rf'):
        f, g = 'boegen', 'RF/Antennenleitung'
    elif k.get('laengengruppe') or k.get('schnittstelle') in ('ddr3', 'sdram', 'qspi', 'spi', 'sdio') :
        f, g = '45r', 'Bus/BGA (Packung und Laengenabgleich)'
    elif (k.get('rang') or 0) >= 5:
        f, g = 'boegen', 'Rang-5-Einzelnetz'
    else:
        f, g = '45r', 'GPIO/analog/langsam'
    return dict(form=f, form_grund=g, form_rmax_mm=float(k.get('form_rmax_mm', .5 if f == '45r' else 3.)),
                **({'form_ausfaecherung': '45r'} if bga else {}))


def bericht(K):
    out = []; N = K['netze']
    c = collections.Counter((k['art'], k.get('strom') or k.get('rang')) for k in N.values())
    out.append('Lagen %d, Signalbreite %.2f mm, GND %s' % (K['lagen'], K['sigw'], K['gnd']))
    for art in ('sw', 'schnell', 'versorgung', 'strom', 'analog', 'signal', 'gnd'):
        ns = sorted([n for n, k in N.items() if k['art'] == art], key=lambda n: (-(N[n].get('rang') or 0), n))
        if not ns: continue
        out.append('%-11s %3d  %s' % (art, len(ns), ', '.join('%s%s' % (basisname(n), '' if art in ('gnd',) else
                                      '[%s]' % (N[n]['strom'] if N[n].get('strom') else N[n]['rang'])) for n in ns[:40]) + (' ...' if len(ns) > 40 else '')))
    fl = {n: k['fluss'] for n, k in N.items() if k.get('fluss')}
    if fl:
        kinder = collections.defaultdict(list)
        for n, f in fl.items():
            if f['von']: kinder[f['von']].append(n)
        zeilen = []
        def baum(n, tiefe):
            f = fl[n]; k = N[n]
            sa = k.get('strom_A')
            if f['einheit'] == 'A': last = '%.3g A (%s)' % (f['gesamt'], f.get('einheit_quelle') or 'A')
            else: last = 'Last %.2f relativ' % f['gesamt']
            zus = '' if sa is None or f.get('einheit_quelle') == 'vorgabe' else ' aus %s' % k.get('strom_A_quelle')
            if k.get('spannung_V') is not None: zus += ', %g V (%s)' % (k['spannung_V'], k.get('spannung_quelle'))
            if k.get('w_quelle'): zus += ', w %.2f mm (%s)' % (k['w'], k['w_quelle'])
            if k.get('flaeche_noetig'): zus += ', Flaeche noetig'
            if k.get('w_defizit_innen'): zus += ', innen fehlen %g mm (Bedarf %g mm)' % (k['w_defizit_innen'], k['w_bedarf_innen'])
            if k.get('w_defizit_aussen'): zus += ', aussen fehlen %g mm (Bedarf %g mm)' % (k['w_defizit_aussen'], k['w_bedarf_aussen'])
            zeilen.append('  ' * tiefe + '%s  [Quelle %s, %s%s]' % (basisname(n), (f['ueber'] or ','.join(f['quelle'][:2])), last, zus))
            for m in sorted(kinder[n], key=lambda m: -fl[m]['gesamt']): baum(m, tiefe + 1)
        for n in sorted([n for n, f in fl.items() if not f['von']], key=lambda n: -fl[n]['gesamt']): baum(n, 0)
        out.append('Stromfluss (Quelle -> Verbraucher; A = Vorgabe oder Schaetzung aus dem Schaltplan, Spannung, Breite nach Strom):')
        out += ['  ' + z for z in zeilen]
    R = collections.Counter(t['rolle'] for t in K['teile'].values())
    out.append('Teile: ' + ', '.join('%s %d' % kv for kv in sorted(R.items())))
    for a, I in sorted(K['inseln'].items(), key=lambda kv: (-len(kv[1]['mitglieder']), kv[0])):
        if not I['mitglieder'] and I['rolle'] == 'stecker': continue
        ent = ' '.join('%s(%s@%s)' % (e['ref'], si(e['C']), basisname(e['netz'])) for e in I['entkopplung'])
        rest = [r for r in I['mitglieder'] if not K['teile'][r]['entkopplung']]
        out.append('Insel %-14s %-9s %2d Teile | Entkopplung: %s | weitere: %s' % (a, I['art'], len(I['mitglieder']), ent or '-', ' '.join(rest)))
    for n, ts in sorted((K.get('abgriffe') or {}).items()):
        out.append('Messabgriff an %-10s %s' % (basisname(n), ' '.join('%s(ab %s)' % (t['pad'], t['start'] or 'stromlos') for t in ts)))
    prog = sorted((r, t['programmier']) for r, t in K['teile'].items() if t.get('programmier'))
    if prog: out.append('Programmierteile (P25): ' + '; '.join('%s %s' % rt for rt in prog))
    dnp = sorted(r for r, t in K['teile'].items() if t.get('dnp'))
    if dnp: out.append('unbestueckt (Courtyard kein Hindernis): ' + ' '.join(dnp))
    frei = [r for r, t in K['teile'].items() if t['rolle'] == 'klein' and not t['insel']]
    if frei: out.append('ohne Insel: ' + ' '.join(sorted(frei)))
    nach = collections.defaultdict(list)
    for r, t in sorted(K['teile'].items()):
        if t['rolle'] == 'klein' and t.get('funktion'): nach[t['funktion']].append(r)
    if nach:
        out.append('Funktion der kleinen Teile (aus dem Zusammenhang im Netzgraphen; Teil -> gestuetzter Pin):')
        def kurz(r):
            t = K['teile'][r]; s = t.get('stuetzt') or {}
            ziel = '%s:%s' % (s['ref'], s['pin']) if s.get('ref') and s.get('pin') else ''
            if s.get('ueber_spule'): ziel += ' Ausgang %s' % s.get('regler')
            w = '%s ' % si(t['C']) if t.get('C') else ''
            return '%s(%s%s)' % (r, w, ziel) if (w or ziel) else r
        for f in list(FUNKTIONEN) + sorted(set(nach) - set(FUNKTIONEN)):
            if nach.get(f): out.append('  %-18s %3d  %s' % (f, len(nach[f]), ' '.join(kurz(r) for r in nach[f])))
        for f in ('rc_glied', 'sonstig'):
            for r in nach.get(f, []):
                if K['teile'][r].get('funktion_grund'): out.append('    %-10s %-9s %s' % (r, f, K['teile'][r]['funktion_grund']))
    return '\n'.join(out)


def si(c):
    for f, u in ((1e-6, 'u'), (1e-9, 'n'), (1e-12, 'p')):
        if c >= f: return ('%g' % round(c / f, 2)) + u
    return '%g' % c

if __name__ == '__main__':
    B, RG, VG, kw = lade(sys.argv)
    K = analysiere(B, RG, VG, impedanz=kw.get('impedanz') == '1', stoer_flanke=kw.get('stoer_flanke') == '1', paare=kw.get('paare', kw.get('paar')) == '1', sw_kompakt=kw.get('sw_kompakt') == '1')
    json.dump(K, open(sys.argv[2], 'w'), indent=1)
    txt = bericht(K)
    open(sys.argv[2].replace('.json', '.txt'), 'w').write(txt + '\n')
    print(txt)
