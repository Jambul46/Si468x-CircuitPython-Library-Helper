# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Jambul46hsd
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Usage:
    import board, busio, digitalio
    spi = busio.SPI(clock=board.GP2, MOSI=board.GP3, MISO=board.GP4) #SPI0 on Pi Pico
    cs = digitalio.DigitalInOut(board.GP5)
    rst = digitalio.DigitalInOut(board.GP6)
    radio = Si468x(spi, cs, rst)

    radio.boot_fm() or radio.boot_dab()
    radio.fm_tune_freq(123456) or radio.fm_seek_start()

"""

import time
import struct
import busio
import digitalio
from adafruit_bus_device.spi_device import SPIDevice
from Si468x.Variables import *


class Si468xError(RuntimeError):
    pass


class Si468x:
    def __init__(
        self,
        spi: busio.SPI,
        cs: digitalio.DigitalInOut,
        rstb: digitalio.DigitalInOut,
        *,
        rst_active_high=True,
        int_active_low=True,
        spi_frequency=8000000,
    ):
        self.spi = spi
        self.rstb = rstb
        self.rst_active_high = rst_active_high
        self.int_active_low = int_active_low

        self.rstb.direction = digitalio.Direction.OUTPUT
        self.rstb.value = not self.rst_active_high

        self.device = SPIDevice(
            self.spi, cs, baudrate=spi_frequency, polarity=0, phase=0
        )

    def _check_rst_held(self):
        if self.rst_active_high and not self.rstb.value:
            raise Si468xError("RSTB must be held active")
        if not self.rst_active_high and self.rstb.value:
            raise Si468xError("RSTB must be held active (low)")

    def reset(self, hold_time=0.01, post_release_delay=0.05):
        self.rstb.value = not self.rst_active_high
        time.sleep(hold_time)
        self.rstb.value = self.rst_active_high
        time.sleep(post_release_delay)

    def _spi_write(self, data: bytes):
        with self.device as spi:
            spi.write(data)

    def _spi_read(self, write: bytes, readlen: int):
        buf = bytearray(readlen)
        with self.device as spi:
            spi.write(write)
            spi.readinto(buf)
        return buf

    def _cmd_write(self, cmd: int, args: bytes = b""):
        self._check_rst_held()
        self._spi_write(bytes([cmd]) + args)

    def _read_reply(self, expected_len=4, timeout=DEFAULT_CTS_TIMEOUT, debug=False):
        start = time.monotonic()
        total = max(4, expected_len) 

        while True:
            data = self._spi_read(bytes([RD_REPLY]), total)

            if not data or len(data) < 4:
                time.sleep(0.001)
                if time.monotonic() - start > timeout:
                    raise Si468xError("CTS timeout or no response")
                continue

            status0 = data[0]

            if debug:
                print("Raw data:", [hex(b) for b in data])

                resp0 = data[0]
                print(
                    "RESP0: CTS={}, ERR_CMD={}, DACQINT={}, DSRVINT={}, RSQINT={}, RDSINT={}, ACFINT={}, STCINT={}".format(
                        bool(resp0 & 0x80),
                        bool(resp0 & 0x40),
                        bool(resp0 & 0x20),
                        bool(resp0 & 0x10),
                        bool(resp0 & 0x08),
                        bool(resp0 & 0x04),
                        bool(resp0 & 0x02),
                        bool(resp0 & 0x01),
                    )
                )

                resp1 = data[1]
                print(
                    "RESP1: DEVNTINT={}, DACFINT={}".format(
                        bool(resp1 & 0x20), bool(resp1 & 0x01)
                    )
                )

                print("RESP2: (no status bits)")

                resp3 = data[3]
                print(
                    "RESP3: PUPSTATE={}, RFFE_ERR={}, DSPERR={}, REPOFERR={}, CMDOFERR={}, ARBERR={}, ERRNR={}".format(
                        (resp3 & 0xC0) >> 6,
                        bool(resp3 & 0x20),
                        bool(resp3 & 0x10),
                        bool(resp3 & 0x08),
                        bool(resp3 & 0x04),
                        bool(resp3 & 0x02),
                        bool(resp3 & 0x01),
                    )
                )

            if (status0 & 0x80) != 0:
                return data

            if time.monotonic() - start > timeout:
                raise Si468xError("CTS timeout")

            time.sleep(0.001)

    def send_command(
        self, cmd: int, args: bytes = b"", resp_len=0, timeout=DEFAULT_CTS_TIMEOUT
    ):
        self._cmd_write(cmd, args)
        return self._read_reply(expected_len=resp_len, timeout=timeout)

    def power_up(self, xtal_freq=19200000, clk_mode=1, ctsien=0):
        arg1 = 0x80 if ctsien else 0x00
        arg2 = (clk_mode << 4) | 7
        arg3 = 0x48
        xtal = struct.pack('<I', xtal_freq)
        arg8 = 0x1F
        arg9 = 0x10
        arg13 = 0x00
        null = 0x00
        args = bytearray([arg1, arg2, arg3] + list(xtal) + [arg8, arg9, null, null, null, arg13, null, null])
        return self.send_command(POWER_UP, args)

    def load_init(self):
        return self.send_command(LOAD_INIT, b'\x00')

    def host_load(self, image):
        
        def send_chunk(chunk: bytes):
            pkt = bytes([HOST_LOAD, 0, 0, 0]) + chunk
            self._spi_write(pkt)
            time.sleep(0.005)
            self._read_reply(0)
            
        if isinstance(image, str):
            with open(image, "rb") as f:
                while True:
                    chunk = f.read(HOST_LOAD_PAYLOAD)
                    if not chunk:
                        break
                    if len(chunk) < HOST_LOAD_PAYLOAD:
                        pad = (-len(chunk)) % 4096
                        if pad:
                            chunk += bytes(pad)
                    send_chunk(chunk)
        else:
            for idx in range(0, len(image), HOST_LOAD_PAYLOAD):
                chunk = image[idx : idx + HOST_LOAD_PAYLOAD]
                if idx + HOST_LOAD_PAYLOAD >= len(image):
                    pad = (-len(chunk)) % 4096
                    if pad:
                        chunk += bytes(pad)
                send_chunk(chunk)
                
        return True

    def flash_load(self, address):
        flashaddr = struct.pack('<I', address)
        null = 0x00
        args = bytes([null, null, null] + list(flashaddr) + [0x00, 0x00, 0x00, 0x00])
        resp = self.send_command(FLASH_LOAD, args, 4)
        if (resp[0] & 0x80) !=0:
            print("Image loaded from FLASH, start at", address)
        else:
            print("CTS Failed")

    def boot(self):
        self.send_command(BOOT, b'\x00')
        print("FW Booted")

    def boot_fm(self, host = True):
        self.reset()
        self.power_up()
        self.load_init()
        if host:
            self.host_load("fw/patch.bin")
        else:
            self.flash_load()     
        self.load_init()
        if host:
            self.host_load("fw/fm.bin")
        else:
            self.flash_load()
        self.boot()
        self.set_property(FM_AUDIO_DE_EMPHASIS, 1)
        self.set_property(FM_RDS_CONFIG, 1)
        print(self.get_sys_state())

    def boot_dab(self, host = True):
        self.reset()
        self.power_up()
        self.load_init()
        if host:
            self.host_load("fw/patch.bin")
        else:
            self.flash_load()     
        self.load_init()
        if host:
            self.host_load("fw/dab.bin")
        else:
            self.flash_load()
        self.boot()
        self.set_property(DAB_TUNE_FE_CFG, 1)
        print(self.get_sys_state())

    def get_part_info(self):
        resp = self.send_command(GET_PART_INFO, b'\x00', 23)
        chiprev = resp[4]
        romid = resp[5]
        part = resp[8] | (resp[9] << 8)
        return {"chiprev": chiprev, "romid": romid, "part": part}

    def get_sys_state(self):
        resp = self.send_command(GET_SYS_STATE, b'\x00', 6)
        return {"sys_state": resp[4]}

    def set_property(self, propid: int, value: int):
        args = bytes([0x00]) + struct.pack('<H', propid) + struct.pack('<H', value)
        return self.send_command(SET_PROPERTY, args)

    def get_property(self, propid: int, count: int = 1):
        args = bytes([count]) + struct.pack('<H', propid)
        resp = self.send_command(GET_PROPERTY, args, 6)
        return {'Data': (resp[4] | (resp[5] << 8))}

    def set_audio_output(self, output: int, master: int = 0):
        if output == 0:
            self.set_property(PIN_CONFIG_ENABLE, (0x80 << 8) | 0x01)
        elif output == 1:
            self.set_property(PIN_CONFIG_ENABLE, (0x80 << 8) | 0x00)
        else:
            print("Invalid value: use 0 for iDAC, 1 for I2S")
            return
            
        if output == 1:
            if master == 0:
                self.set_property(DIGITAL_IO_OUTPUT_SELECT, (0x00 << 8) | 0x00)
            elif master == 1:
                self.set_property(DIGITAL_IO_OUTPUT_SELECT, (0x80 << 8) | 0x00)
            else:
                print("Invalid value: use 0 for Slave, 1 for Master")
                return
                       
        self.fm_seek_start() #Any change of this property will take effect on next tune. BUG! fm_tune_freq doesn't seem to work for this purpose.
    
    def set_volume(self, value: int):
        if 0 <= value <= 63:
            self.set_property(AUDIO_ANALOG_VOLUME, value)
        else:
            print("Volume must be between 0 and 63")

    def mute(self, value: int):
        if 0 <= value <= 3:
            self.set_property(AUDIO_MUTE, value)
        else:
            print("0 Unmute, 1 Mute Left, 2 Mute Right, 3 Mute Both")

    def fm_tune_freq(self, freq_khz: int = 87500, tune_mode = 2):
        freq10 = int(freq_khz / 10)
        freq = struct.pack("<H", freq10)
        mode = (tune_mode & 0x03) << 2
        args = bytes([mode] + list(freq) + [0x00, 0x00, 0x00])
        return self.send_command(FM_TUNE_FREQ, args)

    def fm_seek_start(self, direc=1):
        if 0 <= direc <= 1:
            args = (
                bytes([0x08])
                + bytes([0x01 | ((direc & 1) << 1)])
                + bytes([0x00, 0x00, 0x00])
            )
            return self.send_command(FM_SEEK_START, args)
        else:
            print("1 Seek Up, 0 Seek Down")

    def fm_rsq_status(self):
        resp = self.send_command(FM_RSQ_STATUS, b'\x00', 22)

        return {
            "hdlevel_hint": bool(resp[4] & 0x20),
            "hdlevel_lint": bool(resp[4] & 0x10),
            "snr_hint": bool(resp[4] & 0x08),
            "snr_lint": bool(resp[4] & 0x04),
            "rssi_hint": bool(resp[4] & 0x02),
            "rssi_lint": bool(resp[4] & 0x01),
            "hd_detected": bool(resp[5] & 0x40),
            "flt_hd_detected": bool(resp[5] & 0x20),
            "afcrl": bool(resp[5] & 0x04),
            "valid": bool(resp[5] & 0x02),
            "readfreq": (resp[6] | (resp[7] << 8)) * 10,
            "freq_offset": resp[8],
            "rssi": resp[9],
            "snr": resp[10],
            "mult": resp[11],
            "readantcap": struct.unpack("<H", bytes([resp[12], resp[13]]))[0],
            "hdlevel": resp[15],
            "filtered_hdlevel": resp[16],
        }

    def fm_acf_status(self, value = 0):
        if 0 <= value <= 1:
            resp = self.send_command(FM_ACF_STATUS, bytes([value]), 11)
            return {
                "blend_int": bool((resp[4] & 0x04) >> 2),
                "highcut_int": bool((resp[4] & 0x02) >> 1),
                "smute_int": bool((resp[4] & 0x01)),
                "blend_conv": (resp[5] & 0x40) >> 6,
                "highcut_conv": (resp[5] & 0x20) >> 5,
                "smute_conv": (resp[5] & 0x10) >> 4,
                "blend_state": (resp[5] & 0x04) >> 2,
                "highcut_state": (resp[5] & 0x02) >> 1,
                "smute_state": (resp[5] & 0x01),
                "attn": (resp[6] & 0x1F),
                "highcut": (resp[7]),
                "pilot": (resp[8] & 0x80) >> 7,
                "stblend": (resp[8] & 0x7F) >> 1,
                }
        else:
            print("Must be 1 (clear) or 0 (keep)")

    def fm_rds_status(self, debug=False, timeout=20.0, min_ps_repeats=3, min_rt_repeats=3):
        
        def safe_char(val):
            return chr(val) if 32 <= val <= 126 else " "

        start_time = time.monotonic()
        ps_buf = [" "] * 8
        rt_buf = [" "] * 64
        ps_counts = {}
        rt_counts = {}
        ps_name = ""
        radiotext = ""
        text_ab_flag = None

        blocks = {"A": 0, "B": 0, "C": 0, "D": 0}
        ble = {"A": 0, "B": 0, "C": 0, "D": 0}
        pi = 0
        tp = False
        pty = 0
        rdsfifo_used = 0
        status = {}

        while True:
            resp = self.send_command(FM_RDS_STATUS, b'\x04', 20)

            status = {
                "rds_tppty_int": bool(resp[4] & 0x10),
                "rds_pi_int":    bool(resp[4] & 0x08),
                "rdssync_int":   bool(resp[4] & 0x02),
                "rdsfifo_int":   bool(resp[4] & 0x01),
                "tppty_valid":   bool(resp[5] & 0x10),
                "pi_valid":      bool(resp[5] & 0x08),
                "rdssync":       bool(resp[5] & 0x02),
                "rdsfifo_lost":  bool(resp[5] & 0x01),
            }

            tp   = bool(resp[6] & 0x20)
            pty  = resp[6] & 0x1F
            pi   = resp[8] | (resp[9] << 8)
            rdsfifo_used = resp[10]

            ble = {
                "A": (resp[11] >> 6) & 0x03,
                "B": (resp[11] >> 4) & 0x03,
                "C": (resp[11] >> 2) & 0x03,
                "D": (resp[11] >> 0) & 0x03,
            }

            for _ in range(rdsfifo_used):
                resp = self.send_command(FM_RDS_STATUS, b'\x00', 20)

                block_a = resp[12] | (resp[13] << 8)
                block_b = resp[14] | (resp[15] << 8)
                block_c = resp[16] | (resp[17] << 8)
                block_d = resp[18] | (resp[19] << 8)
                blocks.update({"A": block_a, "B": block_b, "C": block_c, "D": block_d})

                ble_a = (resp[11] >> 6) & 0x03
                ble_b = (resp[11] >> 4) & 0x03
                ble_c = (resp[11] >> 2) & 0x03
                ble_d = (resp[11] >> 0) & 0x03

                if 3 in (ble_a, ble_b, ble_c, ble_d):
                    continue

                group_type = (block_b >> 12) & 0x0F
                version    = (block_b >> 11) & 0x01

                if group_type == 2:
                    new_ab = bool(block_b & 0x0010)
                    if text_ab_flag is None:
                        text_ab_flag = new_ab
                    elif new_ab != text_ab_flag:
                        rt_buf = [" "] * 64
                        rt_counts = {}
                        text_ab_flag = new_ab

                if group_type == 0:
                    seg = block_b & 0x03
                    if 0 <= seg * 2 < 8:
                        ps_buf[seg*2]   = safe_char((block_d >> 8) & 0xFF)
                        ps_buf[seg*2+1] = safe_char(block_d & 0xFF)

                if group_type == 2 and version == 0:
                    seg = block_b & 0x0F
                    if 0 <= seg * 4 < 64:
                        rt_buf[seg*4]   = safe_char((block_c >> 8) & 0xFF)
                        rt_buf[seg*4+1] = safe_char(block_c & 0xFF)
                        rt_buf[seg*4+2] = safe_char((block_d >> 8) & 0xFF)
                        rt_buf[seg*4+3] = safe_char(block_d & 0xFF)

                if group_type == 2 and version == 1:
                    seg = block_b & 0x0F
                    if 0 <= seg * 2 < 32:
                        rt_buf[seg*2]   = safe_char((block_d >> 8) & 0xFF)
                        rt_buf[seg*2+1] = safe_char(block_d & 0xFF)

            cur_ps = "".join(ps_buf).strip()
            if cur_ps:
                ps_counts[cur_ps] = ps_counts.get(cur_ps, 0) + 1
                if ps_counts[cur_ps] >= min_ps_repeats:
                    ps_name = cur_ps

            cur_rt = "".join(rt_buf).strip()
            if cur_rt:
                rt_counts[cur_rt] = rt_counts.get(cur_rt, 0) + 1
                if rt_counts[cur_rt] >= min_rt_repeats:
                    radiotext = cur_rt

            if time.monotonic() - start_time > timeout:
                break

        return {
            "status": status,
            "PS": ps_name,
            "RadioText": radiotext,
            "PI": pi,
            "TP": tp,
            "PTY": pty,
            "blocks": blocks,
            "BLE": ble,
            "FIFO_used": rdsfifo_used,
        }

    def fm_rds_blockcount(self, value = 0):
        if 0 <= value <= 1:
            resp = self.send_command(FM_RDS_BLOCKCOUNT, bytes([value]), 10)
            return {
                "expected": (resp[4] | resp[5] << 8),
                "received": (resp[6] | resp[7] << 8),
                "uncorrectable": (resp[8] | resp[9] << 8),
                }
        else:
            print("Must be 1 (clear) or 0 (keep)")            

    def decode_prog_field(self, byte):
        return {
            "MPS": bool(byte & 0x01),
            "SPS1": bool(byte & 0x02),
            "SPS2": bool(byte & 0x04),
            "SPS3": bool(byte & 0x08),
            "SPS4": bool(byte & 0x10),
            "SPS5": bool(byte & 0x20),
            "SPS6": bool(byte & 0x40),
            "SPS7": bool(byte & 0x80),
        }

    def hd_digrad_status(self, value = 0):
        if 0 <= value <= 1:
            resp = self.send_command(FM_RDS_BLOCKCOUNT, bytes([value]), 23)
            tx_gain_raw = resp[8] & 0x1F
            
            if tx_gain_raw & 0x10:
                tx_gain = tx_gain_raw - 0x20
            else:
                tx_gain = tx_gain_raw
            
            return {
                "hdlogoint": bool(resp[4] & 0x80),
                "srcanaint": bool(resp[4] & 0x40),
                "srcdigint": bool(resp[4] & 0x20),
                "audacqint": bool(resp[4] & 0x08),
                "acqint": bool(resp[4] & 0x04),
                "cdnrhint": bool(resp[4] & 0x02),
                "cdnrlint": bool(resp[4] & 0x01),
                "hdlogo": (resp[5] & 0x80),
                "srcana": (resp[5] & 0x40),
                "srcdig": (resp[5] & 0x20),
                "audacq": (resp[5] & 0x08),
                "acq": (resp[5] & 0x04),
                "cdnrh": (resp[5] & 0x02),
                "cdnrl": (resp[5] & 0x01),
                "bctl": (resp[6] & 0xC0),
                "daai": (resp[6] & 0x3F),
                "cdnr": resp[7],
                "tx_gain": tx_gain,
                "audio_prog_avail": self.decode_prog_field(resp[9]),
                "audio_prog_playing": self.decode_prog_field(resp[10]),
                "audio_ca": self.decode_prog_field(resp[11]),
                "core_audio_err": struct.unpack("<I", resp[12:16])[0],
                "enh_audio_err": struct.unpack("<I", resp[16:20])[0],
                "pty": resp[20],
                "ps_mode": resp[21],
                "codec_mode": resp[22],
                }
        else:
            print("Must be 1 (clear) or 0 (keep)")

    def hd_get_event_status(self, value = 0):
        if 0 <= value <= 1:
            resp = self.send_command(HD_GET_EVENT_STATUS, bytes([value]), 18)

    def hd_get_station_info(self, info_select: int = 1):
        resp = self.send_command(HD_GET_STATION_INFO, bytes([info_select]), 7)
        length = resp[4] | (resp[5] << 8)
        data = resp[6:6+length]
        return data
    
    def hd_get_psd_decode(self, program: int = 0, field: int = 0):
        program &= 0x0F
        field &= 0x0F

        args = bytes([program, field])
    
        resp = self.send_command(HD_GET_PSD_DECODE, args)

        datatype = resp[6] if len(resp) > 6 else 0
        length   = resp[7] if len(resp) > 7 else 0
        data     = resp[8:8+length] if len(resp) > 8 else []

        text = ""
        if data:
            if datatype == 0:  # 8-bit ISO/IEC 8859-1
                text = bytes(data).decode('latin1', errors='ignore')
            elif datatype == 1:  # 16-bit ISO/IEC 10646-1
                text = ''.join(
                    chr((data[i] | (data[i+1] << 8)))
                    for i in range(0, len(data)-1, 2)
                )

        return text
    
    def hd_get_alert_msg(self):
        resp = self.send_command(HD_GET_ALERT_MSG, b'\x00', 7)
        if len(resp) < 6:
            return b""
        
        length = resp[4] | (resp[5] << 8)
        data = resp[6:6+length] if len(resp) >= 6 + length else resp[6:]
        
        return data
    
    def hd_play_alert_tone(self):
        return self.send_command(HD_PLAY_ALERT_TONE, b'\x00', 4)


    def hd_test_get_ber_info(self):
        resp = self.send_command(HD_TEST_GET_BER_INFO, b'\x00', 44)
        return {
            "pids_block_errors": struct.unpack("<I", bytes(resp[4:8]))[0],
            "pids_blocks_tested": struct.unpack("<I", bytes(resp[8:12]))[0],
            "pids_bit_errors": struct.unpack("<I", bytes(resp[12:16]))[0],
            "pids_bits_tested": struct.unpack("<I", bytes(resp[16:20]))[0],
            "p3_bit_errors": struct.unpack("<I", bytes(resp[20:24]))[0],
            "p3_bits_tested": struct.unpack("<I", bytes(resp[24:28]))[0],
            "p2_bit_errors": struct.unpack("<I", bytes(resp[28:32]))[0],
            "p2_bits_tested": struct.unpack("<I", bytes(resp[32:36]))[0],
            "p1_bit_errors": struct.unpack("<I", bytes(resp[36:40]))[0],
            "p1_bits_tested": struct.unpack("<I", bytes(resp[40:44]))[0],
        }


    def hd_set_enabled_ports(self, length, data):
        length = len(ports)
        if length > 64:
            raise ValueError("Maximum 64 ports allowed")
        args = bytes([length]) + b"".join(struct.pack("<H", p) for p in ports)        
        return self.send_command(HD_TEST_GET_BER_INFO, args, 4)
    
    def hd_get_enabled_ports(self):
        resp = self.send_command(HD_TEST_GET_ENABLED_PORTS, b'\x00', resp_len=4 + 2*64)
        length = resp[4]
        ports = [resp[6 + i*2] | (resp[7 + i*2] << 8) for i in range(length)]
        return ports

    
    def hd_acf_status(self, value: int = 0):
        if 0 <= value <= 1:
            resp = self.send_command(HD_ACF_STATUS, bytes([value]), 8)
        return {
            "comf_noise_int": bool(resp[4] & 0x01),
            "comf_noise_conv": (resp[5] & 0x10) >> 4,
            "comf_noise_state": resp[5] & 0x01,
            "comf_noise_level": resp[6] | (resp[7] << 8)
            }

    def test_get_rssi(self):
        resp = self.send_command(TEST_GET_RSSI, b'\x00', 6)
        rssi_bytes = bytes([resp[4], resp[5]])
        raw = struct.unpack('<h', rssi_bytes)[0]
        rssi_dbuv = raw / 256.0
        return {"rssi_raw": raw, "rssi_dbuv": rssi_dbuv}

