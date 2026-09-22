#!/usr/bin/python3
import argparse, glob, os, signal, struct, subprocess, sys, time, zlib
from pathlib import Path

RUNTIME_GLOB = "/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F4*_CDC_in_FS_Mode*-if00"
BOOT_GLOB = "/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F4*_BOOT_CDC*-if00"
APP_BASE=0x08004000; APP_LIMIT=0x08060000; SRAM_END=0x20020000; CHUNK=240
EXPECTED_LAYOUT="AGVBL3-04000-60000"; EXPECTED_BOARD="F411CE01"
PROFILES={
    "f411ce": (0x08060000, "AGVBL3-04000-60000", "F411CE01", 0x20020000),
    "f411cc": (0x08020000, "AGVBL3-04000-20000", "F411CC01", 0x20020000),
    "f401cd": (0x08040000, "AGVBL3-04000-40000", "F401CD01", 0x20018000),
    "f103": (0x0803E000, "AGVBL3-04000-3E000", "F1030101", 0x20005000),
}

def normalize_image(path):
    data=Path(path).read_bytes()
    if len(data)>=16 and data[-8:-5]==b"UFD" and data[-5]==16: data=data[:-16]
    if len(data)<8 or len(data)>APP_LIMIT-APP_BASE: raise RuntimeError(f"invalid application size {len(data)}")
    sp,reset=struct.unpack_from('<II',data,0)
    if not (0x20000000<=sp<=SRAM_END) or (sp&3): raise RuntimeError(f"invalid MSP 0x{sp:08X}")
    if not (reset&1): raise RuntimeError(f"reset vector not Thumb 0x{reset:08X}")
    pc=reset&~1
    if not (APP_BASE<=pc<APP_BASE+len(data)): raise RuntimeError(f"reset vector outside image 0x{pc:08X}")
    return data

def find_one(pattern):
    a=sorted(glob.glob(pattern)); return a[0] if len(a)==1 else None

def wait_one(pattern,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        p=find_one(pattern)
        if p: return p
        time.sleep(.05)
    return None

def rom_dfu_active():
    return subprocess.run(['lsusb','-d','0483:df11'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0

def line_read(ser,timeout):
    end=time.monotonic()+timeout; b=bytearray()
    while time.monotonic()<end:
        x=ser.read(1)
        if not x: continue
        if x==b'\n': return bytes(b).strip().decode(errors='replace')
        if x!=b'\r' and len(b)<4096: b+=x
    return None

def transact(ser,command,prefixes,timeout=3.0):
    ser.write((command+'\n').encode()); ser.flush(); end=time.monotonic()+timeout
    while time.monotonic()<end:
        line=line_read(ser,min(.3,max(.02,end-time.monotonic())))
        if not line: continue
        if any(line.startswith(p) for p in prefixes): return line
        if line.startswith('ERR:'): raise RuntimeError(f"{command.split(':',1)[0]} rejected: {line}")
        print('[BOOT-CDC]',line)
    raise TimeoutError(f"timeout waiting for {command.split(':',1)[0]}")

def port_holders(path):
    real=os.path.realpath(path)
    cp=subprocess.run(['fuser',real],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
    return [int(x) for x in cp.stdout.split() if x.isdigit() and int(x)!=os.getpid()]

def release_port(path):
    pids=port_holders(path)
    if not pids: return
    print('[BOOT-CDC] releasing CDC holders',pids)
    for pid in pids:
        try: os.kill(pid,signal.SIGTERM)
        except ProcessLookupError: pass
    end=time.monotonic()+1.5
    while time.monotonic()<end and port_holders(path): time.sleep(.03)
    for pid in port_holders(path):
        try: os.kill(pid,signal.SIGKILL)
        except ProcessLookupError: pass

def open_serial_claim(path, timeout=5.0):
    import serial
    end=time.monotonic()+timeout
    last=None
    while time.monotonic()<end:
        release_port(path)
        try:
            return serial.Serial(path,1000000,timeout=.05,write_timeout=2,exclusive=True)
        except (OSError,serial.SerialException) as e:
            last=e
            time.sleep(.08)
    raise RuntimeError(f'cannot claim CDC port {path}: {last}')


_runtime_session_token = None

def runtime_resync(ser):
    # Terminate any partial line left by a previous host, then consume all
    # delayed runtime responses until the CDC stream has been quiet long enough
    # to establish a new command boundary.
    for _ in range(3):
        ser.write(b'\n')
        ser.flush()
        time.sleep(.04)
    quiet_since=time.monotonic()
    deadline=quiet_since+1.0
    while time.monotonic()<deadline:
        if ser.in_waiting:
            stale=ser.read(min(4096,ser.in_waiting))
            if stale:
                quiet_since=time.monotonic()
        elif time.monotonic()-quiet_since >= .20:
            break
        time.sleep(.01)
    ser.reset_input_buffer()

def ensure_runtime_session(ser):
    global _runtime_session_token
    runtime_resync(ser)
    if _runtime_session_token is None:
        token=((os.getpid()<<12) ^ int(time.monotonic()*1000) ^ 0xB00710AD) & 0xFFFFFFFF
        _runtime_session_token=token or 1
    token=_runtime_session_token
    command=f'HOST:HELLO:{token}'
    ser.write((command+'\n').encode())
    ser.flush()
    end=time.monotonic()+2.5
    expected=f'ACK:HOST:SESSION:{token}:'
    while time.monotonic()<end:
        line=line_read(ser,min(.3,max(.02,end-time.monotonic())))
        if not line:
            continue
        if line.startswith(expected):
            print('[BOOT-CDC]',line)
            break
        # Stale lines from the previous host/session are diagnostic noise here.
        print('[BOOT-CDC] stale runtime line:',line)
    else:
        raise TimeoutError('timeout establishing runtime host session')
    pong=transact(ser,'PING',['ACK:PONG'],1.5)
    print('[BOOT-CDC]',pong)

def trigger_resident(runtime):
    import serial
    last=None
    for attempt in range(1,6):
        release_port(runtime)
        try:
            with open_serial_claim(runtime,5.0) as s:
                time.sleep(.20)
                # Close any partial command left by a previous host session before
                # arming DFU. Re-open/retry the whole non-destructive negotiation
                # if the USB endpoint stalls before ACK:DFU:ARMED.
                s.write(b'\n\n'); s.flush(); time.sleep(.05); s.reset_input_buffer()
                ensure_runtime_session(s)
                r=transact(s,'BOOT:DFU:ARM',['ACK:DFU:ARMED'],2)
                print('[BOOT-CDC]',r)

                # From this point on do not blindly retry the destructive transition.
                # A detach/serial exception after CONFIRM means the bootloader handoff
                # may already be in progress, so return and let wait_one(BOOT_GLOB)
                # decide authoritatively.
                try:
                    s.write(b'BOOT:DFU:CONFIRM\n'); s.flush()
                except (OSError,serial.SerialException):
                    return

                end=time.monotonic()+1.5
                while time.monotonic()<end:
                    try:
                        line=line_read(s,.15)
                        if not line:
                            continue
                        if line.startswith('ACK:DFU'):
                            print('[BOOT-CDC]',line)
                            return
                        if line.startswith('ERR:'):
                            raise RuntimeError(line)
                    except (OSError,serial.SerialException):
                        return
                return
        except (OSError,serial.SerialException,TimeoutError) as e:
            last=e
            print(f'[BOOT-CDC] runtime transport retry {attempt}/5: {e}')
            time.sleep(.15)
    raise RuntimeError(f'runtime CDC negotiation failed after retries: {last}')

def fallback_rom(image_path):
    script=Path(__file__).resolve().parent/'dfu_upload_blackpill.sh'
    print('[BOOT-CDC] emergency ROM DFU already active; using USB ROM fallback')
    subprocess.run([str(script),str(image_path)],check=True)

def verify_runtime(port):
    import serial
    release_port(port)
    with open_serial_claim(port,5.0) as s:
        time.sleep(.15); s.reset_input_buffer()
        try:
            ensure_runtime_session(s)
            return True
        except Exception as e:
            print(f'[BOOT-CDC] runtime session verification failed: {e}')
            return False

def upload_boot(port,data):
    import serial
    crc=zlib.crc32(data)&0xffffffff
    reconnects=0
    max_reconnects=8
    off=0
    last_pct=-1

    def parse_state(line):
        fields={}
        for item in line.split(':')[2:]:
            if '=' not in item:
                continue
            key,value=item.split('=',1)
            fields[key]=value
        try:
            active=int(fields.get('active','0'),0)
            offset=int(fields.get('offset','0'),0)
            size=int(fields.get('size','0'),0)
            state_crc=int(fields.get('crc','0'),16)
        except Exception as e:
            raise RuntimeError(f'bad STATE response {line}: {e}')
        return active,offset,size,state_crc

    while True:
        current=find_one(BOOT_GLOB)
        if not current:
            current=wait_one(BOOT_GLOB,60)
        if not current:
            raise RuntimeError('resident boot CDC did not reappear during resumable upload')

        try:
            release_port(current)
            with open_serial_claim(current,8.0) as s:
                time.sleep(.2)
                s.reset_input_buffer()
                print('[BOOT-CDC]',transact(s,'PING',['BOOT:PONG'],2))
                info=transact(s,'INFO',['BOOT:INFO:'],2)
                print('[BOOT-CDC]',info)
                if ('proto=3' not in info or f'layout={EXPECTED_LAYOUT}' not in info or
                        f'board={EXPECTED_BOARD}' not in info):
                    raise RuntimeError(
                        f'incompatible resident bootloader target/layout; provision once via ST-Link: {info}')

                state=transact(s,'STATE',['BOOT:STATE:'],2)
                print('[BOOT-CDC]',state)
                active,state_off,state_size,state_crc=parse_state(state)
                if (active==1 and state_size==len(data) and state_crc==crc and
                        0 <= state_off <= len(data)):
                    off=state_off
                    print(f'[BOOT-CDC] resume accepted at offset={off}')
                else:
                    if active==1:
                        print('[BOOT-CDC] resident update state does not match image; restarting BEGIN safely')
                    r=transact(s,f'BEGIN:{len(data)}:{crc:08X}',['ACK:BEGIN:'],12)
                    print('[BOOT-CDC]',r)
                    off=0

                pct=(off*100)//len(data) if data else 100
                last_pct=(pct//5)-1

                while off<len(data):
                    chunk=data[off:off+CHUNK]
                    chunk_crc=zlib.crc32(chunk)&0xffffffff
                    command=f'DATA2:{off}:{chunk_crc:08X}:{chunk.hex().upper()}'
                    expected_next=off+len(chunk)
                    reply=None
                    last_error=None

                    for attempt in range(1,6):
                        try:
                            reply=transact(s,command,['ACK:DATA2:'],3)
                            break
                        except TimeoutError as e:
                            last_error=e
                            print(f'[BOOT-CDC] DATA2 retry {attempt}/5 offset={off}: timeout')
                            try:
                                s.write(b'\n'); s.flush(); time.sleep(.05)
                                while s.in_waiting:
                                    stale=line_read(s,.08)
                                    if stale:
                                        print(f'[BOOT-CDC] resync discard: {stale}')
                            except (OSError,serial.SerialException) as transport_error:
                                raise ConnectionError(str(transport_error))
                            time.sleep(.08)
                        except RuntimeError as e:
                            marker='ERR:DATA2:OFFSET:'
                            msg=str(e)
                            if marker in msg:
                                try:
                                    reported=int(msg.split(marker,1)[1].split()[0],0)
                                except Exception:
                                    reported=-1
                                if reported==expected_next:
                                    reply=f'ACK:DATA2:{reported}:{chunk_crc:08X}'
                                    print(f'[BOOT-CDC] recovered lost DATA2 ACK at offset={off}')
                                    break
                                if reported==off:
                                    last_error=e
                                    print(f'[BOOT-CDC] DATA2 retry {attempt}/5 offset={off}: boot still at same offset')
                                    time.sleep(.08)
                                    continue
                            if any(x in msg for x in (
                                    'ERR:DATA2:FORMAT','ERR:DATA2:CRC',
                                    'ERR:RX:OVERFLOW','ERR:LINE:TOO_LONG')) and attempt < 5:
                                last_error=e
                                print(f'[BOOT-CDC] DATA2 retry {attempt}/5 offset={off}: stale/partial line')
                                try:
                                    s.write(b'\n'); s.flush(); time.sleep(.05)
                                    while s.in_waiting:
                                        _=line_read(s,.05)
                                except (OSError,serial.SerialException) as transport_error:
                                    raise ConnectionError(str(transport_error))
                                continue
                            raise
                        except (OSError,serial.SerialException) as e:
                            raise ConnectionError(str(e))

                    if reply is None:
                        raise ConnectionError(
                            f'DATA2 retries exhausted at offset {off}: {last_error}')

                    try:
                        parts=reply.split(':')
                        nxt=int(parts[2],0)
                        echoed=int(parts[3],16)
                        if echoed != chunk_crc:
                            raise ValueError('chunk CRC echo mismatch')
                    except Exception:
                        raise RuntimeError(f'bad DATA2 ack {reply}')
                    if nxt!=expected_next:
                        raise RuntimeError(
                            f'offset mismatch host={expected_next} boot={nxt}')

                    off=nxt
                    pct=(off*100)//len(data)
                    if EXPECTED_BOARD == 'F1030101':
                        time.sleep(.002)
                    if pct//5!=last_pct//5:
                        last_pct=pct
                        print(f'[BOOT-CDC] write {pct}% ({off}/{len(data)})')

                try:
                    r=transact(s,'END',['ACK:END:OK'],8)
                    print('[BOOT-CDC]',r)
                    return
                except (OSError,serial.SerialException,TimeoutError) as e:
                    print(f'[BOOT-CDC] END transport changed ({e}); runtime verification is authoritative')
                    return

        except (ConnectionError, OSError, serial.SerialException, TimeoutError) as e:
            reconnects+=1
            if reconnects>max_reconnects:
                raise RuntimeError(
                    f'boot CDC transport lost too many times; last offset={off}: {e}')
            print(f'[BOOT-CDC] reconnect {reconnects}/{max_reconnects} after transport loss at offset={off}: {e}')
            time.sleep(.20)
            continue

def main():
    global APP_LIMIT, EXPECTED_LAYOUT, EXPECTED_BOARD, SRAM_END, RUNTIME_GLOB, BOOT_GLOB, CHUNK
    ap=argparse.ArgumentParser(description='AGV F411 resident USB bootloader uploader')
    ap.add_argument('image'); ap.add_argument('--boot-wait',type=float,default=25.0)
    ap.add_argument('--target',choices=sorted(PROFILES),default='f411ce')
    args=ap.parse_args()
    APP_LIMIT, EXPECTED_LAYOUT, EXPECTED_BOARD, SRAM_END = PROFILES[args.target]
    if args.target == 'f103':
        # Keep F103 CDC commands comfortably below the 600-byte boot parser
        # line buffer and reduce USB FS burst pressure on low-cost hubs/clones.
        CHUNK = 16
        RUNTIME_GLOB = '/dev/serial/by-id/usb-STMicroelectronics_BLUEPILL_F103_CDC_in_FS_Mode*-if00'
        BOOT_GLOB = '/dev/serial/by-id/usb-STMicroelectronics_BLUEPILL_F103_BOOT_CDC*-if00'
    data=normalize_image(args.image)
    print(f'[BOOT-CDC] image={len(data)} crc=0x{zlib.crc32(data)&0xffffffff:08X}')
    boot=find_one(BOOT_GLOB)
    if not boot:
        runtime=find_one(RUNTIME_GLOB)
        if runtime:
            trigger_resident(runtime); boot=wait_one(BOOT_GLOB,args.boot_wait)
        elif args.target != 'f103' and rom_dfu_active():
            fallback_rom(Path(args.image)); return 0
        else:
            print('[BOOT-CDC] no runtime/boot CDC; waiting for resident boot CDC or ROM DFU via USB')
            end=time.monotonic()+args.boot_wait
            while time.monotonic()<end and not boot:
                boot=find_one(BOOT_GLOB)
                if not boot and args.target != 'f103' and rom_dfu_active(): fallback_rom(Path(args.image)); return 0
                time.sleep(.1)
    if not boot: raise RuntimeError('resident boot CDC did not appear; USB/NRST/power path unavailable')
    print('[BOOT-CDC] resident port',boot); release_port(boot); upload_boot(boot,data)
    runtime=wait_one(RUNTIME_GLOB,45)
    if not runtime: raise RuntimeError('runtime CDC did not return after committed update')
    if not verify_runtime(runtime): raise RuntimeError('runtime CDC returned but ACK:PONG failed')
    print('[BOOT-CDC] SUCCESS committed image verified and runtime ACK:PONG healthy')
    return 0
if __name__=='__main__':
    try: raise SystemExit(main())
    except Exception as e:
        print('[BOOT-CDC] ERROR:',e,file=sys.stderr); raise SystemExit(1)
