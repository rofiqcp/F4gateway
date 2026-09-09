#!/usr/bin/env python3
import argparse, glob, os, signal, struct, subprocess, sys, time, zlib
from pathlib import Path

RUNTIME_GLOB = "/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F411CE_CDC_in_FS_Mode*-if00"
BOOT_GLOB = "/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F411CE_BOOT_CDC*-if00"
APP_BASE=0x08008000; APP_LIMIT=0x08060000; CHUNK=240

def normalize_image(path):
    data=Path(path).read_bytes()
    if len(data)>=16 and data[-8:-5]==b"UFD" and data[-5]==16: data=data[:-16]
    if len(data)<8 or len(data)>APP_LIMIT-APP_BASE: raise RuntimeError(f"invalid application size {len(data)}")
    sp,reset=struct.unpack_from('<II',data,0)
    if not (0x20000000<=sp<=0x20020000) or (sp&3): raise RuntimeError(f"invalid MSP 0x{sp:08X}")
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
    end=time.monotonic()+3
    while time.monotonic()<end and port_holders(path): time.sleep(.05)
    for pid in port_holders(path):
        try: os.kill(pid,signal.SIGKILL)
        except ProcessLookupError: pass

def trigger_resident(runtime):
    import serial
    release_port(runtime)
    with serial.Serial(runtime,1000000,timeout=.04,write_timeout=1,exclusive=True) as s:
        time.sleep(.15); s.reset_input_buffer()
        r=transact(s,'BOOT:DFU:ARM',['ACK:DFU:ARMED'],2)
        print('[BOOT-CDC]',r)
        s.write(b'BOOT:DFU:CONFIRM\n'); s.flush()
        # Runtime may disappear before ACK reaches host; either ACK or detach is success.
        end=time.monotonic()+1.5; seen=''
        while time.monotonic()<end:
            try:
                line=line_read(s,.15)
                if line:
                    seen=line
                    if line.startswith('ACK:DFU'): break
                    if line.startswith('ERR:'): raise RuntimeError(line)
            except (OSError,serial.SerialException): break
        if seen: print('[BOOT-CDC]',seen)

def fallback_rom(image_path):
    script=Path(__file__).resolve().parent/'dfu_upload_blackpill.sh'
    print('[BOOT-CDC] emergency ROM DFU already active; using USB ROM fallback')
    subprocess.run([str(script),str(image_path)],check=True)

def verify_runtime(port):
    import serial
    release_port(port)
    with serial.Serial(port,1000000,timeout=.05,write_timeout=1,exclusive=True) as s:
        time.sleep(.15); s.reset_input_buffer()
        for _ in range(5):
            s.write(b'PING\n'); s.flush(); end=time.monotonic()+.8; d=b''
            while time.monotonic()<end:
                if s.in_waiting:
                    d+=s.read(min(2048,s.in_waiting))
                    if b'ACK:PONG' in d: return True
                time.sleep(.01)
    return False

def upload_boot(port,data):
    import serial
    crc=zlib.crc32(data)&0xffffffff
    with serial.Serial(port,1000000,timeout=.05,write_timeout=2,exclusive=True) as s:
        time.sleep(.2); s.reset_input_buffer()
        print('[BOOT-CDC]',transact(s,'PING',['BOOT:PONG'],2))
        print('[BOOT-CDC]',transact(s,'INFO',['BOOT:INFO:'],2))
        r=transact(s,f'BEGIN:{len(data)}:{crc:08X}',['ACK:BEGIN:'],12)
        print('[BOOT-CDC]',r)
        off=0; last_pct=-1
        while off<len(data):
            chunk=data[off:off+CHUNK]
            r=transact(s,f'DATA:{off}:{chunk.hex().upper()}',['ACK:DATA:'],3)
            try: nxt=int(r.rsplit(':',1)[1],0)
            except Exception: raise RuntimeError(f'bad DATA ack {r}')
            if nxt!=off+len(chunk): raise RuntimeError(f'offset mismatch host={off+len(chunk)} boot={nxt}')
            off=nxt; pct=(off*100)//len(data)
            if pct//5!=last_pct//5:
                last_pct=pct; print(f'[BOOT-CDC] write {pct}% ({off}/{len(data)})')
        try:
            r=transact(s,'END',['ACK:END:OK'],8)
            print('[BOOT-CDC]',r)
        except (OSError,TimeoutError):
            # Reset may detach USB immediately after ACK; runtime verification below is authoritative.
            print('[BOOT-CDC] END caused USB reset; verifying runtime')

def main():
    ap=argparse.ArgumentParser(description='AGV F411 resident USB bootloader uploader')
    ap.add_argument('image'); ap.add_argument('--boot-wait',type=float,default=25.0)
    args=ap.parse_args(); data=normalize_image(args.image)
    print(f'[BOOT-CDC] image={len(data)} crc=0x{zlib.crc32(data)&0xffffffff:08X}')
    boot=find_one(BOOT_GLOB)
    if not boot:
        runtime=find_one(RUNTIME_GLOB)
        if runtime:
            trigger_resident(runtime); boot=wait_one(BOOT_GLOB,args.boot_wait)
        elif rom_dfu_active():
            fallback_rom(Path(args.image)); return 0
        else:
            print('[BOOT-CDC] no runtime/boot CDC; waiting for resident boot CDC or ROM DFU via USB')
            end=time.monotonic()+args.boot_wait
            while time.monotonic()<end and not boot:
                boot=find_one(BOOT_GLOB)
                if not boot and rom_dfu_active(): fallback_rom(Path(args.image)); return 0
                time.sleep(.1)
    if not boot: raise RuntimeError('resident boot CDC did not appear; USB/NRST/power path unavailable')
    print('[BOOT-CDC] resident port',boot); release_port(boot); upload_boot(boot,data)
    runtime=wait_one(RUNTIME_GLOB,15)
    if not runtime: raise RuntimeError('runtime CDC did not return after committed update')
    if not verify_runtime(runtime): raise RuntimeError('runtime CDC returned but ACK:PONG failed')
    print('[BOOT-CDC] SUCCESS committed image verified and runtime ACK:PONG healthy')
    return 0
if __name__=='__main__':
    try: raise SystemExit(main())
    except Exception as e:
        print('[BOOT-CDC] ERROR:',e,file=sys.stderr); raise SystemExit(1)
