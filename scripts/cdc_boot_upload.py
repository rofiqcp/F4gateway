#!/usr/bin/python3
import argparse, fcntl, glob, os, signal, struct, subprocess, sys, time, zlib
from pathlib import Path

RUNTIME_GLOB = "/dev/serial/by-id/usb-STMicroelectronics_BLUEPILL_F103_CDC_in_FS_Mode*-if00"
BOOT_GLOB = "/dev/serial/by-id/usb-STMicroelectronics_BLUEPILL_F103_BOOT_CDC*-if00"
APP_BASE=0x08002000; APP_LIMIT=0x0800F7F0; SRAM_END=0x20005000; CHUNK=16
EXPECTED_LAYOUT="AGVBL4-C8-02000-F7F0"; EXPECTED_BOARD="F103C801"
UPDATE_LOCK = Path(f'/tmp/f4gateway_firmware_update.{os.getuid()}.lock')
CDC_OWNER_LOCK = Path('/tmp/f4gateway_cdc_owner.lock')
_UPDATE_LOCK_FD = None
_CDC_LOCK_FD = None

PROFILES={
    "f103c8": (0x0800F7F0, "AGVBL4-C8-02000-F7F0", "F103C801", 0x20005000),
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
    patterns=[pattern]
    if pattern == RUNTIME_GLOB:
        patterns.append('/dev/f4gateway')
    elif pattern == BOOT_GLOB:
        patterns.append('/dev/f4gateway-boot')
    by_real={}
    for pat in patterns:
        for path in sorted(glob.glob(pat)):
            if os.path.exists(path):
                by_real.setdefault(os.path.realpath(path), path)
    return next(iter(by_real.values())) if len(by_real)==1 else None

def wait_one(pattern,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        p=find_one(pattern)
        if p: return p
        time.sleep(.05)
    return None

def wait_runtime_or_boot(seconds):
    """Catch either CDC identity so a short runtime enumeration is not missed."""
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        boot=find_one(BOOT_GLOB)
        if boot:
            return None,boot
        runtime=find_one(RUNTIME_GLOB)
        if runtime:
            return runtime,None
        time.sleep(.05)
    return None,None

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

def acquire_update_lock():
    global _UPDATE_LOCK_FD
    fd=os.open(UPDATE_LOCK, os.O_CREAT|os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd)
        raise RuntimeError('another STM32F103C8 firmware update is already active')
    os.ftruncate(fd,0)
    os.write(fd,(str(os.getpid())+'\n').encode())
    os.fsync(fd)
    _UPDATE_LOCK_FD=fd
    print('[BOOT-CDC] firmware-update lock acquired')

def release_update_lock():
    global _UPDATE_LOCK_FD
    if _UPDATE_LOCK_FD is None:
        return
    try:
        # Keep the lock inode persistent. Unlinking a flock file before/after
        # unlock creates an inode race where a second updater can lock a new file.
        fcntl.flock(_UPDATE_LOCK_FD, fcntl.LOCK_UN)
    finally:
        os.close(_UPDATE_LOCK_FD)
        _UPDATE_LOCK_FD=None

def acquire_cdc_owner_lock(timeout=3.0):
    global _CDC_LOCK_FD
    fd=os.open(CDC_OWNER_LOCK, os.O_CREAT|os.O_RDWR, 0o644)
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX|fcntl.LOCK_NB)
            _CDC_LOCK_FD=fd
            print('[BOOT-CDC] CDC ownership lock acquired')
            return
        except BlockingIOError:
            runtime=find_one(RUNTIME_GLOB)
            if runtime:
                release_port(runtime)
            time.sleep(.05)
    os.close(fd)
    raise RuntimeError('CDC ownership lock is busy; another authorized owner is active')

def release_cdc_owner_lock():
    global _CDC_LOCK_FD
    if _CDC_LOCK_FD is None:
        return
    try:
        fcntl.flock(_CDC_LOCK_FD, fcntl.LOCK_UN)
    finally:
        os.close(_CDC_LOCK_FD)
        _CDC_LOCK_FD=None

def _cmdline(pid):
    try:
        return Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
    except OSError:
        return ''

def _owned_cdc_holder(pid):
    cmd=_cmdline(pid)
    gateway='/home/otomasi2/forclift/install/f4gateway/lib/f4gateway/f4gateway_node'
    return cmd == gateway or cmd.startswith(gateway+' ')

def release_port(path):
    deadline=time.monotonic()+1.0
    pids=port_holders(path)
    while pids and time.monotonic()<deadline:
        time.sleep(.04)
        pids=port_holders(path)
    if not pids:
        return

    unknown=[pid for pid in pids if not _owned_cdc_holder(pid)]
    if unknown:
        detail='; '.join(f'PID {pid}: {_cmdline(pid)}' for pid in unknown)
        raise RuntimeError(f'CDC port is owned by an unrelated process; refusing to terminate it: {detail}')

    print('[BOOT-CDC] requesting graceful shutdown of F4 gateway holder(s)',pids)
    for pid in pids:
        try:
            os.kill(pid,signal.SIGTERM)
        except ProcessLookupError:
            pass
    end=time.monotonic()+1.5
    while time.monotonic()<end and port_holders(path):
        time.sleep(.03)
    remaining=port_holders(path)
    if remaining:
        detail='; '.join(f'PID {pid}: {_cmdline(pid)}' for pid in remaining)
        raise RuntimeError(f'F4 gateway did not release CDC after SIGTERM; refusing SIGKILL: {detail}')

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
    release_port(runtime)
    with open_serial_claim(runtime,5.0) as s:
        time.sleep(.15)
        runtime_resync(s)
        # ARM/CONFIRM are intentionally accepted without a HOST session. Do not
        # require a runtime TX reply here: after another host closes CDC, the
        # application RX path can still be healthy while its old IN transfer is
        # stalled. The appearance of BOOT_CDC is the authoritative handoff ACK.
        s.write(b'BOOT:DFU:ARM\n')
        s.flush()
        time.sleep(.05)
        s.write(b'BOOT:DFU:CONFIRM\n')
        s.flush()
        end=time.monotonic()+.75
        while time.monotonic()<end:
            try:
                line=line_read(s,.10)
            except (OSError,serial.SerialException):
                return
            if not line:
                continue
            if line.startswith('ACK:DFU'):
                print('[BOOT-CDC]',line)
            elif line.startswith('ERR:DFU:'):
                raise RuntimeError(line)
            else:
                print('[BOOT-CDC] runtime:',line)

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

def boot_resync(ser):
    # A reconnect can deliver a late DATA2 error/ACK from the previous USB
    # transport instance. Establish a quiet command boundary before PING so
    # stale protocol lines cannot abort a recoverable update.
    try:
        for _ in range(2):
            ser.write(b'\n')
            ser.flush()
            time.sleep(.03)
    except Exception:
        pass
    quiet_since=time.monotonic()
    deadline=quiet_since+.8
    while time.monotonic()<deadline:
        if ser.in_waiting:
            stale=line_read(ser,.08)
            if stale:
                print('[BOOT-CDC] boot resync discard:',stale)
                quiet_since=time.monotonic()
        elif time.monotonic()-quiet_since >= .15:
            break
        time.sleep(.01)
    ser.reset_input_buffer()

def upload_boot(port,data):
    import serial
    crc=zlib.crc32(data)&0xffffffff
    reconnects=0
    max_reconnects=8
    off=0
    last_pct=-1

    while True:
        current=find_one(BOOT_GLOB)
        if not current:
            current=wait_one(BOOT_GLOB,60)
        if not current:
            raise RuntimeError('resident boot CDC did not reappear during upload')

        try:
            release_port(current)
            with open_serial_claim(current,8.0) as s:
                time.sleep(.2)
                boot_resync(s)
                print('[BOOT-CDC]',transact(s,'PING',['BOOT:PONG'],2))
                info=transact(s,'INFO',['BOOT:INFO:'],2)
                print('[BOOT-CDC]',info)
                if ('proto=4' not in info or f'layout={EXPECTED_LAYOUT}' not in info or
                        f'board={EXPECTED_BOARD}' not in info):
                    raise RuntimeError(
                        f'incompatible resident bootloader target/layout; provision once via ST-Link: {info}')
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
                    if EXPECTED_BOARD == 'F103C801':
                        # Keep F1 USB FS/PMA and weak hubs below saturation.
                        time.sleep(.015)
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
            print(f'[BOOT-CDC] reconnect {reconnects}/{max_reconnects}; restart from BEGIN after transport loss at offset={off}: {e}')
            time.sleep(.20)
            continue

def main():
    global APP_LIMIT, EXPECTED_LAYOUT, EXPECTED_BOARD, SRAM_END
    ap=argparse.ArgumentParser(description='STM32F103C8 resident USB bootloader uploader')
    ap.add_argument('image')
    ap.add_argument('--boot-wait',type=float,default=25.0)
    ap.add_argument('--target',choices=sorted(PROFILES),default='f103c8')
    args=ap.parse_args()

    acquire_update_lock()
    acquire_cdc_owner_lock()
    APP_LIMIT, EXPECTED_LAYOUT, EXPECTED_BOARD, SRAM_END = PROFILES[args.target]
    data=normalize_image(args.image)
    print(f'[BOOT-CDC] image={len(data)} crc=0x{zlib.crc32(data)&0xffffffff:08X}')

    boot=find_one(BOOT_GLOB)
    if not boot:
        boot=wait_one(BOOT_GLOB,4.0)

    if not boot:
        runtime=find_one(RUNTIME_GLOB)
        if runtime:
            trigger_resident(runtime)
            boot=wait_one(BOOT_GLOB,args.boot_wait)
        else:
            print('[BOOT-CDC] no runtime/boot CDC; waiting for either identity')
            runtime,boot=wait_runtime_or_boot(args.boot_wait)
            if runtime:
                print('[BOOT-CDC] runtime CDC appeared; requesting resident bootloader')
                trigger_resident(runtime)
                boot=wait_one(BOOT_GLOB,args.boot_wait)

    if not boot:
        runtime_after_handoff=find_one(RUNTIME_GLOB)
        if runtime_after_handoff:
            raise RuntimeError(
                'F103C8 runtime CDC is present but resident boot CDC never appeared. '
                'Provision once with: pio run -e f103_stlink -t upload')
        raise RuntimeError(
            'resident boot CDC did not appear and runtime CDC is absent; '
            'check USB enumeration, NRST, power, and hub path')

    print('[BOOT-CDC] resident port',boot)
    release_port(boot)
    upload_boot(boot,data)

    runtime=wait_one(RUNTIME_GLOB,45)
    if not runtime:
        raise RuntimeError('runtime CDC did not return after committed update')
    if not verify_runtime(runtime):
        raise RuntimeError('runtime CDC returned but ACK:PONG failed')
    print('[BOOT-CDC] SUCCESS committed image verified and runtime ACK:PONG healthy')
    return 0

if __name__=='__main__':
    try:
        raise SystemExit(main())
    except Exception as e:
        print('[BOOT-CDC] ERROR:',e,file=sys.stderr)
        raise SystemExit(1)
    finally:
        release_cdc_owner_lock()
        release_update_lock()
