#!/usr/bin/env python3
"""Offline regression for F411 upload/recovery routing and transactional order."""
from pathlib import Path
import tempfile, struct, subprocess, sys, zlib

ROOT = Path(__file__).resolve().parents[1]
PRE = ROOT / 'scripts/usb_dfu_upload.py'
DFU = ROOT / 'scripts/dfu_upload_blackpill.sh'
MANIFEST = ROOT / 'scripts/make_app_manifest.py'

class FakeEnv:
    def GetProjectOption(self, key, default=''):
        return default
    def AddPreAction(self, *_args, **_kwargs):
        pass

src = PRE.read_text(encoding='utf-8')
src = src.replace('Import("env")', 'env = FakeEnv()', 1)
ns = {'FakeEnv': FakeEnv, '__file__': str(PRE)}
exec(compile(src, str(PRE), 'exec'), ns)


def fail(msg):
    raise AssertionError(msg)


def run_route(name, nodes, port, cdc_ok, stlink_ok, stlink_dfu, manual_ok, expect):
    calls=[]
    # _before_upload asks _usb_nodes once at entry; later waits are mocked directly.
    ns['_acquire_upload_lock'] = lambda: calls.append('lock')
    ns['_prepare_dfu_tool'] = lambda: calls.append('prepare_tool')
    ns['_stop_ros_processes'] = lambda: calls.append('stop_ros')
    ns['_usb_nodes'] = lambda *_: list(nodes)
    ns['_assert_dfu_permission'] = lambda n: calls.append('permission')
    ns['_find_cdc_port'] = lambda: port
    ns['_cdc_to_dfu_with_recovery'] = lambda p: calls.append('cdc') or cdc_ok
    ns['_try_stlink_force_dfu'] = lambda: calls.append('stlink') or stlink_ok
    def wait(sec, manual_hint=False):
        calls.append('manual_wait' if manual_hint else 'dfu_wait')
        return manual_ok if manual_hint else stlink_dfu
    ns['_wait_for_dfu'] = wait
    try:
        ns['_before_upload'](None,None,None)
        outcome='ok'
    except RuntimeError:
        outcome='error'
    if outcome != expect:
        fail(f'{name}: outcome={outcome}, expected={expect}, calls={calls}')
    return calls

# A: Immutable ROM DFU is already active; no CDC/ST-Link branch may run.
c=run_route('already_dfu',['/dev/fake-dfu'],None,False,False,False,False,'ok')
if c != ['lock','prepare_tool','stop_ros','permission']: fail(f'already_dfu unexpected calls {c}')
# B: Runtime CDC transitions itself.
c=run_route('cdc','', '/dev/fake-cdc',True,False,False,False,'ok')
if 'cdc' not in c or 'stlink' in c: fail(f'cdc routing wrong {c}')
# C: Runtime absent; exact F411 ST-Link recovery reaches DFU.
c=run_route('stlink','',None,False,True,True,False,'ok')
if c.count('stlink') != 1 or 'manual_wait' in c: fail(f'stlink routing wrong {c}')
# D: Automatic recovery unavailable; manual BOOT0 path remains supported.
c=run_route('manual','',None,False,False,False,True,'ok')
if 'manual_wait' not in c: fail(f'manual routing missing {c}')
# E: Physical reset/USB impossible -> explicit hard failure, never false success.
run_route('unrecoverable','',None,False,False,False,False,'error')

# ST-Link safety guard: a non-F411 target must be rejected before OpenOCD write/reset.
ns['_probe_stlink_dbgmcu_id'] = lambda: 0x10000410  # STM32F103-class DEV_ID 0x410
if ns['_try_stlink_force_dfu'](): fail('non-F411 ST-Link target was accepted')

# Transaction order: validity metadata is invalidated first and committed LAST.
dfu = DFU.read_text(encoding='utf-8')
order = [
    'run_dfu invalidate-manifest', 'run_dfu write-app', 'run_dfu readback-app',
    'cmp "$RAW" "$READBACK"', 'run_dfu commit-manifest', 'run_dfu readback-manifest',
    'cmp "$MANIFEST" "$READBACK_MANIFEST"', 'leave_bootloader'
]
pos=[]
for token in order:
    i=dfu.find(token)
    if i < 0: fail(f'DFU transaction token missing: {token}')
    pos.append(i)
if pos != sorted(pos): fail(f'DFU transaction order invalid {list(zip(order,pos))}')
if '-s 0x08000000 -D' in dfu or '-s 0x08000000 -U' in dfu:
    fail('ROM-DFU app updater must never rewrite resident bootloader')
if '0x08000000:leave' not in dfu: fail('DFU leave must jump via resident bootloader')

# Manifest binary contract and CRCs.
with tempfile.TemporaryDirectory() as td:
    td=Path(td); app=td/'app.bin'; out=td/'manifest.bin'
    payload=bytearray((i*37+11)&0xff for i in range(4097)); struct.pack_into('<II', payload, 0, 0x2001FFF0, 0x08008021); payload=bytes(payload); app.write_bytes(payload)
    subprocess.run([sys.executable,str(MANIFEST),str(app),str(out)],check=True,capture_output=True,text=True)
    raw=out.read_bytes()
    if len(raw)!=32: fail(f'manifest length {len(raw)}')
    magic,fmt,base,size,app_crc,header_crc,generation,reserved=struct.unpack('<8I',raw)
    if magic!=0x31564741 or fmt!=1 or base!=0x08008000 or size!=len(payload): fail('manifest header fields invalid')
    if app_crc!=(zlib.crc32(payload)&0xffffffff): fail('manifest app CRC mismatch')
    if header_crc!=(zlib.crc32(raw[:20])&0xffffffff): fail('manifest header CRC mismatch')
    if reserved!=0: fail('manifest reserved field must be zero')

boot = (ROOT / 'bootloader/src/main.c').read_text(encoding='utf-8')
for token in ('APP_CRASH_MAGIC', 'APP_CRASH_LIMIT 3UL', 'BOOT_IDLE_TIMEOUT_MS 20000UL',
              'maintenance_loop', 'begin_update', 'program_chunk', 'commit_manifest',
              'erase_sector(FLASH_SECTOR_7)', 'FLASH_SECTOR_2', 'FLASH_SECTOR_6',
              'crc32_bytes((const uint8_t *)APP_BASE, expected_size)', 'application_valid()'):
    if token not in boot: fail(f'resident bootloader token missing: {token}')
# Transaction invariant: manifest sector must be invalidated before application sectors.
if boot.find('erase_sector(FLASH_SECTOR_7)') > boot.find('for (uint32_t s = FLASH_SECTOR_2'):
    fail('resident updater must invalidate manifest before erasing application')
# Manifest commit must only occur after full flash CRC verification.
if boot.find('crc32_bytes((const uint8_t *)APP_BASE, expected_size)') > boot.find('HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MANIFEST_ADDR'):
    fail('manifest commit occurs before full-image CRC verification')
# ROM DFU remains explicit emergency fallback, never the normal boot request path.
if 'if (!strcmp(line, "ROMDFU"))' not in boot:
    fail('explicit ROM DFU emergency command missing')
if 'if(request==BOOT_REQUEST_MAGIC) maintenance_loop(valid);' not in boot:
    fail('application boot request must enter resident USB maintenance, not ROM DFU')
app_src = (ROOT / 'src/main.cpp').read_text(encoding='utf-8')
for token in ('gAppWatchdogArmed = true', 'RTC->BKP1R = kAppCrashMagic',
              'RTC->BKP2R = 0U', 'kCrashCounterClearMs = 10000U'):
    if token not in app_src: fail(f'application watchdog recovery token missing: {token}')
cdc = (ROOT / 'scripts/cdc_boot_upload.py').read_text(encoding='utf-8')
for token in ('BEGIN:', 'DATA:', "transact(s,'END'", 'verify_runtime', 'BOOT_GLOB'):
    if token not in cdc: fail(f'resident host uploader token missing: {token}')

print('PASS F411_RECOVERY_STATE_MACHINE')
print('normal path: runtime CDC -> resident BOOT CDC -> transactional flash -> app heartbeat')
print('power-loss rule: manifest invalidated first; committed only after full CRC; resident bootloader never overwritten')
print('emergency path: immutable STM32 ROM DFU remains explicit USB fallback only')
