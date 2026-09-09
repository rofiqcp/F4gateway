#!/usr/bin/env python3
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

W,H=320,240
BG='#07151c'; PANEL='#0d222c'; PANEL2='#12313d'; CARD='#f4f0e4'; INK='#142026'; TEXT='#dcecf0'; MUTED='#72858d'; CYAN='#37c8e6'; GREEN='#39d98a'; RED='#f35b6f'; AMBER='#ffbf58'; BORDER='#27414b'
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'previews'; OUT.mkdir(exist_ok=True)
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'; BOLD='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
def f(n,b=False): return ImageFont.truetype(BOLD if b else FONT,n)
F8=f(8); F9=f(9); F10=f(10); F11=f(11); F12=f(12,1); F14=f(14,1); F18=f(18,1); F28=f(28,1)

def text(d,xy,s,font=F10,fill=TEXT,anchor='la'):
    d.text(xy,s,font=font,fill=fill,anchor=anchor)

def rr(d,box,r,fill,outline=None,w=1): d.rounded_rectangle(box,radius=r,fill=fill,outline=outline,width=w)

def top(d,title):
    d.rectangle((0,0,W,28),fill=BG); text(d,(10,14),'⌂',F14,CYAN,'lm'); text(d,(34,14),title,F11,TEXT,'lm')
    x=202
    for name,ok in [('GPS',1),('CAM',1),('ESC',1)]:
        d.ellipse((x,10,x+6,16),fill=GREEN if ok else RED); text(d,(x+9,14),name,F8,TEXT,'lm'); x+=39

def bottom(d,active):
    y=185; labels=[('CAMERA','CAMERA'),('GPS','GPS'),('ACTUATOR','ACTUATOR')]
    for i,(key,label) in enumerate(labels):
        x=4+i*105; rr(d,(x,y,x+101,235),7,PANEL2 if active==key else PANEL,BORDER)
        col=CYAN if active==key else TEXT
        icon={'CAMERA':'▣','GPS':'⌖','ACTUATOR':'◆'}[key]
        text(d,(x+50,y+13),icon,F14,col,'ma'); text(d,(x+50,y+35),label,F8,col,'ma')
        if active==key:d.rectangle((x+28,232,x+73,234),fill=CYAN)

def card(d,box=(6,34,314,180)): rr(d,box,8,CARD,'#d6d0c4')

def base(title,active):
    im=Image.new('RGB',(W,H),BG); d=ImageDraw.Draw(im); top(d,title); bottom(d,active); return im,d

def home():
    im,d=base('HOME','HOME'); rr(d,(6,34,234,180),8,CARD,'#d6d0c4'); rr(d,(240,34,314,104),8,CARD,'#d6d0c4'); rr(d,(240,110,314,180),8,CARD,'#d6d0c4')
    text(d,(20,48),'SPEED',F8,MUTED); text(d,(18,72),'12.4',F28,INK); text(d,(105,78),'km/h',F10,INK)
    text(d,(18,114),'VEHICLE READY',F12,GREEN); text(d,(18,145),'Heading 86°',F10,INK); text(d,(18,170),'Menuju: Titik A',F10,INK)
    rr(d,(145,54,218,114),10,'#d7edf1','#b4d7df'); text(d,(181,70),'ADV',F11,INK,'ma'); d.polygon([(158,91),(199,91),(210,108),(151,108)],fill='#53666d'); d.ellipse((158,104,170,116),fill=INK); d.ellipse((194,104,206,116),fill=INK)
    text(d,(277,48),'MODE',F8,MUTED,'ma'); text(d,(277,69),'AUTO',F14,CYAN,'ma'); text(d,(277,90),'TAP',F8,MUTED,'ma')
    text(d,(277,124),'STATE',F8,MUTED,'ma'); text(d,(277,146),'RUN',F14,GREEN,'ma'); text(d,(277,166),'NAV',F8,MUTED,'ma')
    return im

def camera(tab):
    im,d=base('CAMERA','CAMERA'); tabs=['VIEW','DETECT','DRIVE','STATUS']
    for i,t in enumerate(tabs):
        x=8+i*77; rr(d,(x,34,x+72,61),5,PANEL2 if t==tab else PANEL,CYAN if t==tab else BORDER); text(d,(x+36,48),t,F8,CYAN if t==tab else TEXT,'mm')
    rr(d,(6,65,314,180),8,CARD,'#d6d0c4')
    if tab=='VIEW':
        rr(d,(18,81,103,160),8,'#d7edf1','#b4d7df'); d.ellipse((42,102,78,138),outline=INK,width=4); d.rectangle((25,93,96,151),outline=INK,width=3); text(d,(118,82),'Camera Ready',F14,INK); text(d,(118,108),'Perception Active',F10,GREEN); d.line((118,126,296,126),fill='#c8c2b8'); text(d,(118,139),'LIVE TELEMETRY',F8,MUTED); text(d,(118,160),'8.1 FPS',F14,INK)
    elif tab=='DETECT':
        text(d,(20,80),'OBJECT',F8,MUTED); text(d,(20,105),'PERSON',F18,INK); text(d,(173,80),'DISTANCE',F8,MUTED); text(d,(173,105),'2.34 m',F18,INK); d.line((20,126,297,126),fill='#c8c2b8'); text(d,(20,146),'Confidence',F9,MUTED); text(d,(112,146),'92%',F12,INK); text(d,(173,146),'Obstacle',F9,MUTED); text(d,(248,146),'DETECTED',F10,RED,'mm')
    elif tab=='DRIVE':
        text(d,(20,80),'DRIVABLE AREA',F8,MUTED); text(d,(20,108),'CLEAR',F18,GREEN); text(d,(176,80),'OBSTACLE',F8,MUTED); text(d,(176,108),'NONE',F18,GREEN); d.line((20,128,297,128),fill='#c8c2b8'); text(d,(20,147),'Perception',F9,MUTED); text(d,(112,147),'ACTIVE',F12,GREEN); text(d,(20,168),'Path monitor normal',F9,INK)
    else:
        vals=[('Camera','READY',GREEN,20,82),('Perception','ACTIVE',GREEN,180,82),('Frame rate','8.1 FPS',INK,20,140),('GPS link','READY',GREEN,180,140)]
        for a,b,c,x,y in vals:text(d,(x,y),a,F8,MUTED);text(d,(x,y+25),b,F14,c)
    return im

def gps():
    im,d=base('GPS / NAV','GPS'); card(d)
    text(d,(22,47),'⌖',F18,CYAN); text(d,(49,51),'86°',F18,INK); text(d,(49,72),'HEADING',F8,MUTED)
    text(d,(118,43),'LAT',F8,MUTED); text(d,(149,43),'-7.050123',F9,INK); text(d,(118,63),'LON',F8,MUTED); text(d,(149,63),'110.440235',F9,INK)
    d.line((18,85,302,85),fill='#c8c2b8')
    metrics=[('SAT','17',18),('HDOP','0.82',66),('GNSS','3D FIX',123),('IMU','READY',190),('SPEED','1.2',248)]
    for a,b,x in metrics:text(d,(x,94),a,F8,MUTED); text(d,(x,114),b,F9,GREEN if b in ('3D FIX','READY') else INK)
    rr(d,(14,116,306,136),5,PANEL); text(d,(24,126),'Menuju: Titik A',F9,TEXT,'lm')
    buttons=[('<',12,34),('Titik A',49,74),('>',126,34),('SAVE',163,48),('GO',214,44),('STOP',261,47)]
    for label,x,w in buttons:
        col=RED if label=='STOP' else CYAN if label in ('Titik A','GO') else TEXT; rr(d,(x,140,x+w,174),5,PANEL2,col if label in ('Titik A','GO','STOP') else BORDER); text(d,(x+w/2,157),label,F8,col,'mm')
    return im

def actuator():
    im,d=base('ACTUATOR','ACTUATOR'); card(d)
    text(d,(18,46),'MANUAL CONTROL',F9,MUTED); text(d,(18,67),'Mode MANUAL • Tap command',F11,INK); text(d,(218,46),'RPM',F8,MUTED); text(d,(218,67),'328',F14,INK)
    btns=[('MAJU',72,79,52,36,CYAN),('KIRI',14,121,52,36,TEXT),('0°',72,121,52,36,CYAN),('KANAN',130,121,52,36,TEXT),('MUNDUR',188,121,58,36,TEXT),('STOP',252,121,56,36,RED)]
    for lab,x,y,w,h,col in btns:rr(d,(x,y,x+w,y+h),6,PANEL2,col);text(d,(x+w/2,y+h/2),lab,F8,col,'mm')
    text(d,(18,169),'Steer actual 0.4°   Target 0°   Speed 20%',F8,INK)
    return im

def save(name,im): im.save(OUT/name)

def main():
    pages=[('HOME.png',home()),('CAMERA_VIEW.png',camera('VIEW')),('CAMERA_DETECT.png',camera('DETECT')),('CAMERA_DRIVE.png',camera('DRIVE')),('CAMERA_STATUS.png',camera('STATUS')),('GPS_NAV.png',gps()),('ACTUATOR.png',actuator())]
    for n,im in pages:save(n,im)
    sheet=Image.new('RGB',(980,590),'#10191d'); d=ImageDraw.Draw(sheet)
    slots=[(10,35),(330,35),(650,35),(10,315),(330,315),(650,315),(650,315)]
    # first six in 3x2, actuator overlays final lower-right; GPS lower-middle.
    layout=[pages[0],pages[1],pages[2],pages[3],pages[4],pages[5]]
    labels=['HOME','CAMERA • VIEW','CAMERA • DETECT','CAMERA • DRIVE','CAMERA • STATUS','GPS / NAV']
    coords=[(10,35),(330,35),(650,35),(10,315),(330,315),(650,315)]
    for (n,im),lab,(x,y) in zip(layout,labels,coords):text(d,(x,y-18),lab,F11,TEXT);sheet.paste(im,(x,y))
    # separate actuator sheet to avoid shrinking all previews
    act=Image.new('RGB',(340,280),'#10191d'); ad=ImageDraw.Draw(act); text(ad,(10,17),'ACTUATOR',F11,TEXT);act.paste(pages[-1][1],(10,35));act.save(OUT/'ACTUATOR_PREVIEW_CARD.png')
    sheet.save(OUT/'HMI_REVISION_CONTACT_SHEET.png')
    print('Generated:',OUT)
    for p in sorted(OUT.glob('*.png')): print(p.name, Image.open(p).size)
if __name__=='__main__': main()
