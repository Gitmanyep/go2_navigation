"""Render exported traces into an offline GIF; no simulation or robot connections."""
from pathlib import Path
import argparse
import json
import math
import numpy as np
from PIL import Image,ImageDraw,ImageFont


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run');p.add_argument('--output',required=True)
    a=p.parse_args();run=Path(a.run)
    scenarios=['flat_detour','gentle_slope','cloud_dropout']
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',13)
    data=[]
    for name in scenarios:
        rows=[json.loads(line) for line in (run/name/'trace.jsonl').read_text().splitlines()]
        archive=np.load(run/name/'maps.npz');maps={k:archive[k] for k in archive.files};archive.close();data.append((name,rows,maps))
    frames=[]
    for t in np.linspace(0,65,100):
        im=Image.new('RGB',(1140,390),'#f4f7fb');d=ImageDraw.Draw(im)
        d.text((18,10),'Go2 navigation simulation | synthetic terrain | no hardware output',fill='#223144',font=font)
        for panel,(name,rows,maps) in enumerate(data):
            left=panel*380
            def pixel(xy):return (left+35+(xy[0]+1.5)*42,215-xy[1]*42)
            d.text((left+18,44),name,fill='#223144',font=font)
            z=maps['world_z'];origin=maps['world_origin'];res=float(maps['resolution'])
            for iy,ix in np.ndindex(z.shape):
                if maps['world_obstacle'][iy,ix]:color='#c74d55'
                elif not maps['world_ground'][iy,ix]:color='#303744'
                else:
                    q=int(np.clip(232-z[iy,ix]*85,135,240));color=(q,235,215)
                x,y=pixel(origin+[ix*res,iy*res]);d.rectangle((x,y-res*42,x+res*42,y),fill=color)
            past=[r for r in rows if r['t']<=t]
            row=past[-1] if past else rows[0]
            if len(past)>1:d.line([pixel([r['x'],r['y']]) for r in past],fill='#236bd1',width=3)
            x,y=pixel([row['x'],row['y']]);r=.45*42
            d.ellipse((x-r,y-r,x+r,y+r),outline='#236bd1',width=2)
            d.line((x,y,x+r*math.cos(row['yaw']),y-r*math.sin(row['yaw'])),fill='#236bd1',width=2)
            for point,color in [([0,0],'#1a9a69'),([4.2,0],'#8b3dc2')]:
                px,py=pixel(point);d.ellipse((px-4,py-4,px+4,py+4),fill=color)
            if name=='cloud_dropout' and 5<=t<8:d.text((left+30,75),'CLOUD DROPOUT 5-8 s',fill='#b53141',font=small)
            d.text((left+18,360),f"t={min(t,rows[-1]['t']):4.1f}s   v={row['command'][0]:.3f} m/s",fill='#223144',font=small)
        frames.append(im)
    frames[0].save(a.output,save_all=True,append_images=frames[1:],duration=90,loop=0,optimize=True)

if __name__=='__main__':main()
