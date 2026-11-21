"""Deterministic synthetic images; expected corners are independent of detector."""
import cv2
import numpy as np


def scene(width=300, height=220, center=(320,240), cell=9, border=30, solid=False, canvas=(480,640)):
    frame=np.full((*canvas,3),155,np.uint8)
    cx,cy=center
    x,y=int(cx-width/2),int(cy-height/2)
    x0,y0,x1,y1=x-border,y-border,x+width+border,y+height+border
    yy,xx=np.indices(canvas)
    pattern=(((xx-x0)//cell+(yy-y0)//cell)%2)*200+25
    ring=(xx>=x0)&(xx<=x1)&(yy>=y0)&(yy<=y1)
    opening=(xx>x)&(xx<x+width)&(yy>y)&(yy<y+height)
    ring &= ~opening
    frame[ring]=np.repeat((np.full(canvas,25) if solid else pattern)[:,:,None],3,axis=2)[ring]
    q=np.float32([[x,y],[x+width,y],[x+width,y+height],[x,y+height]])
    return frame,q
