"""Fixed body-offset compensation, independent of USB detection. Pure math."""
import math

def compensation(length=.07,pixels=(53.25,15.484375)):
    # Pixel right/down = body right/back. Vehicle moves the opposite direction.
    n=math.hypot(*pixels)
    return (length*pixels[1]/n,length*pixels[0]/n)

def shift(target,heading,forward_left):
    f,l=forward_left;co,si=math.cos(heading),math.sin(heading)
    return (target[0]+co*f-si*l,target[1]+si*f+co*l,target[2])
