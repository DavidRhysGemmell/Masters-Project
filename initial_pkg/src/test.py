#! /usr/bin/env python3

for i in range(720): #essentially index to match angles with laser ranges
    theta= (i/2) *math.pi/180 
    if i==5: #scans for obstacles within 5 meters and the minimum laser range
        print(i)