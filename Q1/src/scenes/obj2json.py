#/usr/bin/python3
#
# for now, this only handles vertices and faces (no normals or materials)
# usage: ./$0 scale rot_x rot_y rot_z trans_x trans_y trans_z < file.obj

import sys, re, math
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("scale", nargs='?', default="1")
parser.add_argument("rot_x", nargs='?', default="0")
parser.add_argument("rot_y", nargs='?', default="0")
parser.add_argument("rot_z", nargs='?', default="0")
parser.add_argument("trans_x", nargs='?', default="0")
parser.add_argument("trans_y", nargs='?', default="0")
parser.add_argument("trans_z", nargs='?', default="0")
args = parser.parse_args()

vs = []
fs = []
pattern = re.compile(r'(\S*)\s*(\S*)\s*(\S*)\s*(\S*)')

for line in sys.stdin:
    line = line.strip().lower()
    m = pattern.match(line)
    if m != None:
        values = m.groups()
        if values[0] == 'v':
            vs.append((float(values[1]), float(values[2]), float(values[3])))
        elif values[0] == 'f':
            fs.append((int(re.sub(r'/.*', '', values[1]))-1, int(re.sub(r'/.*', '', values[2]))-1, int(re.sub(r'/.*', '', values[3]))-1))

centre = [0,0,0]
for v in vs:
    centre = [sum(x) for x in zip(centre, v)]
centre = [x / float(len(vs)) for x in centre]

for i in range(0,len(vs)):
    vs[i] = [(x[0] - x[1]/2) for x in zip(vs[i], centre)]

extent = vs[0]
for v in vs:
    extent = [max(abs(x[0]), x[1]) for x in zip(v, extent)]

size = max(extent)
scale = float(args.scale)
rot_x = float(args.rot_x)
rot_y = float(args.rot_y)
rot_z = float(args.rot_z)
trans_x = float(args.trans_x)
trans_y = float(args.trans_y)
trans_z = float(args.trans_z)
su = math.sin(rot_x);
cu = math.cos(rot_x);
sv = math.sin(rot_y);
cv = math.cos(rot_y);
sw = math.sin(rot_z);
cw = math.cos(rot_z);
for i in range(0,len(vs)):
    v = [x / size * scale for x in vs[i]]
    vs[i] = (
    cv*cw * v[0] + (su*sv*cw - cu*sw) * v[1] + (su*sw + cu*sv*cw) * v[2] + trans_x,
    cv*sw * v[0] + (cu*cw + su*sv*sw) * v[1] + (cu*sv*sw - su*cw) * v[2] + trans_y,
    -sv * v[0] + su*cv * v[1] + (cu*cv) * v[2] + trans_z
    )

for i in range(0,len(fs)):
    f = fs[i]
    print(f'[ [{vs[f[0]][0]},{vs[f[0]][1]},{vs[f[0]][2]}], [{vs[f[1]][0]},{vs[f[1]][1]},{vs[f[1]][2]}], [{vs[f[2]][0]},{vs[f[2]][1]},{vs[f[2]][2]}] ]', end='')
    if i < len(fs) - 1:
        print(',')
    else:
        print()
