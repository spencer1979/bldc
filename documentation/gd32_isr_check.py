#!/usr/bin/env python3
"""Static check: walk direct calls/branches from all interrupt vectors of a VESC
GD32 build and report any reachable function located outside the fast 512 KB
GD32 code area (0x08000000-0x0807FFFF). Indirect calls (function pointers) are
not followed."""
import subprocess, re, sys, struct, collections
elf = sys.argv[1]; T = sys.argv[2]
FAST_END = 0x08080000
nm = subprocess.run([T+'nm','-S','-n',elf],capture_output=True,text=True).stdout
funcs = {}
for l in nm.splitlines():
    p=l.split()
    if len(p)==4 and p[2] in 'tTwW':
        funcs[int(p[0],16)] = (p[3], int(p[1],16))
addrs = sorted(funcs)
import bisect
def owner(a):
    i = bisect.bisect_right(addrs, a)-1
    if i>=0:
        s=addrs[i]; n,sz=funcs[s]
        if s<=a<s+max(sz,2): return s
    return None
dis = subprocess.run([T+'objdump','-d','--no-show-raw-insn',elf],capture_output=True,text=True).stdout
calls = collections.defaultdict(set)
cur=None
for l in dis.splitlines():
    m=re.match(r'^([0-9a-f]{8}) <(.+)>:$',l)
    if m: cur=int(m.group(1),16); continue
    m=re.match(r'^\s*([0-9a-f]+):\s+(b|bl|b\.w|b\S*|cb\S*)\s+([0-9a-f]+)\s+<',l)
    if m and cur is not None:
        t=int(m.group(3),16); o=owner(t)
        src=owner(int(m.group(1),16))
        if o is not None and src is not None and o!=src: calls[src].add(o)
# vector table
data = subprocess.run([T+'objcopy','-O','binary','-j','startup',elf,'/tmp/vec.bin'])
vec=open('/tmp/vec.bin','rb').read()
roots=set()
for i in range(1,len(vec)//4):
    v=struct.unpack_from('<I',vec,i*4)[0] & ~1
    o=owner(v)
    if o is not None and funcs[o][0] not in ('Reset_Handler',): roots.add(o)
EXTRA_NAMES=set(sys.argv[3:])
extra=[a for a,(n,sz) in funcs.items() if n in EXTRA_NAMES]
print("extra roots:", sorted(funcs[a][0] for a in extra))
roots |= set(extra)
seen=set(); stack=list(roots); parent={}
while stack:
    f=stack.pop()
    if f in seen: continue
    seen.add(f)
    for c in calls[f]:
        if c not in seen: parent.setdefault(c,f); stack.append(c)
slow=[f for f in seen if f>=FAST_END]
for n in ("mcpwm_foc_adc_int_handler","mc_interface_mc_timer_isr","mcpwm_foc_tim_sample_int_handler"):
    print(n, "reached" if any(funcs[f][0]==n for f in seen) else "NOT reached")
print(f"ISR roots: {len(roots)}, reachable functions: {len(seen)}, outside fast area: {len(slow)}")
for f in sorted(slow):
    chain=[funcs[f][0]]; x=f
    while x in parent: x=parent[x]; chain.append(funcs[x][0])
    print(f"  0x{f:08x} {funcs[f][0]}  <- " + " <- ".join(chain[1:6]))
