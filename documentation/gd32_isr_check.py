#!/usr/bin/env python3
"""Static check for VESC GD32 builds (GD32F405_COMPAT).

Walks the call graph from all interrupt vectors and reports every reachable
function that is located outside the fast 512 KB GD32 code area
(0x08000000-0x0807FFFF), and every literal reference from reachable code to
read-only data in the slow area.

Indirect calls are approximated conservatively:
  * a function whose address is loaded by reachable code (literal pool) is
    treated as called by it (callbacks passed to drivers, function tables),
  * every function whose address is stored in .data (driver configs such as
    ICUConfig/EXTConfig, initialized function pointers) is an extra root,
  * extra root names can be given on the command line (functions that are
    assigned to ISR-called function pointers at run time, e.g. read_hall_lbm
    for utils_read_hall_fun).

Usage: gd32_isr_check.py <elf> <toolchain-prefix> [extra_root ...] [-not_a_root ...]
  -name: drop a .data function pointer root that is known not to run in
         interrupt context (e.g. -target_printf for the Blackmagic code).
Exit code 1 if anything ISR-reachable is in the slow area.
"""
import subprocess, re, sys, struct, collections, bisect

elf = sys.argv[1]; T = sys.argv[2]
FAST_END = 0x08080000
SLOW_END = 0x08100000

nm = subprocess.run([T+'nm', '-S', '-n', elf], capture_output=True, text=True).stdout
funcs = {}; objs = {}
for l in nm.splitlines():
    p = l.split()
    if len(p) == 4 and p[2] in 'tTwW':
        funcs[int(p[0], 16)] = (p[3], int(p[1], 16))
    elif len(p) == 4 and p[2] in 'rRdD':
        objs[int(p[0], 16)] = (p[3], int(p[1], 16))
faddrs = sorted(funcs); oaddrs = sorted(objs)

def owner(a, tab=funcs, keys=faddrs):
    i = bisect.bisect_right(keys, a) - 1
    if i >= 0:
        s = keys[i]; n, sz = tab[s]
        if s <= a < s + max(sz, 2):
            return s
    return None

def section(name):
    subprocess.run([T+'objcopy', '-O', 'binary', '-j', name, elf, '/tmp/gd32chk.bin'])
    return open('/tmp/gd32chk.bin', 'rb').read()

hdr = subprocess.run([T+'objdump', '-h', elf], capture_output=True, text=True).stdout
secs = {}
for l in hdr.splitlines():
    m = re.match(r'\s*\d+\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)', l)
    if m:
        secs[m.group(1)] = (int(m.group(3), 16), int(m.group(2), 16))  # vma, size

dis = subprocess.run([T+'objdump', '-d', '--no-show-raw-insn', elf],
                     capture_output=True, text=True).stdout
calls = collections.defaultdict(set)      # direct calls/branches
lits = collections.defaultdict(set)       # literal values loaded
mem = {}
for name in ('.text', '.text_slow', '.text2', 'startup', '.rodata'):
    if name in secs:
        vma, size = secs[name]
        mem[name] = (vma, section(name))

def read32(a):
    for vma, b in mem.values():
        if vma <= a < vma + len(b) - 3:
            return struct.unpack_from('<I', b, a - vma)[0]
    return None

cur = None
for l in dis.splitlines():
    m = re.match(r'^([0-9a-f]{8}) <(.+)>:$', l)
    if m:
        cur = int(m.group(1), 16); continue
    if cur is None:
        continue
    m = re.match(r'^\s*([0-9a-f]+):\s+(b|bl|b\.w|b\S*|cb\S*)\s+([0-9a-f]+)\s+<', l)
    if m:
        o = owner(int(m.group(3), 16)); src = owner(int(m.group(1), 16))
        if o is not None and src is not None and o != src:
            calls[src].add(o)
        continue
    m = re.match(r'^\s*([0-9a-f]+):\s+ldr(?:\.w)?\s+\S+,\s+\[pc,\s+#\d+\]\s+@\s+\(([0-9a-f]+)', l)
    if m:
        src = owner(int(m.group(1), 16)); v = read32(int(m.group(2), 16))
        if src is not None and v is not None:
            lits[src].add(v)

# Roots: vectors, .data function pointers, command line
roots = set()
vec = section('startup')
for i in range(1, len(vec) // 4):
    o = owner(struct.unpack_from('<I', vec, i * 4)[0] & ~1)
    if o is not None and funcs[o][0] != 'Reset_Handler':
        roots.add(o)
data_roots = set()
if '.data' in secs:
    d = section('.data')
    for i in range(0, len(d) - 3, 4):
        v = struct.unpack_from('<I', d, i)[0]
        if v & 1 and owner(v & ~1) == (v & ~1):
            data_roots.add(v & ~1)
names = [a for a in sys.argv[3:] if not a.startswith('-')]
ignore = {a[1:] for a in sys.argv[3:] if a.startswith('-')}
extra = {a for a, (n, sz) in funcs.items() if n in names}
data_roots = {a for a in data_roots if funcs[a][0] not in ignore}
print("extra roots:", sorted(funcs[a][0] for a in extra))
print(".data function pointers used as roots:", len(data_roots))
roots |= data_roots | extra

seen = set(); stack = list(roots); parent = {}
while stack:
    f = stack.pop()
    if f in seen:
        continue
    seen.add(f)
    nxt = set(calls[f])
    for v in lits[f]:
        if v & 1 and owner(v & ~1) == (v & ~1):
            nxt.add(v & ~1)       # function address taken -> may be called
    for c in nxt:
        if c not in seen:
            parent.setdefault(c, f); stack.append(c)

def chain(f):
    c = []; x = f
    while x in parent and len(c) < 5:
        x = parent[x]; c.append(funcs[x][0])
    return " <- ".join(c)

for n in ("mcpwm_foc_adc_int_handler", "mc_interface_mc_timer_isr",
          "mcpwm_foc_tim_sample_int_handler"):
    print(n, "reached" if any(funcs[f][0] == n for f in seen) else "NOT reached")

slow = sorted(f for f in seen if FAST_END <= f < SLOW_END)
print(f"roots: {len(roots)}, reachable functions: {len(seen)}, slow functions: {len(slow)}")
for f in slow:
    print(f"  SLOW 0x{f:08x} {funcs[f][0]}  <- {chain(f)}")

slowdata = []
for f in sorted(seen):
    for v in lits[f]:
        if FAST_END <= v < SLOW_END and owner(v & ~1) is None:
            o = owner(v, objs, oaddrs)
            slowdata.append((funcs[f][0], v, objs[o][0] if o else '?'))
print(f"literal refs to slow-area data from reachable code: {len(slowdata)}")
for fn, v, on in slowdata:
    print(f"  DATA {fn}: 0x{v:08x} ({on})")
sys.exit(1 if slow or slowdata else 0)
