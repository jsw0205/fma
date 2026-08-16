import csv
import math
import re

LOG_FILE = "pygpsdata-20260703165206.log"
CSV_FILE = "nmea_parsed.csv"
SVG_FILE = "nmea_scatter.svg"

gga_re = re.compile(r"\$GNGGA,([^,]*),([^,]*),([NS]),([^,]*),([EW]),(\d),")

def nmea_to_deg(value, hemi, is_lon=False):
    if not value:
        return None

    deg_len = 3 if is_lon else 2
    deg = int(value[:deg_len])
    minute = float(value[deg_len:])
    out = deg + minute / 60.0

    if hemi in ("S", "W"):
        out = -out

    return out

points = []

with open(LOG_FILE, "r", encoding="utf-8", errors="ignore") as f:
    for line in f:
        m = gga_re.search(line)
        if not m:
            continue

        t, lat_raw, ns, lon_raw, ew, quality = m.groups()

        lat = nmea_to_deg(lat_raw, ns, is_lon=False)
        lon = nmea_to_deg(lon_raw, ew, is_lon=True)

        if lat is None or lon is None:
            continue

        points.append((t, lat, lon, int(quality)))

if not points:
    raise RuntimeError("GGA 좌표를 못 찾음")

lat0 = sum(p[1] for p in points) / len(points)
lon0 = sum(p[2] for p in points) / len(points)

m_per_deg_lat = 111320.0
m_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))

rows = []
for i, (t, lat, lon, quality) in enumerate(points):
    east_cm = (lon - lon0) * m_per_deg_lon * 100.0
    north_cm = (lat - lat0) * m_per_deg_lat * 100.0
    rows.append((i, t, lat, lon, east_cm, north_cm, quality))

with open(CSV_FILE, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["index", "time", "lat", "lon", "east_cm", "north_cm", "quality"])
    w.writerows(rows)

east_vals = [r[4] for r in rows]
north_vals = [r[5] for r in rows]

print("samples:", len(rows))
print("mean lat/lon:", lat0, lon0)
print("east cm min/max:", min(east_vals), max(east_vals))
print("north cm min/max:", min(north_vals), max(north_vals))
print("quality counts:", {q: sum(1 for r in rows if r[6] == q) for q in sorted(set(r[6] for r in rows))})

# SVG 산점도 생성
W, H = 800, 800
PAD = 60

min_x, max_x = min(east_vals), max(east_vals)
min_y, max_y = min(north_vals), max(north_vals)

span_x = max(max_x - min_x, 1e-9)
span_y = max(max_y - min_y, 1e-9)
span = max(span_x, span_y)

cx = (min_x + max_x) / 2
cy = (min_y + max_y) / 2

def map_x(x):
    return PAD + ((x - (cx - span / 2)) / span) * (W - 2 * PAD)

def map_y(y):
    return H - PAD - ((y - (cy - span / 2)) / span) * (H - 2 * PAD)

svg_points = []
for _, _, _, _, x, y, q in rows:
    px = map_x(x)
    py = map_y(y)
    color = "#1f77b4" if q == 4 else "#ff7f0e"
    svg_points.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="2" fill="{color}" opacity="0.45"/>')

zero_x = map_x(0)
zero_y = map_y(0)

svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">
<rect width="100%" height="100%" fill="white"/>
<text x="{W/2}" y="30" text-anchor="middle" font-size="20">RTK Fixed Position Jitter</text>
<line x1="{PAD}" y1="{zero_y:.2f}" x2="{W-PAD}" y2="{zero_y:.2f}" stroke="#999"/>
<line x1="{zero_x:.2f}" y1="{PAD}" x2="{zero_x:.2f}" y2="{H-PAD}" stroke="#999"/>
<rect x="{PAD}" y="{PAD}" width="{W-2*PAD}" height="{H-2*PAD}" fill="none" stroke="#333"/>
{chr(10).join(svg_points)}
<text x="{W/2}" y="{H-15}" text-anchor="middle" font-size="14">East-West offset (cm)</text>
<text x="20" y="{H/2}" text-anchor="middle" font-size="14" transform="rotate(-90 20 {H/2})">North-South offset (cm)</text>
<text x="{PAD}" y="{H-35}" font-size="12">samples: {len(rows)}</text>
<text x="{PAD}" y="{H-20}" font-size="12">quality 4 = RTK Fixed</text>
</svg>
'''

with open(SVG_FILE, "w") as f:
    f.write(svg)

print("csv:", CSV_FILE)
print("svg:", SVG_FILE)
