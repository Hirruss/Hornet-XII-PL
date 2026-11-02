"""Redraw the user's P&L architecture as SVG + PNG. Requires Pillow for PNG."""
from html import escape
from pathlib import Path
import math

from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parents[1] / 'docs/images'
COLORS = {'ink': '#18304b', 'muted': '#526880', 'background': '#f6f8fc',
          'implemented': '#d8f3ed', 'planned': '#ece9f8', 'output': '#dceafd',
          'config': '#fff3ce', 'feedback': '#b56b05', 'line': '#536c88'}


class Diagram:
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
                    '<title>Hornet XII Perception and Localization framework</title>',
                    f'<rect width="100%" height="100%" fill="{COLORS["background"]}"/>']
        self.image = Image.new('RGB', (width, height), COLORS['background'])
        self.draw = ImageDraw.Draw(self.image)

    def font(self, size, bold=False):
        candidates = ['C:/Windows/Fonts/segoeuib.ttf' if bold else 'C:/Windows/Fonts/segoeui.ttf',
                      '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf' if bold else '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']
        for path in candidates:
            if Path(path).is_file():
                return ImageFont.truetype(path, size)
        return ImageFont.load_default(size=size)

    def text(self, x, y, lines, size=25, bold=False, color='ink', anchor='left'):
        if isinstance(lines, str):
            lines = [lines]
        for index, text in enumerate(lines):
            cy = y + index * (size + 9)
            align = 'middle' if anchor == 'center' else 'start'
            self.svg.append(f'<text x="{x}" y="{cy+size}" text-anchor="{align}" font-family="Segoe UI,DejaVu Sans,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" fill="{COLORS[color]}">{escape(text)}</text>')
            font = self.font(size, bold)
            width = self.draw.textlength(text, font=font)
            self.draw.text((x-width/2 if anchor == 'center' else x, cy), text, font=font, fill=COLORS[color])

    def box(self, x, y, width, height, title, subtitle, kind='planned'):
        fill = COLORS[kind]
        self.svg.append(f'<rect x="{x}" y="{y}" width="{width}" height="{height}" rx="18" fill="{fill}" stroke="#aebed0" stroke-width="2"/>')
        self.draw.rounded_rectangle((x, y, x+width, y+height), radius=18, fill=fill, outline='#aebed0', width=2)
        self.text(x+width/2, y+18, title, size=26, bold=True, anchor='center')
        if subtitle:
            self.text(x+width/2, y+59, subtitle, size=20, color='muted', anchor='center')

    def arrow(self, points, color='line', dashed=False):
        shade = COLORS[color]
        coords = ' '.join(f'{x},{y}' for x, y in points)
        dash = ' stroke-dasharray="9 7"' if dashed else ''
        self.svg.append(f'<polyline points="{coords}" fill="none" stroke="{shade}" stroke-width="3"{dash}/>')
        if dashed:
            for (x1,y1),(x2,y2) in zip(points, points[1:]):
                length = math.hypot(x2-x1,y2-y1)
                for start in range(0, int(length), 16):
                    a,b=start/max(length,1),min(start+9,length)/max(length,1)
                    self.draw.line([(x1+(x2-x1)*a,y1+(y2-y1)*a),(x1+(x2-x1)*b,y1+(y2-y1)*b)], fill=shade, width=3)
        else:
            self.draw.line(points, fill=shade, width=3)
        x,y=points[-1]
        px,py=points[-2]
        angle=math.atan2(y-py,x-px)
        tri=[(x,y)]+[(x-14*math.cos(angle+d),y-14*math.sin(angle+d)) for d in (-0.45,0.45)]
        self.draw.polygon(tri, fill=shade)
        self.svg.append(f'<polygon points="{" ".join(f"{a},{b}" for a,b in tri)}" fill="{shade}"/>')

    def save(self, name):
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT/f'{name}.svg').write_text('\n'.join(self.svg+['</svg>']), encoding='utf-8')
        self.image.save(OUT/f'{name}.png')


def framework():
    d=Diagram(2000,1250)
    d.text(50,25,'HORNET XII  /  PERCEPTION & LOCALIZATION',42,True)
    d.text(50,88,'Green: implemented    Purple: work to assign    Blue: outputs    Dashed: YAML reads / feedback',23,color='muted')
    d.text(50,180,'FRONT STEREO',23,True,color='muted')
    d.box(50,270,230,110,'Front camera',['Driver integration'])
    d.box(340,270,250,110,'Calibration',['Split + rectify'],'implemented')
    d.box(730,210,270,110,'Object detection',['Stereo centers / class'])
    d.box(730,390,270,110,'Color thresholding',['Stereo centers / class'])
    d.box(1150,300,300,130,'Depth estimation',['Triangulate + select','Transform into map'],'implemented')
    d.box(1630,300,310,130,'Object positions',['Per class + instance','XYZ in fixed map'],'output')
    d.box(340,500,250,100,'Stereo YAML',['Saved before runtime'],'config')
    d.arrow([(280,325),(340,325)])
    d.text(285,285,'raw',20,color='muted')
    d.arrow([(590,325),(655,325),(655,265),(730,265)])
    d.arrow([(655,325),(655,445),(730,445)])
    d.text(595,210,['Left / right','rectified'],20,color='muted')
    d.arrow([(1000,265),(1080,265),(1080,345),(1150,345)])
    d.arrow([(1000,445),(1080,445),(1080,390),(1150,390)])
    d.text(1008,475,['Per-class','observations'],20,color='muted')
    d.arrow([(1450,365),(1630,365)])
    d.arrow([(465,500),(465,380)],color='feedback',dashed=True)
    d.arrow([(590,550),(1220,550),(1220,430)],color='feedback',dashed=True)
    d.text(705,555,'Read geometry once',21,color='feedback')
    d.arrow([(1300,300),(1300,155),(865,155),(865,210)],color='feedback',dashed=True)
    d.arrow([(865,155),(695,155),(695,445),(730,445)],color='feedback',dashed=True)
    d.text(900,120,'Range + uncertainty / source feedback',21,color='feedback')
    d.text(50,735,'BOTTOM VISION + LOCALIZATION',23,True,color='muted')
    d.box(50,840,230,110,'Bottom camera',['Driver integration'])
    d.box(340,840,250,110,'Calibration',['Mono rectify'],'implemented')
    d.box(730,750,270,110,'Optical flow',['Metric motion + scale'])
    d.box(730,1020,270,120,'Bottom color',['Objects on known plane'])
    d.box(730,595,270,100,'IMU + depth',['Sensor integration'])
    d.box(1150,780,300,130,'EKF / localization',['Pose + covariance','Map / TF authority'])
    d.box(1630,780,310,130,'Robot localization',['Pose in fixed map'],'output')
    d.box(1630,1020,310,120,'Bottom objects',['Positions in map'],'output')
    d.box(340,1050,250,100,'Bottom YAML',['Saved before runtime'],'config')
    d.arrow([(280,895),(340,895)])
    d.arrow([(590,895),(655,895),(655,805),(730,805)])
    d.arrow([(655,895),(655,1080),(730,1080)])
    d.arrow([(465,1050),(465,950)],color='feedback',dashed=True)
    d.arrow([(1000,805),(1150,805)])
    d.arrow([(1000,645),(1060,645),(1060,860),(1150,860)])
    d.arrow([(1450,845),(1630,845)])
    d.arrow([(1000,1080),(1630,1080)])
    d.arrow([(1300,780),(1300,665),(1520,665),(1520,480),(1380,480),(1380,430)])
    d.text(1535,560,['TF at image time','+ camera extrinsics'],21,color='muted')
    d.arrow([(1225,910),(1225,975),(865,975),(865,1020)])
    d.text(900,935,'Pose + known plane / altitude',20,color='muted')
    d.text(50,1190,'Map coordinates do not remove pose drift. Calibration YAMLs shipped here are synthetic examples, blocked by default.',22,color='muted')
    d.save('framework')


def overview():
    d=Diagram(1350,620)
    d.text(50,30,'HORNET XII PL  /  SYSTEM BOUNDARY',38,True)
    d.box(50,160,250,100,'Cameras',['Front stereo + bottom'])
    d.box(50,350,250,100,'IMU + depth',['Time-aligned sensors'])
    d.box(470,230,360,150,'Perception &',['Localization'],'implemented')
    d.box(1010,160,290,100,'Object positions',['Metric map coordinates'],'output')
    d.box(1010,350,290,100,'Robot localization',['Pose + uncertainty'],'output')
    d.arrow([(300,210),(390,210),(390,270),(470,270)])
    d.arrow([(300,400),(390,400),(390,340),(470,340)])
    d.arrow([(830,270),(920,270),(920,210),(1010,210)])
    d.arrow([(830,340),(920,340),(920,400),(1010,400)])
    d.text(50,540,'Implementation scope: calibration / rectification + sparse stereo depth. Other node roles are assignment scaffolds.',22,color='muted')
    d.save('system-overview')


if __name__ == '__main__':
    framework()
    overview()
    print(f'Diagrams written to {OUT}')
