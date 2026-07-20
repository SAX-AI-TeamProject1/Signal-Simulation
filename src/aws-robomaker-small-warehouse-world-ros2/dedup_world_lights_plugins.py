#!/usr/bin/env python3
"""
worlds/*/*.world 안에 중복 삽입된 <light>, <plugin> 을 제거하는 스크립트.

port_classic_to_gzsim.py 를 같은 world 파일에 2번 이상 돌렸을 때 발생하는
"Error Code 2: light with name[sun] already exists" 같은 중복 삽입을 정리한다.
키는 light -> name 속성, plugin -> (filename, name) 조합. 각 키의 첫 등장만 남기고
이후 중복은 제거.

원본은 <file>.bak 으로 백업.

원본 (cmd에서 즉석으로 실행했던 버전, 백업 없이 바로 덮어씀):

    cd ~/Desktop/AWS_GAZEBO/aws-robomaker-small-warehouse-world-ros2
    python3 - << 'PY'
    import glob, xml.etree.ElementTree as ET
    for f in glob.glob('worlds/*/*.world'):
        t = ET.parse(f); w = t.getroot().find('world')
        seen = set()
        for tag, key in (('light', lambda e: e.get('name')),
                         ('plugin', lambda e: (e.get('filename'), e.get('name')))):
            for e in list(w.findall(tag)):
                k = (tag, key(e))
                if k in seen:
                    w.remove(e)          # 중복 제거
                else:
                    seen.add(k)
        ET.indent(t, space="  ")
        t.write(f, encoding="utf-8", xml_declaration=True)
        w = ET.parse(f).getroot().find('world')
        print(f, '| light:', len(w.findall('light')), '| plugin:', len(w.findall('plugin')))
    PY
"""
import glob
import shutil
import xml.etree.ElementTree as ET

if __name__ == "__main__":
    for f in glob.glob("worlds/*/*.world"):
        t = ET.parse(f)
        w = t.getroot().find("world")
        seen = set()
        for tag, key in (
            ("light", lambda e: e.get("name")),
            ("plugin", lambda e: (e.get("filename"), e.get("name"))),
        ):
            for e in list(w.findall(tag)):
                k = (tag, key(e))
                if k in seen:
                    w.remove(e)  # 중복 제거
                else:
                    seen.add(k)

        shutil.copy(f, f + ".bak")
        ET.indent(t, space="  ")
        t.write(f, encoding="utf-8", xml_declaration=True)

        w = ET.parse(f).getroot().find("world")
        print(f, "| light:", len(w.findall("light")), "| plugin:", len(w.findall("plugin")))
