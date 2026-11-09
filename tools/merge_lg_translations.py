"""Merge LGroundControl Chinese wording into the Flight QGC catalog.

The reference catalog is extracted from LGroundControl.exe with lconvert.
Exact (context, source) matches win; otherwise a source string that has only
one LG translation is used.  A small explicit table fixes the Application
Settings vocabulary where the two QGC versions use different context names.
"""
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from lxml import etree


OVERRIDES = {
    ("SettingsPages.json", "General"): "基本配置",
    ("SettingsPages.json", "Plan View"): "航线规划",
    ("SettingsPages.json", "ADSB Server"): "ADSB 服务器",
    ("SettingsPages.json", "Remote ID"): "远程 ID",
    ("SettingsPages.json", "Telemetry"): "遥测数据",
    ("SettingsPages.json", "Video"): "视频服务",
    ("SettingsPages.json", "Help"): "关于",
    ("SelectViewDropdown", "Plan"): "航线规划",
    ("AppSettings", "Telemetry"): "遥测数据",
    ("AppSettings", "Video"): "视频服务",
    ("ADSBVehicleManager.SettingsUI.json", "ADSB Server"): "ADSB 服务器",
    ("FlyView.SettingsUI.json", "General"): "基本配置",
    ("General.SettingsUI.json", "General"): "基本配置",
    ("PlanView.SettingsUI.json", "General"): "基本配置",
    ("Viewer3D.SettingsUI.json", "General"): "基本配置",
    ("App.SettingsGroup.json", "Application Load/Save Path"): "应用程序加载/保存路径",
    ("App.SettingsGroup.json", "Stream GCS Position"): "流动地面站位置",
    ("App.SettingsGroup.json", "Color Scheme"): "配色方案",
    ("App.SettingsGroup.json", "Language"): "语言",
    ("App.SettingsGroup.json", "Never,Always,When in Follow Me Flight Mode"):
        "从不,始终,处于\"跟随我\"飞行模式时",
    ("App.SettingsGroup.json",
     "Streams the ground station GPS location to the vehicle for Follow Me mode."):
        "将地面站 GPS 位置发送给载具，用于跟随我模式。",
    ("UnitsSettings", "Area"): "区域",
    ("UnitsSettings", "Horizontal Distance"): "水平距离",
    ("UnitsSettings", "Vertical Distance"): "垂直距离",
    ("UnitsSettings", "Speed"): "速度",
    ("UnitsSettings", "Temperature"): "温度",
    ("UnitsSettings", "Meters"): "米",
    ("UnitsSettings", "SquareMeters"): "平方米",
    ("UnitsSettings", "Celsius"): "摄氏度",
    ("UnitsSettings", "Units"): "单位",
    # Strings newer than the LGroundControl catalog: keep the same wording style.
    ("ParameterDiffDialog", "%1 will be changed (including %2 not currently on the Vehicle)"):
        "将修改 %1 项（其中 %2 项当前不在载具上）",
    ("ParameterDiffDialog", "%1 will be changed"): "将修改 %1 项",
    ("ParameterDiffDialog", "1 already matches the Vehicle"): "1 项与载具一致",
    ("ParameterDiffDialog", "%1 already match the Vehicle"): "%1 项与载具一致",
    ("ParameterDiffDialog", "1 read-only parameter will not be sent"): "1 项只读参数不会发送",
    ("ParameterDiffDialog", "%1 read-only parameters will not be sent"): "%1 项只读参数不会发送",
    ("ParameterDiffDialog", "%1 not found on the Vehicle and cannot be sent"):
        "载具上找不到 %1 项，无法发送",
    ("ParameterDiffDialog", "Loaded 1 parameter from file."): "从文件加载了 1 项参数。",
    ("ParameterDiffDialog", "Loaded 1 parameter from file: %1."): "从文件加载了 1 项参数：%1。",
    ("ParameterDiffDialog", "Loaded %1 parameters from file."): "从文件加载了 %1 项参数。",
    ("ParameterDiffDialog", "Loaded %1 parameters from file: %2."):
        "从文件加载了 %1 项参数：%2。",
    ("ParameterDiffDialog", "Click 'Ok' to update the parameters below on the Vehicle."):
        "点击“确定”将以下参数更新到载具。",
    ("ParameterDiffDialog", "N/A — not on Vehicle"): "不可用 — 载具上没有",
    ("ParameterDiffDialog", "N/A — new to Vehicle"): "不可用 — 载具新增",
    ("ParameterDiffDialog",
     "Parameters marked 'new to Vehicle' have not been reported by the Vehicle. "
     "They may only become visible after they are sent and the Vehicle is rebooted."):
        "标记为“载具新增”的参数尚未由载具上报；发送并重启载具后它们才可能出现。",
    ("ParameterDiffDialog",
     "Parameters marked 'not on Vehicle' cannot be sent since the file does not include "
     "type information. They may only exist after another parameter is changed and the "
     "Vehicle is rebooted, after which you can load this file again."):
        "标记为“载具上没有”的参数因文件不含类型信息而无法发送；只有在修改其他参数并重启载具后"
        "它们才可能出现，届时可以重新加载此文件。",
    ("ParameterDiffDialog", "File"): "文件",
    ("ParameterDiffDialog", "Vehicle"): "载具",
    ("ParameterDiffDialog", "Comp ID"): "组件 ID",
    ("ParameterDiffDialog", "Name"): "名称",
}


def load(path: Path):
    root = ET.parse(path).getroot()
    exact = {}
    by_source = defaultdict(set)
    for context in root.findall("context"):
        name = context.findtext("name", "")
        for message in context.findall("message"):
            translation = message.find("translation")
            if translation is None:
                continue
            if translation.get("type") == "unfinished":
                continue
            text = translation.text or ""
            if not text.strip():
                continue
            if translation.findall("numerusform"):
                continue
            key = (name, message.findtext("source", ""))
            exact[key] = text
            by_source[message.findtext("source", "")].add(text)
    unique_source = {source: next(iter(values)) for source, values in by_source.items()
                     if len(values) == 1}
    return exact, unique_source


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ours", required=True, type=Path)
    parser.add_argument("--lg", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args()

    exact, unique_source = load(arguments.lg)
    tree = etree.parse(str(arguments.ours))
    root = tree.getroot()

    changed_exact = changed_source = changed_override = 0
    for context in root.findall("context"):
        name = context.findtext("name", "")
        for message in context.findall("message"):
            translation = message.find("translation")
            if translation is None or translation.findall("numerusform"):
                continue
            source = message.findtext("source", "")
            replacement = None
            if (name, source) in OVERRIDES:
                replacement = OVERRIDES[(name, source)]
                changed_override += 1
            elif (name, source) in exact:
                replacement = exact[(name, source)]
                changed_exact += 1
            elif source in unique_source:
                replacement = unique_source[source]
                changed_source += 1
            if replacement is None:
                continue
            for child in list(translation):
                translation.remove(child)
            translation.text = replacement
            translation.attrib.pop("type", None)

    tree.write(str(arguments.output), encoding="utf-8", xml_declaration=True,
               pretty_print=True, doctype="<!DOCTYPE TS>")
    print(f"exact={changed_exact} source={changed_source} override={changed_override} "
          f"-> {arguments.output}")


if __name__ == "__main__":
    main()
