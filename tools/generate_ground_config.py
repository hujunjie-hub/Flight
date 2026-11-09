"""Generate the Flight parameter catalog used by the firmware and the ground station.

The five YAML files in config/ stay authoritative.  This tool reads them,
resolves the parameters that are actually applied to the running controller and
vehicle model, and emits:

  ground_parameters.h   C table (name, default, limits, writability) consumed by
                        the Host SIL and the flight firmware;
  ground_parameters.json  the same catalog plus Chinese titles, units, groups and
                        descriptions consumed by the customized ground station;
  ground_parameters.md   human-readable reference generated from the catalog.

Only parameters that are written into ProductControllerConfig are exposed: the
estimator and sensor YAML files are still DRAFT_* and carry TBD values, so
publishing them as writable MAVLink parameters would claim evidence the project
does not have.  They are still recorded in the "sources" section for audit.

Usage:
    python tools/generate_ground_config.py [--output DIR] [--check]
"""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "middleware/protocol/mavlink/parameters"
FILES = {
    "vehicle_model": "VEH",
    "sensor_alignment": "SNS",
    "estimator_tuning": "INS",
    "controller_tuning": "CTL",
    # Session calibration values are stored in the same MAVLink parameter
    # catalog but are not fields of ProductControllerConfig.
    "calibration": "CAL",
}

# Fields that the Host SIL and the firmware copy into ProductControllerConfig.
VEHICLE_FIELDS = {
    "mass_kg", "inertia_frd_kgm2", "motor_position_frd_m", "motor_yaw_torque_ratio_m",
}
CONTROLLER_FIELDS = {
    "so3_gain_per_s", "rate_kp_nm_per_radps", "rate_ki_nm_per_rad", "rate_kd_nm_per_radps2",
    "rate_kaw_per_s", "rate_d_cutoff_hz", "rate_integral_limit_nm", "manual_hover_throttle_norm",
    "fallback_kp_per_s2", "fallback_kd_per_s", "mpc_q_diag", "mpc_qf_diag", "mpc_r_diag",
    "mpc_s_diag", "actuation_preview_delay_s", "mpc_seed_position_error_limit_m",
    "mpc_seed_velocity_error_limit_mps",
}

AXIS = ("X", "Y", "Z")
AXIS_CN = ("滚转", "俯仰", "偏航")
NED_CN = ("北", "东", "地")

# name template, Chinese title template, group, units, minimum, maximum, description
SCALARS = {
    ("vehicle_model", "mass_kg"): (
        "VEH_MASS", "整机质量", "机体模型", "kg", 0.1, 50.0,
        "控制器推力分配和制导模型使用的整机质量；与实际起飞重量不一致会造成高度偏差。"),
    ("controller_tuning", "inner_loop", "rate_d_cutoff_hz"): (
        "CTL_RATE_DCUT", "角速度环 D 项截止频率", "姿态角速度环", "Hz", 1.0, 449.0,
        "角速度误差微分的低通截止频率，必须大于 0 且小于 450 Hz。"),
    ("controller_tuning", "inner_loop", "manual_hover_throttle_norm"): (
        "CTL_HOVER_THR", "手动悬停油门", "姿态角速度环", "norm", 0.05, 0.95,
        "手动模式下维持悬停的归一化油门，必须在 0 与 1 之间。"),
    ("controller_tuning", "outer_loop", "actuation_preview_delay_s"): (
        "CTL_ACT_DLY", "执行预览延迟", "MPC 权重", "s", 0.0, 0.04,
        "把执行器响应延迟前馈到参考中的时间，允许 0 到 0.04 s。"),
    ("controller_tuning", "outer_loop", "mpc_seed_position_error_limit_m"): (
        "CTL_SEED_POS", "MPC 种子位置误差限", "MPC 权重", "m", 0.01, 100.0,
        "MPC 热启动允许的位置误差上限，必须大于 0。"),
    ("controller_tuning", "outer_loop", "mpc_seed_velocity_error_limit_mps"): (
        "CTL_SEED_VEL", "MPC 种子速度误差限", "MPC 权重", "m/s", 0.01, 100.0,
        "MPC 热启动允许的速度误差上限，必须大于 0。"),
}

VECTORS = {
    ("controller_tuning", "inner_loop", "so3_gain_per_s"): (
        "CTL_SO3_{a}", "SO(3) 姿态增益（{c}）", "SO3 姿态增益", "1/s", 0.1, 50.0,
        "SO(3) 姿态误差到期望角速度的增益，必须大于 0。"),
    ("controller_tuning", "inner_loop", "rate_kp_nm_per_radps"): (
        "CTL_RATE_KP_{a}", "角速度环 P（{c}）", "姿态角速度环", "N·m/(rad/s)", 0.0, 20.0,
        "角速度误差比例增益，允许 0。"),
    ("controller_tuning", "inner_loop", "rate_ki_nm_per_rad"): (
        "CTL_RATE_KI_{a}", "角速度环 I（{c}）", "姿态角速度环", "N·m/rad", 0.0, 20.0,
        "角速度误差积分增益，允许 0。"),
    ("controller_tuning", "inner_loop", "rate_kd_nm_per_radps2"): (
        "CTL_RATE_KD_{a}", "角速度环 D（{c}）", "姿态角速度环", "N·m/(rad/s²)", 0.0, 20.0,
        "角速度误差微分增益，允许 0；D 项再经过 CTL_RATE_DCUT 低通。"),
    ("controller_tuning", "inner_loop", "rate_kaw_per_s"): (
        "CTL_RATE_KAW_{a}", "角速度环抗饱和（{c}）", "姿态角速度环", "1/s", 0.01, 100.0,
        "积分抗饱和反馈增益，必须大于 0。"),
    ("controller_tuning", "inner_loop", "rate_integral_limit_nm"): (
        "CTL_RATE_ILIM_{a}", "角速度积分限幅（{c}）", "姿态角速度环", "N·m", 0.001, 10.0,
        "积分项力矩限幅，必须大于 0。"),
    ("controller_tuning", "outer_loop", "fallback_kp_per_s2"): (
        "CTL_FB_KP_{a}", "位置环回退 P（{n}）", "位置环回退", "1/s²", 0.0, 50.0,
        "MPC 不可用时位置环回退控制的比例增益，允许 0。"),
    ("controller_tuning", "outer_loop", "fallback_kd_per_s"): (
        "CTL_FB_KD_{a}", "位置环回退 D（{n}）", "位置环回退", "1/s", 0.0, 50.0,
        "MPC 不可用时位置环回退控制的阻尼增益，允许 0。"),
    ("controller_tuning", "outer_loop", "mpc_q_diag"): (
        "CTL_MPC_Q{i}", "MPC 状态权重 Q{i}", "MPC 权重", "-", 0.001, 1000.0,
        "MPC 运行代价的对角权重，必须大于 0。"),
    ("controller_tuning", "outer_loop", "mpc_qf_diag"): (
        "CTL_MPC_QF{i}", "MPC 终端权重 Qf{i}", "MPC 权重", "-", 0.001, 1000.0,
        "MPC 终端代价的对角权重，必须大于 0。"),
    ("controller_tuning", "outer_loop", "mpc_r_diag"): (
        "CTL_MPC_R{i}", "MPC 控制权重 R{i}", "MPC 权重", "-", 0.0, 1000.0,
        "MPC 控制量代价的对角权重，允许 0。"),
    ("controller_tuning", "outer_loop", "mpc_s_diag"): (
        "CTL_MPC_S{i}", "MPC 控制增量权重 S{i}", "MPC 权重", "-", 0.001, 100.0,
        "MPC 控制增量代价的对角权重，必须大于 0。"),
}

INERTIA = {
    (0, 0): "IXX", (0, 1): "IXY", (0, 2): "IXZ",
    (1, 0): "IXY", (1, 1): "IYY", (1, 2): "IYZ",
    (2, 0): "IXZ", (2, 1): "IYZ", (2, 2): "IZZ",
}


class Parameter:
    def __init__(self, name, title, group, units, value, minimum, maximum, description,
                 file, path, kind, field, access):
        self.access = access
        self.field = field
        self.name = name
        self.title = title
        self.group = group
        self.units = units
        self.value = value
        self.minimum = minimum
        self.maximum = maximum
        self.description = description
        self.file = file
        self.path = path
        self.kind = kind

    def as_json(self):
        return {
            "name": self.name,
            "title": self.title,
            "group": self.group,
            "units": self.units,
            "value": float(self.value),
            "minimum": float(self.minimum),
            "maximum": float(self.maximum),
            "writable": True,
            "kind": self.kind,
            "file": self.file + ".yaml",
            "path": list(self.path),
            "description": self.description,
        }


def leaves(value, path=()):
    if isinstance(value, dict):
        for key, item in value.items():
            yield from leaves(item, (*path, key))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            yield from leaves(item, (*path, index))
    else:
        yield path, value


def is_number(value):
    return type(value) in (int, float) and math.isfinite(value) and abs(value) <= 3.4e38


def number_literal(value):
    if not is_number(value):
        raise ValueError(f"invalid float32 value {value!r}")
    return f"{float(value):.9e}f"


def resolve(data, path):
    node = data
    for key in path:
        if isinstance(node, list) and isinstance(key, int):
            node = node[key]
        elif isinstance(node, dict) and key in node:
            node = node[key]
        else:
            raise KeyError(path)
    return node


def build_catalog(groups):
    parameters = []
    for group, data in groups.items():
        for field in sorted(VEHICLE_FIELDS | CONTROLLER_FIELDS):
            matches = [(key, "scalar") for key in SCALARS if key[0] == group and key[-1] == field]
            matches += [(key, "vector") for key in VECTORS if key[0] == group and key[-1] == field]
            if not matches:
                continue  # matrices and motor arrays are expanded below
            key, kind = matches[0]
            path = key[1:]
            value = resolve(data, path)
            if kind == "vector":
                parameters.extend(expand_vector(group, field, key, path, value))
            else:
                parameters.append(expand_scalar(group, field, key, path, value))
    # The vehicle model carries dedicated matrix and motor arrays.
    vehicle = groups["vehicle_model"]
    parameters.extend(expand_inertia("vehicle_model", vehicle["inertia_frd_kgm2"]))
    parameters.extend(expand_motors("vehicle_model", vehicle["motor_position_frd_m"]))
    parameters.extend(expand_yaw_ratios("vehicle_model", vehicle["motor_yaw_torque_ratio_m"]))
    # Calibration values are deliberately stored-only: the sensor service
    # applies them, not ProductControllerConfig.
    for index, entry in enumerate(groups.get("calibration", {}).get("parameters", [])):
        parameters.append(Parameter(
            entry["name"], entry["title"], entry["group"], entry.get("units", "-"),
            entry["value"], entry["minimum"], entry["maximum"],
            entry.get("description", ""), "calibration",
            ("parameters", index, "value"), "stored", entry["name"], ("none",)))
    return parameters


def expand_scalar(group, field, key, path, value):
    if not is_number(value):
        raise ValueError(f"{key} is not a float32 number: {value!r}")
    name, title, title_group, units, minimum, maximum, description = SCALARS[key]
    if not minimum <= float(value) <= maximum:
        raise ValueError(f"{name} default {value} is outside [{minimum}, {maximum}]")
    return Parameter(name, title, title_group, units, value, minimum, maximum, description,
                     group, path, "scalar", field, ("scalar", field))


def expand_vector(group, field, key, path, value):
    name_t, title_t, title_group, units, minimum, maximum, description = VECTORS[key]
    result = []
    if len(value) != 3 and len(value) not in (6,):
        raise ValueError(f"{key} has unsupported length {len(value)}")
    for index, item in enumerate(value):
        if not is_number(item):
            raise ValueError(f"{key}[{index}] is not a float32 number: {item!r}")
        if "fallback_k" in field and index < 3:
            axis, axis_cn = AXIS[index], NED_CN[index]
        elif index < 3:
            axis, axis_cn = AXIS[index], AXIS_CN[index]
        else:
            axis, axis_cn = str(index), str(index)
        name = name_t.format(a=axis, i=index)
        title = title_t.format(c=axis_cn, a=axis, i=index, n=NED_CN[index] if index < 3 else axis)
        result.append(Parameter(name, title, title_group, units, item, minimum, maximum,
                                description, group, (*path, index), "vector", field,
                                ("array", field, index)))
    return result


def expand_inertia(group, matrix):
    if len(matrix) != 9 or isinstance(matrix[0], list):
        raise ValueError("inertia_frd_kgm2 must be nine row-major numbers")
    result = []
    for row in range(3):
        for column in range(3):
            value = matrix[row * 3 + column]
            if row > column:
                continue  # symmetric tensor: expose the upper triangle only
            name = f"VEH_{INERTIA[(row, column)]}"
            title = f"转动惯量 {INERTIA[(row, column)]}"
            minimum, maximum = (0.001, 5.0) if row == column else (-1.0, 1.0)
            result.append(Parameter(
                name, title, "机体模型", "kg·m²", value, minimum, maximum,
                "机体惯性张量的上三角元素（行优先，FRD 机体系）；必须保持对称正定，"
                "否则参数写入会被拒绝。", group, ("inertia_frd_kgm2", row, column), "matrix",
                "inertia_frd_kgm2", ("inertia", row, column)))
    return result


def expand_motors(group, positions):
    result = []
    for motor, position in enumerate(positions):
        for axis in range(3):
            name = f"VEH_M{motor}_{AXIS[axis]}"
            title = f"电机 {motor + 1} 位置 {AXIS[axis]}（{('前', '右', '下')[axis]}）"
            result.append(Parameter(
                name, title, "机体模型", "m", position[axis], -1.0, 1.0,
                "电机安装位置，机体系 FRD，相对重心。四个电机位置共同决定控制分配矩阵。",
                group, ("motor_position_frd_m", motor, axis), "matrix", "motor_position_frd_m",
                ("matrix", "motor_position_frd_m", motor, axis)))
    return result


def expand_yaw_ratios(group, ratios):
    result = []
    for motor, value in enumerate(ratios):
        name = f"VEH_M{motor}_YAW"
        result.append(Parameter(
            name, f"电机 {motor + 1} 反扭矩系数", "机体模型", "m", value, -0.2, 0.2,
            "螺旋桨反扭矩与推力的比值，符号表示旋向；参与偏航控制分配。",
            group, ("motor_yaw_torque_ratio_m", motor), "vector", "motor_yaw_torque_ratio_m",
            ("array", "motor_yaw_torque_ratio_m", motor)))
    return result


def build_rows(groups):
    rows = []
    for group, data in groups.items():
        if group == "calibration":
            continue  # stored-only values are documented by ground_parameters.md
        for path, value in leaves(data):
            rows.append({
                "file": group + ".yaml",
                "path": list(path),
                "value": value,
                "mavlink_id": None,
            })
    return rows


def attach_names(rows, parameters):
    by_path = {(p.file, tuple(p.path)): p.name for p in parameters}
    for row in rows:
        row["mavlink_id"] = by_path.get((row["file"][:-5], tuple(row["path"])))
    return rows


def validate(parameters):
    names = set()
    for parameter in parameters:
        if len(parameter.name) > 16:
            raise ValueError(f"MAVLink parameter name too long: {parameter.name}")
        if not all(c.isupper() or c.isdigit() or c == "_" for c in parameter.name):
            raise ValueError(f"MAVLink parameter name must be uppercase: {parameter.name}")
        if parameter.name in names:
            raise ValueError(f"duplicate MAVLink parameter name: {parameter.name}")
        names.add(parameter.name)
        if not parameter.minimum <= float(parameter.value) <= parameter.maximum:
            raise ValueError(f"{parameter.name} default outside limits")
    return names


def controller_fields(parameters):
    """Check the catalog keeps every ProductControllerConfig field covered."""
    fields = {parameter.field for parameter in parameters}
    missing = (VEHICLE_FIELDS | CONTROLLER_FIELDS) - fields
    if missing:
        raise ValueError(f"controller fields without a parameter: {sorted(missing)}")


def shape(field):
    return {
        "mass_kg": (), "inertia_frd_kgm2": (9,), "motor_position_frd_m": (4, 3),
        "motor_yaw_torque_ratio_m": (4,), "mpc_q_diag": (6,), "mpc_qf_diag": (6,),
        "rate_d_cutoff_hz": (), "manual_hover_throttle_norm": (), "actuation_preview_delay_s": (),
        "mpc_seed_position_error_limit_m": (), "mpc_seed_velocity_error_limit_mps": (),
    }.get(field, (3,))


def flatten(value, dimensions):
    if not dimensions:
        yield (), number_literal(value)
    elif isinstance(value, list) and len(value) == dimensions[0]:
        for index, item in enumerate(value):
            for indexes, literal in flatten(item, dimensions[1:]):
                yield (index, *indexes), literal
    else:
        raise ValueError(f"invalid array shape, expected {dimensions}: {value!r}")


def write_outputs(out, groups, parameters, rows):
    digest = hashlib.sha256(json.dumps(
        [p.as_json() for p in parameters], sort_keys=True, ensure_ascii=False).encode()).hexdigest()
    sources = {
        name: hashlib.sha256((ROOT / "config" / f"{name}.yaml").read_bytes()).hexdigest()
        for name in FILES
    }
    out.mkdir(parents=True, exist_ok=True)

    literals = [f'    {{"{p.name}", {number_literal(p.value)}, {number_literal(p.minimum)}, '
                f'{number_literal(p.maximum)}, 1}},'
                for p in parameters]
    assignments = []
    vehicle = groups["vehicle_model"]
    controller = groups["controller_tuning"]
    active = {field: vehicle[field] for field in sorted(VEHICLE_FIELDS)}
    for section in ("inner_loop", "outer_loop"):
        for key, value in controller[section].items():
            if key in CONTROLLER_FIELDS:
                active[key] = value
    for field, value in active.items():
        for indexes, literal in flatten(value, shape(field)):
            suffix = "".join(f"[{index}]" for index in indexes)
            assignments.append(f"    config->payload.{field}{suffix} = {literal};")

    from_lines = [
        "",
        "/* Copy the parameter catalog order into the controller payload. */",
        "static void ground_payload_from_values(ProductControllerConfigPayload *p,",
        "                                      const float *v)",
        "{",
    ]
    to_lines = [
        "",
        "/* Read the controller payload back into the catalog order. */",
        "static void ground_values_from_payload(float *v,",
        "                                      const ProductControllerConfigPayload *p)",
        "{",
    ]
    for index, parameter in enumerate(parameters):
        access = parameter.access
        if access[0] == "scalar":
            from_lines.append(f"    p->{access[1]} = v[{index}];")
            to_lines.append(f"    v[{index}] = p->{access[1]};")
        elif access[0] == "array":
            _, field, element = access
            from_lines.append(f"    p->{field}[{element}] = v[{index}];")
            to_lines.append(f"    v[{index}] = p->{field}[{element}];")
        elif access[0] == "matrix":
            _, field, row, column = access
            from_lines.append(f"    p->{field}[{row}][{column}] = v[{index}];")
            to_lines.append(f"    v[{index}] = p->{field}[{row}][{column}];")
        elif access[0] == "inertia":
            _, row, column = access
            from_lines.append(f"    p->inertia_frd_kgm2[{row * 3 + column}] = v[{index}];")
            if row != column:
                from_lines.append(f"    p->inertia_frd_kgm2[{column * 3 + row}] = v[{index}];")
            to_lines.append(f"    v[{index}] = p->inertia_frd_kgm2[{row * 3 + column}];")
        elif access[0] == "none":
            # Stored-only parameter (sensor calibration): the catalog keeps the
            # value, ProductControllerConfig has no matching field.
            continue
        else:
            raise ValueError(f"unknown access {access}")
    from_lines.append("}")
    to_lines.append("}")

    header = [
        "/* Generated from the five YAML files by generate_ground_config.py; do not edit. */",
        "#ifndef GROUND_PARAMETERS_H",
        "#define GROUND_PARAMETERS_H",
        "",
        "#include \"product_contract/product_contract.h\"",
        "",
        "typedef struct {",
        "    char name[16];",
        "    float value;",
        "    float minimum;",
        "    float maximum;",
        "    unsigned char writable;",
        "} GroundParameter;",
        "",
        "static const GroundParameter ground_parameters[] = {",
        *literals,
        "};",
        f"enum {{ GROUND_PARAMETER_COUNT = {len(parameters)} }};",
        "",
        "static void ground_configure_controller(ProductControllerConfig *config)",
        "{",
        f"    config->parameter_hash = 0x{int(digest[:8], 16):08x}u;",
        *assignments,
        "}",
        *from_lines,
        *to_lines,
        "",
        "#endif",
        "",
    ]
    (out / "ground_parameters.h").write_text("\n".join(header), encoding="utf-8")

    document = {
        "schema": 2,
        "profile": "QUAD_X4",
        "generator": "tools/generate_ground_config.py",
        "sha256": digest,
        "sources": sources,
        "parameters": [p.as_json() for p in parameters],
        "fields": rows,
    }
    (out / "ground_parameters.json").write_text(
        json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    lines = [
        "# Flight 参数目录",
        "",
        f"由 `tools/generate_ground_config.py` 从 config/ 五份 YAML 生成，SHA256 `{digest}`。",
        "参数在运行时通过 MAVLink 参数协议读写，写入只在本上电周期有效；重启后恢复 YAML 默认值。",
        "",
        "| 名称 | 标题 | 分组 | 默认值 | 范围 | 单位 |",
        "|---|---|---|---|---|---|",
    ]
    for p in parameters:
        lines.append(f"| `{p.name}` | {p.title} | {p.group} | {p.value:g} | "
                     f"{p.minimum:g} … {p.maximum:g} | {p.units} |")
    lines.append("")
    (out / "ground_parameters.md").write_text("\n".join(lines), encoding="utf-8")


def generate(out, check=False):
    groups = {
        group: yaml.safe_load((ROOT / "config" / f"{group}.yaml").read_text(encoding="utf-8"))
        for group in FILES
    }
    if any(data["profile"] != "QUAD_X4" or data["schema_version"] != 1 for data in groups.values()):
        raise ValueError("expected the QUAD_X4 schema 1 parameter set")
    structure = groups["controller_tuning"]["outer_loop"]["mpc_structure"]
    if structure != {"sample_time_s": 0.02, "prediction_horizon": 20, "control_horizon": 5}:
        raise ValueError("regenerate the MPC solver before changing the MPC structure")
    parameters = build_catalog(groups)
    validate(parameters)
    controller_fields(parameters)
    rows = attach_names(build_rows(groups), parameters)
    if check:
        before = {name: (out / name).read_bytes() for name in
                  ("ground_parameters.h", "ground_parameters.json", "ground_parameters.md")
                  if (out / name).is_file()}
        if len(before) != 3:
            raise SystemExit(f"parameter catalog is missing under {out}")
        write_outputs(out, groups, parameters, rows)
        changed = [name for name, old in before.items() if (out / name).read_bytes() != old]
        if changed:
            raise SystemExit(f"parameter catalog is stale: {', '.join(changed)}")
        print(f"parameter catalog up to date: {len(parameters)} parameters")
        return
    write_outputs(out, groups, parameters, rows)
    print(f"generated {len(parameters)} parameters in {out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    generate(arguments.output, arguments.check)

