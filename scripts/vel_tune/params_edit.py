"""Edit sim/config/model_params.yaml's `robots` entries in place, keeping its comments: set a model's vel_ctl /
wheel_friction (PhysX, the model's own keys) or its `newton:` overrides. Used by calibrate_all.py.

    set_params(path, model, engine, vel_ctl=None, wheel_friction=None, note=None, wheel_max_force=None,
               wheel_damping=None)

wheel_max_force (Newton's wheel torque cap) and wheel_damping (its drive damping, N*m*s/rad) go only under `newton:`.

`robots:` must be the file's last top-level key (it is); a model without a block gets one appended."""
import re


def _fmt_list(vals):
    return "[" + ", ".join(f"{v:g}" for v in vals) + "]"


def _fmt_friction(f):
    return "{" + f"static: {f['static']:g}, dynamic: {f['dynamic']:g}, combine: {f.get('combine', 'min')}" + "}"


def set_params(path, model, engine, vel_ctl=None, wheel_friction=None, note=None, wheel_max_force=None,
               wheel_damping=None):
    lines = open(path).read().splitlines()
    robots = next(i for i, l in enumerate(lines) if l.rstrip() == "robots:")
    head = next((i for i in range(robots + 1, len(lines)) if lines[i].rstrip() == f"  {model}:"), None)
    if head is None:
        lines.append(f"  {model}:")
        head = len(lines) - 1
    # the block: lines after the head indented > 2 (blank lines inside allowed), up to the next "  key:" or "  #"
    end = head + 1
    while end < len(lines) and (lines[end].startswith("   ") or lines[end].strip() == ""):
        end += 1
    while end > head + 1 and lines[end - 1].strip() == "":
        end -= 1
    block = lines[head + 1:end]
    if engine == "physx":
        keys = {"vel_ctl": _fmt_list(vel_ctl) if vel_ctl else None,
                "wheel_friction": _fmt_friction(wheel_friction) if wheel_friction else None}
        for key, val in keys.items():
            if val is None:
                continue
            idx = next((i for i, l in enumerate(block) if re.match(rf"    {key}:", l)), None)
            if idx is None:
                # before a newton: sub-block, if any, so the model's own keys come first
                nidx = next((i for i, l in enumerate(block) if l.startswith("    newton:")), len(block))
                block.insert(nidx, f"    {key}: {val}")
            else:
                block[idx] = f"    {key}: {val}"
    else:
        nidx = next((i for i, l in enumerate(block) if l.startswith("    newton:")), None)
        if nidx is None:
            block.append("    newton:")
            nidx = len(block) - 1
        nend = nidx + 1
        while nend < len(block) and block[nend].startswith("      "):
            nend += 1
        sub = block[nidx + 1:nend]
        for key, val in (("vel_ctl", _fmt_list(vel_ctl) if vel_ctl else None),
                         ("wheel_friction", _fmt_friction(wheel_friction) if wheel_friction else None),
                         ("wheel_max_force", f"{wheel_max_force:g}" if wheel_max_force else None),
                         ("wheel_damping", f"{wheel_damping:g}" if wheel_damping else None)):
            if val is None:
                continue
            idx = next((i for i, l in enumerate(sub) if re.match(rf"      {key}:", l)), None)
            if idx is None:
                sub.append(f"      {key}: {val}")
            else:
                sub[idx] = f"      {key}: {val}"
        block[nidx + 1:nend] = sub
    if note:
        tag = f"    # {note}"
        block = [l for l in block if not l.startswith("    # calibrate_all") or engine not in l] + [tag]
    lines[head + 1:end] = block
    open(path, "w").write("\n".join(lines) + "\n")
