from pathlib import Path


def design_rules_save(src):
    base = Path(src)
    return {ext: base.with_suffix(ext).read_bytes() for ext in ('.kicad_pro', '.kicad_dru')
            if base.with_suffix(ext).exists()}


def design_rules_goal_clear(out):
    for ext in ('.kicad_pro', '.kicad_dru'):
        Path(out).with_suffix(ext).unlink(missing_ok=True)


def design_rules_copy(data, out, generated_pro=False):
    for ext in ('.kicad_pro', '.kicad_dru'):
        goal = Path(out).with_suffix(ext)
        if ext in data:
            goal.write_bytes(data[ext])
        elif not (generated_pro and ext == '.kicad_pro'):
            goal.unlink(missing_ok=True)
