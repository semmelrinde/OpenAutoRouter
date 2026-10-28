""
from pathlib import Path


def regeln_sichern(src):
    base = Path(src)
    return {ext: base.with_suffix(ext).read_bytes() for ext in ('.kicad_pro', '.kicad_dru')
            if base.with_suffix(ext).exists()}


def regeln_ziel_leeren(out):
    ""
    for ext in ('.kicad_pro', '.kicad_dru'):
        Path(out).with_suffix(ext).unlink(missing_ok=True)


def regeln_kopieren(daten, out, erzeugtes_pro=False):
    ""
    for ext in ('.kicad_pro', '.kicad_dru'):
        ziel = Path(out).with_suffix(ext)
        if ext in daten:
            ziel.write_bytes(daten[ext])
        elif not (erzeugtes_pro and ext == '.kicad_pro'):
            ziel.unlink(missing_ok=True)
