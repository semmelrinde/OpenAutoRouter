

def measure(B, R):
    import math
    from common.zones import board_geometry
    from postprocessing.short_circuit_protection import export_contacts, track_preview
    try:
        for t in R['tracks']:
            q, _ = track_preview(t)
            if len(q['pts']) < 2 or len(q['w']) != len(q['pts']) - 1:
                raise ValueError('R44: incomplete track geometry')
            if not all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in q['pts']):
                raise ValueError('R44: invalid track points')
            if not all(math.isfinite(w) and w > 0 for w in q['w']):
                raise ValueError('R44: invalid track width')
        default_d = float(str((R.get('options') or {}).get('via', '.45/.20')).split('/')[0])
        for v in R['vias']:
            d = float(v.get('d', v.get('diameter', default_d)))
            if not all(math.isfinite(v[k]) for k in ('x','y')) or not math.isfinite(d) or d <= 0:
                raise ValueError('R44: invalid via geometry')
        contacts, inherited = export_contacts(board_geometry(B, R), R)
        def pairs(rows):
            return {(q['layer_name'], *sorted(q['net_list'])) for q in rows}
        movable, pinned = pairs(contacts), pairs(inherited)
        return dict(version='r44-contact_pairs-v1', measured=True,
                    contact_pairs=len(movable | pinned), movable_pairs=len(movable),
                    inherited_pad_pairs=len(pinned), contacts=len(contacts),
                    inherited_pad_contacts=len(inherited),
                    examples=(contacts + inherited)[:8])
    except Exception as exc:

        return dict(version='r44-contact_pairs-v1', measured=False,
                    contact_pairs=None, faults=repr(exc)[:200])


def passed(m):
    return m.get('measured') is True and m.get('contact_pairs') == 0
